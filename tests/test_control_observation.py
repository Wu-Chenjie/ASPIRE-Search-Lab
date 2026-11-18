import json,math,threading
from pathlib import Path
from types import SimpleNamespace as N
import pytest
from annual_runtime.control_observation import ObservationLog, record, reference_record, reference_payload, setpoint_record
from annual_runtime.control_timing import ControlTiming
from annual_runtime.px4_frames import trajectory_to_ned

def v(x,y,z):return N(x=x,y=y,z=z)
def message(stamp=10):
 p=N(transforms=[N(translation=v(3.,6.,1.5),rotation=N(x=0.,y=0.,z=0.,w=1.))],velocities=[N(linear=v(.5,0.,0.),angular=v(0.,0.,.4))],accelerations=[N(linear=v(0.,.2,0.))])
 return N(header=N(frame_id='world',stamp=N(sec=stamp,nanosec=0)),points=[p])
def node(sink):
 return N(control_timing=N(observation=sink),get_clock=lambda:N(now=lambda:N(nanoseconds=10**10)),
          origin=[2.,6.,.24],position=None,last_sample=0,resets=None,attitude_reset=None,attitude_sample=0,
          position_reset_pending=False,alignment_fault=None)
def rows(p):return [json.loads(s) for s in p.read_text().splitlines()]

def test_disabled_is_lazy_and_does_not_modify_publish(tmp_path,monkeypatch):
 monkeypatch.delenv('RACER_CONTROL_OBSERVATION',raising=False);t=ControlTiming(tmp_path,0,'off');n=node(None)
 record(n,'disabled',lambda:1/0);sent=[];msg=message();t.publish(N(publish=sent.append),msg,'reference_publish',10**10);t.close()
 assert sent==[msg] and t.observation is None and not list(tmp_path.glob('*observation*'))

def test_existing_timing_trace_does_not_block_publications(tmp_path, caplog):
    previous = tmp_path / 'drone-0-bridge.jsonl'
    previous.write_text('previous session\n')
    timing = ControlTiming(tmp_path, 0, 'bridge')
    sent = []
    timing.publish(N(publish=sent.append), 'heartbeat', 'heartbeat_publish', 1)
    timing.close()
    assert sent == ['heartbeat']
    assert previous.read_text() == 'previous session\n'
    assert 'timing trace' in caplog.text


def test_timing_write_failure_leaves_control_publisher_errors_visible(caplog):
    def fail_write(row):
        raise OSError('disk full')

    def fail_publish(message):
        raise RuntimeError('control publisher unavailable')

    timing = ControlTiming('', 0, 'bridge')
    timing.file = N(write=fail_write, close=lambda: None)
    timing.event('tick')
    sent = []
    timing.publish(N(publish=sent.append), 'heartbeat', 'heartbeat_publish', 1)
    assert sent == ['heartbeat'] and timing.file is None
    assert 'disk full' in caplog.text

    with pytest.raises(RuntimeError, match='control publisher unavailable'):
        timing.publish(N(publish=fail_publish), 'heartbeat', 'heartbeat_publish', 2)


def test_received_duplicate_and_distinct_payload_have_exact_keys(tmp_path):
 a=ObservationLog(tmp_path,0,'executor');b=ObservationLog(tmp_path,0,'bridge');na,nb=node(a),node(b);msg=message();before=reference_payload(msg)
 record(na,'reference_sent',lambda:reference_record(na,msg));record(nb,'reference_received',lambda:reference_record(nb,msg));record(nb,'reference_accepted',lambda:reference_record(nb,msg,accepted=True))
 target=trajectory_to_ned(before['position'],before['velocity'],before['acceleration'],before['yaw'],before['yaw_rate'],na.origin)
 m=N(timestamp=10**7,yawspeed=target['yawspeed'],**{k:target[k] for k in ('position','velocity','acceleration','yaw')})
 for _ in range(2):record(nb,'setpoint_sent',lambda:setpoint_record(nb,m))
 msg.points[0].transforms[0].translation.z+=.001
 record(nb,'reference_received',lambda:reference_record(nb,msg));a.close();b.close();ar=rows(a.path);br=rows(b.path)
 assert ar[0]['reference_key']==br[0]['reference_key']==br[1]['reference_key']==br[2]['reference_key']==br[3]['reference_key']
 assert br[4]['reference_key']!=ar[0]['reference_key'] and br[3]['event_seq']==2
 assert ar[0]['enu']==before and ar[0]['enu']['position'][2]==1.5
 fields = ('origin', 'position', 'position_sample_us', 'resets', 'attitude_reset', 'attitude_sample', 'reset_pending', 'alignment_fault')
 assert all(ar[0][field] is None for field in fields)
 assert br[1]['origin'] == nb.origin and br[1]['position_sample_us'] == 0 and br[1]['reset_pending'] is False
 assert br[2]['ned']['position']==[0.,1.,-1.26] and br[2]['ned']['velocity']==[0.,.5,0.]
 assert br[2]['ned']['acceleration']==[.2,0.,0.] and br[2]['ned']['yaw_rate']==-.4
 assert abs(br[2]['ned']['yaw']-math.pi/2)<1e-12

def test_mutable_fields_detached_and_nonfinite_explicit(tmp_path):
 s=ObservationLog(tmp_path/'new',0,'freeze');x=[1.,float('nan')];s.event('snapshot',x=x);x[0]=99;s.close();r=rows(s.path)
 assert r[0]['x']==[1.,'nan'];status=json.loads(s.status_path.read_text());assert status['written']==1 and status['dropped']==0 and status['write_errors']==0

def test_queue_overflow_and_byte_limit_are_reported(tmp_path):
 release=threading.Event()
 class Slow(ObservationLog):
  def _write(self):release.wait(2.);super()._write()
 s=Slow(tmp_path,0,'overflow',capacity=2,max_bytes=300)
 for i in range(30):s.event('packet',value='x'*100)
 release.set();s.close();status=json.loads(s.status_path.read_text());assert status['dropped']>0 and not status['writer_alive'];assert status['bytes']<=300

def test_supplier_failure_or_file_collision_never_raises_into_control(tmp_path):
 s=ObservationLog(tmp_path,0,'error');n=node(s);record(n,'malformed',lambda:1/0);s.close();assert rows(s.path)[0]['event']=='observation_error'
 collision=ObservationLog(tmp_path,0,'error');record(node(collision),'packet',lambda:dict(p=[1,2,3]));collision.close();assert collision.write_errors and collision.dropped


def test_gzip_observation_roundtrip_preserves_records_and_close_status(tmp_path):
    import gzip,json
    from annual_runtime.control_observation import ObservationLog
    log=ObservationLog(tmp_path,0,'executor',compress=True)
    for i in range(25):log.event('reference_sent',sim_ns=i*20000000,reference=[.1,2.,3.],nested={'unobserved':True})
    log.close()
    assert log.path.name=='drone-0-executor-observation.jsonl.gz'
    with gzip.open(log.path,'rt') as f:rows=[json.loads(l) for l in f]
    assert len(rows)==25 and [r['event_seq'] for r in rows]==list(range(1,26))
    assert all(r['reference']==[.1,2.,3.] and r['nested']=={'unobserved':True} for r in rows)
    status=json.loads(log.status_path.read_text())
    assert status['written']==25 and not status['dropped'] and not status['write_errors']
    assert status['compression']=='gzip' and status['byte_budget_basis']=='uncompressed_utf8'
    assert status['stored_bytes']==log.path.stat().st_size


def test_gzip_keeps_original_uncompressed_byte_budget(tmp_path):
    import gzip,json
    from annual_runtime.control_observation import ObservationLog
    log=ObservationLog(tmp_path,0,'bridge',compress=True,max_bytes=200)
    log.event('large',payload='a'*2000);log.close()
    with gzip.open(log.path,'rt') as f:assert f.read()==''
    status=json.loads(log.status_path.read_text())
    assert status['dropped']==1 and status['written']==0 and status['max_bytes']==200


def test_control_timing_wires_opt_in_gzip_without_changing_record_fields(tmp_path,monkeypatch):
    import gzip,json
    from annual_runtime.control_timing import ControlTiming
    monkeypatch.setenv('RACER_CONTROL_OBSERVATION','1')
    monkeypatch.setenv('RACER_CONTROL_OBSERVATION_GZIP','1')
    trace=ControlTiming(tmp_path,0,'executor')
    trace.observation.event('reference_sent',sim_ns=100,token='native-test',reference=[1,2,3])
    target=trace.observation.path;trace.close()
    with gzip.open(target,'rt') as f:rows=[json.loads(l) for l in f]
    assert len(rows)==1 and rows[0]['sim_ns']==100 and rows[0]['token']=='native-test'
    assert rows[0]['reference']==[1,2,3]
    assert not (tmp_path/'drone-0-executor-observation.jsonl').exists()
