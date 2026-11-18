import json,queue,sys,time
from pathlib import Path
from types import SimpleNamespace as N
import numpy as np
import pytest
from annual_runtime.trigger_evidence import EvidenceRing,optional_ring
from annual_runtime.map_evidence import capture_safety,QueryTraceCorridor

def saved(ring):
    return [dict(meta=json.loads(bytes(z['metadata_json']).decode()),array=z['a'].copy()) for p in sorted(ring.folder.glob('*.npz')) for z in [np.load(p)]]

def test_trigger_keeps_immutable_pre_and_real_post_and_deduplicates(tmp_path):
    ring=EvidenceRing(tmp_path,0,'mapper',pre_records=2,post_records=2)
    a=np.arange(12,dtype=np.float32)
    for t in [1.,2.,3.]:ring.record('input',t,{'sequence':t},{'a':a})
    ring.trigger('test',3,trigger_id='one');assert ring.trigger('duplicate',3,trigger_id='one') is None
    a[:]=99
    for t in [4.,5.]:ring.record('input',t,{'sequence':t},{'a':a})
    ring.close();rows=saved(ring)
    assert [x['meta']['record']['sim_s'] for x in rows]==[2,3,4,5]
    np.testing.assert_array_equal(rows[0]['array'],np.arange(12));assert rows[-1]['array'][0]==99
    status=json.loads((ring.folder/'status.json').read_text());assert status['windows'][0]['post_captured']==2
    assert status['written']==4 and not status['writer_alive'] and status['pending']==0
    assert status['queue_dropped']==status['write_errors']==0

def test_caps_memory_record_size_trigger_count_and_disk(tmp_path):
    ring=EvidenceRing(tmp_path,0,'mapper',memory_bytes=256,max_record_bytes=256,max_disk_bytes=1,max_triggers=2,post_records=0)
    for i in range(20):ring.record('input',i*.1,{}, {'a':np.ones(20,np.uint8)})
    assert ring.ring_bytes<=256
    ring.record('oversize',3,{}, {'a':np.ones(1024,np.uint8)});assert ring.oversize==1
    assert ring.trigger('one',3) and ring.trigger('two',3)
    assert ring.trigger('three',3) is None and ring.trigger_dropped==1
    ring.close();assert ring.disk_dropped>0 and ring.disk_bytes==0

def test_wrong_session_or_drone_trigger_ignored_and_shutdown_tail_explicit(tmp_path):
    ring=EvidenceRing(tmp_path,0,'mapper');ring.record('input',1,{}, {'a':np.zeros(3)})
    ring.receive_trigger(json.dumps(dict(schema=1,kind='map_evidence_trigger',id='x',drone=1,session=ring.session,reason='x',sim_s=1)))
    assert not ring.windows;ring.close();w=ring.windows[0]
    assert w['trigger']['reason']=='shutdown_tail_no_trigger'
    assert w['post_captured']==0 and w['post_requested']==2

def test_invalid_trigger_payloads_leave_diagnostics_available(tmp_path):
    ring = EvidenceRing(tmp_path, 0, 'mapper')
    trigger = dict(schema=1, kind='map_evidence_trigger', id='valid', drone=0,
                   session=ring.session, reason='test', sim_s=1)
    try:
        nested = '[' * 1200 + '0' + ']' * 1200
        for data in ('null', '[]', '"text"', nested,
                     json.dumps(dict(trigger, sim_s=10**400))):
            ring.receive_trigger(data)
        assert ring.write_errors == 5
        assert not ring.windows
        ring.receive_trigger(json.dumps(trigger))
        assert ring.windows[0]['trigger']['id'] == 'valid'
    finally:
        ring.close()

def test_queue_backpressure_is_nonblocking_and_never_claims_post_capture(tmp_path,monkeypatch):
    ring=EvidenceRing(tmp_path,0,'mapper',post_records=1)
    ring.trigger('x',1)
    def full(*a,**kw):raise queue.Full
    monkeypatch.setattr(ring.queue,'put_nowait',full)
    start=time.monotonic();ring.record('post',2,{}, {'a':np.ones(3)})
    assert time.monotonic()-start<.2 and ring.queue_dropped==1
    assert ring.windows[0]['post_captured']==0
    ring.close()

@pytest.mark.parametrize('raise_policy_error', [False, True])
def test_trigger_publisher_failure_preserves_policy_outcome(tmp_path, monkeypatch, raise_policy_error):
    class PublisherError(RuntimeError):
        pass

    def publish(message):
        raise PublisherError('diagnostic publisher unavailable')

    monkeypatch.setitem(sys.modules, 'std_msgs.msg', N(String=lambda **kwargs: N(**kwargs)))
    ring = EvidenceRing(tmp_path, 0, 'executor')
    state = np.zeros((1, 1, 1), dtype=np.int8)
    position = np.zeros(3)
    grid = N(state=state, origin=position, resolution=.3, clearance=.5, flight_limits=(.7, 3.), version=1)
    native = N(state=state, origin=position, resolution=.3, stamp=1., sequence=1)
    node = N(map_evidence=ring, map_evidence_pub=N(publish=publish), safety_fault=None,
             safety_replica=N(map=grid), native_safety=native, token='current', epoch=1,
             native_plan_id=1, safety_stamp=1., id=0, stamps={0: 1.}, actual_yaw=0., heading=0.,
             yaw_rate=0., curve=None, curve_time=0., safety_stopping=False, safety_stop_deadline=None,
             observation_failure=None, ref=position, positions={0: position}, velocity=position,
             acceleration=position)

    @capture_safety
    def policy(node, t):
        node.safety_fault = 'policy fault'
        if raise_policy_error:
            raise RuntimeError('original policy error')
        return 'policy outcome'

    try:
        if raise_policy_error:
            with pytest.raises(RuntimeError, match='original policy error'):
                policy(node, 1.)
        else:
            assert policy(node, 1.) == 'policy outcome'
        assert ring.write_errors == 1
        assert ring.records == 1
        assert ring.windows[0]['trigger']['reason'] == 'execution_safety_fault'
    finally:
        ring.close()

def test_opt_in_off_and_source_tree_destination_rejected(monkeypatch):
    monkeypatch.delenv('RACER_MAP_EVIDENCE_DIR',raising=False);assert optional_ring(0,'mapper') is None
    monkeypatch.setenv('RACER_MAP_EVIDENCE_DIR',str(Path(__file__).resolve().parents[1]/'forbidden-evidence'))
    assert optional_ring(0,'mapper') is None
    assert not (Path(__file__).resolve().parents[1]/'forbidden-evidence').exists()

def test_observed_corridor_returns_identical_results_without_mutating_map():
    from core.exploration.voxel_mapping import VoxelMap
    from core.planning.execution_safety import NativeSafetyMap,ExecutionCorridor
    m=VoxelMap([[0,0,0],[6,6,4]],flight_limits=(.7,3.));m.state[:]=0;m.rebuild();n=NativeSafetyMap();n.ingest([1,1,1,.3,0,0,0,*m.shape,*np.ones(m.state.size)])
    base=ExecutionCorridor(m,n);trace=QueryTraceCorridor(base);before=m.state.copy()
    for a,b in [(np.array([2,2,2]),np.array([2.2,2.2,2.2])),(np.array([-1,2,2]),np.array([-1,2,2]))]:assert base.box_safe(a,b)==trace.box_safe(a,b)
    assert trace.query_count==2 and trace.reject_count==1;np.testing.assert_array_equal(before,m.state)


def test_receipt_metadata_is_bounded_and_interpolation_endpoints_are_preserved(tmp_path):
    from annual_runtime.map_evidence import mapper_received,mapper_before
    from annual_runtime.pose_history import PoseHistory
    from core.exploration.voxel_mapping import VoxelMap
    ring=EvidenceRing(tmp_path,0,'mapper');poses=PoseHistory()
    poses.add(1.,[1,2,1],[0,0,0,1]);poses.add(1.02,[1.02,2,1],[0,0,0,1])
    m=N(header=N(stamp=N(sec=1,nanosec=10000000),frame_id='depth'),encoding='32FC1',step=4,is_bigendian=False,width=1,height=1,data=np.array([2.],np.float32).tobytes())
    cal=N(header=m.header,width=1,height=1,k=[1.,0,0,0,1.,0,0,0,1.])
    vm=VoxelMap([[0,0,0],[6,6,4]])
    node=N(map_evidence=ring,_map_evidence_received={},_map_evidence_receipt_ordinal=0,pending=[],get_clock=lambda:N(now=lambda:N(nanoseconds=1020000000)),poses=poses,
        calibration=cal,session='test',sequence=0,map=vm,belief=np.zeros(vm.shape,np.float32),ego_cells=set())
    messages=[N() for _ in range(9)]
    for msg in messages:mapper_received(node,msg)
    assert len(node._map_evidence_received)==8 and id(messages[0]) not in node._map_evidence_received
    mapper_received(node,m);context=mapper_before(node,m,1.01,poses.at(1.01))
    assert context['metadata']['receipt']['receipt_ordinal']==10
    assert context['metadata']['pose_endpoint_samples_complete']
    np.testing.assert_array_equal(context['arrays']['pose_endpoint_positions'],[[1,2,1],[1.02,2,1]])
    np.testing.assert_allclose(context['arrays']['estimated_position'],[1.01,2,1])
    assert id(m) not in node._map_evidence_received;ring.close()


def test_live_health_is_atomic_bounded_and_surfaces_idle_errors(tmp_path):
    ring=EvidenceRing(tmp_path,0,'mapper');end=time.monotonic()+3
    while not (ring.folder/'health.json').exists():
        assert time.monotonic()<end;time.sleep(.01)
    ring.write_errors+=1
    while True:
        x=json.loads((ring.folder/'health.json').read_text())
        if x['write_errors']==1:break
        assert time.monotonic()<end;time.sleep(.01)
    assert x['records']==0 and x['sequence']>=2
    assert (ring.folder/'health.json').stat().st_size<=4096;ring.close()

def test_live_capture_gate_rejects_error_stale_session_and_delay():
    from tools.flight.capture_health import health_fault
    x=dict(schema=1,session='a',wall_ns=10**9,disk_limit=33554432,hook_max_wall_s=.001)
    assert health_fault(x,2*10**9,'a') is None
    for patch in [dict(write_errors=1),dict(queue_dropped=1),dict(oversize=1),dict(wall_ns=-4*10**9),dict(session='b'),dict(closed=True),dict(hook_max_wall_s=.201)]:
        assert health_fault(dict(x,**patch),2*10**9,'a')
