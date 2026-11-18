"""Actual bridge methods with controlled clocks/messages, no DDS or aircraft."""
import json,os,time
from pathlib import Path
from types import SimpleNamespace
import pytest
pytest.importorskip('rclpy');pytest.importorskip('px4_msgs')
from builtin_interfaces.msg import Time
from px4_msgs.msg import VehicleAttitude,VehicleStatus,EstimatorStatusFlags
from test_px4_bridge import bridge,reference


class Stream:
    def __init__(self,node,position,monkeypatch,rtf=.2,flags_period=.1):
        self.node=node;self.position=position;self.rtf=rtf;self.flags_period=flags_period
        self.wall=100.;self.sim=10.;self.last={k:10. for k in ('attitude','position','flags','status')};self.rows=[]
        monkeypatch.setattr(time,'monotonic',lambda:self.wall)
        monkeypatch.setattr(node,'get_clock',lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=round(self.sim*1e9))))
        node.flags_wall=node.position_wall=node.attitude_wall=node.status_wall=100.
        node.flags_time=node.status_time=10.;node.reference_wall=100.;node.trajectory(reference())
        node.tick()

    def step(self,*,stop=(),aligned=True,paused=False):
        self.wall+=.02
        if not paused:self.sim+=.02*self.rtf
        stamp=round(self.sim*1e6);node=self.node
        def due(k,period):
            if paused or k in stop or self.sim-self.last[k]<period-1e-9:return False
            self.last[k]=self.sim;return True
        if due('flags',self.flags_period):node.estimator_status(EstimatorStatusFlags(timestamp=stamp,cs_tilt_align=True,cs_yaw_align=aligned))
        if due('attitude',.004):node.vehicle_attitude(VehicleAttitude(timestamp_sample=stamp,q=[1.,0.,0.,0.]))
        if due('status',.1):node.vehicle_status(VehicleStatus(timestamp=stamp,arming_state=VehicleStatus.ARMING_STATE_ARMED,nav_state=VehicleStatus.NAVIGATION_STATE_OFFBOARD,pre_flight_checks_pass=True))
        if due('position',.008):self.position.timestamp_sample=stamp;node.local_position(self.position)
        if not paused and 'reference' not in stop:
            r=reference();ns=round(self.sim*1e9);r.header.stamp=Time(sec=ns//10**9,nanosec=ns%10**9);node.trajectory(r)
        before=node.heartbeat_count;node.tick();row=dict(wall_elapsed=self.wall-100.,sim=self.sim,sent=node.heartbeat_count>before,reason=node.reason,**node.heartbeat_diagnostics.last_snapshot);self.rows.append(row);return row

    def run(self,seconds,**kwargs):
        return [self.step(**kwargs) for _ in range(round(seconds/.02))]

    def save(self,name):
        if os.environ.get('HEARTBEAT_DIAG_DIR'):
            p=Path(os.environ['HEARTBEAT_DIAG_DIR']);p.mkdir(parents=True,exist_ok=True)
            (p/(name+'.json')).write_text(json.dumps(self.rows,indent=2))


@pytest.mark.parametrize('rtf',[1.,.2,.1,.05])
def test_real_10hz_flags_keep_old_deadlines_at_low_rtf(bridge,monkeypatch,rtf):
    s=Stream(*bridge,monkeypatch,rtf=rtf);rows=s.run(8.)
    assert all(r['sent'] for r in rows)
    s.save('steady-'+str(rtf))


def test_old_1hz_flags_reproduce_failure_at_point_two(bridge,monkeypatch):
    s=Stream(*bridge,monkeypatch,flags_period=1.);rows=s.run(6.)
    bad=[r for r in rows if not r['sent']];assert bad
    assert all('flags_wall_age' in r['failed_gates'] for r in bad)
    assert all(r['ages']['flags_sim_s']<2. for r in bad)
    s.save('old-period-control')


@pytest.mark.parametrize('fault,gate',[('flags','flags_wall_age'),('position','position_wall_age'),('attitude','attitude_wall_age'),('reference','reference_wall_age')])
def test_real_stream_loss_still_stops_heartbeat(bridge,monkeypatch,fault,gate):
    s=Stream(*bridge,monkeypatch);s.run(1.);rows=s.run(4.,stop=(fault,));bad=[r for r in rows if not r['sent']]
    assert bad and gate in rows[-1]['failed_gates']
    assert bad[0]['wall_elapsed']<=4.03 if fault=='flags' else bad[0]['wall_elapsed']<=1.53
    s.save('loss-'+fault)


def test_clock_pause_stops_with_unchanged_wall_guard(bridge,monkeypatch):
    s=Stream(*bridge,monkeypatch);s.run(1.);rows=s.run(1.,paused=True)
    assert not rows[-1]['sent'] and 'position_wall_age' in rows[-1]['failed_gates']
    assert next(r['wall_elapsed'] for r in rows if not r['sent'])<=1.53
    s.save('pause')


@pytest.mark.parametrize('jump',[-1.,5.])
def test_clock_jump_retires_session_even_with_new_inputs(bridge,monkeypatch,jump):
    s=Stream(*bridge,monkeypatch);s.run(1.);s.sim+=jump;s.step()
    assert s.node.alignment_fault=='ros_clock_discontinuity_requires_new_session'
    assert not any(r['sent'] for r in s.run(1.))
    s.save('jump-'+str(jump))


def test_alignment_failure_stops_and_only_fresh_alignment_recovers(bridge,monkeypatch):
    s=Stream(*bridge,monkeypatch);s.run(1.)
    n=s.node;stamp=round(s.sim*1e6)+1;n.estimator_status(EstimatorStatusFlags(timestamp=stamp,cs_tilt_align=True,cs_yaw_align=False))
    assert not s.step(aligned=False)['sent']
    s.run(.8,aligned=False);assert not s.rows[-1]['sent']
    assert any(r['sent'] for r in s.run(1.,aligned=True))
    # Once PX4 failsafe is observed, no in-session alignment recovery may rearm.
    n.vehicle_status(VehicleStatus(timestamp=round(s.sim*1e6)+1,arming_state=VehicleStatus.ARMING_STATE_ARMED,failsafe=True))
    assert not any(r['sent'] for r in s.run(1.,aligned=True))
    assert n.failsafe_latched
    s.save('alignment-and-failsafe')
