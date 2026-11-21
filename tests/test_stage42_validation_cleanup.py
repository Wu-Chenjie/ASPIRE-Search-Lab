"""Retained dynamics rejection, serialization-free certificates, owned teardown."""
from types import SimpleNamespace as NS
import signal
import numpy as np
import pytest

def test_certified_brake_keeps_exact_jet_without_packet_statistics(monkeypatch):
    from test_execution_safety import recorded_curve,maps
    from core.planning.continuous_trajectory import ContinuousTrajectory
    from core.planning.execution_safety import certified_brake,ExecutionCorridor
    curve=recorded_curve();runtime,native=maps();progress=7.46
    def forbidden(self):raise AssertionError('Packet statistics must not be needed for safety admission')
    monkeypatch.setattr(ContinuousTrajectory,'to_dict',forbidden)
    stop=certified_brake(curve,progress,ExecutionCorridor(runtime,native))
    for a,b in zip(curve.sample(progress),stop.sample(progress)):np.testing.assert_allclose(a,b,atol=1e-10)
    end=stop.sample(stop.duration)
    assert np.linalg.norm(end[1])+np.linalg.norm(end[2])+abs(end[4])+abs(stop.yaw_acceleration(stop.duration))<1e-9

def test_every_candidate_still_passes_dynamics_admission(monkeypatch):
    from test_execution_safety import recorded_curve,maps
    import core.planning.execution_safety as m
    curve=recorded_curve();runtime,native=maps();calls=[]
    limits = m.motion_limits()
    monkeypatch.setattr(m,'motion_limits',lambda:NS(acceleration=limits.acceleration,jerk=limits.jerk,
        accepts=lambda curve:calls.append(curve) or False))
    with pytest.raises(ValueError,match='No certified C2 stop'):
        m.certified_brake(curve,7.46,m.ExecutionCorridor(runtime,native))
    assert len(calls)==11 and len(calls[-1].durations)==3

def test_client_pre_stop_rejects_reused_pid_and_foreign_role(monkeypatch):
    import tools.flight.gazebo_shutdown as m
    scans=[]
    def table():
        scans.append('initial')
        return {101:(1,'old'),999:(1,'foreign')}
    monkeypatch.setattr(m,'snapshot',table)
    monkeypatch.setattr(m,'same_process',lambda pid,birth:pid==101 and birth=='new')
    signals=[];monkeypatch.setattr(m.os,'kill',lambda *x:signals.append(x))
    a=NS(pid=101,poll=lambda:None);foreign=NS(pid=999,poll=lambda:None)
    r=m.signal_owned_clients([a,foreign],{101:'px4',999:'someone_else'},wait_s=0)
    assert not r['signals'] and not signals and scans==['initial']

def test_transport_clients_quiesce_before_public_server_stop(monkeypatch,tmp_path):
    import tools.flight.gazebo_shutdown as m
    events=[];states={1:None,2:None,3:None,4:None}
    children=[NS(pid=i,poll=lambda i=i:states[i]) for i in states]
    names={1:'px4',2:'sensor_bridge',3:'gazebo',4:'xvfb'}
    monkeypatch.setattr(m,'snapshot',lambda:{i:(0,str(i)) for i in states if states[i] is None})
    monkeypatch.setattr(m,'same_process',lambda pid,birth:states[pid] is None and birth==str(pid))
    def kill(pid,sig):
        assert pid in [1,2] and sig==signal.SIGINT;events.append(('client',pid));states[pid]=0
    monkeypatch.setattr(m.os,'kill',kill)
    def public_stop(process,out,budget_s):
        assert states[1]==states[2]==0;events.append(('server',process.pid));return dict(accepted=True)
    monkeypatch.setattr(m,'stop_owned_server',public_stop)
    def terminate(children,timeout,defer_pids,before_signals):
        assert timeout==8 and defer_pids==[4];before_signals();return dict(forced_kill_pids=[])
    monkeypatch.setattr(m,'terminate',terminate)
    r=m.terminate_flight(children,names,tmp_path,timeout=8)
    assert events==[('client',1),('client',2),('server',3)] and r['non_display_budget_s']==8
