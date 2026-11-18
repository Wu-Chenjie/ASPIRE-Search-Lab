"""Fault fixture safety boundaries; actual message loss is validated in SITL."""
import json,signal
from types import SimpleNamespace
import pytest
from tools.flight.fault_injection import FaultSchedule


@pytest.fixture
def fixture(tmp_path,monkeypatch):
    wall=[100.];monkeypatch.setattr('tools.flight.fault_injection.time.monotonic',lambda:wall[0])
    sent=[];agent=SimpleNamespace(pid=123456789,poll=lambda:None,send_signal=sent.append)
    return tmp_path,wall,agent,sent


def test_requires_ready_armed_offboard_and_no_prior_fault(fixture):
    p,w,a,s=fixture;f=FaultSchedule('flags',p,a)
    for flight,execution in [({},{}),({'armed':True,'offboard':False},{'ready':True}),({'armed':True,'offboard':True,'flight_session_fault':'prior'},{'ready':True})]:
        f.step(61.,flight,execution);assert f.state=='waiting'
    assert not (p/'fault-control.json').exists();f.close(61.)


@pytest.mark.parametrize('mode',['flags','position'])
def test_selective_drop_restores_and_only_expected_failsafe_is_observed(fixture,mode):
    p,w,a,s=fixture;f=FaultSchedule(mode,p,a)
    f.step(61.,{'armed':True,'offboard':True},{'ready':True})
    assert json.loads((p/'fault-control.json').read_text())=={'drop':True}
    assert f.allows_observation('px4_failsafe_requires_new_session')
    assert not f.allows_observation('attitude_reset_requires_new_map_session')
    w[0]+=12.;f.step(63.,{},{});assert f.state=='restored'
    assert json.loads((p/'fault-control.json').read_text())=={'drop':False}
    w[0]+=30.;assert f.step(67.,{},{});assert not s;f.close(67.)


def test_cleanup_resumes_only_owned_stopped_agent(fixture):
    p,w,a,s=fixture;f=FaultSchedule('dds',p,a)
    f.apply(True,61.);f.close(62.)
    assert s==[signal.SIGSTOP,signal.SIGCONT]
