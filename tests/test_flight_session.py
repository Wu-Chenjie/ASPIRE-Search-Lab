import pytest
from annual_runtime.flight_session import FlightSessionFault


def test_terminal_fault_is_latched_and_healthy_reports_cannot_rearm_session():
    gate=FlightSessionFault(1)
    assert not gate.observe(dict(drone=1,time=10.,alignment_fault=None),10.)
    assert gate.observe(dict(drone=1,time=10.1,alignment_fault='frame_reset'),10.1)
    assert not gate.observe(dict(drone=1,time=10.2,alignment_fault=None),10.2)
    assert (gate.reason,gate.time)==('frame_reset',10.1)


@pytest.mark.parametrize('packet', [
    dict(drone=2,time=10.,alignment_fault='reset'),
    dict(drone=True,time=10.,alignment_fault='reset'),
    dict(drone=1,time=7.,alignment_fault='reset'),
    dict(drone=1,time=11.,alignment_fault='reset'),
    dict(drone=1,time=float('nan'),alignment_fault='reset'),
    dict(drone=1,time=10.,armed=False,failsafe=True),
    {}, None,
])
def test_invalid_foreign_or_nonterminal_report_cannot_retire_session(packet):
    gate=FlightSessionFault(1)
    assert not gate.observe(packet,10.) and gate.reason is None


def test_latched_failsafe_is_terminal_even_after_landing():
    gate=FlightSessionFault(0)
    assert gate.observe(dict(drone=0,time=2.,armed=False,failsafe=False,failsafe_latched=True),2.)
    assert gate.reason=='px4_failsafe_requires_new_session'
