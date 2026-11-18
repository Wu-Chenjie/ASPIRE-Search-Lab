"""Check fixture ownership/cleanup; not evidence of autonomous flight."""
import signal
from types import SimpleNamespace

import pytest

from tools.flight.map_delay_fixture import MeasuredMapDelay


def test_bounded_mapper_pause_resumes_owned_child_and_requires_brake(tmp_path):
    signals = []
    process = SimpleNamespace(pid=123, poll=lambda: None, send_signal=signals.append)
    fixture = MeasuredMapDelay(process, tmp_path)
    flight = dict(armed=True, offboard=True)
    moving = dict(ready=True, arrived=False, token='test', reference_velocity=[.2, 0., 0.],
                  trajectory_duration=6., trajectory_time=1., stopping_reserve={'reaction_horizon_s': 1.2})
    fixture.step(60., flight, dict(moving, ready=False))
    assert signals == []
    fixture.step(61., flight, moving)
    assert signals == [signal.SIGSTOP]
    fixture.step(62.2, flight, dict(moving, safety_revision={'number': 1}))
    fixture.step(62.9, flight, moving)
    fixture.validate()
    fixture.step(70., flight, moving)
    fixture.close(71.)
    assert signals == [signal.SIGSTOP, signal.SIGCONT]


def test_failed_run_cleanup_resumes_mapper_without_claiming_success(tmp_path):
    signals = []
    process = SimpleNamespace(pid=123, poll=lambda: None, send_signal=signals.append)
    fixture = MeasuredMapDelay(process, tmp_path)
    fixture.step(60., dict(armed=True, offboard=True), dict(ready=True, arrived=False,
        token='test', reference_velocity=[.2, 0., 0.], trajectory_duration=6., trajectory_time=1.,
        stopping_reserve={'reaction_horizon_s': 1.2}))
    with pytest.raises(RuntimeError, match='did not complete'):
        fixture.validate()
    fixture.close(60.1)
    assert signals == [signal.SIGSTOP, signal.SIGCONT]
