import pytest

from tools.hm3d import replay_map_probe


@pytest.mark.parametrize('ready_after_spin',[False,True])
def test_diagnostic_wait_checks_delivery_before_final_timeout(monkeypatch,ready_after_spin):
    clock=[0.]
    replied=[False]
    monkeypatch.setattr(replay_map_probe.time,'monotonic',lambda:clock[0])

    def spin(seconds):
        clock[0]+=seconds+.02
        replied[0]=ready_after_spin

    args=(lambda:replied[0],spin,.03,'No diagnostic reply')
    if ready_after_spin:
        replay_map_probe.wait_until(*args)
    else:
        with pytest.raises(TimeoutError,match='No diagnostic reply'):
            replay_map_probe.wait_until(*args)
    assert clock[0]>=.03
