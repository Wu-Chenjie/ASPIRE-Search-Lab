import numpy as np
import pytest
from scipy.interpolate import BSpline
from core.planning.bspline_trajectory import native_trajectory,append_safe_brake,native_prefix,safe_native_execution
from core.planning.handoff import continuity
from types import SimpleNamespace
from core.planning.continuous_trajectory import ContinuousTrajectory


def test_native_curve_and_yaw_are_preserved_through_serialization():
    rng=np.random.default_rng(18)
    # Long knot intervals keep the deliberately random curve within the common envelope.
    knots=np.arange(-3,10)*8.
    points=rng.uniform(-.1,.1,(9,3)); yaw=rng.uniform(-.08,.08,9)
    packet=dict(order=3,knots=knots.tolist(),points=points.tolist(),yaw_points=yaw.tolist(),yaw_dt=8.)
    curve=native_trajectory(packet); restored=ContinuousTrajectory.from_dict(curve.to_dict())
    pos=BSpline(knots,points,3); heading=BSpline(knots,yaw,3)
    for t in np.r_[np.linspace(0,48,117),np.arange(0,49,8)]:
        p,v,a,y,r=restored.sample(t)
        for k,actual in enumerate((p,v,a)): np.testing.assert_allclose(actual,pos(t,nu=k),atol=1e-12)
        assert y==pytest.approx(heading(t),abs=1e-12)
        assert r==pytest.approx(heading(t,nu=1),abs=1e-12)
        assert restored.yaw_acceleration(t)==pytest.approx(heading(t,nu=2),abs=1e-12)
    batch=restored.sample_many(np.linspace(0,48,121))
    np.testing.assert_allclose(batch[3],heading(np.linspace(0,48,121)),atol=1e-12)


def test_native_limits_are_rejected_instead_of_rescaled():
    packet=dict(order=3,knots=list(np.arange(-3,6)),points=[[i*3.,0,0] for i in range(5)],yaw_points=[0.]*5,yaw_dt=1.)
    with pytest.raises(ValueError,match='dynamics envelope'): native_trajectory(packet)


def test_native_yaw_domain_mismatch_is_rejected():
    packet=dict(order=3,knots=list(np.arange(-3,6)),points=[[0.,0,0]]*5,yaw_points=[0.]*5,yaw_dt=2.)
    with pytest.raises(ValueError,match='domains differ'): native_trajectory(packet)


def test_contingency_preserves_prefix_and_stops_with_c2_boundary():
    packet=dict(order=3,knots=list(np.arange(-3,6)*4.),points=[[i*.3,0,1.5] for i in range(5)],yaw_points=[i*.04 for i in range(5)],yaw_dt=4.)
    native=native_trajectory(packet)
    curve=append_safe_brake(native,SimpleNamespace(safe_path=lambda p:True))
    for t in np.linspace(0,native.duration,29):
        for a,b in zip(native.sample(t),curve.sample(t)):np.testing.assert_allclose(a,b,atol=1e-12)
    for value in continuity(native,ContinuousTrajectory([curve.durations[-1]],curve.coefficients[-1:],curve.sample(native.duration)[3],
        curve.yaw_delta,yaw_coefficients=curve.yaw_segments[-1]),native.duration).values(): assert value<1e-10
    end=curve.sample(curve.duration)
    np.testing.assert_allclose(end[1:3],np.zeros((2,3)),atol=1e-12)
    assert abs(end[4])+abs(curve.yaw_acceleration(curve.duration))<1e-12
    with pytest.raises(ValueError,match='terminal brake'):append_safe_brake(native,SimpleNamespace(safe_path=lambda p:False))


def test_shortened_horizon_retains_native_position_and_independent_yaw():
    packet=dict(order=3,knots=list(np.arange(-3,10)*4.),points=[[i*.3,0,1.5] for i in range(9)],
        yaw_points=[i*.04 for i in range(14)],yaw_dt=24./11.)
    curve=native_trajectory(packet)
    for end in (4.,7.17,curve.duration):
        cut=native_prefix(curve,end)
        for t in np.linspace(0,end,39):
            for a,b in zip(curve.sample(t),cut.sample(t)):np.testing.assert_allclose(a,b,atol=1e-12)
            assert curve.yaw_acceleration(t)==pytest.approx(cut.yaw_acceleration(t),abs=1e-12)
    # The full curve crosses unknown space; only a prefix whose entire brake
    # also fits may be returned. Its stopped endpoint must stay inside the gate.
    gate=SimpleNamespace(safe_path=lambda p:bool(np.all(p[:,0]<=.9)))
    stopped,end=safe_native_execution(curve,gate)
    assert .5<=end<curve.duration
    assert gate.safe_path(stopped.path(.01))
    assert np.linalg.norm(stopped.sample(stopped.duration)[1])<1e-12
    assert abs(stopped.sample(stopped.duration)[4])<1e-12
    for t in np.linspace(0,end,19):
        for a,b in zip(curve.sample(t),stopped.sample(t)):np.testing.assert_allclose(a,b,atol=1e-12)


def test_frontier_step_rejects_a_brake_outside_the_continuous_corridor(monkeypatch):
    # Point samples alone accept everything. The continuous checker rejects
    # first the long route and then a brake that crosses the measured boundary.
    monkeypatch.setenv('ASPIRE_FRONTIER_STEPS', '1')
    packet=dict(order=3,knots=list(np.arange(-3,10)*4.),points=[[i*.3,0,1.5] for i in range(9)],
                yaw_points=[0.]*9,yaw_dt=4.)
    native=native_trajectory(packet)
    limit=native.sample(native.duration/2)[0][0]+.001
    gate=SimpleNamespace(safe_path=lambda p:True,
        safe_curve=lambda c:bool(np.max(c.path(.02)[:,0])<=limit))
    stopped,used=safe_native_execution(native,gate)
    assert 0<used<native.duration/2
    assert gate.safe_curve(stopped)
    end=stopped.sample(stopped.duration)
    assert np.linalg.norm(end[1])+np.linalg.norm(end[2])<1e-8
    with pytest.raises(ValueError,match='No known-free'):
        safe_native_execution(native,SimpleNamespace(safe_curve=lambda c:False,safe_path=lambda p:True))


def test_next_brake_duration_is_checked_before_shortening_native_prefix(monkeypatch):
    monkeypatch.setenv('ASPIRE_FRONTIER_STEPS', '1')
    packet = dict(order=3, knots=list(np.arange(-3, 6)*4.),
                  points=[[i*.3, 0, 1.5] for i in range(5)], yaw_points=[0.]*5, yaw_dt=4.)
    native = native_trajectory(packet)
    native_end = native.sample(native.duration)[0][0]
    # Model a continuous gate rejecting the first dynamically feasible (.6 s)
    # stop. A different terminal stop may satisfy the gate at the same prefix.
    def safe_curve(candidate):
        end = candidate.sample(candidate.duration)[0][0]
        return not native_end+.02 < end < native_end+.027

    gate = SimpleNamespace(safe_curve=safe_curve, safe_path=lambda p: True)
    stopped, used = safe_native_execution(native, gate)
    assert used == native.duration
    assert stopped.duration-used == pytest.approx(.8)
    assert gate.safe_curve(stopped)


def test_stopping_reserve_rejection_tries_another_brake_at_same_prefix():
    packet = dict(order=3, knots=list(np.arange(-3, 6)*.5),
                  points=[[i*.0375, 0, 1.5] for i in range(5)], yaw_points=[0.]*5, yaw_dt=.5)
    native = native_trajectory(packet)
    gate = SimpleNamespace(frontier_sitl=True, safe_curve=lambda c: True, safe_path=lambda p: True)
    checked = []

    def validate_stop(candidate):
        checked.append(candidate.durations[-1])
        if candidate.durations[-1] < .8:
            raise ValueError('no stopping reserve')

    stopped, used = safe_native_execution(native, gate, validate_stop=validate_stop, reserve_horizon=1.2)
    assert used == native.duration
    assert stopped.duration-used == pytest.approx(.8)
    assert checked == [.6, .8]


def test_fixed_horizon_reserve_is_checked_once_per_long_prefix():
    packet = dict(order=3, knots=list(np.arange(-3, 6)*4.),
                  points=[[i*.3, 0, 1.5] for i in range(5)], yaw_points=[0.]*5, yaw_dt=4.)
    native = native_trajectory(packet)
    gate = SimpleNamespace(frontier_sitl=True, safe_curve=lambda c: True, safe_path=lambda p: True)
    checked = []

    def validate_stop(candidate):
        assert candidate.method == 'native_racer_bspline_with_brake'
        checked.append(candidate.duration)
        if candidate.knots[-2] == native.duration:
            raise ValueError('reserve failed before any terminal brake')

    stopped, used = safe_native_execution(native, gate, validate_stop=validate_stop, reserve_horizon=1.2)
    assert 1.2 < used < native.duration
    assert checked == [native.duration+.6, stopped.duration]


@pytest.mark.parametrize('clear',[False,True])
def test_stopped_native_prefix_checks_same_measured_path_once(monkeypatch,clear):
    monkeypatch.delenv('ASPIRE_FRONTIER_STEPS',raising=False)
    packet=dict(order=3,knots=list(np.arange(-3,6)),points=[[1.,2.,1.5]]*5,
                yaw_points=[0.]*5,yaw_dt=1.)
    curve=native_trajectory(packet)
    calls=[]
    def check_path(points):
        calls.append(points)
        return clear
    gate=SimpleNamespace(safe_path=check_path)
    if clear:
        stopped,end=safe_native_execution(curve,gate)
        assert stopped is curve and end==curve.duration
        assert len(calls)==1
    else:
        with pytest.raises(ValueError,match='No known-free native prefix'):
            safe_native_execution(curve,gate)
        assert calls


def test_stepped_continuous_certification_skips_redundant_sampled_paths(monkeypatch):
    monkeypatch.setenv('ASPIRE_FRONTIER_STEPS', '1')
    packet = dict(order=3, knots=list(np.arange(-3, 6) * 4.),
                  points=[[i * .3, 0, 1.5] for i in range(5)], yaw_points=[0.] * 5, yaw_dt=4.)
    native = native_trajectory(packet)
    path_calls = []
    curve_calls = []

    def check_path(points):
        path_calls.append(points)
        return True

    def check_curve(candidate):
        curve_calls.append(candidate)
        return True

    gate = SimpleNamespace(safe_curve=check_curve, safe_path=check_path)
    stopped, used = safe_native_execution(native, gate)
    assert used == native.duration
    assert stopped.duration > used
    assert path_calls == []
    assert len(curve_calls) == 2
