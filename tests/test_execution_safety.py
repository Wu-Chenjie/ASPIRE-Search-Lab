import json
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from core.exploration.voxel_mapping import VoxelMap
from core.planning.bspline_trajectory import native_trajectory,append_safe_brake
from core.planning.continuous_trajectory import ContinuousTrajectory
from core.planning.execution_safety import NativeSafetyMap,ExecutionCorridor,certified_brake,polynomial_box
from core.planning.execution_safety import stopped_planning_reference
from core.planning.motion_limits import motion_limits


def maps():
    runtime=VoxelMap([[0,0,0],[18.3,25.5,3.9]],flight_limits=(.7,3.))
    runtime.state[:]=0; runtime.rebuild()
    native=NativeSafetyMap()
    native.ingest([1,89.2,1,.3,0,0,0,*runtime.shape,*np.ones(runtime.state.size)])
    return runtime,native


def recorded_curve():
    packet=json.loads((Path(__file__).parent/'fixtures/active06_native_id9.json').read_text())
    return append_safe_brake(native_trajectory(packet),SimpleNamespace(safe_path=lambda p:True))


def test_clearance_recheck_keeps_the_certified_subdivision():
    runtime, native = maps()
    native.state[4, 5, 5] = 5  # Free but inflated. The diagonal skirts its corner.
    coefficients = np.zeros((6, 3))
    coefficients[0] = [1.2, 1.1999, 1.65]
    coefficients[1] = [.6, .6, 0.]
    curve = ContinuousTrajectory([1.], [coefficients], 0., 0., 'diagonal')
    corridor = ExecutionCorridor(runtime, native)
    for progress in (0., .123, .499, .5, 1.):
        assert corridor.safe_curve(curve, progress)
    native.state[5, 5, 5] = 5  # This cell intersects the remaining trajectory.
    assert not corridor.safe_curve(curve, .123)


def test_native_planning_reference_uses_fixed_hover_instead_of_odometry_drift():
    runtime, native = maps()
    runtime.state[8, 8, 4] = 1
    runtime.rebuild()
    corridor = ExecutionCorridor(runtime, native)
    fixed = [2.55, 2.55, 2.055]
    actual = [2.55, 2.55, 2.045]
    assert corridor.box_safe(fixed, fixed)
    assert not corridor.box_safe(actual, actual)
    report = dict(time=8., position_stamp=7.95, epoch=0, ready=True, arrived=True,
                  reference=fixed, actual_position=actual, reference_yaw=.3,
                  reference_velocity=[0., 0., 0.], reference_acceleration=[0., 0., 0.],
                  reference_yaw_rate=0.)
    assert stopped_planning_reference(report, -1, False, 8.1) == [8., *fixed, .3]
    completed = dict(report, token='accepted', native_plan_id=5, epoch=3)
    assert stopped_planning_reference(completed, 5, True, 8.1) == [8., *fixed, .3]
    assert stopped_planning_reference(completed, 4, True, 8.1) is None
    assert stopped_planning_reference(completed, -1, False, 8.1) is None
    assert stopped_planning_reference(completed, 5, False, 8.1) is None

    for invalid in (dict(ready=False), dict(arrived=False), dict(pending_token='pending'), dict(epoch=3),
                    dict(time=7.5), dict(position_stamp=7.5), dict(actual_position=[2.55, 2.55, 2.315]),
                    dict(reference_velocity=[0., 0., .01]), dict(reference_acceleration=[0., 0., .01]),
                    dict(reference_yaw_rate=.01)):
        assert stopped_planning_reference(dict(report, **invalid), -1, False, 8.1) is None


def test_actual_id9_new_hits_invalidate_brake_and_allow_early_c2_stop():
    runtime,native=maps(); curve=recorded_curve(); corridor=ExecutionCorridor(runtime,native)
    assert corridor.safe_curve(curve)
    for z in range(4,8):runtime.state[31,46,z]=1
    runtime.rebuild()
    # Original executor's physical envelope accepts this historical counterexample.
    assert runtime.safe_path(curve.path(.02))
    assert not corridor.safe_curve(curve)
    # React at first endpoint hits, before slower native occupancy threshold.
    progress=7.46
    stopped=certified_brake(curve,progress,corridor)
    assert corridor.safe_curve(stopped,progress)
    for left,right in zip(curve.sample(progress),stopped.sample(progress)):
        np.testing.assert_allclose(left,right,atol=1e-10)
    end=stopped.sample(stopped.duration)
    np.testing.assert_allclose(end[1:3],0.,atol=1e-10)
    assert abs(end[4])+abs(stopped.yaw_acceleration(stopped.duration))<1e-10
    # No fabricated position/velocity discontinuity before the stopping boundary.
    np.testing.assert_allclose(stopped.path(.1)[0],curve.path(.1)[0])
    assert stopped.duration<curve.duration


def test_capture_bearing_changes_only_brake_yaw_and_retains_original_c2_boundary():
    runtime,native=maps(); curve=recorded_curve(); corridor=ExecutionCorridor(runtime,native)
    progress=7.46
    original=certified_brake(curve,progress,corridor)
    goal=original.sample(original.duration)[3]+.15
    directed=certified_brake(curve,progress,corridor,measured_bearing=goal)
    np.testing.assert_allclose(directed.coefficients,original.coefficients)
    for left,right in zip(curve.sample(progress),directed.sample(progress)):
        np.testing.assert_allclose(left,right,atol=1e-10)
    assert abs(directed.yaw_acceleration(progress)-curve.yaw_acceleration(progress))<1e-9
    assert abs(directed.sample(directed.duration)[3]-goal)<abs(original.sample(original.duration)[3]-goal)
    assert abs(directed.sample(directed.duration)[4])+abs(directed.yaw_acceleration(directed.duration))<1e-9
    assert motion_limits().accepts(directed)


def test_native_only_inflation_and_unknown_remain_blocking():
    runtime,native=maps(); curve=recorded_curve(); corridor=ExecutionCorridor(runtime,native)
    endpoint=runtime.indices(curve.sample(curve.duration)[0])
    for value in (0,2,4,5,6):
        native.state[tuple(endpoint)]=value
        assert not corridor.safe_curve(curve)
    native.state[tuple(endpoint)]=1
    assert corridor.safe_curve(curve)
    runtime.state[tuple(endpoint)]=-1;runtime.rebuild()
    assert not corridor.safe_curve(curve)


def test_blocked_current_envelope_refuses_escape_or_unsafe_brake():
    runtime,native=maps(); curve=recorded_curve(); progress=7.46
    cell=runtime.indices(curve.sample(progress)[0]);runtime.state[tuple(cell)]=1;runtime.rebuild()
    with pytest.raises(ValueError,match='No certified C2 stop'):
        certified_brake(curve,progress,ExecutionCorridor(runtime,native))


def test_snapshot_age_order_and_geometry_validation():
    runtime,native=maps()
    assert native.fresh(89.5) and native.fresh(90.3) and not native.fresh(91.3) and not native.fresh(89.)
    packet=[1,89.3,2,.3,0,0,0,*runtime.shape,*np.ones(runtime.state.size)]
    assert native.ingest(packet)
    assert not native.ingest(packet)
    packet[2]=3; packet[1]=89.1
    assert not native.ingest(packet)
    packet[1]=89.4
    for cell in (np.nan,np.inf,-np.inf,3,1.25):
        packet[-1]=cell
        with pytest.raises(ValueError):native.ingest(packet)
        assert native.sequence==2 and native.stamp==89.3
    packet[-1]=6
    assert native.ingest(packet)


def test_bernstein_enclosure_catches_between_sample_excursion():
    c=np.zeros((6,3));c[:,0]=[0,4,-4,0,0,0];c[:,2]=[1.5,0,0,0,0,0]
    lo,hi=polynomial_box(c,0,1)
    assert lo[0]<=0 and hi[0]>=1
    runtime,native=maps()
    # Both endpoints agree, but the interior traverses occupied space.
    c[0,:2]=[3.,3.];c[:,0]=[3,4,-4,0,0,0]
    curve=ContinuousTrajectory([20.],[c],0,0)
    runtime.state[13,10,5]=1;runtime.rebuild()
    assert not ExecutionCorridor(runtime,native).safe_curve(curve)


def test_latency_reserve_checked_before_motion_and_new_hit_brakes_early():
    from core.planning.execution_safety import stopping_reserve, REACTION_HORIZON
    runtime,native=maps();curve=recorded_curve();corridor=ExecutionCorridor(runtime,native)
    progress=6.46
    reserve,boundary=stopping_reserve(curve,progress,corridor)
    assert boundary-progress>=REACTION_HORIZON-1e-10
    assert corridor.safe_curve(reserve,progress)
    for t in np.linspace(progress,boundary,11):
        for a,b in zip(curve.sample(t),reserve.sample(t)):np.testing.assert_allclose(a,b,atol=1e-9)
    for z in range(4,8):runtime.state[31,46,z]=1
    runtime.rebuild()
    # The delayed stop is unavailable BEFORE the reference enters the new
    # native cubic exclusion. Continuing motion may no longer be granted.
    assert corridor.box_safe(curve.sample(progress)[0],curve.sample(progress)[0])
    with pytest.raises(ValueError):stopping_reserve(curve,progress,corridor)
    early=certified_brake(curve,progress,corridor)
    assert corridor.safe_curve(early,progress)
    # Waiting until the historical terminal boundary is too late under the
    # preserved nominal clearance: no "escape" certificate is fabricated.
    with pytest.raises(ValueError):certified_brake(curve,curve.duration-.1,corridor)
    with pytest.raises(ValueError,match='full latency'):stopping_reserve(curve,0.,corridor,horizon=.05)


def test_sitl_corridor_relaxes_inflation_only_while_moving():
    runtime, native = maps()
    curve = recorded_curve()
    strict = ExecutionCorridor(runtime, native)
    relaxed = ExecutionCorridor(runtime, native, relaxed_sitl=True)
    middle = tuple(runtime.indices(curve.sample(curve.duration / 2)[0]))
    endpoint = tuple(runtime.indices(curve.sample(curve.duration)[0]))
    assert middle != endpoint

    native.state[middle] = 5
    assert not strict.safe_curve(curve)
    assert relaxed.safe_curve(curve)
    for blocked in (0, 2, 4, 6):
        native.state[middle] = blocked
        assert not strict.safe_curve(curve) and not relaxed.safe_curve(curve)
    native.state[middle] = 5

    runtime.state[middle] = 1
    runtime.rebuild()
    assert not relaxed.safe_curve(curve)
    runtime.state[middle] = 0
    runtime.rebuild()

    native.state[endpoint] = 5
    final_position = curve.sample(curve.duration)[0]
    assert relaxed.box_safe(final_position, final_position)
    assert not strict.safe_curve(curve) and not relaxed.safe_curve(curve)
    native.state[endpoint] = 2
    assert not strict.safe_curve(curve) and not relaxed.safe_curve(curve)


@pytest.mark.parametrize('profile', ['relaxed', 'frontier'])
def test_observed_corridor_preserves_sitl_inflation_and_endpoint_policy(profile):
    from annual_runtime.map_evidence import QueryTraceCorridor

    runtime, native = maps()
    curve = recorded_curve()
    base = ExecutionCorridor(runtime, native, relaxed_sitl=True,
                             frontier_sitl=profile == 'frontier')
    traced = QueryTraceCorridor(base)
    middle = curve.sample(curve.duration / 2)[0]
    native.state[tuple(runtime.indices(middle))] = 5

    assert base.safe_curve(curve)
    assert traced.safe_curve(curve)
    assert traced.query_count > 0
    assert traced.box_safe(middle, middle)
    endpoint = curve.sample(curve.duration)[0]
    native.state[tuple(runtime.indices(endpoint))] = 5
    if profile == 'frontier':
        assert base.safe_curve(curve) and traced.safe_curve(curve)
    else:
        assert not base.safe_curve(curve)
        assert not traced.safe_curve(curve)
    native.state[tuple(runtime.indices(endpoint))] = 2
    assert not base.safe_curve(curve)
    assert traced.safe_curve(curve) == base.safe_curve(curve)


def test_sitl_tracking_and_static_handoff_thresholds_are_opt_in(monkeypatch):
    from core.planning.execution_safety import (
        EXECUTION_PROFILE_ENV, SITL_RELAXED_PROFILE, static_reference_jump_limit,
        tracking_brake_trigger,
    )

    monkeypatch.delenv(EXECUTION_PROFILE_ENV, raising=False)
    assert static_reference_jump_limit() == .12 and tracking_brake_trigger() == .15
    monkeypatch.setenv(EXECUTION_PROFILE_ENV, SITL_RELAXED_PROFILE)
    assert static_reference_jump_limit() == .15 and tracking_brake_trigger() == .18


def test_reference_step_distance_checks_both_sides_at_inclusive_knots(monkeypatch):
    from core.planning import execution_safety

    curve = ContinuousTrajectory([.5, .5, .5], np.zeros((3, 6, 3)), 0., 0.)
    checked = []
    original = execution_safety.polynomial_box

    def record_span(coefficients, low, high):
        checked.append((low, high))
        return original(coefficients, low, high)

    monkeypatch.setattr(execution_safety, 'polynomial_box', record_span)
    assert execution_safety.reference_step_distance(curve, .5, seconds=.5) == 0.
    assert checked == [(1., 1.), (0., 1.), (0., 0.)]
    checked.clear()
    assert execution_safety.reference_step_distance(curve, curve.duration) == 0.
    assert checked == [(1., 1.)]


def test_endpoint_proof_preserves_latest_reason_and_uses_older_eligible_frame():
    from core.exploration.observation_completion import ObservationCompletion

    completion = ObservationCompletion(0)
    for sequence, (stamp, position) in enumerate(((1., [0., 0., 1.]), (1.5, [0., 0., 1.]),
                                                  (2.25, [.2, 0., 1.]), (3., [0., 0., 1.])), 1):
        packet = dict(source=0, sensor='gz_depth', integration_completed=True,
                      sensor_session='mission', sequence=sequence, time=stamp, pose_time=stamp,
                      sensor_position=position, sensor_yaw=0., map_version=sequence,
                      shape=[5, 5, 5], origin=[0., 0., 0.], resolution=.3,
                      ray_cell_count=1, point_count=1)
        assert completion.ingest(packet, stamp)

    goal = [0., 0., 1.]
    proof = completion.proof(2.25, 1.5, goal, 0.)
    assert proof['sequence'] == 2 and completion.wait_reason is None
    assert completion.proof(4.5, 3., goal, 0.)['sequence'] == 4
    assert completion.proof(3.25, 2.5, goal, 0., grid='wrong') is None
    assert completion.wait_reason == 'sensor_grid_mismatch'
    assert completion.proof(6.5, 2.5, goal, 0.) is None
    assert completion.wait_reason == 'integrated_frame_stale'
    assert completion.proof(6.5, 4., goal, 0.) is None
    assert completion.wait_reason == 'awaiting_post_arrival_integrated_frame'


def test_strict_box_safe_combines_native_check_and_distinguishes_margin_band():
    runtime, native = maps()
    corridor = ExecutionCorridor(runtime, native)
    center = np.array([3.15, 3.15, 1.5])
    cell = tuple(runtime.indices(center))
    outer = tuple(runtime.indices(center + np.array([corridor.margin, 0., 0.])))
    assert outer != cell

    calls = []
    original = native.box_safe

    def counted_box_safe(*args, **kwargs):
        calls.append((args, kwargs))
        return original(*args, **kwargs)

    native.box_safe = counted_box_safe
    assert corridor.box_safe(center, center)
    assert len(calls) == 1

    calls.clear()
    native.state[outer] = 5
    assert corridor.box_safe(center, center)
    assert len(calls) == 1

    calls.clear()
    native.state[cell] = 5
    assert not corridor.box_safe(center, center)
    assert len(calls) == 1


def test_corridor_uses_l2_tracking_ball_for_diagonal_unknown_voxels(monkeypatch):
    from core.planning.bspline_trajectory import safe_native_execution

    runtime = VoxelMap([[-3.0, -13.0, 0.3], [3.0, -7.0, 4.2]], resolution=.3, clearance=.6,
                       flight_limits=(-2.85, 7.1))
    runtime.state[:] = 0
    for cx in (-0.15, 0.15):
        runtime.state[tuple(runtime.indices([cx, -11.65, 2.85]))] = -1
    runtime.rebuild()
    native = NativeSafetyMap()
    native.ingest([1, 58.0, 1, .3, *runtime.origin, *runtime.shape,
                   *np.where(runtime.state == 0, 1, 0).ravel(order='C')])

    corridor = ExecutionCorridor(runtime, native)
    root = np.array([0., -11., 2.2])
    drifted = np.array([0.011, -11.039, 2.206])
    assert corridor.box_safe(root, root)
    assert corridor.box_safe(drifted, drifted)

    goal = np.array([0., -9.5, 2.2])
    coeff = np.zeros((1, 6, 3))
    coeff[0, 0] = root
    coeff[0, 3] = 10. * (goal - root)
    coeff[0, 4] = -15. * (goal - root)
    coeff[0, 5] = 6. * (goal - root)
    curve = ContinuousTrajectory([4.5], coeff, 0., 0., yaw_durations=[4.5], yaw_segments=np.zeros((1, 6)))
    monkeypatch.setenv('ASPIRE_FRONTIER_STEPS', '1')
    stopped, duration = safe_native_execution(curve, corridor)
    assert duration == pytest.approx(4.5)
    np.testing.assert_allclose(stopped.sample(stopped.duration)[0], goal, atol=1e-9)

    # Diagonal infringement below 0.25 m from E(0.60, 0.30) must fail closed.
    closer = np.array([0., -11.10, 2.25])
    assert not corridor.box_safe(closer, closer)

    # Pure axis boundaries: 0.60 + 0.25 = 0.85 m horizontal, 0.30 + 0.25 = 0.55 m vertical.
    clear_map, clear_native = maps()
    clear_map.state[8, 8, 5] = 1
    clear_map.rebuild()
    clear_native.state[8, 8, 5] = 2
    axis_corridor = ExecutionCorridor(clear_map, clear_native)
    # Voxel [8, 8, 5] spans [2.4, 2.7] x [2.4, 2.7] x [1.5, 1.8].
    assert not axis_corridor.box_safe([2.55, 1.56, 1.65], [2.55, 1.56, 1.65])
    assert axis_corridor.box_safe([2.55, 1.54, 1.65], [2.55, 1.54, 1.65])
    assert not axis_corridor.box_safe([2.55, 2.55, 2.34], [2.55, 2.55, 2.34])
    assert axis_corridor.box_safe([2.55, 2.55, 2.36], [2.55, 2.55, 2.36])

    # Spherical profile (rx == rz == 0.43 m): boundary at 0.43 + 0.25 = 0.68 m.
    monkeypatch.setenv('ANNUAL_VOXEL_MAP', json.dumps(dict(collision_profile='x500_swept_sphere')))
    sphere_map = VoxelMap([[0, 0, 0], [6., 6., 3.9]], flight_limits=(.7, 3.))
    sphere_map.state[:] = 0
    sphere_map.state[8, 8, 5] = 1
    sphere_map.rebuild()
    sphere_native = NativeSafetyMap()
    sphere_native.ingest([1, 89.2, 1, .3, 0, 0, 0, *sphere_map.shape, *np.ones(sphere_map.state.size)])
    sphere_native.state[8, 8, 5] = 2
    sphere_corridor = ExecutionCorridor(sphere_map, sphere_native)
    assert not sphere_corridor.box_safe([2.55, 2.55, 2.47], [2.55, 2.55, 2.47])
    assert sphere_corridor.box_safe([2.55, 2.55, 2.49], [2.55, 2.55, 2.49])
