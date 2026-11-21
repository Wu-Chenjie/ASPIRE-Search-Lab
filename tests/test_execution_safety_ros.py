"""Executor callbacks with actual ROS message classes, without a DDS process.

Only the Node I/O/clock is isolated. NativeSafetyMap, measured-map merging,
continuous corridor checks and C2 braking execute their real implementations.
The explicit free-grid fixture is a unit-test setup, not flight evidence.
"""
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('rclpy')
pytest.importorskip('std_msgs.msg')
from rclpy.node import Node
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, String
from trajectory_msgs.msg import MultiDOFJointTrajectory
import annual_runtime.view_executor_node as executor_module
from annual_runtime.view_executor_node import ViewExecutor
from core.planning.bspline_trajectory import native_trajectory, append_safe_brake
from core.planning.execution_safety import ExecutionCorridor, NativeSafetyMap
from annual_runtime.static_cancel_gateway import StaticCancelGateway


class AdapterFixture(StaticCancelGateway, SimpleNamespace):
    """Original adapter callbacks with the real default-disabled cancel gateway."""
    def __init__(self, **fields):
        super().__init__(**fields)
        self._static_gateway_setup('')
        self.collision_stop_enabled=False



def set_odometry(harness, position, yaw=0., *, offset_ns=0):
    msg = Odometry()
    stamp_ns = harness.clock[0] + offset_ns
    msg.header.stamp = Time(sec=stamp_ns // 1_000_000_000, nanosec=stamp_ns % 1_000_000_000)
    msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = map(float, position)
    msg.pose.pose.orientation.w = float(np.cos(yaw / 2))
    msg.pose.pose.orientation.z = float(np.sin(yaw / 2))
    harness.node.odom(0, msg)


def snapshot(harness, state=None, sequence=2):
    node = harness.node
    runtime = node.safety_replica.map
    values = np.ones(runtime.shape, dtype=np.uint8) if state is None else state
    data = [1., harness.now(), float(sequence), runtime.resolution,
            *runtime.origin, *runtime.shape, *values.ravel(order='C')]
    message = Float64MultiArray(data=list(map(float, data)))
    node.native_snapshot(message)
    return message


def observation(harness, indices, sequence=1):
    node = harness.node
    runtime = node.safety_replica.map
    packet = dict(source=0, sequence=sequence, sensor_session='ros-unit-test',
                  time=harness.now(), sensor='gz_depth', integration_completed=True,
                  indices=indices, values=[1] * len(indices),
                  sensor_position=node.ref.tolist(), sensor_yaw=float(node.heading),
                  pose_time=harness.now(), map_version=sequence,
                  shape=list(map(int, runtime.shape)), origin=runtime.origin.tolist(),
                  resolution=runtime.resolution, ray_cell_count=len(indices), point_count=len(indices))
    node.observation(String(data=json.dumps(packet)))


def command_message(curve, token='native-test-1', epoch=1):
    return String(data=json.dumps(dict(epoch=epoch, token=token,
                                      path=curve.path().tolist(),
                                      yaw=float(curve.sample(curve.duration)[3]),
                                      trajectory=curve.to_dict())))


def activate(harness, progress=7.46):
    node, curve = harness.node, harness.curve
    node.command(command_message(curve))
    assert node.token == 'native-test-1' and node.curve is not None
    node.curve_time = progress
    node.ref, node.velocity, node.acceleration, node.heading, node.yaw_rate = node.curve.sample(progress)
    set_odometry(harness, node.ref, node.heading)
    node.peer(0, String(data=json.dumps(dict(time=harness.now(),
               intent=dict(token=node.token, committed=True, voters=[])))))
    node.last = harness.now()
    return node.curve


@pytest.fixture
def executor(monkeypatch):
    monkeypatch.setenv('RACER_EXECUTION_SAFETY', '1')
    recorded = json.loads((Path(__file__).parent / 'fixtures/active06_native_id9.json').read_text())
    curve = append_safe_brake(native_trajectory(recorded), SimpleNamespace(safe_path=lambda path: True))
    start = curve.sample(0.)[0].tolist()
    params = dict(drone_id=0, bounds=json.dumps([[0, 0, 0], [18.3, 25.5, 3.9]]),
                  safety_flight_limits=json.dumps([.7, 3.]),
                  fleet_starts=json.dumps([start]), start=json.dumps(start))
    monkeypatch.setattr(Node, '__init__', lambda *args, **kwargs: None)
    monkeypatch.setattr(Node, 'declare_parameter',
                        lambda self, key, default: SimpleNamespace(value=params.get(key, default)))
    subscriptions = {}
    monkeypatch.setattr(Node, 'create_subscription',
                        lambda self, kind, topic, callback, qos: subscriptions.setdefault(topic, callback))
    monkeypatch.setattr(Node, 'create_timer', lambda *args, **kwargs: None)
    publishers = {}

    def publisher(self, kind, topic, qos):
        pub = SimpleNamespace(messages=[], message_type=kind)
        pub.publish = pub.messages.append
        publishers[topic] = pub
        return pub

    monkeypatch.setattr(Node, 'create_publisher', publisher)
    clock = [100_000_000_000]

    def stamp():
        return Time(sec=clock[0] // 1_000_000_000, nanosec=clock[0] % 1_000_000_000)

    monkeypatch.setattr(Node, 'get_clock', lambda self: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=clock[0], to_msg=stamp)))
    wall = [10.]
    monkeypatch.setattr(executor_module, 'time', SimpleNamespace(monotonic=lambda: wall[0]))
    node = ViewExecutor()
    node.ready = True
    node.ref = np.asarray(start)
    # This explicit unit-test map fixture provides room to exercise real braking.
    # Dynamic invalidation below enters through actual ROS callback messages.
    node.safety_replica.map.state[:] = 0
    node.safety_replica.map.rebuild()
    node.safety_stamp = clock[0] * 1e-9
    harness = SimpleNamespace(node=node, curve=curve, clock=clock, stamp=stamp, wall=wall,
                              now=lambda: clock[0] * 1e-9,
                              publishers=publishers, subscriptions=subscriptions)
    set_odometry(harness, start)
    message = snapshot(harness, sequence=1)
    assert isinstance(message, Float64MultiArray)
    assert '/baseline/safety_map_0' in subscriptions
    yield harness
    node.control_timing.close()


@pytest.mark.parametrize('changed_map', ['native', 'mapper'])
def test_dynamic_map_callback_certifies_c2_stop_and_preserves_token_prefix(executor, changed_map):
    node = executor.node
    original = activate(executor)
    boundary = node.curve_time
    node.pending = {'token': 'obsolete-pending-command'}
    if changed_map == 'native':
        state = np.ones(node.safety_replica.map.shape, dtype=np.uint8)
        # Exact native cubic inflation of the four measured active06 support cells.
        state[29:34, 44:49, 2:10] = 5
        state[31, 46, 4:8] = 6
        snapshot(executor, state)
    else:
        observation(executor, [[31, 46, z] for z in range(4, 8)])
    assert node.safety_dirty
    node.check_execution_safety(executor.now())
    assert node.safety_fault is None and node.safety_stopping
    assert node.token == 'native-test-1' and node.epoch == 1
    assert node.curve_time == boundary and node.pending is None
    assert node.safety_revision['reason'] == 'remaining_corridor_changed'
    assert node.safety_revision['boundary'] == boundary
    assert boundary < node.curve.duration < original.duration
    assert ExecutionCorridor(node.safety_replica.map, node.native_safety).safe_curve(node.curve, boundary)
    for t in np.linspace(0., boundary, 17):
        for old, new in zip(original.sample(t), node.curve.sample(t)):
            np.testing.assert_allclose(old, new, atol=1e-9)
        np.testing.assert_allclose(original.yaw_acceleration(t), node.curve.yaw_acceleration(t), atol=1e-9)
    terminal = node.curve.sample(node.curve.duration)
    np.testing.assert_allclose(terminal[1:3], 0., atol=1e-9)
    assert abs(terminal[4]) + abs(node.curve.yaw_acceleration(node.curve.duration)) < 1e-9
    node.tick()
    assert isinstance(node.trajectory_pub.messages[-1], MultiDOFJointTrajectory)
    node.report()
    report = json.loads(node.report_pub.messages[-1].data)
    assert report['token'] == 'native-test-1' and report['safety_revision']['number'] == 1
    assert not report['arrived'] and report['safety_fault'] is None


@pytest.mark.parametrize('changed_map', ['native', 'mapper'])
def test_blocked_current_envelope_reports_explicit_safety_fault(executor, changed_map):
    node = executor.node
    original = activate(executor)
    before_reference = node.ref.copy()
    cell = node.safety_replica.map.indices(node.ref)
    if changed_map == 'native':
        state = np.ones(node.safety_replica.map.shape, dtype=np.uint8)
        state[tuple(cell)] = 6
        snapshot(executor, state)
    else:
        observation(executor, [cell.tolist()])
    node.pending = {'token': 'must-not-activate'}
    node.tick()
    assert node.safety_fault and 'remaining_corridor_changed' in node.safety_fault
    assert 'No certified C2 stop' in node.safety_fault
    assert node.curve is original and node.pending is None
    assert node.safety_revision is None and not node.safety_stopping
    np.testing.assert_allclose(node.ref, before_reference)
    published = node.trajectory_pub.messages[-1].points[0]
    assert isinstance(node.trajectory_pub.messages[-1], MultiDOFJointTrajectory)
    assert published.velocities[0].linear.x == published.velocities[0].linear.y == published.velocities[0].linear.z == 0.
    node.report()
    report = json.loads(node.report_pub.messages[-1].data)
    assert report['safety_fault'] and not report['ready'] and not report['arrived']
    assert report['token'] == 'native-test-1'


def test_stale_map_selects_a_finite_stop_without_repeated_extension(executor):
    node = executor.node
    original = activate(executor)
    boundary = node.curve_time
    executor.clock[0] += 2_100_000_000
    set_odometry(executor, node.ref, node.heading)
    node.check_execution_safety(executor.now())
    assert node.safety_fault is None and node.safety_stopping
    assert node.safety_revision['reason'] == 'measured_map_stale'
    assert node.token == 'native-test-1' and node.curve_time == boundary
    assert boundary < node.curve.duration <= boundary + 4.
    for old, new in zip(original.sample(boundary), node.curve.sample(boundary)):
        np.testing.assert_allclose(old, new, atol=1e-9)
    end = node.curve.sample(node.curve.duration)
    np.testing.assert_allclose(end[1:3], 0., atol=1e-9)
    assert abs(end[4]) + abs(node.curve.yaw_acceleration(node.curve.duration)) < 1e-9
    stopped = node.curve
    revision = node.safety_revision.copy()
    executor.clock[0] += 10_000_000_000
    node.check_execution_safety(executor.now())
    assert node.curve is stopped and node.safety_revision == revision


def test_command_activation_uses_latest_received_native_map(executor):
    node = executor.node
    packet = command_message(executor.curve)
    assert node.curve is None and node.epoch == 0
    # A command prepared against a free map must not activate after a new snapshot.
    state = np.ones(node.safety_replica.map.shape, dtype=np.uint8)
    state[29:34, 44:49, 2:10] = 5
    state[31, 46, 4:8] = 6
    snapshot(executor, state)
    node.command(packet)
    assert node.curve is None and node.token is None and node.epoch == 0
    assert node.reason == 'latest_execution_corridor_rejected'
    assert node.safety_fault is None


def test_stale_map_cannot_activate_a_new_command(executor):
    node = executor.node
    executor.clock[0] += 2_100_000_000
    set_odometry(executor, node.ref)
    node.command(command_message(executor.curve))
    assert node.curve is None and node.token is None and node.epoch == 0
    assert node.reason == 'latest_execution_corridor_rejected'


def test_native_adapter_accepts_brake_revision_without_duplicate_time_key():
    import io
    from tools.research.frontier_baselines.live_ros2 import NativeLeaseAgent
    node=AdapterFixture(intent=dict(token='flight-token',native_duration=8.),pending=None,
        completed=set(),log=io.StringIO(),now=lambda:104.)
    node.event=lambda kind,**fields:NativeLeaseAgent.event(node,kind,**fields)
    revision=dict(number=1,time=103.95,boundary=6.,path=[[0,0,1.5],[0,1,1.5]])
    NativeLeaseAgent.executed(node,String(data=json.dumps(dict(token='flight-token',safety_revision=revision))))
    row=json.loads(node.log.getvalue())
    assert row['time']==104. and row['decision_time']==103.95
    assert node.intent['native_duration']==6. and node.intent['safety_revision']==1


@pytest.mark.parametrize('offset_target,offset', [('actual', .26), ('reference', .13)])
def test_activation_rejects_root_outside_actual_or_reference_tube(executor, offset_target, offset):
    node = executor.node
    shifted = node.ref + np.array([offset, 0., 0.])
    if offset_target == 'actual':
        set_odometry(executor, shifted)
    else:
        node.ref = shifted
    node.command(command_message(executor.curve))
    assert node.curve is None and node.token is None and node.epoch == 0
    assert node.activation_rejected['reason'] == 'activation_root_outside_tracking_tube'
    assert node.activation_rejected['token'] == 'native-test-1'
    assert json.loads(node.report_pub.messages[-1].data)['activation_rejected'] == node.activation_rejected


def begin_stale_stop(executor):
    node = executor.node
    activate(executor)
    executor.clock[0] += 2_100_000_000
    set_odometry(executor, node.ref, node.heading)
    node.check_execution_safety(executor.now())
    assert node.safety_stopping and node.safety_fault is None
    return node.curve, node.safety_stop_deadline, node.safety_revision.copy()


def test_new_native_frames_cannot_extend_stop_with_stale_mapper(executor):
    node = executor.node
    stopped, deadline, revision = begin_stale_stop(executor)
    initial_time = executor.now()
    initial_progress = node.curve_time
    for sequence in range(2, 6):
        executor.clock[0] += 50_000_000
        node.curve_time = initial_progress + executor.now() - initial_time
        node.ref, node.velocity, node.acceleration, node.heading, node.yaw_rate = stopped.sample(node.curve_time)
        set_odometry(executor, node.ref, node.heading)
        snapshot(executor, sequence=sequence)
        assert executor.now() - node.safety_stamp > 2.
        node.check_execution_safety(executor.now())
        assert node.safety_fault is None
        assert node.curve is stopped and node.safety_stop_deadline == deadline
        assert node.safety_revision == revision


def test_stopping_without_dirty_map_still_checks_tracking(executor):
    node = executor.node
    begin_stale_stop(executor)
    node.safety_dirty = False
    node.last_safety_check = executor.now()
    set_odometry(executor, node.ref + np.array([.251, 0., 0.]))
    node.check_execution_safety(executor.now())
    assert node.safety_fault is None and not node.safety_hold_valid
    assert node.reason == 'stopping_tracking_tube_exceeded'


def test_stop_absolute_deadline_faults_even_without_dirty_map(executor):
    node = executor.node
    stopped, deadline, revision = begin_stale_stop(executor)
    executor.clock[0] = int((deadline + .001) * 1e9)
    node.safety_dirty = False
    node.last_safety_check = executor.now()
    node.check_execution_safety(executor.now())
    assert node.safety_fault == 'certified_stop_deadline_exceeded'
    assert node.curve is stopped and node.safety_revision == revision
    node.report()
    report = json.loads(node.report_pub.messages[-1].data)
    assert not report['ready'] and not report['arrived']


def test_completed_reference_stop_does_not_require_sensor_arrival_receipt(executor):
    node = executor.node
    stopped, deadline, revision = begin_stale_stop(executor)
    node.curve_time = stopped.duration
    node.ref, node.velocity, node.acceleration, node.heading, node.yaw_rate = stopped.sample(stopped.duration)
    set_odometry(executor, node.ref, node.heading)
    executor.clock[0] = int((deadline + .001) * 1e9)
    node.safety_stamp = executor.now()
    snapshot(executor, sequence=3)
    set_odometry(executor, node.ref, node.heading)
    # A post-arrival integrated depth receipt is distinct from ending the
    # finite reference stop. Waiting for that receipt must not extend the stop,
    # report observation completion, or incorrectly claim the stop ran late.
    assert not node.arrived and node.observation_proof is None
    node.check_execution_safety(executor.now())
    assert node.safety_fault is None
    assert node.curve is stopped and node.safety_stop_deadline == deadline
    assert node.safety_revision == revision
    node.report()
    report = json.loads(node.report_pub.messages[-1].data)
    assert not report['arrived'] and report['observation_proof'] is None


def test_reference_step_headroom_triggers_finite_stop_before_tube_limit(executor):
    node = executor.node
    original = activate(executor)
    direction = node.velocity/np.linalg.norm(node.velocity)
    set_odometry(executor, node.ref-.18*direction, node.heading)
    node.check_execution_safety(executor.now())
    assert node.safety_fault is None and node.safety_stopping
    assert node.safety_revision['reason']=='tracking_step_headroom'
    assert node.safety_stop_deadline is not None
    np.testing.assert_allclose(original.sample(node.curve_time)[0], node.curve.sample(node.curve_time)[0])


def test_next_reference_outside_tube_is_never_published(executor):
    node = executor.node
    activate(executor)
    direction = node.velocity/np.linalg.norm(node.velocity)
    set_odometry(executor, node.ref-.251*direction, node.heading)
    before=node.ref.copy()
    executor.clock[0]+=40_000_000
    node.tick()
    assert node.safety_fault is None and node.reason=='tracking_error_hold'
    np.testing.assert_array_equal(node.ref,before)


def test_growth_during_stop_is_given_headroom_before_next_step_limit(executor):
    node=executor.node
    original=activate(executor)
    direction=node.velocity/np.linalg.norm(node.velocity)
    set_odometry(executor,node.ref-.151*direction,node.heading)
    node.safety_dirty=False;node.last_safety_check=executor.now()
    node.check_execution_safety(executor.now())
    assert node.safety_fault is None and node.safety_stopping
    assert node.safety_revision['reason']=='tracking_step_headroom'
    assert node.safety_stop_deadline is not None
    np.testing.assert_allclose(original.sample(node.curve_time)[0],node.curve.sample(node.curve_time)[0])


@pytest.mark.parametrize('offset_ns', [-20_000_000, -600_000_000, 200_000_000])
def test_reordered_stale_or_future_odometry_cannot_latch_false_tracking_fault(executor, offset_ns):
    node = executor.node
    activate(executor, progress=1.)
    executor.clock[0] += 20_000_000
    set_odometry(executor, node.ref, node.heading)
    set_odometry(executor, node.ref + np.array([.2, 0., 0.]), node.heading, offset_ns=offset_ns)

    node.report()
    node.tick()

    assert node.safety_fault is None
    assert node.stamps[node.id] == pytest.approx(executor.now()) and node.curve_time > 1.


def test_expired_odometry_holds_before_a_tracking_decision(executor):
    node = executor.node
    activate(executor, progress=1.)
    set_odometry(executor, node.ref + np.array([.2, 0., 0.]), node.heading)
    executor.clock[0] += 600_000_000

    node.report()
    node.tick()
    assert node.safety_fault is None and not node.safety_hold_valid
    assert node.curve_time == 1. and node.reason == 'odometry_timeout'

    executor.clock[0] += 20_000_000
    set_odometry(executor, node.ref, node.heading)
    node.tick()
    assert node.safety_fault is None and node.safety_hold_valid and node.curve_time > 1.


def test_report_does_not_hide_tracking_tube_violation_between_checks(executor):
    node = executor.node
    activate(executor)
    set_odometry(executor, node.ref+np.array([.251,0.,0.]), node.heading)
    node.report()
    report=json.loads(node.report_pub.messages[-1].data)
    assert report['tracking_error'] > .25 and report['safety_fault'] is None


@pytest.mark.parametrize('state_value', [0, 2, 4, 6])
def test_arrived_hold_loses_validity_when_root_unknown_or_inflated(executor, state_value):
    node = executor.node
    activate(executor, progress=executor.curve.duration)
    node.arrived = True
    state = np.ones(node.safety_replica.map.shape, dtype=np.uint8)
    state[tuple(node.safety_replica.map.indices(node.ref))] = state_value
    snapshot(executor, state)
    node.check_execution_safety(executor.now())
    assert node.safety_fault is None and node.reason == 'arrived_root_needs_replan'
    assert not node.safety_hold_valid
    node.report()
    report = json.loads(node.report_pub.messages[-1].data)
    assert not report['ready'] and not report['arrived']


def test_arrived_stale_hold_is_not_reported_as_success(executor):
    node = executor.node
    activate(executor, progress=executor.curve.duration)
    node.arrived = True
    executor.clock[0] += 2_100_000_000
    set_odometry(executor, node.ref, node.heading)
    node.check_execution_safety(executor.now())
    assert not node.safety_hold_valid and node.safety_fault is None
    node.report()
    report = json.loads(node.report_pub.messages[-1].data)
    assert not report['ready'] and not report['arrived']
    assert report['reason'] == 'hold_waiting_fresh_safety_map'


def test_duplicate_mapper_packet_does_not_refresh_accepted_stamp(executor):
    node = executor.node
    observation(executor, [[1, 1, 1]], sequence=10)
    accepted_stamp = node.safety_stamp
    executor.clock[0] += 2_000_000_000
    node.safety_dirty = False
    observation(executor, [[1, 1, 1]], sequence=10)
    assert node.safety_stamp == accepted_stamp
    assert not node.safety_dirty
    assert node.safety_replica.sensor_stamps[0] == accepted_stamp


def test_execution_safety_forbids_dynamic_handoff(executor):
    node = executor.node
    packet = json.loads(command_message(executor.curve).data)
    packet['handoff'] = {'time': 100., 'token': 'old-token'}
    node.command(String(data=json.dumps(packet)))
    assert node.curve is None and node.pending is None and node.token is None
    assert node.activation_rejected['reason'] == 'execution_safety_requires_static_handoff'


@pytest.mark.parametrize('elapsed', [.12, .201])
def test_stopping_reserve_budget_uses_monotonic_not_frozen_ros_clock(executor, monkeypatch, elapsed):
    node = executor.node
    activate(executor, progress=1.)
    called = []
    monkeypatch.setattr(executor_module, 'stopping_reserve', lambda *a, **kw: called.append(True))
    sim_before = executor.now()
    executor.wall[0] += elapsed
    node.check_execution_safety(sim_before)
    assert executor.now() == sim_before
    assert not called and node.safety_fault is None and node.safety_hold_valid
    status = node.stopping_reserve_status
    assert status['check_sim_s'] is None and status['native_input_age_s'] == 0. and status['mapper_input_age_s'] == 0.



@pytest.mark.parametrize('case', ['late_transition', 'expired_mapper', 'missed_lease'])
def test_handoff_rejection_keeps_predecessor_and_rechecks_before_resuming(executor, monkeypatch, case):
    import annual_runtime.guarded_c2 as c2_module
    from core.planning.handoff import curve_fingerprint
    from test_guarded_handoff import pair

    node = executor.node
    old, successor, _, _ = pair()
    executor.curve = old
    node.ref = old.sample(0.)[0].copy()
    set_odometry(executor, node.ref)
    old = activate(executor, progress=1.7)
    node._static_cancel_setup('123')
    node._c2_setup(True)
    old_task = dict(session='123', task=1, proposal=11, map_revision=9)
    node._static_task_activated(dict(static_task=old_task, token=node.token))
    monkeypatch.setattr(c2_module, 'time', executor_module.time)

    packet = json.loads(command_message(successor, token='candidate', epoch=2).data)
    packet.update(authorization_session='123',
                  static_task=dict(old_task, task=2, proposal=12),
                  handoff=dict(from_epoch=1, from_token=node.token, trajectory_time=2.,
                               old_curve_sha256=curve_fingerprint(old)))
    node.command(String(data=json.dumps(packet)))
    assert node.pending['token'] == 'candidate'

    node.curve_time = 1.99
    node.ref, _, _, node.heading, _ = old.sample(node.curve_time)
    previous_reference = node.ref.copy()
    executor.clock[0] += 20_000_000
    set_odometry(executor, node.ref, node.heading)
    if case != 'missed_lease':
        node.peer(0, String(data=json.dumps(dict(time=executor.now(),
                 intent=dict(token=node.token, committed=True, voters=[]),
                 pending_intent=dict(token='candidate', committed=True, voters=[])))))
        delay = .201 if case == 'late_transition' else .06
        if case == 'expired_mapper':
            node.safety_stamp = executor.now() - 1.95
        def slow_snapshot(kind, *args, **kwargs):
            if kind == 'c2_transition':
                executor.wall[0] += delay
        monkeypatch.setattr(node, 'save_safety_certificate', slow_snapshot)
    node.tick()

    assert node.pending is None and node.curve is old and node.token == 'native-test-1'
    assert node.static_task == old_task and node.static_cancel_ack is None
    assert node.epoch == 1 and node.handoff_count == 0 and node.safety_fault is None
    assert node.observation_proof is None
    if case == 'missed_lease':
        assert node.curve_time > 2.
        assert node.activation_rejected['reason'] == 'handoff_boundary_already_consumed'
        return

    assert node.curve_time == 1.99 and node.stopping_reserve_status is None
    assert not node.safety_hold_valid and node.safety_dirty
    assert node.activation_rejected['reason'] == (
        'handoff_safety_check_delay_budget_exceeded' if case == 'late_transition' else 'handoff_measured_map_stale')
    node.command(String(data=json.dumps(packet)))
    assert node.pending is None and node.token == 'native-test-1'
    assert node.activation_rejected['reason'] == 'static_task_cancelled_or_invalid'
    point = node.trajectory_pub.messages[-1].points[0]
    translation = point.transforms[0].translation
    velocity = point.velocities[0].linear
    np.testing.assert_allclose([translation.x, translation.y, translation.z], previous_reference)
    np.testing.assert_allclose([velocity.x, velocity.y, velocity.z], 0.)

    executor.clock[0] += 20_000_000
    set_odometry(executor, node.ref, node.heading)
    node.safety_stamp = executor.now()
    node.tick()
    assert node.safety_hold_valid and not node.safety_fault
    assert node.token == 'native-test-1' and node.curve_time > 2.
    assert node.stopping_reserve_status['checked_at'] == executor.now()


def test_required_brake_overrun_applies_stop_without_wall_clock_kill(executor, monkeypatch):
    node = executor.node
    original = activate(executor)
    state = np.ones(node.safety_replica.map.shape, dtype=np.uint8)
    state[29:34, 44:49, 2:10] = 5
    state[31, 46, 4:8] = 6
    snapshot(executor, state)
    real_brake = executor_module.certified_brake
    def slow_brake(*args, **kwargs):
        brake = real_brake(*args, **kwargs)
        executor.wall[0] += .201
        return brake
    monkeypatch.setattr(executor_module, 'certified_brake', slow_brake)

    node.tick()
    assert node.safety_fault is None and node.safety_stopping and node.safety_hold_valid


def test_activation_allows_slow_wall_clock_when_sim_maps_are_fresh(executor, monkeypatch):
    node = executor.node
    monkeypatch.setattr(executor_module, 'stopping_reserve', lambda *a, **kw: pytest.fail('unexpected duplicate stopping_reserve'))
    node.command(command_message(executor.curve))
    assert node.curve is not None and node.token == 'native-test-1'


def test_execution_report_binds_native_plan_to_activated_token_and_estimated_pose(executor):
    node = executor.node
    packet = json.loads(command_message(executor.curve).data)
    packet['native_id'] = 37
    node.command(String(data=json.dumps(packet)))
    assert node.native_plan_id == 37
    set_odometry(executor, node.ref + np.array([.01, 0., 0.]))
    node.report()
    report = json.loads(node.report_pub.messages[-1].data)
    assert report['native_plan_id'] == 37 and report['token'] == packet['token']
    np.testing.assert_allclose(report['actual_position'], node.positions[0])
    assert report['position_stamp'] == executor.now()
    # Rejected or stale commands must not relabel the current active trajectory.
    packet.update(native_id=99, token='unaccepted', epoch=0)
    node.command(String(data=json.dumps(packet)))
    node.report()
    assert json.loads(node.report_pub.messages[-1].data)['native_plan_id'] == 37
    node.stop('unit_test_end')
    node.report()
    report = json.loads(node.report_pub.messages[-1].data)
    assert report['native_plan_id'] is None and report['token'] is None


def test_stopped_reference_waits_without_fabricating_observation(executor):
    node=executor.node
    activate(executor,progress=executor.curve.duration)
    original=node.curve.to_dict()
    set_odometry(executor,node.ref+np.array([.151,0.,0.]))
    node.check_execution_safety(executor.now())
    assert node.safety_fault is None and node.terminal_hold_status['tracking_pressure']
    assert not node.arrived and node.observation_proof is None
    assert node.curve.to_dict()==original and not node.safety_stopping
    deadline=node.terminal_hold_status['absolute_deadline']
    for step in (1.5,2.99):
        executor.clock[0]=int((100.+step)*1e9)
        node.safety_stamp=executor.now();snapshot(executor,sequence=10+int(step*100));set_odometry(executor,node.positions[node.id])
        node.check_execution_safety(executor.now())
        assert node.terminal_hold_status['absolute_deadline']==deadline
    executor.clock[0]=103_001_000_000
    set_odometry(executor,node.positions[node.id])
    node.check_execution_safety(executor.now())
    assert node.safety_fault is None and node.observation_failure['outcome']=='unobserved'
    assert node.observation_failure['observation_deadline']==deadline
    assert not node.arrived and node.observation_proof is None
    assert 'replan_deadline' not in node.observation_failure
    failure = dict(node.observation_failure)
    executor.clock[0] += 8_000_000_000
    set_odometry(executor,node.ref);node.safety_stamp=executor.now();snapshot(executor,sequence=200)
    node.check_execution_safety(executor.now())
    assert node.safety_fault is None and node.safety_hold_valid
    assert node.observation_failure == failure and node.observation_proof is None
    assert node.terminal_hold_status['checked_at'] == executor.now()


@pytest.mark.parametrize('bad', ['tracking','native_unknown','mapper_unknown','reference','derivative'])
def test_pending_terminal_hold_keeps_original_guards(executor,bad):
    node=executor.node;activate(executor,progress=executor.curve.duration)
    if bad=='tracking':set_odometry(executor,node.ref+np.array([.2501,0.,0.]))
    if bad=='native_unknown':
        state=np.ones(node.safety_replica.map.shape,dtype=np.uint8)
        state[tuple(node.safety_replica.map.indices(node.ref))]=0;snapshot(executor,state)
    if bad=='mapper_unknown':
        node.safety_replica.map.state[tuple(node.safety_replica.map.indices(node.ref))]=-1
        node.safety_dirty=True
    if bad=='reference':node.ref=node.ref+np.array([.001,0.,0.]);set_odometry(executor,node.ref)
    if bad=='derivative':
        node.curve.coefficients[-1,1,0]+=.01
        node.ref=node.curve.sample(node.curve.duration)[0];set_odometry(executor,node.ref)
    node.check_execution_safety(executor.now())
    assert not node.safety_hold_valid and node.observation_proof is None and not node.arrived
    assert (node.safety_fault is not None) == (bad in ('reference', 'derivative'))


def test_terminal_hold_state_reset_on_cancel_and_new_activation(executor):
    node=executor.node;activate(executor,progress=executor.curve.duration)
    node.check_execution_safety(executor.now());assert node.terminal_hold_status
    node.stop('test');assert node.terminal_hold_since is None and node.terminal_hold_status is None
    node.ref=executor.curve.sample(0)[0];set_odometry(executor,node.ref)
    node.command(command_message(executor.curve,token='next',epoch=2))
    assert node.token=='next' and node.terminal_hold_since is None


def abort_terminal(executor):
    node=executor.node;activate(executor,progress=executor.curve.duration)
    node.check_execution_safety(executor.now())
    executor.clock[0]=103_001_000_000
    node.safety_stamp=executor.now()-1.5;snapshot(executor,sequence=20)
    node.native_safety.stamp=executor.now()-1.5
    node.safety_stamp=executor.now();snapshot(executor,sequence=21);set_odometry(executor,node.ref)
    node.check_execution_safety(executor.now())
    assert node.observation_failure and node.safety_fault is None
    return node


def test_unobserved_task_cannot_later_award_completion_or_change_reference(executor):
    node=abort_terminal(executor);reference=node.ref.copy();original=node.curve.to_dict()
    observation(executor,[[1,1,1]],sequence=30);set_odometry(executor,node.ref)
    # Even an otherwise valid later observation cannot resurrect a closed task.
    node.tick();node.report()
    report=json.loads(node.report_pub.messages[-1].data)
    assert not report['arrived'] and report['observation_proof'] is None
    assert report['observation_failure']['outcome']=='unobserved'
    np.testing.assert_array_equal(node.ref,reference);assert node.curve.to_dict()==original
    assert not node.safety_fault


@pytest.mark.parametrize('bad', ['tracking','unknown'])
def test_unobserved_exit_never_bypasses_hard_safety(executor,bad):
    node=abort_terminal(executor)
    if bad=='tracking':set_odometry(executor,node.ref+np.array([.2501,0,0]))
    if bad=='unknown':
        state=np.ones(node.safety_replica.map.shape,dtype=np.uint8)
        state[tuple(node.safety_replica.map.indices(node.ref))]=0;snapshot(executor,state,sequence=22)
    node.check_execution_safety(executor.now())
    assert not node.safety_hold_valid and not node.arrived


@pytest.mark.parametrize('input_stream', ['map','odometry'])
def test_stopped_hold_recovers_with_fresh_inputs(executor, input_stream):
    node=abort_terminal(executor)
    reference = node.ref.copy()
    executor.clock[0] += 2_100_000_000
    if input_stream == 'map':
        set_odometry(executor, node.ref)
    else:
        node.safety_stamp = executor.now()
        snapshot(executor, sequence=22)
    node.check_execution_safety(executor.now())
    node.report()
    report=json.loads(node.report_pub.messages[-1].data)
    assert not report['ready'] and not report['arrived'] and not report['safety_fault']
    assert report['observation_proof'] is None
    set_odometry(executor, node.ref)
    node.safety_stamp = executor.now()
    snapshot(executor, sequence=22)
    node.check_execution_safety(executor.now())
    assert node.safety_hold_valid and node.safety_fault is None and not node.arrived
    np.testing.assert_array_equal(node.ref, reference)


def test_fourth_unobserved_view_keeps_stationary_root_available(executor):
    executor.node.observation_failure_count = 3
    node=abort_terminal(executor)
    assert node.observation_failure['mission_failure_count'] == 4
    assert node.safety_hold_valid and not node.arrived and node.observation_proof is None


def test_bad_map_packet_does_not_advance_sequence_or_fault_executor(executor):
    node=executor.node
    node.native_snapshot(Float64MultiArray(data=[1.,executor.now(),99.]))
    assert node.native_safety.sequence==1 and node.safety_fault is None

    good = dict(source=0, sequence=8, sensor_session='ros-unit-test',
                time=executor.now(), sensor='gz_depth', integration_completed=True,
                indices=[[1,1,1]], measured_indices=[[1,1,1]], values=[1],
                sensor_position=node.ref.tolist(),sensor_yaw=0.,pose_time=executor.now(),
                map_version=8,shape=list(map(int, node.safety_replica.map.shape)),
                origin=node.safety_replica.map.origin.tolist(),
                resolution=node.safety_replica.map.resolution,ray_cell_count=1,point_count=1)
    malformed=dict(good,measured_indices=[[999,1,1]])
    node.observation(String(data=json.dumps(malformed)))
    assert not node.safety_replica.sequences and not node.observation_completion.frames
    assert node.safety_fault is None and node.safety_replica.map.state[1,1,1]==0
    node.observation(String(data=json.dumps(good)))
    assert node.safety_replica.sequences[0]==8 and node.observation_completion.sequence==8
    assert node.safety_replica.map.state[1,1,1]==1
    snapshot(executor,sequence=2)
    assert node.native_safety.sequence==2


def test_failed_certificate_write_leaves_no_published_file(executor, monkeypatch, tmp_path):
    import queue
    import threading

    node=executor.node
    node.safety_output=str(tmp_path)
    node.safety_certificate_queue=queue.Queue(maxsize=8)
    node.safety_certificate_stop=threading.Event()
    node.safety_certificate_stop.set()
    warnings=[]
    monkeypatch.setattr(node, 'get_logger', lambda: SimpleNamespace(
        warning=lambda message, **kwargs: warnings.append(message)), raising=False)
    activate(executor)
    path=Path(node.save_safety_certificate('activation', node.curve, 0.))
    assert not path.exists()

    def fail_after_partial_write(stream, **arrays):
        stream.write(b'partial zip')
        raise OSError('fixture write failure')

    monkeypatch.setattr(executor_module.np, 'savez_compressed', fail_after_partial_write)
    node._write_safety_certificates()
    assert warnings and list(tmp_path.iterdir()) == []
    assert node.safety_fault is None and node.token == 'native-test-1'


def test_malformed_callback_packets_preserve_existing_executor_state(executor):
    node = executor.node
    activate(executor)
    previous = (node.token, node.epoch, node.peers.copy())
    for bad in ('{', '[]', '{}'):
        message = String(data=bad)
        node.command(message)
        node.peer(0, message)
        node.control(message)
        node.flight_status(message)
        assert (node.token, node.epoch, node.peers) == previous
        assert not node.paused and not node.done and node.flight_fault.reason is None
    node.control(String(data='{"paused":[0]}'))
    assert node.paused and node.token is None


def test_failed_observation_releases_only_native_intent_and_no_coverage(executor):
    from tools.research.frontier_baselines.live_ros2 import NativeLeaseAgent
    node=abort_terminal(executor);node.report();message=node.report_pub.messages[-1]
    events=[]
    agent=AdapterFixture(execution={},intent={'token':node.token,'native_id':7},pending=None,
        safety_enabled=True,now=executor.now,last_sensor=executor.now(),dirty=False,
        replica=SimpleNamespace(map=node.safety_replica.map),native_safety=node.native_safety,
        corridor=lambda:ExecutionCorridor(node.safety_replica.map,node.native_safety),
        failed_observations={},completed=set(),event=lambda kind,**kw:events.append(dict(kind=kind,**kw)))
    NativeLeaseAgent.executed(agent,message)
    assert agent.intent is None and agent.completed==set() and node.token in agent.failed_observations
    assert events[-1]['completed_observation_awarded'] is False
    assert events[-1]['native_feedback_active']==-1
    NativeLeaseAgent.executed(agent,message);assert len(events)==1
    feedback=[]
    agent.transport=SimpleNamespace(send=feedback.append)
    agent.flight_fault=SimpleNamespace(reason=None)
    agent.id=0;agent.last_rejection=-1;agent.collision_stop_released=None
    NativeLeaseAgent.send_execution_feedback(agent, executor.now())
    assert len(feedback[-1]['data'])==8 and feedback[-1]['data'][0]==-1.
    assert feedback[-1]['data'][2]==0. and feedback[-1]['data'][7]==7.
    NativeLeaseAgent.send_execution_feedback(agent, executor.now()+1.1)
    assert feedback[-1]['data'][7]==7. and not feedback[-1]['ready']
    agent.flight_fault.reason='coordinate_session_retired'
    NativeLeaseAgent.send_execution_feedback(agent, executor.now()+1.2)
    assert len(feedback[-1]['data'])==7 and not feedback[-1]['ready']


def test_native_collision_request_brakes_and_releases_after_measured_rest(executor):
    from tools.research.frontier_baselines.live_ros2 import NativeLeaseAgent

    node=executor.node
    activate(executor)
    node._static_cancel_setup('123')
    node.collision_stop_enabled=True
    node.native_plan_id=7
    request=dict(native_id=7,epoch=node.epoch,token=node.token,requested_at=executor.now())

    def send(request):
        node.command(String(data=json.dumps(dict(native_collision_stop=request,authorization_session='123'))))

    send(dict(request,epoch=0))
    assert node.collision_stop_request is None
    original=node.curve
    send(request)
    node.check_execution_safety(executor.now())
    assert node.safety_revision['reason']=='native_collision_alert'
    assert node.curve.duration<original.duration and not node.arrived
    node.curve_time=node.curve.duration
    node.ref,_,_,node.heading,_=node.curve.sample(node.curve_time)
    set_odometry(executor,node.ref,node.heading)
    node.check_execution_safety(executor.now())
    executor.clock[0]+=200_000_000
    set_odometry(executor,node.ref,node.heading)
    node.actual_speed=.1
    node.check_execution_safety(executor.now())
    assert node.collision_stop_receipt is None and not node.arrived
    executor.clock[0]+=200_000_000
    set_odometry(executor,node.ref,node.heading)
    node.check_execution_safety(executor.now())
    assert node.arrived and node.observation_proof is None
    assert node.collision_stop_receipt['observation_complete'] is False

    events=[]
    agent=AdapterFixture(intent=dict(native_id=7,token=node.token,collision_stop_request=request),
        now=executor.now,last_sensor=executor.now(),dirty=False,native_safety=node.native_safety,
        corridor=lambda:ExecutionCorridor(node.safety_replica.map,node.native_safety),
        event=lambda kind,**fields:events.append((kind,fields)))
    node.report()
    agent.execution=json.loads(node.report_pub.messages[-1].data)
    NativeLeaseAgent.release_collision_stop(agent,dict(node.collision_stop_receipt,requested_at=1.))
    assert agent.intent is not None
    NativeLeaseAgent.release_collision_stop(agent,node.collision_stop_receipt)
    assert agent.intent is None and agent.collision_stop_released==(7,request['requested_at'])
    assert events[0][1]['completed_observation_awarded'] is False

    # A map update may be the last input to arrive. Recheck this exact report
    # without asking the executor to produce another stop certificate.
    agent.fast_stop_feedback=True
    agent.stop_release_wait=None
    agent.flight_fault=SimpleNamespace(reason=None)
    agent.intent=dict(native_id=7,token=node.token,collision_stop_request=request)
    agent.collision_stop_released=None
    agent.release_collision_stop=lambda receipt,**fields:NativeLeaseAgent.release_collision_stop(agent,receipt,**fields)
    agent.trace_stop_release=lambda *args:NativeLeaseAgent.trace_stop_release(agent,*args)
    feedback=[]
    agent.send_execution_feedback=lambda stamp:feedback.append((stamp,agent.intent,agent.collision_stop_released))
    executor.clock[0]+=400_000_000
    agent.last_sensor=executor.now()
    agent.native_safety=NativeSafetyMap()
    agent.corridor=lambda:ExecutionCorridor(node.safety_replica.map,agent.native_safety)
    agent.release_collision_stop(node.collision_stop_receipt)
    assert agent.intent is not None and not feedback and not events[-1][1]['native_fresh']
    NativeLeaseAgent.native_snapshot(agent,snapshot(executor,sequence=3))
    assert feedback==[(executor.now(),None,(7,request['requested_at']))]
    assert events[-3][1]['trigger']=='native_map' and events[-3][1]['root_safe']


def test_elective_review_stop_preserves_the_original_route_if_endpoint_is_ineligible(executor,monkeypatch):
    from tools.research.frontier_baselines.live_ros2 import fresh_static_source_frame
    node=executor.node
    original=activate(executor)
    node._static_cancel_setup('123');node.collision_stop_enabled=True;node.native_plan_id=7
    request=dict(native_id=7,epoch=node.epoch,token=node.token,requested_at=executor.now(),purpose='review')
    msg=String(data=json.dumps(dict(native_collision_stop=request,authorization_session='123')))
    solver=executor_module.certified_brake

    def without_elective_endpoint(curve,progress,corridor,**kwargs):
        if kwargs:raise ValueError('no eligible elective endpoint')
        return solver(curve,progress,corridor)

    monkeypatch.setattr(executor_module,'certified_brake',without_elective_endpoint)
    node.command(msg);node.check_execution_safety(executor.now())
    assert node.curve is original and node.safety_fault is None and node.collision_stop_request is None
    assert node.collision_stop_refusal['reason']=='no eligible elective endpoint'
    node.command(msg);assert node.collision_stop_request is None
    frame=dict(counter=25,sequence=6,capture_s=90.5)
    assert fresh_static_source_frame(frame,91.25) and not fresh_static_source_frame(frame,91.251)
    assert not fresh_static_source_frame(frame,90.49) and not fresh_static_source_frame(None,91.0)


@pytest.mark.parametrize('source', ['solver', 'certificate'])
def test_late_elective_review_stop_keeps_original_route_after_a_fresh_recheck(executor, monkeypatch, source):
    node = executor.node
    original = activate(executor)
    node._static_cancel_setup('123')
    node.collision_stop_enabled = True
    node.native_plan_id = 7
    request = dict(native_id=7, epoch=node.epoch, token=node.token,
                   requested_at=executor.now(), purpose='review')
    message = String(data=json.dumps(dict(native_collision_stop=request, authorization_session='123')))
    solver = executor_module.certified_brake
    if source == 'solver':
        def slow_review(*args, **kwargs):
            brake = solver(*args, **kwargs)
            if kwargs:
                executor.wall[0] += .201
            return brake
        monkeypatch.setattr(executor_module, 'certified_brake', slow_review)
    else:
        def slow_certificate(kind, *args, **kwargs):
            if kind == 'brake':
                executor.wall[0] += .201
        monkeypatch.setattr(node, 'save_safety_certificate', slow_certificate)

    node.command(message)
    assert node.collision_stop_request == request
    executor.clock[0] += 20_000_000
    set_odometry(executor, node.ref, node.heading)
    progress = node.curve_time
    node.tick()

    assert node.safety_fault is None and not node.safety_hold_valid
    assert node.collision_stop_receipt is None and node.collision_stop_request is None
    assert node.collision_stop_refusal['reason'] == 'elective_stop_delay_budget_exceeded'
    assert node.curve is original and node.curve_time == progress and node.safety_stop_deadline is None

    executor.clock[0] += 20_000_000
    set_odometry(executor, node.ref, node.heading)
    node.tick()
    assert node.safety_hold_valid and node.safety_fault is None
    assert node.curve is original and node.curve_time > progress
    assert node.token == request['token'] and node.observation_proof is None


def test_real_collision_brake_runs_before_an_elective_request(executor, monkeypatch):
    node = executor.node
    activate(executor)
    node._static_cancel_setup('123')
    node.collision_stop_enabled = True
    node.native_plan_id = 7
    request = dict(native_id=7, epoch=node.epoch, token=node.token,
                   requested_at=executor.now(), purpose='review')
    message = String(data=json.dumps(dict(native_collision_stop=request, authorization_session='123')))
    node.command(message)
    state = np.ones(node.safety_replica.map.shape, dtype=np.uint8)
    state[29:34, 44:49, 2:10] = 5
    state[31, 46, 4:8] = 6
    snapshot(executor, state)
    solver = executor_module.certified_brake

    def no_elective_computation(*args, **kwargs):
        assert not kwargs
        return solver(*args, **kwargs)

    monkeypatch.setattr(executor_module, 'certified_brake', no_elective_computation)
    node.tick()
    assert node.safety_stopping and node.safety_fault is None
    assert node.safety_revision['reason'] == 'remaining_corridor_changed'
    assert node.collision_stop_receipt is None and node.collision_stop_request is None
    assert node.collision_stop_refusal['reason'] == 'elective_stop_preempted_by_safety_brake'


def test_elective_brake_waits_for_measured_speed_before_releasing(executor):
    node=executor.node
    activate(executor)
    node._static_cancel_setup('123');node.collision_stop_enabled=True;node.native_plan_id=7
    request=dict(native_id=7,epoch=node.epoch,token=node.token,requested_at=executor.now(),purpose='review')
    node.command(String(data=json.dumps(dict(native_collision_stop=request,authorization_session='123'))))
    node.check_execution_safety(executor.now())
    assert node.safety_revision['reason']=='fresh_review_requested_stop'
    node.curve_time=node.curve.duration
    node.ref,_,_,node.heading,_=node.curve.sample(node.curve_time)
    set_odometry(executor,node.ref,node.heading)
    node.check_execution_safety(executor.now())
    executor.clock[0]+=1_600_000_000
    snapshot(executor);node.safety_stamp=executor.now();set_odometry(executor,node.ref,node.heading)
    node.actual_speed=.054
    node.check_execution_safety(executor.now())
    assert node.safety_fault is None and node.collision_stop_receipt is None
    executor.clock[0]+=180_000_000
    set_odometry(executor,node.ref,node.heading)
    node.check_execution_safety(executor.now())
    assert node.collision_stop_receipt and node.arrived and node.observation_proof is None


@pytest.mark.parametrize('relaxed', [False, True])
def test_failed_view_does_not_relax_next_native_reference_root(executor,monkeypatch,relaxed):
    import tools.research.frontier_baselines.live_ros2 as live
    from core.planning.continuous_trajectory import ContinuousTrajectory
    from core.planning.execution_safety import EXECUTION_PROFILE_ENV, SITL_RELAXED_PROFILE

    if relaxed:
        monkeypatch.setenv(EXECUTION_PROFILE_ENV, SITL_RELAXED_PROFILE)
    else:
        monkeypatch.delenv(EXECUTION_PROFILE_ENV, raising=False)
    node=abort_terminal(executor);reference=node.ref.copy()
    original=executor.curve;shift=reference+np.array([.16,0,0])-original.sample(0)[0]
    coefficients=original.coefficients.copy();coefficients[:,0]+=shift
    curve=ContinuousTrajectory(original.durations,coefficients,original.yaw,original.yaw_delta,
        yaw_durations=original.yaw_durations,yaw_segments=original.yaw_segments)
    monkeypatch.setattr(live,'native_trajectory',lambda row:curve)
    monkeypatch.setattr(live,'safe_native_execution',lambda value,runtime:(value,value.duration))
    events=[];agent=AdapterFixture(pose=curve.sample(0)[0],execution={'reference':reference.tolist()},
        flight_fault=SimpleNamespace(reason=None),pending=None,intent=None,safety_enabled=True,dirty=False,
        replica=SimpleNamespace(map=node.safety_replica.map),native_safety=node.native_safety,
        now=executor.now,last_sensor=executor.now(),diagnostics=3,
        corridor=lambda **kwargs:ExecutionCorridor(node.safety_replica.map,node.native_safety),
        event=lambda kind,**kw:events.append(dict(kind=kind,**kw)))
    agent.reject=lambda row,reason:events.append(dict(kind='rejected',reason=reason))
    live.NativeLeaseAgent.propose(agent,dict(traj_id=99,from_id=-1,boundary_progress=-1))
    assert events[-1]['reason']=='native_static_reference_discontinuity'
    np.testing.assert_array_equal(node.ref,reference)
