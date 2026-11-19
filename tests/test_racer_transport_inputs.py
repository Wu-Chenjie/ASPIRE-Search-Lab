"""Exercise native transport input isolation on a host without ROS."""
import gzip
import importlib.util
import json
from pathlib import Path
import queue
import sys
from threading import Lock
from types import ModuleType, SimpleNamespace

import pytest


@pytest.fixture
def native_transport(monkeypatch):
    modules = {
        'rclpy': (),
        'rclpy.node': ('Node',),
        'rclpy.qos': ('QoSProfile', 'DurabilityPolicy'),
        'ament_index_python.packages': ('get_package_share_directory', 'get_package_prefix'),
        'builtin_interfaces.msg': ('Time',),
        'geometry_msgs.msg': ('PoseStamped',),
        'nav_msgs.msg': ('Odometry',),
        'sensor_msgs.msg': ('PointCloud2', 'PointField'),
        'std_msgs.msg': ('Float64MultiArray',),
        'racer_interfaces.msg': ('Bspline',),
    }
    for name, members in modules.items():
        module = ModuleType(name)
        for member in members:
            setattr(module, member, SimpleNamespace)
        monkeypatch.setitem(sys.modules, name, module)
    path = Path(__file__).resolve().parents[1]/'tools/racer_ros2/transport.py'
    spec = importlib.util.spec_from_file_location('tools.racer_ros2._input_fixture', path)
    module = importlib.util.module_from_spec(spec)

    def pose():
        return SimpleNamespace(header=SimpleNamespace(), pose=SimpleNamespace(
            orientation=SimpleNamespace(), position=SimpleNamespace()))

    class PointField(SimpleNamespace):
        FLOAT32 = 7

    class Node(SimpleNamespace):
        def __init__(self, _):
            self.received = []
            self.warnings = []

        def get_logger(self):
            return SimpleNamespace(warning=self.warnings.append)

        def create_publisher(self, *_):
            return SimpleNamespace(publish=self.received.append, get_subscription_count=lambda: 0)

        def create_guard_condition(self, _):
            return SimpleNamespace(trigger=lambda: None)

        def set_parameters(self, _):
            pass

        def create_subscription(self, *_):
            pass

        def create_timer(self, *_):
            pass

    # Replace the base class in the isolated module before loading the transport.
    sys.modules['rclpy.node'].Node = Node
    sys.modules['rclpy'].parameter = SimpleNamespace(
        Parameter=lambda name, value: SimpleNamespace(name=name, value=value))
    sys.modules['rclpy.qos'].DurabilityPolicy = SimpleNamespace(TRANSIENT_LOCAL=1)
    spec.loader.exec_module(module)
    module.PoseStamped = pose
    module.PointField = PointField
    monkeypatch.delenv('ASPIRE_SAFETY_MAP_TIMING', raising=False)
    return module


@pytest.fixture
def transport(native_transport):
    module = native_transport
    gateway = module.NativeDDSTransport.__new__(module.NativeDDSTransport)
    gateway.fleet_size = 2
    gateway.closing = gateway.triggered = gateway.fast_stop_feedback = False
    gateway.started = True
    gateway.error = gateway.input_stream = gateway.map_timing = gateway.raw = None
    gateway.ready = [False, False]
    gateway.warnings, gateway.recording_failures = [], {}
    gateway.metadata, gateway.waiting = {}, {}
    gateway.incoming, gateway.outgoing = queue.Queue(), queue.Queue()
    gateway.drops = 0
    gateway.children = []
    gateway.counts = dict(frame=0, feedback=0)
    gateway.received = []
    gateway.publishers_by_kind = {
        (kind, i): SimpleNamespace(publish=gateway.received.append)
        for kind in ('sensor', 'cloud', 'feedback') for i in range(2)
    }
    return gateway


def feedback(time, drone=0, ready=True, **updates):
    return dict(dict(type='feedback', time=time, drone=drone, ready=ready,
                     data=[-1., 0., 0., -1., -1., time, 0.]), **updates)


def test_bad_observation_and_feedback_do_not_poison_following_samples(transport):
    gateway = transport
    gateway.send(dict(type='frame', drone=0, time=1., origin=[float('nan'), 0, 0], points=[[0, 0, 1]]))
    gateway.send(feedback(1., data=[-1., 0., 0.]))
    gateway.send(feedback(1.1, data=[1.5, 0., 0., -1., -1., 1.1, 2.]))
    gateway.poll()
    assert gateway.ready == [False, False] and not gateway.received
    assert gateway.drops == 3 and gateway.error is None

    valid = dict(type='frame', drone=0, time=2., origin=[0, 0, 0], points=[[0, 0, 1]])
    release = feedback(2.2, data=[-1., 1., 0., -1., -1., 2.2, 0., 7.])
    gateway.send(valid)
    gateway.send(release)
    gateway.poll()
    assert gateway.ready == [True, False] and gateway.counts == dict(frame=1, feedback=1)
    assert gateway.received[0].header.stamp.sec == 2
    assert gateway.received[-1].data == release['data']
    assert gateway.error is None


def test_settled_reference_supplements_legacy_feedback_for_single_aircraft(transport):
    transport.fleet_size = 1
    reference = [-1., 8., .68, -5.96, 2.20, 0.]
    original = feedback(8.1)
    transport.send(dict(original, stopped_reference=reference))
    transport.send(dict(original, stopped_reference=[4., *reference[1:]]))
    transport.send(dict(original, stopped_reference=[-1., 7., *reference[2:]]))
    transport.send(dict(original, stopped_reference=['invalid', *reference[1:]]))
    transport.poll()
    assert [message.data for message in transport.received] == [
        original['data'], reference, original['data'], original['data'], original['data']]
    assert transport.drops == 0


def test_subnanosecond_negative_stamp_is_rejected_before_rounding(transport):
    frame = dict(type='frame', drone=0, origin=[0, 0, 0], points=[[0, 0, 1]])
    transport.send(dict(frame, time=-1e-12))
    transport.send(dict(frame, time=1e-12))
    transport.poll()
    assert transport.drops == 1 and transport.counts['frame'] == 1
    assert transport.received[0].header.stamp.sec == 0
    assert transport.received[0].header.stamp.nanosec == 0


def test_bad_boundary_cannot_pair_or_overwrite_a_proposal(transport):
    gateway = transport
    original = dict(from_id=-1, boundary_progress=-1., planning_wall_s=.02)
    gateway.metadata[0, 4] = original
    gateway.waiting[0, 5] = dict(traj_id=5)
    for data in (
        [4., -1., float('nan'), .03],
        [5., float('inf'), -1., .03],
        [5.25, -1., -1., .04],
        [5., -1., -2., .04],
    ):
        gateway.boundary(0, SimpleNamespace(data=data))
    assert gateway.metadata == {(0, 4): original}
    assert gateway.incoming.empty() and (0, 5) in gateway.waiting
    gateway.boundary(0, SimpleNamespace(data=[5., -1., -1., .03]))
    assert gateway.incoming.get_nowait() == dict(traj_id=5, **dict(original, planning_wall_s=.03))


def test_recording_failure_leaves_gateway_and_next_feedback_active(transport):
    class BrokenRecord:
        def write(self, _):
            raise OSError('recording storage unavailable')

        def close(self):
            raise OSError('delayed recording write error')

    gateway = transport
    gateway.input_stream = BrokenRecord()
    gateway.raw = BrokenRecord()
    gateway.metadata[0, 3] = dict(from_id=-1)
    gateway.deliver((0, 3), dict(traj_id=3))
    assert gateway.incoming.get_nowait()['traj_id'] == 3
    gateway.send(feedback(1., ready=False))
    gateway.send(feedback(2.))
    gateway.poll()
    assert gateway.raw is None and gateway.input_stream is None
    assert [msg.data[5] for msg in gateway.received] == [1., 2.]
    assert gateway.ready[0] and gateway.error is None
    assert len(gateway.warnings) == 2
    assert set(gateway.recording_failures) == {'trajectories.jsonl', 'measured-input.jsonl.gz'}


def test_worker_exit_and_initialization_errors_remain_fatal(transport):
    gateway = transport
    gateway.children = [SimpleNamespace(poll=lambda: 9)]
    gateway.send(feedback(1.))
    gateway.poll()
    assert gateway.error == 'A native ROS2 RACER process exited' and not gateway.received

    def fail_start(_):
        raise ValueError('native startup parameters missing')

    gateway.children.clear()
    gateway.outgoing.get_nowait()
    gateway.error = None
    gateway.started = False
    gateway.start = fail_start
    gateway.send(dict(type='hello'))
    gateway.poll()
    assert gateway.error == 'native startup parameters missing'


def test_optional_recording_open_and_protocol_failures_leave_native_core_running(
        native_transport, monkeypatch, tmp_path):
    module = native_transport
    monkeypatch.setenv('RACER_RECORD_INPUT', '1')
    monkeypatch.setenv('ASPIRE_FAST_STOP_HANDOFF', '1')
    file_open = Path.open
    write_text = Path.write_text

    def open_file(path, *args, **kwargs):
        if path.name in ('trajectories.jsonl', 'stop-feedback.jsonl', 'planner.log'):
            raise PermissionError('recording open refused')
        return file_open(path, *args, **kwargs)

    def write_file(path, data, **kwargs):
        if path.name == 'protocol.json':
            raise OSError('protocol recording storage unavailable')
        return write_text(path, data, **kwargs)

    def open_gzip(*args, **kwargs):
        raise PermissionError('gzip open refused')

    monkeypatch.setattr(Path, 'open', open_file)
    monkeypatch.setattr(Path, 'write_text', write_file)
    monkeypatch.setattr(gzip, 'open', open_gzip)
    child = SimpleNamespace(poll=lambda: None, send_signal=lambda _: None, wait=lambda **_: 0)
    launches = []

    def launch(args, **kwargs):
        launches.append(kwargs)
        return child

    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(tmp_path))
    monkeypatch.setattr(module, 'get_package_prefix', lambda _: str(tmp_path))
    monkeypatch.setattr(module, 'configuration', lambda *_: [
        dict(name='planner', executable='exploration_node', parameters={}, remappings=[])])
    monkeypatch.setattr(module.subprocess, 'Popen', launch)
    output = tmp_path/'capture'
    gateway = module.NativeDDSTransport(output, fleet_size=1)
    gateway.send(dict(type='hello'))
    gateway.poll()
    assert gateway.started and gateway.error is None and gateway.children == [child]
    assert (output/'planner.yaml').is_file() and launches[0]['stdout'] == module.subprocess.DEVNULL
    assert gateway.raw is None and gateway.input_stream is None
    failures = {'trajectories.jsonl', 'measured-input.jsonl.gz', 'planner.log', 'protocol.json'}
    if hasattr(gateway, 'flush_feedback'):
        assert gateway.stop_feedback_log is None
        failures.add('stop-feedback.jsonl')
    gateway.publish_row(feedback(1.))
    assert gateway.ready == [True] and len(gateway.received) == 1
    gateway.close()
    assert len(gateway.warnings) == len(failures)
    assert set(json.loads((output/'gateway-counts.json').read_text())['recording_failures']) == failures
    with pytest.raises(FileExistsError):
        module.NativeDDSTransport(output, fleet_size=1)


def test_core_config_write_and_process_spawn_failures_still_abort(
        native_transport, monkeypatch, tmp_path):
    module = native_transport
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(tmp_path))
    monkeypatch.setattr(module, 'get_package_prefix', lambda _: str(tmp_path))
    monkeypatch.setattr(module, 'configuration', lambda *_: [
        dict(name='planner', executable='exploration_node', parameters={}, remappings=[])])
    write_text = Path.write_text

    def write_file(path, data, **kwargs):
        if path.parent.name == 'blocked-config' and path.suffix == '.yaml':
            raise PermissionError('cannot write core configuration')
        return write_text(path, data, **kwargs)

    def launch(*args, **kwargs):
        raise FileNotFoundError('native planner executable unavailable')

    monkeypatch.setattr(Path, 'write_text', write_file)
    monkeypatch.setattr(module.subprocess, 'Popen', launch)
    for name, error in (
        ('blocked-config', 'cannot write core configuration'),
        ('blocked-spawn', 'native planner executable unavailable'),
    ):
        gateway = module.NativeDDSTransport(tmp_path/name, fleet_size=1)
        gateway.send(dict(type='hello'))
        gateway.poll()
        assert not gateway.started and gateway.error == error
        gateway.close()
        assert gateway.error == error


def test_optional_close_failure_preserves_child_exit_reason(transport, monkeypatch, tmp_path):
    def fail_close():
        raise OSError('buffered evidence flush failed')

    gateway = transport
    gateway.output = tmp_path
    gateway.children = [SimpleNamespace(poll=lambda: 4, wait=lambda **_: 4)]
    broken_stream = SimpleNamespace(close=fail_close)
    gateway.logs = [('planner.log', broken_stream)]
    gateway.raw = gateway.input_stream = broken_stream
    failures = {'trajectories.jsonl', 'measured-input.jsonl.gz', 'planner.log'}
    if hasattr(gateway, 'flush_feedback'):
        gateway.fast_stop_feedback = True
        gateway.stop_feedback_log = broken_stream
        failures.add('stop-feedback.jsonl')

    def fail_summary(*args, **kwargs):
        raise OSError('summary storage unavailable')

    monkeypatch.setattr(Path, 'write_text', fail_summary)
    gateway.poll()
    gateway.close()
    gateway.close()
    assert gateway.error == 'A native ROS2 RACER process exited'
    assert set(gateway.recording_failures) == failures | {'gateway-counts.json'}
    assert len(gateway.warnings) == len(failures) + 1


def test_fast_feedback_keeps_valid_pending_and_survives_serialization_error(transport):
    gateway = transport
    if not hasattr(gateway, 'flush_feedback'):
        pytest.skip('Frontier fast feedback path unavailable')
    gateway.fast_stop_feedback = True
    gateway.feedback_pending = {}
    gateway.feedback_lock = Lock()
    gateway.feedback_guard = SimpleNamespace(trigger=lambda: None)
    def fail_stop_record(_):
        raise OSError('collision stop recording unavailable')

    gateway.last_stop_feedback = {}
    gateway.stop_feedback_log = SimpleNamespace(write=fail_stop_record, close=lambda: None)
    gateway.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=2_200_000_000))
    valid = feedback(1.2, data=[-1., 0., 0., -1., -1., 1.2, 0., 7., .7])
    gateway.send(valid)
    gateway.send(feedback(1.4, data=[-1., 0., 0., -1., -1., float('nan'), 0.]))
    gateway.flush_feedback()
    assert gateway.received[0].data == valid['data'] and gateway.ready == [True, False]
    assert gateway.stop_feedback_log is None and len(gateway.warnings) == 1

    def fail_serialize(_):
        raise ValueError('ROS serialization rejected one message')

    gateway.publishers_by_kind['feedback', 0].publish = fail_serialize
    gateway.send(feedback(2., ready=False))
    gateway.send(feedback(2.1, drone=1))
    gateway.goal = SimpleNamespace(get_subscription_count=lambda: 0)
    gateway.flush_feedback()
    assert gateway.received[-1].data[5] == 2.1 and gateway.ready == [True, True]
    assert gateway.drops == 2 and gateway.error is None
