"""Run the normal SITL entry point against deterministic ROS and child-process stubs."""
import hashlib
import json
import os
from pathlib import Path
import sys
import subprocess
from types import SimpleNamespace as NS

import cv2
import pytest

from tools.flight import run


@pytest.fixture
def harness(tmp_path, monkeypatch):
    root = tmp_path / 'source'
    (root / 'config').mkdir(parents=True)
    (root / 'config/search.rviz').write_bytes((run.ROOT / 'config/search.rviz').read_bytes())
    monkeypatch.setattr(run, 'ROOT', root)
    monkeypatch.setattr(os, 'environ', {'RACER_CPU_PROFILE': 'render3-perception2',
                                       'RACER_GZ_SHUTDOWN_GUARD': '/missing/guard.so',
                                       'RACER_PX4_LOGGER_TOPICS': '/missing/topics'})
    monkeypatch.setattr(os, 'sched_getaffinity', lambda _: {0, 1}, raising=False)

    external = tmp_path / 'external'
    prefix = tmp_path / 'native'
    for path in (external / 'PX4-Autopilot/build/px4_sitl_default/bin/px4',
                 external / 'agent_install/bin/MicroXRCEAgent',
                 prefix / 'lib/racer_core/exploration_node'):
        path.parent.mkdir(parents=True)
        path.touch()
    startup = external / 'PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/rcS'
    startup.parent.mkdir(parents=True)
    startup.write_text('uxrce_dds_client start -t udp -p $uxrce_dds_port $uxrce_dds_ns\n')
    prepared = tmp_path / 'prepared'
    (prepared / 'models').mkdir(parents=True)
    (prepared / 'scene.sdf').write_text('<sdf><world/></sdf>')
    (prepared / 'bridge.yaml').write_text('[]')
    (prepared / 'runtime.json').write_text(json.dumps(dict(
        layout='original', performance_profile='legacy', models=str(prepared / 'models'),
        camera_transforms=[], bounds=[[-3,-4,0],[7,4,4]], fleet_starts=[[0,0,2.2]],
        vehicles=[dict(model='annual_x500_0',start=[0,0,2.2],spawn=[0,0,.24],
                       initial_position=[0,0,.24])], bootstrap_altitudes=[2.2])))

    clock = NS(wall=0., sim=0., active=True)
    state = NS(clock=clock, children={}, subscriptions={}, mandatory_exit=None, user_stop=False,
               png_fails=False, png_attempts=0, snapshot_fails=False, viewer_config_fails=False,
               flight_fault=None, safety_fault=None, return_blocked=False, odometry_at=40., takeoff_at=70.)

    def sleep(seconds):
        clock.wall += seconds

    monkeypatch.setattr(run, 'time', NS(monotonic=lambda: clock.wall, time=lambda: 0., sleep=sleep))

    def subscribe(_type, topic, callback, _qos):
        state.subscriptions[topic] = callback

    node = NS(create_subscription=subscribe, set_parameters=lambda _: None,
              destroy_node=lambda: setattr(state, 'node_destroyed', True),
              get_clock=lambda: NS(now=lambda: NS(nanoseconds=round(clock.sim * 1e9))))

    def spin(_node, timeout_sec):
        clock.wall += 25.
        if state.user_stop:
            clock.active = False
            return
        sim = clock.sim
        flying=state.takeoff_at is not None and sim>=state.takeoff_at
        state.subscriptions['/drone_0/flight_status'](NS(data=json.dumps(dict(
            origin=[0,0,0], origin_calibration='calibrated', armed=flying, offboard=flying,
            flight_session_fault=state.flight_fault if 45. <= sim < 65. else None))))
        state.subscriptions['/drone_0/view_execution'](NS(data=json.dumps(dict(
            ready=False, reason='initial_yaw_scan', time=sim, tracking_error=.8,
            safety_fault=state.safety_fault if 45. <= sim < 65. else None))))
        if state.odometry_at is not None and sim>=state.odometry_at:
            header = NS(stamp=NS(sec=int(sim), nanosec=0), frame_id='map')
            state.subscriptions['/drone_0/estimated_odometry'](NS(header=header, pose=NS(pose=NS(
                position=NS(x=0.,y=0.,z=.55 if flying else .24), orientation=NS(x=0.,y=0.,z=0.,w=1.)))))
            if '/drone_0/rgb/image' in state.subscriptions:
                state.subscriptions['/drone_0/rgb/image'](NS(
                    header=header, height=1,width=1,step=3,encoding='rgb8',data=b'\0\0\0'))
            depth_data = b'' if state.snapshot_fails and sim == 40. else b'\0\0\0\0'
            if '/drone_0/depth/image' in state.subscriptions:
                state.subscriptions['/drone_0/depth/image'](NS(
                    header=header,height=1,width=1,step=4,encoding='32FC1',is_bigendian=False,data=depth_data))
            for topic in ('rgb','depth'):
                info_topic = f'/drone_0/{topic}/camera_info'
                if info_topic in state.subscriptions:
                    state.subscriptions[info_topic](NS(header=header,k=[1.]*9))
        if state.mandatory_exit and sim >= 50.:
            state.children[state.mandatory_exit].returncode = 3
        if state.return_blocked and flying:
            state.subscriptions['/baseline/return_blocked_0'](NS())
        clock.sim += 5.

    def transform():
        return NS(transform=NS(translation=NS(), rotation=NS()))

    def install(name, **attrs):
        monkeypatch.setitem(sys.modules, name, NS(**attrs))

    install('ament_index_python.packages', get_package_prefix=lambda _: prefix)
    install('rclpy', init=lambda:None, create_node=lambda _:node, spin_once=spin,
            ok=lambda:clock.active, shutdown=lambda:None, parameter=NS(Parameter=lambda *a,**k:None))
    install('rclpy.qos', qos_profile_sensor_data=0, QoSProfile=lambda **k:k, DurabilityPolicy=NS(TRANSIENT_LOCAL=1))
    install('nav_msgs.msg', Odometry=object)
    install('sensor_msgs.msg', Image=object, CameraInfo=object)
    install('std_msgs.msg', Empty=object, String=object, Float64MultiArray=object)
    install('geometry_msgs.msg', TransformStamped=transform)
    install('tf2_ros', TransformBroadcaster=lambda _:NS(sendTransform=lambda _:None))
    install('px4_msgs.msg', EstimatorStatusFlags=object, VehicleLocalPosition=object)
    install('annual_runtime.px4_bridge_node', px4_topic=lambda root,*parts:root+'/'+('/'.join(parts[:-1])))

    def popen(args, *, stdout, **options):
        name = Path(stdout.name).stem
        if name == 'rviz':
            raise FileNotFoundError('rviz2 unavailable')
        process = NS(pid=100+len(state.children), returncode=4 if name in ('evaluation','continuous_rgb') else None,
                     args=args)
        process.poll = lambda:process.returncode
        state.children[name] = process
        return process

    def terminate(children, names, out, timeout):
        return {'direct_returncodes': [dict(pid=child.pid, returncode=child.returncode or 0)
                                       for child in children]}

    monkeypatch.setattr(run.subprocess, 'Popen', popen)
    monkeypatch.setattr(run, 'terminate_flight', terminate)
    def imwrite(*args):
        state.png_attempts += 1
        return not state.png_fails

    monkeypatch.setattr(cv2, 'imwrite', imwrite)

    def launch(*args, seconds=80.):
        if state.viewer_config_fails: (root / 'config/search.rviz').unlink()
        out = tmp_path / 'run'
        monkeypatch.setattr(sys, 'argv', ['run.py', '--external', str(external), '--prepared', str(prepared),
            '--output', str(out), '--seconds', str(seconds), '--budget-clock', 'sim', '--record-video', '--rviz',
            *args])
        state.result = lambda: json.loads((out / 'result.json').read_text())
        run.main()
        if (out/'result.json').exists():return state.result()

    state.launch = launch
    state.prepared = prepared
    state.output = tmp_path / 'run'
    return state


def test_prepared_static_aircraft_is_rejected_before_start(harness):
    model = harness.prepared/'models/annual_x500_0/model.sdf'
    model.parent.mkdir()
    model.write_text('<sdf><model name="annual_x500_0"><static>1</static></model></sdf>')
    (harness.prepared/'scene.sdf').write_text(
        '<sdf><world><include><uri>model://annual_x500_0</uri></include></world></sdf>')
    with pytest.raises(ValueError, match='static aircraft'):
        harness.launch()
    assert not harness.children


@pytest.mark.parametrize('record_input', [False, True])
def test_prepared_direct_dds_applies_launch_prerequisites(harness, monkeypatch, record_input):
    config_path = harness.prepared/'runtime.json'
    config = json.loads(config_path.read_text())
    config['native_frame_source'] = 'mapper_dds'
    config_path.write_text(json.dumps(config))
    monkeypatch.setenv('RACER_EXECUTION_SAFETY', '1')
    if record_input:
        monkeypatch.setenv('RACER_RECORD_INPUT', '1')
    result = harness.launch(seconds=0.)
    runtime = json.loads((harness.output/'runtime.json').read_text())
    assert runtime['native_frame_source'] == ('mapper_observation' if record_input else 'mapper_dds')
    assert ('native_map_dds' in result['diagnostics']) == record_input


def test_run_continues_after_diagnostics_and_optional_process_failures(harness):
    harness.snapshot_fails = True
    harness.png_fails = True
    result = harness.launch('--record-debug-evidence', '--segmentation-weights', '/missing/weights')
    assert result['error'] is None
    assert result['budget_end_sim'] == 80.
    assert result['wall_s'] > 300.
    assert result['effective_harness_wall_watchdog_seconds'] is None
    assert result['deferred_graph_started'] and result['startup']['ready']
    assert result['startup']['odom_at']['sim_s'] == 40.
    assert result['startup']['ready_at']['sim_s'] == pytest.approx(70.)
    assert result['startup']['takeoff_observed'] and not result['startup']['executor_ready']
    assert result['captured_pairs'] == 0 and harness.png_attempts > 0
    assert set(result['diagnostics']) >= {
        'cpu_profile', 'gazebo_guard_configuration', 'px4_logger_configuration', 'rviz_spawn',
        'evaluation_exit', 'continuous_rgb_exit', 'startup_odometry', 'startup_ready',
        'bootstrap_tracking', 'rgbd_snapshot',
        'segmentation_weights'}
    assert result['cpu_profile']['name'] == 'inherit'


def test_missing_viewer_config_and_optional_modes_leave_base_run_available(harness):
    harness.viewer_config_fails = True
    result = harness.launch('--no-perception', '--confirmation-mode', 'registry', '--review-continuation',
                            '--registry-cue-fixture', '--map-delay-fixture',
                            '--static-guidance-settings', '/missing/settings')
    assert result['error'] is None
    assert result['budget_end_sim'] == 80.
    assert result['confirmation_mode'] == 'legacy'
    assert not result['review_continuation']
    assert 'person_search' not in harness.children
    assert set(result['diagnostics']) >= {
        'viewer_configuration', 'perception', 'registry_cue_fixture', 'map_delay_fixture',
        'review_continuation', 'static_pair'}


@pytest.mark.parametrize(('backend', 'custom_model'), [
    ('yolo11n-seg', False), ('yolo11n-seg', True), ('rfdetr-seg-nano', False),
])
def test_segmentation_backend_reaches_registry_node(harness, tmp_path, monkeypatch, backend, custom_model):
    weights = tmp_path/'model weights.pt'
    weights.write_bytes(b'test weights')
    manifest = tmp_path/'model manifest.json'
    manifest.write_text(json.dumps({'model_sha256': hashlib.sha256(weights.read_bytes()).hexdigest()}))
    monkeypatch.chdir(tmp_path)
    options = ['--detector-manifest', manifest.name] if custom_model else []
    weights_arg = weights.name if custom_model else str(weights)

    result = harness.launch('--confirmation-mode', 'registry', '--detector', backend,
                            '--segmentation-weights', weights_arg, *options)
    perception_args = harness.children['person_search'].args
    assert result['error'] is None and result['confirmation_mode'] == 'registry'
    assert f'detector_backend:={backend}' in perception_args
    assert f'segmentation_weights:={weights}' in perception_args
    assert [arg for arg in perception_args if arg.startswith('detector_manifest:=')] == (
        [f'detector_manifest:={manifest}'] if custom_model else [])
    assert result['detector_manifest_sha256'] == (
        hashlib.sha256(manifest.read_bytes()).hexdigest() if custom_model else None)
    planner_args = harness.children['racer_adapter'].args
    assert all('detector_manifest' not in arg for arg in planner_args)


def test_missing_evidence_hashes_do_not_stop_owned_processes(harness, monkeypatch):
    (run.ROOT/'tools').mkdir()
    source = run.ROOT/'tools/unreadable.py'
    source.touch()
    read_bytes = Path.read_bytes

    def unavailable(path):
        if path.name in {source.name, 'exploration_node', 'px4'}:
            raise PermissionError(f'Cannot read {path.name} for hashing')
        return read_bytes(path)

    monkeypatch.setattr(Path, 'read_bytes', unavailable)
    result = harness.launch('--record-debug-evidence')
    assert result['error'] is None and result['budget_end_sim'] == 80.
    assert result['deferred_graph_started'] and result['startup']['ready']
    assert result['code_sha256'] == {}
    assert result['native_binary_sha256'] is result['px4_binary_sha256'] is None
    assert {'runtime_source_archive','native_binary_hash','px4_binary_hash'} <= set(result['diagnostics'])


def test_final_report_destination_failure_still_releases_owned_resources(harness, monkeypatch, capsys):
    original = Path.write_text

    def write(path, *args, **kwargs):
        if path.name == 'result.json':
            raise OSError('report directory quota')
        return original(path, *args, **kwargs)

    monkeypatch.setattr(Path, 'write_text', write)
    harness.launch()
    report = json.loads(capsys.readouterr().out)
    assert harness.node_destroyed and report['startup']['ready']
    assert report['cleanup']['direct_returncodes']
    assert report['diagnostics']['result_write'] == 'report directory quota'


def test_preserves_explicit_wall_watchdog(harness):
    with pytest.raises(SystemExit) as exit_info:
        harness.launch('--wall-watchdog-seconds', '140')
    assert exit_info.value.code == 1
    result = harness.result()
    assert result['error'] == 'Explicit wall watchdog exceeded'
    assert not result['deferred_graph_started']


def test_blocked_return_ends_run_as_failure_and_cleans_up(harness):
    harness.return_blocked = True
    with pytest.raises(SystemExit) as stopped:
        harness.launch('--recover-no-path', seconds=10_000.)
    result = harness.result()
    assert stopped.value.code == 1
    assert result['error'] == 'Return blocked: no authorized route home'
    assert result['last']['return_blocked']['reason'] == 'no_authorized_return_route'
    assert result['budget_end_sim'] == harness.takeoff_at
    assert result['cleanup']['direct_returncodes']


def test_preserves_core_process_exit_failure(harness):
    harness.mandatory_exit = 'agent'
    with pytest.raises(SystemExit) as exit_info:
        harness.launch()
    assert exit_info.value.code == 1
    result = harness.result()
    assert result['budget_end_sim'] < 80.
    assert 'Owned process exited' in result['error']
    assert any(row['name']=='agent' and row['returncode']==3 for row in result['unexpected_exits'])


def test_user_stop_ends_run_and_reports_cleanup(harness):
    harness.user_stop = True
    with pytest.raises(SystemExit) as exit_info:
        harness.launch()
    assert exit_info.value.code == 1
    result = harness.result()
    assert result['error'] == 'KeyboardInterrupt'
    assert result['cleanup']['direct_returncodes']


def test_unavailable_affinity_tool_still_runs_workload(tmp_path):
    from tools.flight.cpu_profile import CPUProfile

    cpu = CPUProfile('render3-perception2', allowed=range(5))
    args = cpu.best_effort_command('gazebo', [sys.executable, '-c', 'print("workload")'])
    proc = subprocess.run(args, env={'PATH':str(tmp_path)}, capture_output=True, text=True, check=True)
    assert proc.stdout.strip() == 'workload'



@pytest.mark.parametrize(('attribute','prefix'), [
    ('flight_fault','Latched flight fault'),
    ('safety_fault','Execution safety fault'),
])
def test_terminal_mission_fault_fails_result_after_full_budget(harness, attribute, prefix):
    setattr(harness, attribute, 'fixture_latch')
    with pytest.raises(SystemExit) as exit_info:
        harness.launch()
    result = harness.result()
    assert exit_info.value.code == 1
    assert result['budget_end_sim'] == 80.
    assert result['error'] == f'{prefix}: fixture_latch'
    assert result['cleanup']['direct_returncodes']


@pytest.mark.parametrize('completed', [True, False])
def test_explicit_fault_fixture_accepts_expected_latch_and_requires_completion(harness, monkeypatch, completed):
    from tools.flight import fault_injection

    class FixtureSchedule(fault_injection.FaultSchedule):
        def step(self, sim, flight, execution):
            if sim >= 60. and completed:
                self.state = 'complete'
            elif sim >= 40.:
                self.state = 'active'
            return self.state == 'complete'

    monkeypatch.setattr(fault_injection, 'FaultSchedule', FixtureSchedule)
    harness.flight_fault = 'px4_failsafe_requires_new_session'
    harness.takeoff_at = None
    if completed:
        result = harness.launch('--fault-injection', 'dds')
        assert result['error'] is None
        assert result['budget_end_sim'] == pytest.approx(60.)
    else:
        with pytest.raises(SystemExit):
            harness.launch('--fault-injection', 'dds')
        result = harness.result()
        assert result['budget_end_sim'] == 80.
        assert result['error'] == 'Fault fixture did not complete within budget'


def test_zero_sim_budget_ends_immediately(harness):
    result = harness.launch(seconds=0.)
    assert result['error'] is None and result['budget_end_sim'] == 0.
    assert not result['deferred_graph_started']



@pytest.mark.parametrize(('odometry_at','reason'), [
    (None,'Run budget expired before valid estimated odometry and search startup'),
    (40.,'Run budget expired before measured flight readiness'),
])
def test_incomplete_startup_fails_only_after_positive_budget(harness, odometry_at, reason):
    harness.odometry_at = odometry_at
    harness.takeoff_at = None
    with pytest.raises(SystemExit) as exit_info:
        harness.launch()
    result = harness.result()
    assert exit_info.value.code == 1 and result['error'] == reason
    assert result['budget_end_sim'] == 80.
    assert result['deferred_graph_started'] is (odometry_at is not None)
    assert not result['startup']['ready']



@pytest.mark.parametrize('safety,override,effective', [(True,None,1.),(True,0.,1.),(False,2.,2.)])
def test_launch_reports_and_applies_supported_pacing_to_scene_copy(harness, safety, override, effective):
    if safety:
        os.environ['RACER_EXECUTION_SAFETY']='1'
    source=harness.prepared/'scene.sdf'
    original=('<sdf><world><physics><max_step_size>.004</max_step_size>'
              '<real_time_factor>3</real_time_factor></physics></world></sdf>')
    source.write_text(original)
    args=[] if override is None else ['--sim-real-time-factor',str(override)]
    result=harness.launch(*args)
    assert result['error'] is None
    assert result['requested_sim_real_time_factor']==(3. if override is None else override)
    assert result['effective_sim_real_time_factor']==effective
    assert ('sim_real_time_factor' in result['diagnostics']) is safety
    assert source.read_text()==original
    assert f'<real_time_factor>{effective}</real_time_factor>' in (harness.output/'scene.sdf').read_text()
    if safety:
        runtime=json.loads((harness.output/'runtime.json').read_text())
        assert runtime['execution_safety_timing_contract']['physics_real_time_factor']==effective



def test_launch_applies_explicit_pacing_in_world_without_physics(harness):
    original=(harness.prepared/'scene.sdf').read_text()
    result=harness.launch('--sim-real-time-factor','2')
    assert result['error'] is None
    assert result['requested_sim_real_time_factor']==result['effective_sim_real_time_factor']==2.
    assert (harness.prepared/'scene.sdf').read_text()==original
    assert '<real_time_factor>2.0</real_time_factor>' in (harness.output/'scene.sdf').read_text()


def test_team_bundle_relocates_models_and_retains_negative_case(harness):
    manifest = {"version": 1, "negative": True}
    (harness.prepared / "bundle.json").write_text(json.dumps(manifest))
    result = harness.launch()
    assert result["negative"] is True
    config = json.loads((harness.output / "runtime.json").read_text())
    assert config["models"] == str(run.ROOT / "external/PX4-gazebo-models/models")
    assert json.loads((harness.output / "bundle.json").read_text()) == manifest


def test_default_run_skips_debug_evidence_and_warns_only_on_post_budget_cleanup(harness, monkeypatch):
    import yaml

    def terminate_with_cleanup_crash(children, names, out, timeout):
        return {'direct_returncodes': [
            dict(pid=child.pid, returncode=134 if names[child.pid] == 'gazebo' else 0)
            for child in children]}

    monkeypatch.setattr(run, 'terminate_flight', terminate_with_cleanup_crash)
    result = harness.launch()
    assert result['error'] is None and 'cleanup_abnormal_exit' in result['diagnostics']
    assert not (harness.output / 'runtime-source.tar.gz').exists()
    assert harness.png_attempts == 0
    nodes = yaml.safe_load((harness.output / 'nodes.yaml').read_text())['/**']['ros__parameters']
    assert nodes['control_timing_dir'] == '' and nodes['execution_safety_dir'] == ''
    assert all('evidence_dir:=' not in arg for arg in harness.children['person_search'].args)
    assert not any(topic in harness.subscriptions for topic in (
        '/drone_0/rgb/image', '/drone_0/depth/image',
        '/drone_0/rgb/camera_info', '/drone_0/depth/camera_info'))
