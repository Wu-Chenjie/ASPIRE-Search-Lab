"""Compile the same C++ scheduler used by the native FSM, with no ROS mock."""
from pathlib import Path
import shutil
import subprocess
import pytest
from tools.racer_ros2.configuration import configuration

def test_native_recovery_failure_flood_and_state_transitions(tmp_path):
    compiler=shutil.which('c++')
    if compiler is None:pytest.skip('C++ compiler unavailable')
    root=Path(__file__).resolve().parents[1];binary=tmp_path/'recovery-policy-test'
    subprocess.run([compiler,'-std=c++17','-Wall','-Wextra','-Werror','-I',str(root/'tools/flight'),str(root/'tests/recovery_policy_test.cpp'),'-o',str(binary)],check=True,capture_output=True,text=True)
    result=subprocess.run([str(binary)],check=True,capture_output=True,text=True)
    assert 'PASS:' in result.stdout

def test_no_path_switch_keeps_original_safety_and_nominal_limits(tmp_path):
    root=Path(__file__).resolve().parents[1]
    hello=dict(starts=[[0,-.9,2.2]],bounds=[[-3,-4,0],[7,4,4]],intrinsics=[216,0,160,0,216,120,0,0,1],motion_limits=dict(speed=1,acceleration=1.2,jerk=4,yaw_rate=.9),planning_limits=dict(speed=.3,acceleration=.4,jerk=4,yaw_rate=.325),flight_limits=[.7,3.],min_time=False,no_path_recovery=True,stop_and_plan=True)
    cfg=configuration(hello,tmp_path,root/'external/racer_ros2_ws/src/racer_core/config/single_drone_planner.xml');p=next(c['parameters'] for c in cfg if c['executable']=='exploration_node')
    assert p['search.no_path_recovery'] and p['search.stop_and_plan']
    assert p['fsm.authorized_execution'] and p['sdf_map.obstacles_inflation']>=.6
    assert p['manager.max_vel']==.3 and p['manager.max_acc']==.4
    assert not p['manager.min_time'] and not p['search.feasible_time']


@pytest.mark.parametrize('case', ['axis', 'recover', 'blocked', 'inputs', 'invalid', 'rollback',
                                 'gap', 'bearing', 'registry', 'transit', 'return-kino', 'return-waypoint'])
def test_real_native_recovery_and_optimizer_failures(tmp_path,monkeypatch,case):
    pytest.importorskip('rclpy')
    from ament_index_python.packages import get_package_prefix
    import yaml,json
    import numpy as np
    from core.planning.bspline_trajectory import native_trajectory,safe_native_execution
    from core.exploration.voxel_mapping import VoxelMap
    from core.planning.motion_limits import motion_limits
    root=Path(__file__).resolve().parents[1]
    binary=Path(get_package_prefix('racer_core'))/'lib/racer_core/task_recovery_probe'
    fixed_return = case in ('return-kino', 'return-waypoint')
    if fixed_return and not binary.with_name('task_authorization_recovery_probe').is_file():
        pytest.skip('Fixed return requires an execution-safety RACER build')
    limits=dict(speed=1,acceleration=1.2,jerk=4,yaw_rate=.9)
    monkeypatch.setenv('ANNUAL_MOTION_LIMITS',json.dumps(limits))
    monkeypatch.setenv('ANNUAL_VOXEL_MAP',json.dumps(dict(resolution=.15,collision_profile='legacy')))
    hello=dict(starts=[[0,0,1.8]],bounds=[[-3,-4,0],[7,4,4]],intrinsics=[216,0,160,0,216,120,0,0,1],flight_limits=[.7,3.],motion_limits=limits,planning_limits=dict(speed=.3,acceleration=.4,jerk=4,yaw_rate=.325),min_time=False)
    if case=='registry':hello['confirmation_mode']='registry'
    cfg=configuration(hello,tmp_path,root/'external/racer_ros2_ws/src/racer_core/config/single_drone_planner.xml')
    params=next(c['parameters'] for c in cfg if c['executable']=='exploration_node');params['use_sim_time']=False
    if case=='transit':
        monkeypatch.setenv('ASPIRE_KNOWN_TRANSIT','1')
        params['task_probe.end_yaw']=.25
    if fixed_return:
        monkeypatch.setenv('ASPIRE_KNOWN_TRANSIT', '0')
        params['optimization.max_iteration_time2'] = .05
    if case == 'return-kino':
        params['search.horizon'] = 1.
    path=tmp_path/'probe.yaml';path.write_text(yaml.safe_dump({'/**':{'ros__parameters':params}}))
    result=subprocess.run([str(binary),'--case','axis' if case=='transit' else case,'--output',str(tmp_path/'probe'),'--ros-args','--params-file',str(path)],capture_output=True,text=True,timeout=45)
    (tmp_path/'native-recovery.log').write_text(result.stdout+'\n'+result.stderr)
    assert result.returncode==0,result.stdout+'\n'+result.stderr
    markers=dict(axis='PASS_AXIS',recover='PASS_NATIVE_RECOVERY',blocked='PASS_BLOCKED',inputs='PASS_INPUTS',invalid='PASS_INVALID',rollback='PASS_ROLLBACK',gap='PASS_GAP',bearing='PASS_NATIVE_BEARING',registry='PASS_NATIVE_REGISTRY',transit='KNOWN_TRANSIT length=')
    markers.update({'return-kino': 'PASS_FIXED_RETURN_KINO', 'return-waypoint': 'PASS_FIXED_RETURN_WAYPOINT'})
    assert markers[case] in result.stdout+result.stderr
    if fixed_return or case in ('axis','recover','rollback','bearing','registry','transit'):
        packet=json.loads((tmp_path/'probe-curve.json').read_text())
        curve=native_trajectory(packet)  # Original C2, finite/domain and continuous dynamics checks.
        if case=='transit':
            t=.4*curve.duration
            p,v,_,h,_=curve.sample(t)
            assert p[0]>.1 and v[0]>.1 and abs(h)<1e-8
            assert curve.sample(curve.duration)[3]==pytest.approx(.25)
            assert curve.yaw_rate_limit()<=.325+1e-8
            gap=subprocess.run([str(binary),'--case','gap','--ros-args','--params-file',str(path)],
                               capture_output=True,text=True,timeout=45)
            assert gap.returncode==0 and 'PASS_GAP' in gap.stdout, gap.stdout+gap.stderr
        observed=json.loads((tmp_path/'probe-map.json').read_text())
        bounds=np.array([observed['origin'],np.array(observed['origin'])+observed['size']])
        runtime=VoxelMap(bounds,resolution=observed['resolution'],clearance=.6,flight_limits=(.7,3.))
        assert tuple(runtime.shape)==tuple(observed['shape'])
        runtime.state[:]=np.array(observed['state'],dtype=np.int8).reshape(runtime.shape);runtime.rebuild()
        assert runtime.safe_path(curve.path(.01))
        stopped,end=safe_native_execution(curve,runtime)  # Existing body/unknown-space and C2 stop checks.
        assert end==pytest.approx(curve.duration)
        assert runtime.safe_path(stopped.path(.01)) and motion_limits().accepts(stopped)
        assert np.linalg.norm(stopped.sample(stopped.duration)[1])<1e-5
        (tmp_path/'execution-check.json').write_text(json.dumps(dict(original_gate_passed=True,full_native_duration=curve.duration,stopped_duration=stopped.duration,limits=stopped.limits(),yaw_rate=stopped.yaw_rate_limit(),no_flight=True),indent=2))
