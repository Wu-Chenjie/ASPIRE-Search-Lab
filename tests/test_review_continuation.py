"""Same-request bounded native lifecycle; no simulated flight claims."""
import json
from pathlib import Path
import shutil
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]


def test_real_cpp_review_lifecycle_and_cancel_protocol(tmp_path):
    compiler = shutil.which('c++')
    if compiler is None:
        pytest.skip('C++ compiler unavailable')
    binary = tmp_path/'review-continuation-policy'
    subprocess.run([compiler, '-std=c++17', '-Wall', '-Wextra', '-Werror',
                    '-I', str(ROOT/'tools/flight'), str(ROOT/'tests/review_continuation_test.cpp'),
                    '-o', str(binary)], check=True, capture_output=True, text=True)
    result = subprocess.run([str(binary)], check=True, capture_output=True, text=True)
    assert 'same_request_three_segments=1 budgets_no_reset=1 fresh_matching_done=1 exact_integer_identity=1 deferred_cancel=1' in result.stdout


def test_default_disabled_and_bounded_runtime_parameters(tmp_path):
    from tools.racer_ros2.configuration import configuration
    hello = dict(starts=[[0,0,1.8]], bounds=[[-3,-4,0],[7,4,4]],
        intrinsics=[216,0,160,0,216,120,0,0,1], flight_limits=[.7,3.],
        motion_limits=dict(speed=1,acceleration=1.2,jerk=4,yaw_rate=.9),
        planning_limits=dict(speed=.3,acceleration=.4,jerk=4,yaw_rate=.325), min_time=False)
    source = ROOT/'external/racer_ros2_ws/src/racer_core/config/single_drone_planner.xml'
    def params():
        return next(c['parameters'] for c in configuration(hello,tmp_path,source) if c['executable']=='exploration_node')
    p = params()
    assert not p['search.review_continuation']
    assert p['search.review_observation_hold_s']==5.
    assert (p['search.review_max_segments'], p['search.review_max_duration_s'], p['search.review_max_path_m'])==(3,60.,6.)
    hello.update(review_continuation=True,review_max_segments=2,review_max_duration_s=40.,review_max_path_m=4.,review_observation_hold_s=3.)
    p = params()
    assert p['search.review_continuation']
    assert p['search.review_observation_hold_s']==3.
    assert (p['search.review_max_segments'],p['search.review_max_duration_s'],p['search.review_max_path_m'])==(2,40.,4.)
    assert p['manager.max_vel']==.3 and p['manager.max_acc']==.4
    assert p['sdf_map.obstacles_inflation']>=.6 and p['fsm.authorized_execution']


@pytest.mark.parametrize('fixture',[False,True])
def test_overlay_build_preserves_planner_and_execution_authorization(tmp_path,fixture):
    build = tmp_path/'generated'
    flags=['--execution-safety','--consistent-inflation']+(['--flight-fixture'] if fixture else [])
    subprocess.run([sys.executable,str(ROOT/'tools/search/build_native.py'),str(build),*flags],
                   check=True,capture_output=True,text=True)
    core = build/'src/racer_core'
    s=(core/'src/exploration_manager/fast_exploration_fsm.cpp').read_text()
    assert 'if(!task_review_continuation_)task_review_pending_=false;' in s
    assert 'bounded same-request review continuation' in s
    assert 'if(taskReviewContinuationTick())return;' in s
    feedback=s.split('void FastExplorationFSM::executionFeedback(')[1].split('void FastExplorationFSM::swarmTrajTimerCallback')[0]
    assert 'const double received_at=racer_ros::Time::now().toSec();' in feedback
    assert feedback.index('if (d[5] < feedback_stamp_ || fabs(received_at-d[5]) > 0.5)')<feedback.index('feedback_stamp_ = d[5];')
    assert 'if(task_stop_and_plan_ && live_execution_ && executed_id_>=0 && !executed_done_ && !task_c2_requested) return;' in s
    assert 'const bool task_c2_requested=!task_fresh_review_&&task_static_c2_.enabled&&task_static_work_&&' in s
    assert '!task_static_gate_.published&&live_execution_' in s
    assert 'endpoint_not_currently_free' in s
    assert 'proposed_duration_exceeds_remaining_budget' in s
    assert 'task_review_continuation_probe' in (core/'CMakeLists.txt').read_text()
    assert json.loads((build/'task-overlay.json').read_text())['execution_safety']
    from tools.flight.review_continuation_overlay import apply
    import inspect
    source=inspect.getsource(apply)
    assert 'src/exploration_manager/fast_exploration_fsm.cpp' in source
    assert 'src/plan_env/' not in source and 'src/plan_manage/' not in source


def test_real_ros_native_review_callback(tmp_path):
    pytest.importorskip('rclpy')
    from ament_index_python.packages import get_package_prefix
    from tools.racer_ros2.configuration import configuration
    import yaml
    binary=Path(get_package_prefix('racer_core'))/'lib/racer_core/task_review_continuation_probe'
    if not binary.is_file():
        pytest.skip('New execution-safety native build required')
    hello=dict(starts=[[0,0,1.8]],bounds=[[-3,-4,0],[7,4,4]],
        intrinsics=[216,0,160,0,216,120,0,0,1],flight_limits=[.7,3.],
        motion_limits=dict(speed=1,acceleration=1.2,jerk=4,yaw_rate=.9),
        planning_limits=dict(speed=.3,acceleration=.4,jerk=4,yaw_rate=.325),
        min_time=False,active_review=True,stop_and_plan=True,review_continuation=True)
    cfg=configuration(hello,tmp_path,ROOT/'external/racer_ros2_ws/src/racer_core/config/single_drone_planner.xml')
    params=next(c['parameters'] for c in cfg if c['executable']=='exploration_node');params['use_sim_time']=False
    path=tmp_path/'probe.yaml';path.write_text(yaml.safe_dump({'/**':{'ros__parameters':params}}))
    result=subprocess.run([str(binary),'--ros-args','--params-file',str(path)],capture_output=True,text=True,timeout=45)
    (tmp_path/'native-review-continuation.log').write_text(result.stdout+'\n'+result.stderr)
    assert result.returncode==0,result.stdout+'\n'+result.stderr
    assert 'PASS_NATIVE_REVIEW_HOLD same_budget=1 bounded_5s=1 matching_evidence_release=1 return_release=1' in result.stdout
    assert 'PASS_NATIVE_REVIEW_CALLBACK correlation=1 fresh_actual_arrival=1 unchanged_arrival_0.25=1 unknown_map_rejected=1 deferred_cancel=1 timeout=1 no_flight=1' in result.stdout

