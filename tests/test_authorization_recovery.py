"""Bound authorization rejection episodes without changing native safety gates."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]


def test_distinct_authorization_rejections_enter_bounded_real_cpp_policy(tmp_path):
    compiler = shutil.which('c++')
    if compiler is None:
        pytest.skip('C++ compiler unavailable')
    binary = tmp_path/'authorization-recovery'
    subprocess.run([compiler, '-std=c++17', '-Wall', '-Wextra', '-Werror',
                    '-I', str(ROOT/'tools/flight'), str(ROOT/'tests/authorization_recovery_test.cpp'),
                    '-o', str(binary)], check=True, capture_output=True, text=True)
    result = subprocess.run([str(binary)], check=True, capture_output=True, text=True)
    assert 'trigger=3 recovery_attempts=9 duplicate_feedback=300 activation_only_reset=1' in result.stdout


def hashes(folder):
    return {str(p.relative_to(folder)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in folder.rglob('*') if p.is_file()}


def generate(path, *flags):
    subprocess.run([sys.executable, str(ROOT/'tools/search/build_native.py'), str(path), *flags],
                   check=True, capture_output=True, text=True)


def test_opt_in_overlay_touches_only_fsm_and_admission_helper(tmp_path):
    from tools.flight.authorization_recovery_overlay import apply
    build = tmp_path/'default';generate(build)
    core = build/'src/racer_core'
    before = hashes(core)
    text = (core/'src/exploration_manager/fast_exploration_fsm.cpp').read_text()
    assert 'task_np_policy_.active() && candidate!=task_review_proposal_' in text
    assert not (core/'task_authorization_recovery_probe.cpp').exists()
    assert 'task_authorization_recovery_probe' not in (core/'CMakeLists.txt').read_text()
    apply(core)
    after = hashes(core)
    assert {name for name in set(before)|set(after) if before.get(name)!=after.get(name)} == {
        'include/exploration_manager/fast_exploration_fsm.h',
        'include/exploration_manager/task_authorization_recovery.hpp',
        'src/exploration_manager/fast_exploration_fsm.cpp'}
    changed = (core/'src/exploration_manager/fast_exploration_fsm.cpp').read_text()
    for gate in [
        'if (!live_execution_ || (msg->data.size() != 7 && msg->data.size() != 8)) return;',
        'if (candidate == fd_->newest_traj_.traj_id && d[4] < 0.) {',
        'if (d[5] < feedback_stamp_ || fabs(racer_ros::Time::now().toSec()-d[5]) > 0.5) return;',
        'task_no_path_recovery_ && task_np_policy_.active() && !taskRecoveryReady()) return;',
        'task_np_policy_.activated();']:
        assert changed.count(gate)==text.count(gate)==1
    # Callback uses the saved proposal/goal, not mutable ed metadata or spline end.
    callback = changed[changed.index('void FastExplorationFSM::executionFeedback'):]
    assert 'authorizationRejected(task_np_policy_,task_no_path_recovery_' in callback
    assert 'task_authorization_proposal_,task_review_proposal_,task_last_rejected_' in callback
    assert 'task_authorization_goal_.x()' in callback
    # The actual local alternative argument is captured at its native call.
    assert 'task_planned_alternative=true; task_alternative_goal=goal;\n      res=expl_manager_->planTrajToView' in changed
    assert 'task_authorization_goal_=task_planned_alternative?task_alternative_goal:expl_manager_->ed_->next_pos_;' in changed
    assert before['include/exploration_manager/task_recovery_policy.hpp']==after['include/exploration_manager/task_recovery_policy.hpp']


def test_execution_safety_build_installs_real_fsm_callback_probe(tmp_path):
    build = tmp_path/'safety';generate(build, '--execution-safety')
    core = build/'src/racer_core'
    assert (core/'task_authorization_recovery_probe.cpp').is_file()
    cmake = (core/'CMakeLists.txt').read_text()
    assert 'add_executable(task_authorization_recovery_probe task_authorization_recovery_probe.cpp src/exploration_manager/fast_exploration_fsm.cpp)' in cmake
    assert 'target_link_libraries(task_authorization_recovery_probe racer_algorithms)' in cmake
    assert json.loads((build/'task-overlay.json').read_text())['execution_safety']
    fsm = (core/'src/exploration_manager/fast_exploration_fsm.cpp').read_text()
    assert 'transitState(IDLE,"waiting for measured frontiers");' in fsm
    assert 'task_return_.started() && (res==NO_GRID || res==NO_FRONTIER)' in fsm
    assert 'const bool observation_stop=d.size()==8;' in fsm
    assert 'task_return_.stopped(executed_id_,d[5])' in fsm
    callback = fsm.split('void FastExplorationFSM::executionFeedback(', 1)[1]
    assert callback.index('(!collision_stop && !collision_hold_feedback)') < callback.index('feedback_stamp_ = d[5];')
    assert callback.index('if(collision_hold_feedback &&') < callback.index('taskGuidanceFeedback(')


def test_collision_receipt_dispatch_at_generated_cpp_callback(tmp_path):
    compiler = shutil.which('c++')
    if compiler is None:
        pytest.skip('C++ compiler unavailable')
    build = tmp_path/'native'
    generate(build, '--execution-safety')
    core = build/'src/racer_core'
    fsm = (core/'src/exploration_manager/fast_exploration_fsm.cpp').read_text()
    start = fsm.index('void FastExplorationFSM::executionFeedback(')
    end = fsm.index('  if (active >= 0 &&', start)
    source = tmp_path/'collision_feedback.cpp'
    # Compile the generated callback through dispatch and return accounting.
    # Stand-ins supply the ROS message/clock and observe downstream dispatch.
    source.write_text(r'''
#include <cassert>
#include <exploration_manager/task_return_policy.hpp>
#include <memory>
#include <vector>
#define ROS_WARN(...) do {} while (false)
namespace std_msgs {
struct Message { std::vector<double> data; };
using Float64MultiArrayConstPtr=std::shared_ptr<Message>;
}
namespace racer_ros {
struct Time {
    static Time now() { return {}; }
    double toSec() const { return 10.; }
};
}
enum State { EXEC_TRAJ, PLAN_TRAJ, PUB_TRAJ };
struct FastExplorationFSM {
    struct Planner {
        int calls=0;
        void taskGuidanceFeedback(int,bool,int,bool) { ++calls; }
    } planner;
    struct Data {
        struct Trajectory { int traj_id=42; } newest_traj_;
        bool avoid_collision_=true;
    } data;
    Planner* expl_manager_=&planner;
    Data* fd_=&data;
    racer_task::ReturnPolicy task_return_;
    State state_=EXEC_TRAJ;
    bool live_execution_=true, live_sent_=false, executed_done_=false;
    bool task_collision_stop_enabled_=true, task_collision_stopped_=false, task_stop_for_review_=false;
    double feedback_stamp_=9.9, task_collision_stop_stamp_=9.8;
    int executed_id_=42, task_collision_stop_id_=42, task_collision_stop_released_id_=-1;
    int dispatched=0;
    int settled_references=0;
    void taskReturnLog(const char*) {}
    void settledReferenceFeedback(const std_msgs::Float64MultiArrayConstPtr&) { ++settled_references; }
    void executionFeedback(const std_msgs::Float64MultiArrayConstPtr&);
};
''' + fsm[start:end] + r'''
    ++dispatched;
}
int main() {
    FastExplorationFSM f;
    f.task_return_.begin({0.,0.,1.8});
    assert(f.task_return_.attempt(9.,false));
    f.task_return_.proposed(42);
    assert(f.task_return_.activated(42));
    auto ref=std::make_shared<std_msgs::Message>();
    ref->data={42.,9.9,0.,0.,1.8,0.};
    f.executionFeedback(ref);
    assert(f.settled_references==1 && f.dispatched==0);
    auto send = [&](double sent, int id, double request, double done=0., int candidate=-1) {
        auto msg=std::make_shared<std_msgs::Message>();
        msg->data={-1.,.564,done,double(candidate),-1.,sent,0.,double(id),request};
        f.executionFeedback(msg);
    };
    send(9.9,41,9.7);
    send(10.,41,9.8);
    send(10.,42,9.7);
    send(10.,42,9.8,1.);
    assert(f.feedback_stamp_==9.9 && f.task_return_.executing() && f.executed_id_==42);
    assert(f.dispatched==0 && f.planner.calls==0);
    send(9.9,42,9.8);
    assert(f.task_collision_stopped_ && f.task_collision_stop_id_<0 && f.dispatched==1);
    assert(!f.task_return_.executing() && !f.task_return_.terminal());
    // Stand in for the rest of the callback clearing owned execution on stop.
    f.executed_id_=-1;
    f.state_=PLAN_TRAJ;
    send(10.,42,9.8,0.,42);
    assert(f.feedback_stamp_==10. && f.dispatched==1 && f.planner.calls==1);
    f.state_=PUB_TRAJ;
    f.live_sent_=true;
    f.data.newest_traj_.traj_id=43;
    send(10.,42,9.8,0.,43);
    assert(f.dispatched==2 && f.planner.calls==2);
    f.executed_id_=43;
    send(10.,42,9.8,0.,43);
    assert(f.dispatched==2 && f.planner.calls==2);
}
''')
    binary = tmp_path/'collision-feedback'
    subprocess.run([compiler, '-std=c++17', '-Wall', '-Wextra', '-Werror',
                    '-I', str(core/'include'), str(source), '-o', str(binary)],
                   check=True, capture_output=True, text=True)
    subprocess.run([str(binary)], check=True, capture_output=True, text=True)


def test_real_ros_native_authorization_recovery_callback(tmp_path):
    pytest.importorskip('rclpy')
    from ament_index_python.packages import get_package_prefix
    from tools.racer_ros2.configuration import configuration
    import yaml
    binary = Path(get_package_prefix('racer_core'))/'lib/racer_core/task_authorization_recovery_probe'
    if not binary.is_file():
        pytest.skip('Opt-in execution-safety native build required')
    hello = dict(starts=[[0,0,1.8]], bounds=[[-3,-4,0],[7,4,4]],
        intrinsics=[216,0,160,0,216,120,0,0,1], flight_limits=[.7,3.],
        motion_limits=dict(speed=1,acceleration=1.2,jerk=4,yaw_rate=.9),
        planning_limits=dict(speed=.3,acceleration=.4,jerk=4,yaw_rate=.325),
        min_time=False,no_path_recovery=True,stop_and_plan=True)
    cfg = configuration(hello,tmp_path,ROOT/'external/racer_ros2_ws/src/racer_core/config/single_drone_planner.xml')
    params = next(c['parameters'] for c in cfg if c['executable']=='exploration_node')
    params['use_sim_time'] = False
    config = tmp_path/'probe.yaml'
    config.write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}))
    result = subprocess.run([str(binary), '--ros-args', '--params-file', str(config)],
                            capture_output=True,text=True,timeout=45)
    (tmp_path/'native-authorization-recovery.log').write_text(result.stdout+'\n'+result.stderr)
    assert result.returncode==0,result.stdout+'\n'+result.stderr
    assert 'PASS_NATIVE_AUTHORIZATION_RECOVERY trigger=3 recovery_attempts=9 real_ros_callback=1 no_flight=1' in result.stdout
    assert 'PASS_NATIVE_SHORT_STOP progress=0.564 duration=5.69 stale_and_wrong_id_rejected=1 native_empty_map_idle=1' in result.stdout

    assert 'PASS_NATIVE_STOPPED_FEEDBACK original_saved_fields=1 plan_state_released=1 no_observation_credit=1 stale_rejected=1 native_empty_map_idle=1 no_flight=1' in result.stdout
    assert 'PASS_NATIVE_RETURN_RETRY empty_trigger_waits=1 no_goal_retries=1 verified_stop_resumes=1 stale_and_wrong_id_rejected=1 loss_isolated=1 no_flight=1' in result.stdout
    assert 'PASS_NATIVE_COLLISION_FEEDBACK same_stamp_isolated=1 exact_release=1 duplicate_hold_only=1 current_denial=1 no_flight=1' in result.stdout
