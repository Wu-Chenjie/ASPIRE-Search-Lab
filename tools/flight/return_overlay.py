"""Return mission fix on the generated RACER copy; original snapshot untouched."""
from pathlib import Path
import shutil
from tools.flight.optimizer_overlay import replace_once


def apply(core):
    shutil.copyfile(Path(__file__).with_name('return_policy.hpp'), core/'include/exploration_manager/task_return_policy.hpp')
    p=core/'include/exploration_manager/fast_exploration_fsm.h';s=p.read_text()
    s=replace_once(s,'#include <exploration_manager/task_recovery_policy.hpp>', '#include <exploration_manager/task_recovery_policy.hpp>\n#include <exploration_manager/task_return_policy.hpp>')
    s=replace_once(s,'  bool task_no_path_recovery_', '''  racer_task::ReturnPolicy task_return_;
  double task_return_odom_stamp_=-1., task_return_log_stamp_=-1.;
  double task_return_execution_stamp_=-1.;
  double task_idle_frontier_retry_=-1.;
  bool taskReturnTick();
  void taskReturnLog(const char* reason);
  bool task_no_path_recovery_''');p.write_text(s)
    p=core/'src/exploration_manager/fast_exploration_fsm.cpp';s=p.read_text()
    s=replace_once(s,'  switch (state_) {','  if(taskReturnTick())return;\n  switch (state_) {')
    # Keep the original idle-to-home timer, but allow delayed observations to
    # produce a first or subsequent frontier in the usual live entry point.
    s=replace_once(s,'      if (check_interval > 100.0) {','''      const double now=racer_ros::Time::now().toSec();
      if(live_execution_ && check_interval<=100. && now>=task_idle_frontier_retry_) {
        task_idle_frontier_retry_=now+5.;
        if(expl_manager_->updateFrontierStruct(fd_->odom_pos_)!=0) {
          transitState(PLAN_TRAJ,"measured frontiers available");
          break;
        }
      }
      if (check_interval > 100.0) {''')
    s=replace_once(s,'''  } else
    transitState(FINISH, "triggerCallback");''','''  } else if(live_execution_) {
    fd_->last_check_frontier_time_=racer_ros::Time::now();
    task_idle_frontier_retry_=fd_->last_check_frontier_time_.toSec()+5.;
    transitState(IDLE,"waiting for measured frontiers");
  } else
    transitState(FINISH, "triggerCallback");''')
    s=replace_once(s,'''      if (res == SUCCEED) {
        transitState(PUB_TRAJ, "FSM");''','''      if (res == SUCCEED) {
        task_idle_frontier_retry_=-1.;
        transitState(PUB_TRAJ, "FSM");''')
    s=replace_once(s,'''      } else if (res == NO_GRID) {
        fd_->static_state_ = true;
        fd_->last_check_frontier_time_ = racer_ros::Time::now();''','''      } else if(task_return_.started() && (res==NO_GRID || res==NO_FRONTIER)) {
        // Return remains in PLAN_TRAJ until its retry is due.
        fd_->static_state_=true;
      } else if (res == NO_GRID) {
        fd_->static_state_ = true;
        if(task_idle_frontier_retry_<0. || !live_execution_)
          fd_->last_check_frontier_time_=racer_ros::Time::now();
        task_idle_frontier_retry_=racer_ros::Time::now().toSec()+5.;''')
    s=replace_once(s,'''            fd_->last_check_frontier_time_ = racer_ros::Time::now();
            transitState(IDLE, "FSM");''','''            fd_->last_check_frontier_time_ = racer_ros::Time::now();
            task_idle_frontier_retry_=fd_->last_check_frontier_time_.toSec()+5.;
            transitState(IDLE, "FSM");''')
    s=replace_once(s,'        fd_->go_back_ = true;', '''        task_return_.begin({fd_->start_pos_.x(),fd_->start_pos_.y(),fd_->start_pos_.z()});
        taskReturnLog("no_frontier_return_started");
        fd_->go_back_ = true;''')
    s=replace_once(s,'      if(task_no_path_recovery_ && task_np_policy_.active() && !taskRecoveryReady()) return;', '''      if(task_return_.started() && (!task_return_.mayPlan(racer_ros::Time::now().toSec()) ||
          (live_execution_ && executed_id_>=0 && !executed_done_))) return;
      if((!task_return_.started() || task_return_.mayUseTemporary()) &&
          task_no_path_recovery_ && task_np_policy_.active() && !taskRecoveryReady()) return;''')
    start=s.index('        // Check if reach goal\n');end=s.index('\n      }\n\n      break;',start)
    s=s[:start]+'''        // The authorized executor owns progress and observation completion.
        // Never finish from a planned point near a mutable recovery goal.
        taskReturnTick();'''+s[end:]
    s=replace_once(s,'  int res;\n  bool review', '''  bool recovery=task_no_path_recovery_ && task_np_policy_.active();
  if(task_return_.started()) {
    recovery=recovery && task_return_.mayUseTemporary();
    if(!task_return_.attempt(racer_ros::Time::now().toSec(),recovery))return NO_FRONTIER;
    if(!recovery) {
      const auto& h=task_return_.home();
      expl_manager_->ed_->next_pos_=Eigen::Vector3d(h[0],h[1],h[2]);
      expl_manager_->ed_->next_yaw_=0.;
    }
    taskReturnLog("planning_attempt");
  }
  int res;
  bool review''')
    s=replace_once(s,'  if(!review && task_no_path_recovery_ && task_np_policy_.active()) {',
                   '  if(!review && recovery) {')
    s=replace_once(s,'const auto goal=task_np_policy_.active()?task_np_goal_:',
                   'const auto goal=recovery?task_np_goal_:')
    s=replace_once(s,'if(task_no_path_recovery_ && !review && task_np_policy_.active() && res==SUCCEED)',
                   'if(!review && recovery && res==SUCCEED)')
    s=s.replace('if(task_recover_ && !task_np_policy_.active()', 'if(task_recover_ && !task_return_.started() && !task_np_policy_.active()')
    s=replace_once(s,'  if (res == SUCCEED) {\n    auto info', '''  if(task_return_.started()) {
    if(res==SUCCEED)task_return_.proposed(live_counter_+1);
    else if(res==NO_FRONTIER || res==NO_GRID)task_return_.noGoal(racer_ros::Time::now().toSec());
    else task_return_.failed(racer_ros::Time::now().toSec());
    taskReturnLog(res==SUCCEED?"proposal":res==FAIL?"native_failure":"no_goal_hold");
  }
  if (res == SUCCEED) {\n    auto info''')
    s=replace_once(s,'    task_rejections_=0;', '''    if(task_return_.activated(active))taskReturnLog("authorized_activation");
    task_rejections_=0;''')
    s=replace_once(s,'  if (!live_execution_ || msg->data.size() != 7) return;',
                   '  if (!live_execution_ || (msg->data.size() != 7 && msg->data.size() != 8)) return;')
    s=replace_once(s,'  feedback_stamp_ = d[5];', '''  if(!std::isfinite(d[5]))return;
  const bool observation_stop=d.size()==8;
  // The extension identifies a stopped, dual-map-verified R26 release.
  // A delayed receipt must not de-own a different running execution.
  if(observation_stop && (d[0]!=-1. || d[2]!=0. || !(d[7]>=0.) ||
       d[7]>std::numeric_limits<int>::max() || std::floor(d[7])!=d[7] ||
       (executed_id_>=0 && d[7]!=executed_id_)))return;
  feedback_stamp_ = d[5];''')
    s=replace_once(s,'  const int active = int(d[0]), candidate = int(d[3]);', '''  const int active = int(d[0]), candidate = int(d[3]);
  bool verified_stop=observation_stop;''')
    s=replace_once(s,'  if (active >= 0 && active != executed_id_ && active == fd_->newest_traj_.traj_id) {','''  if(task_return_.terminal())return;
  if(task_return_.started() && active<0 && executed_id_>=0) {
    if(verified_stop) {
      if(task_return_.stopped(executed_id_,d[5]))taskReturnLog("verified_stop_resume_original_home");
    } else if(task_return_.executing() || !executed_done_) {
      if(task_return_.stopped(executed_id_,d[5]))taskReturnLog("execution_lost_retry_home");
    }
  }
  if (active >= 0 && active != executed_id_ && active == fd_->newest_traj_.traj_id &&
      (!task_return_.started() || task_return_.awaiting(active))) {''')
    s=replace_once(s, '    executed_done_ = d[2] > 0.;',
                   '    executed_done_ = d[2] > 0.;\n    task_return_execution_stamp_=d[5];')
    s=replace_once(s,'  if (candidate == fd_->newest_traj_.traj_id && d[4] < 0.) {', '''  if (candidate == fd_->newest_traj_.traj_id && d[4] < 0.) {
    if(task_return_.rejected(candidate,d[5]))taskReturnLog("authorization_rejected");''')
    s=replace_once(s,'void FastExplorationFSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {', '''void FastExplorationFSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
  task_return_odom_stamp_=rclcpp::Time(msg->header.stamp).seconds();''')
    s=replace_once(s,'void FastExplorationFSM::taskReview(', Path(__file__).with_name('return_methods.cpp').read_text()+'\nvoid FastExplorationFSM::taskReview(')
    p.write_text(s)
