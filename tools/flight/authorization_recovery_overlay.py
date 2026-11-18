"""Execution-safety build: bounded denial recovery and completed-stop replanning."""
from pathlib import Path
import shutil
from tools.flight.optimizer_overlay import replace_once


def apply(core):
    source = Path(__file__).with_name('authorization_recovery.hpp')
    shutil.copyfile(source, core/'include/exploration_manager/task_authorization_recovery.hpp')
    path = core/'include/exploration_manager/fast_exploration_fsm.h'
    text = path.read_text()
    text = replace_once(text, '#include <exploration_manager/task_recovery_policy.hpp>',
                        '#include <exploration_manager/task_recovery_policy.hpp>\n#include <exploration_manager/task_authorization_recovery.hpp>')
    text = replace_once(text, '  bool task_no_path_recovery_', '''  // Captured from the actual planning call, never from a truncated spline end.
  int task_authorization_proposal_ = -1;
  Eigen::Vector3d task_authorization_goal_ = Eigen::Vector3d::Zero();
  friend struct TaskAuthorizationRecoveryProbe;
  bool task_no_path_recovery_''')
    path.write_text(text)
    path = core/'src/exploration_manager/fast_exploration_fsm.cpp'
    text = path.read_text()
    # A certified early stop can finish before every original trajectory-time
    # threshold. Only fresh, matching execution feedback may release this wait;
    # retain the native frontier update and all proposal authorization gates.
    text = replace_once(text, '''        bool need_replan = false;
        if (t_cur > fp_->replan_thresh2_''', '''        bool need_replan = false;
        if (task_stop_and_plan_ && live_execution_ && executed_id_ >= 0 && executed_done_) {
          ROS_WARN("TASK_EXECUTION completed_stop_replan id=%d progress=%.6f original_duration=%.6f",
              executed_id_, executed_progress_, info->duration_);
          need_replan = true;
        } else if (t_cur > fp_->replan_thresh2_''')
    text = replace_once(text, '  int res;\n  bool review', '''  bool task_planned_alternative = false;
  Eigen::Vector3d task_alternative_goal = Eigen::Vector3d::Zero();
  int res;
  bool review''')
    # This branch's local goal is not reflected by ed_->next_pos_. Record it
    # at the actual call, without changing the planner's goal/state semantics.
    anchor = '      res=expl_manager_->planTrajToView(fd_->start_pt_,fd_->start_vel_,fd_->start_acc_,fd_->start_yaw_,goal,expl_manager_->ed_->yaws_[i]);'
    text = replace_once(text, anchor, '''      task_planned_alternative=true; task_alternative_goal=goal;
'''+anchor)
    text = replace_once(text, '  if (res == SUCCEED) {\n    auto info', '''  if (res == SUCCEED) {
    task_authorization_proposal_=live_counter_+1;
    task_authorization_goal_=task_planned_alternative?task_alternative_goal:expl_manager_->ed_->next_pos_;
    auto info''')
    old = '''    if(task_no_path_recovery_ && task_np_policy_.active() && candidate!=task_review_proposal_ && candidate!=task_last_rejected_)
      task_np_policy_.failed({task_np_goal_.x(),task_np_goal_.y(),task_np_goal_.z()},racer_ros::Time::now().toSec());'''
    text = replace_once(text, old, '''    if(racer_task::authorizationRejected(task_np_policy_,task_no_path_recovery_,candidate,
        task_authorization_proposal_,task_review_proposal_,task_last_rejected_,
        {task_authorization_goal_.x(),task_authorization_goal_.y(),task_authorization_goal_.z()},
        racer_ros::Time::now().toSec()))
      ROS_WARN("TASK_RECOVERY authorization_rejected sim=%.6f id=%d failures=%d active=%d attempts=%d requested_goal=%.6f,%.6f,%.6f",
        racer_ros::Time::now().toSec(),candidate,task_np_policy_.failures(),int(task_np_policy_.active()),task_np_policy_.attempts(),
        task_authorization_goal_.x(),task_authorization_goal_.y(),task_authorization_goal_.z());''')
    path.write_text(text)
