"""Opt-in bounded native planning-failure recovery, on the build copy only."""
from pathlib import Path
import shutil

from .optimizer_overlay import replace_once


def apply(core):
    shutil.copyfile(Path(__file__).with_name('recovery_policy.hpp'),core/'include/exploration_manager/task_recovery_policy.hpp')
    p=core/'include/path_searching/astar2.h';s=p.read_text().replace('  double lambda_heu_;','  int task_failure_reason_ = 0; // 1: timeout, 2: node pool, 3: exhausted\n  double lambda_heu_;');p.write_text(s)
    p=core/'src/path_searching/astar2.cpp';s=p.read_text();s=s.replace('  NodePtr cur_node = path_node_pool_[0];','  task_failure_reason_=0;\n  NodePtr cur_node = path_node_pool_[0];',1);s=s.replace('      return NO_PATH;','      task_failure_reason_=1; return NO_PATH;',1);s=s.replace('              return NO_PATH;','              task_failure_reason_=2; return NO_PATH;',1);s=s.replace('  return NO_PATH;\n}', '  task_failure_reason_=3; return NO_PATH;\n}',1);p.write_text(s)
    p=core/'include/exploration_manager/fast_exploration_fsm.h';s=p.read_text().replace('#include <Eigen/Eigen>','#include <Eigen/Eigen>\n#include <exploration_manager/task_recovery_policy.hpp>');anchor='  bool task_stop_and_plan_'
    s=replace_once(s,anchor,'''  bool task_no_path_recovery_ = false;
  racer_task::RecoveryPolicy task_np_policy_;
  std::vector<Eigen::Vector3d> task_np_breadcrumbs_;
  Eigen::Vector3d task_np_goal_ = Eigen::Vector3d::Zero();
  bool taskRecoveryReady();
  int taskRecoveryPlan();
'''+anchor);p.write_text(s)
    p=core/'src/exploration_manager/fast_exploration_fsm.cpp';s=p.read_text();s=s.replace('  nh.param("search/stop_and_plan",','  nh.param("search/no_path_recovery", task_no_path_recovery_, false);\n  nh.param("search/stop_and_plan",',1)
    anchor='      if(task_stop_and_plan_ && live_execution_ && executed_id_>=0 && !executed_done_) return;'
    s=replace_once(s,anchor,anchor+'\n      if(task_no_path_recovery_ && task_np_policy_.active() && !taskRecoveryReady()) return;')
    s=s.replace('  if(review) {','  if(!review && task_no_path_recovery_ && task_np_policy_.active()) {\n    res=taskRecoveryPlan();\n  } else if(review) {',1)
    s=s.replace('if(task_recover_ && !review && fd_->static_state_', 'if(task_recover_ && !task_np_policy_.active() && !review && fd_->static_state_',1)
    anchor='  if (res == SUCCEED) {\n    auto info = &planner_manager_->local_data_;'
    s=replace_once(s,anchor,'''  if(task_no_path_recovery_ && !review && fd_->static_state_ && res==FAIL) {
    const auto goal=task_np_policy_.active()?task_np_goal_:expl_manager_->ed_->next_pos_;
    task_np_policy_.failed({goal.x(),goal.y(),goal.z()},racer_ros::Time::now().toSec());
    auto map=expl_manager_->sdf_map_;
    ROS_WARN("TASK_RECOVERY failure sim=%.6f count=%d active=%d astar=%d root=%.6f,%.6f,%.6f root_occ=%d root_inflate=%d goal=%.6f,%.6f,%.6f goal_occ=%d goal_inflate=%d",
      racer_ros::Time::now().toSec(),task_np_policy_.failures(),int(task_np_policy_.active()),planner_manager_->path_finder_->task_failure_reason_,
      fd_->start_pt_.x(),fd_->start_pt_.y(),fd_->start_pt_.z(),map->getOccupancy(fd_->start_pt_),map->getInflateOccupancy(fd_->start_pt_),
      goal.x(),goal.y(),goal.z(),map->getOccupancy(goal),map->getInflateOccupancy(goal));
  }
  if(task_no_path_recovery_ && !review && task_np_policy_.active() && res==SUCCEED)
    ROS_WARN("TASK_RECOVERY proposal sim=%.6f id=%d attempts=%d",racer_ros::Time::now().toSec(),live_counter_+1,task_np_policy_.attempts());
'''+anchor)
    anchor='    task_rejections_=0;'
    s=replace_once(s,anchor,'''    if(task_no_path_recovery_) {
      if(task_np_policy_.active() && active!=task_review_proposal_)ROS_WARN("TASK_RECOVERY activated sim=%.6f id=%d",racer_ros::Time::now().toSec(),active);
      auto map=expl_manager_->sdf_map_;
      if(map->getOccupancy(fd_->start_pt_)==SDFMap::FREE && map->getInflateOccupancy(fd_->start_pt_)==0 &&
         (task_np_breadcrumbs_.empty() || (task_np_breadcrumbs_.back()-fd_->start_pt_).norm()>.5)) {
        task_np_breadcrumbs_.push_back(fd_->start_pt_);if(task_np_breadcrumbs_.size()>12)task_np_breadcrumbs_.erase(task_np_breadcrumbs_.begin());
      }
      task_np_policy_.activated();
    }
'''+anchor)
    anchor='    if(candidate!=task_last_rejected_){++task_rejections_;task_last_rejected_=candidate;}'
    s=replace_once(s,anchor,'''    if(task_no_path_recovery_ && task_np_policy_.active() && candidate!=task_review_proposal_ && candidate!=task_last_rejected_)
      task_np_policy_.failed({task_np_goal_.x(),task_np_goal_.y(),task_np_goal_.z()},racer_ros::Time::now().toSec());
'''+anchor)
    anchor='void FastExplorationFSM::taskReview('
    methods=Path(__file__).with_name('recovery_methods.cpp').read_text()
    s=replace_once(s,anchor,methods+'\n'+anchor);p.write_text(s)
