// Build overlay: immutable return destination and actual completion.
void FastExplorationFSM::taskReturnLog(const char* reason) {
  const auto& h=task_return_.home();
  ROS_WARN("TASK_RETURN sim=%.6f state=%s reason=%s attempts=%d home=%.6f,%.6f,%.6f goal=%.6f,%.6f,%.6f actual=%.6f,%.6f,%.6f active=%d done=%d segment=%d",
    racer_ros::Time::now().toSec(),task_return_.name(),reason,task_return_.attempts(),h[0],h[1],h[2],
    expl_manager_->ed_->next_pos_.x(),expl_manager_->ed_->next_pos_.y(),expl_manager_->ed_->next_pos_.z(),
    fd_->odom_pos_.x(),fd_->odom_pos_.y(),fd_->odom_pos_.z(),executed_id_,int(executed_done_),task_return_.active());
}

bool FastExplorationFSM::taskReturnTick() {
  if(!task_return_.started())return false;
  const double now=racer_ros::Time::now().toSec();
  const double pose_age=now-task_return_odom_stamp_;
  const double execution_age=now-task_return_execution_stamp_;
  const bool fresh=live_execution_ && fd_->have_odom_ && pose_age>=0. && pose_age<=.5 &&
      execution_age>=0. && execution_age<=.5;
  if(task_return_.observe(executed_id_,executed_done_,
      {fd_->odom_pos_.x(),fd_->odom_pos_.y(),fd_->odom_pos_.z()},fresh)) {
    if(task_return_.state()==racer_task::ReturnPolicy::State::returned) {
      taskReturnLog("home_pose_and_execution_complete");
      transitState(FINISH,"verified return");
    } else {
      const auto& h=task_return_.home();
      expl_manager_->ed_->next_pos_=Eigen::Vector3d(h[0],h[1],h[2]);
      expl_manager_->ed_->next_yaw_=0.;fd_->static_state_=true;
      taskReturnLog("segment_complete_resume_original_home");
      transitState(PLAN_TRAJ,"return continues");
    }
  }
  if(task_return_.terminal()) {
    if(now-task_return_log_stamp_>=5.){taskReturnLog("terminal_hold");task_return_log_stamp_=now;}
    return true;
  }
  return false;
}
