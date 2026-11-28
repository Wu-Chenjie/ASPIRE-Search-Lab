// Build overlay: immutable return destination and actual completion.
void FastExplorationFSM::taskReturnLog(const char* reason) {
  const auto& h=task_return_.home();
  ROS_WARN("TASK_RETURN sim=%.6f state=%s reason=%s attempts=%d home=%.6f,%.6f,%.6f goal=%.6f,%.6f,%.6f actual=%.6f,%.6f,%.6f active=%d done=%d segment=%d",
    racer_ros::Time::now().toSec(),task_return_.name(),reason,task_return_.attempts(),h[0],h[1],h[2],
    expl_manager_->ed_->next_pos_.x(),expl_manager_->ed_->next_pos_.y(),expl_manager_->ed_->next_pos_.z(),
    fd_->odom_pos_.x(),fd_->odom_pos_.y(),fd_->odom_pos_.z(),executed_id_,int(executed_done_),task_return_.active());
}

bool FastExplorationFSM::taskReturnTick() {
  const double now=racer_ros::Time::now().toSec();
  // Only change missions at a stopped planning boundary. A proposal awaiting
  // authorization or an aircraft still executing a segment keeps ownership.
  if(state_==PLAN_TRAJ && live_execution_ && task_stop_and_plan_ && task_no_path_recovery_ &&
      (executed_id_<0 || executed_done_)) {
    if(!task_return_.started() && task_np_policy_.stalled(now)) {
      const auto& home=fd_->start_pos_;
      task_return_.begin({home.x(),home.y(),home.z()},true);
      fd_->go_back_=true;
      taskReturnLog("stalled_search_return_started");
    } else if(task_return_.blockIfStalled(now)) {
      taskReturnLog("no_authorized_return_route");
      task_return_blocked_pub_.publish(std_msgs::Empty());
      transitState(FINISH,"return blocked");
      return true;
    }
  }
  if(!task_return_.started())return false;
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
