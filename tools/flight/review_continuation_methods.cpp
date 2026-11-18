// Task-owned bounded lifecycle. This file never publishes an actuator command
// or changes RACER, occupancy, clearance, arrival or execution safety limits.
void FastExplorationFSM::taskReviewReport(const std::string& state,const std::string& reason,
    std::int64_t id,const Eigen::Vector3d* override_goal) {
  const auto& r=task_review_session_;
  const Eigen::Vector3d goal=override_goal?*override_goal:Eigen::Vector3d(r.goal[0],r.goal[1],r.goal[2]);
  std::ostringstream s;s<<std::setprecision(17);
  const auto quote=racer_task::reviewJsonString;
  s<<"{\"request_id\":"<<quote(std::to_string(id<0?r.request_id:id))
   <<",\"native_plan_id\":"<<(id<0?r.plan_id:-1)<<",\"state\":"<<quote(state)
   <<",\"goal\":["<<goal.x()<<','<<goal.y()<<','<<goal.z()<<']'
   <<",\"segment_index\":"<<(id<0?r.segment_index:0)<<",\"reason\":"<<quote(reason)
   <<",\"time\":"<<racer_ros::Time::now().toSec()<<",\"path_used_m\":"<<r.path_used
   <<",\"distance_m\":";
  const double distance=fd_->have_odom_?(fd_->odom_pos_-goal).norm():INFINITY;
  if(fd_->have_odom_ && std::isfinite(distance))s<<distance;else s<<"null";
  s<<",\"max_segments\":"<<r.max_segments<<",\"max_duration_s\":"<<r.max_duration_s
   <<",\"max_path_m\":"<<r.max_path_m<<",\"observation_hold_s\":"<<r.observation_hold_s<<'}';
  std_msgs::msg::String message;message.data=s.str();task_review_status_pub_.publish(message);
  ROS_WARN("TASK_REVIEW_STATUS %s",message.data.c_str());
}

void FastExplorationFSM::taskReviewExit(const std::string& state,const std::string& reason) {
  auto& r=task_review_session_;
  r.requestExit(state,reason);task_review_pending_=false;
  // Cancellation/timeout prevents new segments. An already published or
  // authorized segment remains owned by the unchanged certified executor.
  const bool in_flight=r.phase==racer_task::ReviewContinuation::Phase::executing ||
      (r.phase==racer_task::ReviewContinuation::Phase::proposed && state_==PUB_TRAJ &&
       live_sent_ && fd_->newest_traj_.traj_id==r.plan_id);
  if(in_flight) {
    if(!task_review_exit_reported_) {taskReviewReport("cancel_pending",r.exit_reason);task_review_exit_reported_=true;}
    return;
  }
  if(r.active()) {r.finishExit();taskReviewReport(r.exit_state,r.exit_reason);}
  if(state_==PUB_TRAJ && !live_sent_)transitState(PLAN_TRAJ,"unpublished review retired");
}

bool FastExplorationFSM::taskReviewContinuationReady() {
  auto& r=task_review_session_;
  if(!task_review_enabled_ || !task_stop_and_plan_ || !live_execution_ || !r.active() ||
     r.phase!=racer_task::ReviewContinuation::Phase::queued || !r.exit_state.empty())return false;
  const double now=racer_ros::Time::now().toSec();r.bounds(now);
  if(fd_->go_back_)r.requestExit("invalid","return_in_progress");
  const auto map=expl_manager_->sdf_map_;
  if(map->getOccupancy(task_review_goal_)!=SDFMap::FREE || map->getInflateOccupancy(task_review_goal_)!=0)
    r.requestExit("invalid","endpoint_not_currently_free");
  if(!r.exit_state.empty()) {taskReviewExit(r.exit_state,r.exit_reason);return false;}
  // Current native map is read here; complete path/map freshness, dynamics and
  // body clearance are independently required by external authorization.
  const double pose_age=now-task_return_odom_stamp_, feedback_age=now-feedback_stamp_;
  return fd_->have_odom_ && fd_->odom_pos_.allFinite() && pose_age>=0. && pose_age<=.5 &&
      feedback_age>=0. && feedback_age<=.5 && (executed_id_<0 || executed_done_);
}

bool FastExplorationFSM::taskReviewContinuationTick() {
  auto& r=task_review_session_;
  if(!task_review_continuation_ || !r.active())return false;
  const double now=racer_ros::Time::now().toSec();r.bounds(now);
  if(fd_->go_back_)r.requestExit("invalid","return_in_progress");
  const double pose_age=now-task_return_odom_stamp_, feedback_age=now-feedback_stamp_;
  const bool fresh=live_execution_ && fd_->have_odom_ && fd_->odom_pos_.allFinite() &&
      pose_age>=0. && pose_age<=.5 && feedback_age>=0. && feedback_age<=.5;
  if(r.complete(executed_id_,executed_done_,
      {fd_->odom_pos_.x(),fd_->odom_pos_.y(),fd_->odom_pos_.z()},fresh)) {
    taskReviewReport("segment_completed","fresh_matching_executor_completion");
    if(!r.active()) {
      task_review_pending_=false;taskReviewReport(r.exit_state,r.exit_reason);
      if(!r.startObservation(now))return false;
      fd_->static_state_=true;
    } else taskReviewReport("continuing","segment_complete_goal_not_reached");
  }
  if(r.phase==racer_task::ReviewContinuation::Phase::holding) {
    if(!live_execution_ || executed_id_!=r.plan_id || !executed_done_)
      r.requestExit("invalid","observation_execution_changed");
    if(fresh && (fd_->odom_pos_-task_review_goal_).norm()>.25)
      r.requestExit("invalid","observation_actual_pose_departed");
    if(r.waitingObservation(now))return true;
  }
  if(!r.exit_state.empty()) {taskReviewExit(r.exit_state,r.exit_reason);return r.active();}
  if(r.phase==racer_task::ReviewContinuation::Phase::queued) {
    if(!taskReviewContinuationReady())return r.active();
    task_review_pending_=true;fd_->static_state_=true;
    if(state_==IDLE || state_==EXEC_TRAJ)transitState(PLAN_TRAJ,"bounded same-request review continuation");
  }
  return false;
}

void FastExplorationFSM::taskReviewContinuationRequest(const geometry_msgs::PoseStampedConstPtr& msg) {
  const double now=racer_ros::Time::now().toSec();
  const std::int64_t id=static_cast<std::int64_t>(msg->header.stamp.sec)*1000000000LL+msg->header.stamp.nanosec;
  const double stamp=static_cast<double>(msg->header.stamp.sec)+msg->header.stamp.nanosec*1e-9,age=now-stamp;
  const auto q=msg->pose.orientation;
  Eigen::Vector3d goal(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
  const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
  const char* why=!task_review_enabled_ || !task_stop_and_plan_ || !live_execution_?"mode_not_authorized":
      msg->header.frame_id!="map"?"frame_not_map":id<=0 || msg->header.stamp.nanosec>=1000000000U?"invalid_request_id":
      !std::isfinite(age) || age<0. || age>1.?"capture_age_invalid":
      !goal.allFinite() || !std::isfinite(norm) || fabs(norm-1.)>1e-5?"pose_invalid":
      fd_->go_back_?"return_in_progress":nullptr;
  // Nonfinite rejected poses cannot be serialized as valid JSON goals.
  if(why) {if(!goal.allFinite())goal.setZero();taskReviewReport("rejected",why,id,&goal);return;}
  auto& r=task_review_session_;
  if(r.active()) {
    if(id==r.request_id) {taskReviewReport("accepted","duplicate_ignored_budget_unchanged");return;}
    taskReviewReport("rejected","another_review_in_progress",id,&goal);return;
  }
  if(id<=r.last_request_id) {taskReviewReport("rejected","replayed_or_nonmonotonic_request_id",id,&goal);return;}
  const auto map=expl_manager_->sdf_map_;
  if(map->getOccupancy(goal)!=SDFMap::FREE || map->getInflateOccupancy(goal)!=0) {
    taskReviewReport("rejected","endpoint_not_currently_free",id,&goal);return;
  }
  if(!r.begin(id,{goal.x(),goal.y(),goal.z()},now)) {taskReviewReport("rejected","invalid_request_or_budget",id,&goal);return;}
  task_review_goal_=goal;task_review_yaw_=atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
  task_review_stamp_=stamp;task_review_received_=now;task_review_pending_=true;
  task_review_exit_reported_=false;taskReviewReport("accepted","fresh_visual_request_current_free_endpoint");
  if(state_==IDLE && taskReviewContinuationReady())transitState(PLAN_TRAJ,"fresh visual review");
}

void FastExplorationFSM::taskReviewCancel(const std_msgs::msg::String::ConstSharedPtr& msg) {
  if(!task_review_continuation_)return;
  std::int64_t id=0;std::string reason;
  if(!racer_task::parseReviewCancel(msg->data,id,reason)) {ROS_WARN("TASK_REVIEW malformed_cancel_ignored");return;}
  if(!task_review_session_.active() || id!=task_review_session_.request_id) {
    ROS_WARN("TASK_REVIEW unmatched_cancel_ignored request_id=%lld",static_cast<long long>(id));return;
  }
  const bool evidence_complete=reason=="active_multiview_confirmed" || reason=="reliable_direct_confirmed";
  const bool arrived=task_review_session_.phase==racer_task::ReviewContinuation::Phase::holding;
  taskReviewExit(arrived && evidence_complete?"released":"cancelled",reason);
}
