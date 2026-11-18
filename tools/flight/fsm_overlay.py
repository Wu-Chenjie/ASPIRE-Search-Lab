"""Opt-in task overlays on a fresh verified RACER build; never edit the snapshot."""
from .optimizer_overlay import replace_once


def apply(core):
    header=core/'include/exploration_manager/fast_exploration_fsm.h'
    text=header.read_text();anchor='  bool live_execution_ = false, live_sent_ = false;'
    text=replace_once(text,anchor,'''  bool task_stop_and_plan_ = false, task_review_enabled_ = false, task_recover_ = false;
  bool task_review_pending_ = false;
  int task_rejections_ = 0, task_last_rejected_ = -1, task_review_proposal_ = -1;
  Eigen::Vector3d task_review_goal_;
  double task_review_yaw_ = 0., task_review_stamp_ = -1., task_review_received_ = -1.;
  racer_ros::Subscriber task_review_sub_;
  bool taskReviewReady();
  void taskReview(const geometry_msgs::PoseStampedConstPtr& msg);
'''+anchor);header.write_text(text)
    path=core/'src/exploration_manager/fast_exploration_fsm.cpp';text=path.read_text()
    anchor='  fp_.reset(new FSMParam);'
    text=replace_once(text,anchor,'''  nh.param("search/stop_and_plan", task_stop_and_plan_, false);
  nh.param("search/active_review", task_review_enabled_, false);
  nh.param("search/recover_frontiers", task_recover_, false);
  if(task_review_enabled_) task_review_sub_=nh.subscribe("/search/review_goal", 2, &FastExplorationFSM::taskReview, this);
'''+anchor)
    anchor='    case PLAN_TRAJ: {'
    text=replace_once(text,anchor,anchor+'''
      // Real dynamics remain active. Wait for sensor-certified arrival before
      // the next native proposal; no unsupported moving C2 handoff is authorized.
      if(task_stop_and_plan_ && live_execution_ && executed_id_>=0 && !executed_done_) return;
''')
    anchor='  int res;\n  if (fd_->avoid_collision_'
    text=replace_once(text,anchor,'''  int res;
  bool review = fd_->static_state_ && taskReviewReady();
  task_review_pending_=false;
  if(review) {
    expl_manager_->ed_->next_pos_=task_review_goal_;
    expl_manager_->ed_->next_yaw_=task_review_yaw_;
    res=expl_manager_->planTrajToView(fd_->start_pt_,fd_->start_vel_,fd_->start_acc_,fd_->start_yaw_,task_review_goal_,task_review_yaw_);
    ROS_WARN("Task review planned using native RACER; external authorization still required id=%d status=%d request_stamp=%.9f goal=%.6f,%.6f,%.6f",live_counter_+1,res,task_review_stamp_,task_review_goal_[0],task_review_goal_[1],task_review_goal_[2]);
  } else if (fd_->avoid_collision_''')
    anchor='  if (res == SUCCEED) {\n    auto info = &planner_manager_->local_data_;'
    text=replace_once(text,anchor,'''  if(review && res==SUCCEED)task_review_proposal_=live_counter_+1;
  if(task_recover_ && !review && fd_->static_state_ && task_rejections_>0 && !expl_manager_->ed_->points_.empty()) {
    const size_t i=size_t(task_rejections_-1)%expl_manager_->ed_->points_.size();
    const auto goal=expl_manager_->ed_->points_[i];
    if((goal-fd_->start_pt_).norm()>.3 && expl_manager_->sdf_map_->getOccupancy(goal)==SDFMap::FREE && expl_manager_->sdf_map_->getInflateOccupancy(goal)==0) {
      res=expl_manager_->planTrajToView(fd_->start_pt_,fd_->start_vel_,fd_->start_acc_,fd_->start_yaw_,goal,expl_manager_->ed_->yaws_[i]);
      ROS_WARN("Trying alternative measured-map native frontier after execution rejection");
    }
  }
'''+anchor)
    anchor='    executed_id_ = active;'
    text=replace_once(text,anchor,'    if(active==task_review_proposal_)ROS_WARN("TASK_REVIEW authorized id=%d",active);\n    task_rejections_=0;\n'+anchor)
    anchor='  if (candidate == fd_->newest_traj_.traj_id && d[4] < 0.) {'
    text=replace_once(text,anchor,anchor+'\n    if(candidate!=task_last_rejected_){++task_rejections_;task_last_rejected_=candidate;}')
    anchor='void FastExplorationFSM::executionFeedback('
    text=replace_once(text,anchor,'''bool FastExplorationFSM::taskReviewReady() {
  if(!task_review_enabled_ || !task_review_pending_)return false;
  const double now=racer_ros::Time::now().toSec(), queued_age=now-task_review_received_;
  auto map=expl_manager_->sdf_map_;
  const int occupancy=map->getOccupancy(task_review_goal_), inflated=map->getInflateOccupancy(task_review_goal_);
  const bool ready=racer_task::reviewReady(true,true,fd_->go_back_,now,task_review_received_,
      [&](){return occupancy==SDFMap::FREE && inflated==0;});
  if(!ready) {
    const char* reason=fd_->go_back_?"return_in_progress":
        !std::isfinite(queued_age)||queued_age<0.?"invalid_queue_clock":
        queued_age>=20.?"queue_expired":"endpoint_not_currently_free";
    ROS_WARN("TASK_REVIEW dropped sim=%.6f request_stamp=%.9f reason=%s queued_age=%.6f occupancy=%d inflated=%d",
        now,task_review_stamp_,reason,queued_age,occupancy,inflated);
    task_review_pending_=false;
  }
  return ready;
}

void FastExplorationFSM::taskReview(const geometry_msgs::PoseStampedConstPtr& msg) {
  const double now=racer_ros::Time::now().toSec(), stamp=rclcpp::Time(msg->header.stamp).seconds(), age=now-stamp;
  Eigen::Vector3d p(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
  const auto q=msg->pose.orientation;
  const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
  const char* reason=!task_review_enabled_?"disabled":msg->header.frame_id!="map"?"frame_not_map":
      !std::isfinite(age)||age<0.||age>1.?"capture_age_invalid":
      !p.allFinite()||!std::isfinite(norm)||fabs(norm-1.)>1e-5?"pose_invalid":
      fd_->go_back_?"return_in_progress":nullptr;
  if(reason){ROS_WARN("TASK_REVIEW rejected sim=%.6f request_stamp=%.9f reason=%s age=%.6f",now,stamp,reason,age);return;}
  const int occupancy=expl_manager_->sdf_map_->getOccupancy(p), inflated=expl_manager_->sdf_map_->getInflateOccupancy(p);
  if(occupancy!=SDFMap::FREE || inflated!=0) {
    ROS_WARN("TASK_REVIEW rejected sim=%.6f request_stamp=%.9f reason=endpoint_not_free occupancy=%d inflated=%d goal=%.6f,%.6f,%.6f",
        now,stamp,occupancy,inflated,p.x(),p.y(),p.z());return;
  }
  if(task_review_pending_)ROS_WARN("TASK_REVIEW replaced request_stamp=%.9f by=%.9f",task_review_stamp_,stamp);
  task_review_goal_=p;task_review_yaw_=atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
  task_review_stamp_=stamp;task_review_received_=now;task_review_pending_=true;
  ROS_WARN("TASK_REVIEW received sim=%.6f request_stamp=%.9f max_queue_s=20 state=%d goal=%.6f,%.6f,%.6f",
      now,stamp,int(state_),p.x(),p.y(),p.z());
  // Wake an idle planner; moving executions still finish before this request.
  if(state_==IDLE && taskReviewReady() && (!live_execution_ || executed_id_<0 || executed_done_))
    transitState(PLAN_TRAJ,"fresh visual review");
}

'''+anchor)
    path.write_text(text)
