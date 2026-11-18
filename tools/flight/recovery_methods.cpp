// Inserted into the original native FSM build, never a replacement planner.
bool FastExplorationFSM::taskRecoveryReady() {
  // Bypass recovery scheduling only for a fresh measured-map review. Native
  // planning and external authorization still apply.
  if(taskReviewReady())return true;
  const double now=racer_ros::Time::now().toSec();
  if(!task_np_policy_.ready(now))return false;
  // Refresh frontiers while holding in PLAN_TRAJ; upstream normally refreshes
  // only on trigger or after an executed trajectory.
  expl_manager_->updateFrontierStruct(fd_->odom_pos_);
  return true;
}

int FastExplorationFSM::taskRecoveryPlan() {
  const double now=racer_ros::Time::now().toSec();
  auto map=expl_manager_->sdf_map_;auto ed=expl_manager_->ed_;
  const racer_task::Point root={fd_->start_pt_.x(),fd_->start_pt_.y(),fd_->start_pt_.z()};
  auto points=ed->points_;auto yaws=ed->yaws_;
  // Last observed, previously authorized roots are optional native-planned
  // retreat goals. Their entire path is rechecked against the current map.
  for(auto it=task_np_breadcrumbs_.rbegin();it!=task_np_breadcrumbs_.rend();++it) {
    points.push_back(*it);yaws.push_back(fd_->start_yaw_[0]);
  }
  std::vector<racer_task::Point> candidates;
  for(const auto& p:points)candidates.push_back({p.x(),p.y(),p.z()});
  const int selected=racer_task::selectGoal(task_np_policy_,candidates,yaws,root,now,
    [&](const racer_task::Point& v) {
      const Eigen::Vector3d p(v[0],v[1],v[2]);
      return map->getOccupancy(p)==SDFMap::FREE && map->getInflateOccupancy(p)==0;
    });
  if(selected<0) {
    task_np_policy_.hold(now);
    ROS_WARN("TASK_RECOVERY hold sim=%.6f reason=no_eligible_measured_goal attempts=%d root_occ=%d root_inflate=%d",
      now,task_np_policy_.attempts(),map->getOccupancy(fd_->start_pt_),map->getInflateOccupancy(fd_->start_pt_));
    return NO_FRONTIER; // Keep holding; no proposal, no execution cancellation.
  }
  task_np_goal_=points[selected];
  task_np_policy_.attempted({task_np_goal_.x(),task_np_goal_.y(),task_np_goal_.z()},now);
  ed->next_pos_=task_np_goal_;ed->next_yaw_=yaws[selected];
  ROS_WARN("TASK_RECOVERY attempt sim=%.6f index=%d attempt=%d kind=%s goal=%.6f,%.6f,%.6f",
    now,selected,task_np_policy_.attempts(),selected<int(ed->points_.size())?"frontier":"retreat",
    task_np_goal_.x(),task_np_goal_.y(),task_np_goal_.z());
  return expl_manager_->planTrajToView(fd_->start_pt_,fd_->start_vel_,fd_->start_acc_,fd_->start_yaw_,task_np_goal_,ed->next_yaw_);
}
