bool FastExplorationFSM::taskStaticEventTick(){
 if(!task_static_replan_)return false;
 auto& manager=*expl_manager_;const double now=racer_ros::Time::now().toSec();
 if(task_static_work_){
  if(task_static_gate_.published)return false; // executor owns an already submitted proposal
  if(!task_static_budget_.allows(now,0.)){
   taskStaticRelease(false,task_static_budget_.reason(now,0.));
   return true;
  }
  if(!manager.task_static_events_.valid(task_static_work_->token)){
   taskStaticRelease(false,"event_invalid");return true;
  }
  if(fd_->go_back_||fd_->avoid_collision_||task_review_pending_||task_review_session_.active()||task_np_policy_.active()){
   taskStaticRelease(true,"competing_mission_deferred_event");return true;
  }
  return false;
 }
 if(!task_static_budget_.allows(now,0.))return false;
 // Frontier refresh can enter PLAN_TRAJ before the accepted curve finishes.
 // Its stop-and-plan wait is still an executing vehicle, so admit one static
 // evaluation there. Publication/C2 authorization and competing missions keep
 // their existing gates; never overwrite a PUB_TRAJ proposal or a stopped solve.
 const bool waiting_execution=state_==PLAN_TRAJ&&task_stop_and_plan_&&
   executed_data_&&executed_id_>=0&&!executed_done_;
 if(state_!=EXEC_TRAJ&&state_!=IDLE&&!waiting_execution)return false;
 if(state_==EXEC_TRAJ&&(!executed_data_||executed_id_<0))return false;
 const bool competing=fd_->go_back_||fd_->avoid_collision_||task_review_pending_||
   task_review_session_.active()||task_np_policy_.active();
 const double odom_age=now-task_return_odom_stamp_;
 if(!std::isfinite(odom_age)||odom_age<0.||odom_age>.5||!fd_->odom_pos_.allFinite()||
    !task_static_gate_.ready(now,feedback_stamp_,competing))return false;
 task_static_work_=manager.task_static_events_.start(now);if(!task_static_work_)return false;
 task_static_gate_.started(now);manager.task_static_evaluation_=task_static_work_;
 task_static_prior_goal_valid_=false;
 if(state_==IDLE){
  // No current exploration goal is guaranteed in IDLE. Let the native planner
  // establish its baseline instead of reading an uninitialized/stale ED goal.
  transitState(PLAN_TRAJ,"static event wakes idle planner");return true;
 }
 // Bounded early cost evaluation uses the native candidate/path checks now,
 // while the original executing trajectory and all authorization gates remain.
 Eigen::Vector3d goal=manager.ed_->next_pos_;double goal_yaw=manager.ed_->next_yaw_;
 task_static_prior_goal_=goal;task_static_prior_yaw_=goal_yaw;task_static_prior_goal_valid_=true;
 const auto policy=manager.task_bearing_policy_;
 const auto pending=manager.task_bearing_pending_,changed=manager.task_bearing_changed_;
 const auto target=manager.task_bearing_target_;const int cue=manager.task_bearing_selected_cue_;
 auto restore=[&](){manager.task_bearing_policy_=policy;manager.task_bearing_preview_=false;
  manager.task_bearing_pending_=pending;manager.task_bearing_changed_=changed;
  manager.task_bearing_target_=target;manager.task_bearing_selected_cue_=cue;};
 bool useful=false;manager.task_bearing_preview_=true;
 try{useful=manager.taskBearingRerank(fd_->odom_pos_,fd_->odom_vel_,
      Eigen::Vector3d(fd_->odom_yaw_,0.,0.),goal,goal_yaw);}
 catch(...){restore();taskStaticRelease(false,"preview_exception");throw;}
 restore();
 ROS_INFO("STATIC_EVENT preview token=%llu useful=%d stop_and_plan=%d",
   static_cast<unsigned long long>(task_static_work_->token),int(useful),int(task_stop_and_plan_));
 if(!useful){taskStaticRelease(false,"native_preview_prefers_exploration");return false;}
 // This is a replan request, not permission to publish. Existing PLAN_TRAJ
 // stop-and-plan guard and the external executor stay authoritative.
 transitState(PLAN_TRAJ,"static event reevaluated");return true;
}

void FastExplorationFSM::taskStaticRelease(bool retry,const char* reason){
 if(!task_static_work_||task_static_gate_.published)return;
 const auto token=task_static_work_->token;
 if(retry)expl_manager_->task_static_events_.retry(token);
 else expl_manager_->task_static_events_.finish(token);
 ROS_INFO("STATIC_EVENT release token=%llu retry=%d reason=%s",
   static_cast<unsigned long long>(token),int(retry),reason);
 task_static_work_.reset();expl_manager_->task_static_evaluation_.reset();
 task_static_gate_.resetProposal();
 // Retire a cancelled unpublished goal as well as its spline. Higher-priority
 // missions may already own ED; never restore an exploration goal over them.
 if(task_static_prior_goal_valid_&&!fd_->go_back_&&!fd_->avoid_collision_&&
    !task_review_pending_&&!task_review_session_.active()&&!task_np_policy_.active()){
  expl_manager_->ed_->next_pos_=task_static_prior_goal_;
  expl_manager_->ed_->next_yaw_=task_static_prior_yaw_;
 }
 task_static_prior_goal_valid_=false;
 // Restore the executor-owned trajectory only for unpublished proposals.
 if(executed_data_){planner_manager_->local_data_=*executed_data_;fd_->newest_traj_=executed_spline_;}
 if(state_==PLAN_TRAJ||state_==PUB_TRAJ){
  fd_->static_state_=executed_id_<0||executed_done_;
  transitState(fd_->static_state_?PLAN_TRAJ:EXEC_TRAJ,"static event proposal retired");
 }
}
