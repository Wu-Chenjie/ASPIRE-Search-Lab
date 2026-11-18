void FastExplorationFSM::taskStaticExecutionFeedback(int active,bool done,int candidate,bool rejected,
                                                   double now,double observation_s){
  if(task_static_c2_.enabled){
    // This runs after the existing callback has applied matching active feedback.
    // Publishing or authorization-pending feedback cannot release a solver slot.
    task_static_monitor_.feedback(active,done,now,observation_s);
    if(task_static_work_&&task_static_gate_.published&&executed_data_&&
       task_static_monitor_.accept(*task_static_work_,task_static_cancel_,active,executed_id_,
          candidate==task_static_gate_.proposal&&rejected)){
      expl_manager_->task_static_events_.finish(task_static_work_->token);
      task_static_work_.reset();expl_manager_->task_static_evaluation_.reset();
      task_static_prior_goal_valid_=false;task_static_cancel_.reset();
      task_static_gate_.resetProposal();task_static_c2_.reset();
      task_static_monitor_.feedback(active,done,now,observation_s);
    }
    expl_manager_->task_static_events_.setExecutingSources(task_static_monitor_.reservedSources());
  }
}
