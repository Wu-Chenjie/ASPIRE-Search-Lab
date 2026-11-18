"""Revalidate paired same-source observations around an early stopped solve."""
from .optimizer_overlay import replace_once


def apply(core):
    include = core/'include/exploration_manager'
    p = include/'fast_exploration_fsm.h'
    s = replace_once(p.read_text(), '  bool task_static_replan_=false;', '''  bool task_fresh_review_=false, task_stop_for_review_=false;
  bool task_review_source_retention_=false, task_review_scheduling_priority_=false;
  double task_review_stop_heading_=0.;
  const racer_task::TargetObservation* taskFreshReviewFrame(double now,double minimum_capture=-1.) const;
  const racer_task::TargetObservation* taskReviewNavigationFrame(const racer_task::TargetObservation& paired,double now) const;
  const racer_task::TargetRecord* taskReviewSource() const;
  void taskRequestReviewStop(double now);
  bool task_static_replan_=false;''')
    p.write_text(s)

    p = core/'src/exploration_manager/fast_exploration_fsm.cpp'
    s = '#include <exploration_manager/fresh_review.hpp>\n'+p.read_text()
    s = replace_once(s, '  if(task_collision_stop_enabled_)\n    task_collision_stop_pub_', '''  const char* fresh_review=std::getenv("ASPIRE_STATIC_FRESH_SIGNING");
  task_fresh_review_=task_collision_stop_enabled_&&fresh_review&&std::string(fresh_review)=="1";
  const char* retention=std::getenv("ASPIRE_REVIEW_SOURCE_RETENTION");
  task_review_source_retention_=task_fresh_review_&&retention&&std::string(retention)=="1";
  const char* priority=std::getenv("ASPIRE_REVIEW_SCHEDULING_PRIORITY");
  task_review_scheduling_priority_=task_fresh_review_&&priority&&std::string(priority)=="1";
  if(task_collision_stop_enabled_)
    task_collision_stop_pub_''')
    s = replace_once(s, '      const bool task_c2_requested=task_static_c2_.enabled&&task_static_work_&&', '''      if(task_fresh_review_&&task_collision_stop_id_>=0&&task_collision_stop_id_==executed_id_&&!executed_done_)return;
      const bool task_c2_requested=!task_fresh_review_&&task_static_c2_.enabled&&task_static_work_&&''')
    s = replace_once(s, '      const auto task_static_planning_map=', '''      std::optional<racer_task::TargetObservation> solved_pair;
      if(task_fresh_review_&&task_static_work_){
        const double now=racer_ros::Time::now().toSec();
        const auto* paired=taskFreshReviewFrame(now);
        const auto* navigation=paired?taskReviewNavigationFrame(*paired,now):nullptr;
        if(!navigation)return; // Same-source wait is bounded by taskStaticEventTick.
        solved_pair=*paired;
        expl_manager_->task_static_evaluation_=task_static_work_;
        auto& evaluation=*expl_manager_->task_static_evaluation_;
        evaluation.evidence=*navigation;
        evaluation.paired_capture=paired->capture;
        ROS_INFO("STATIC_FRESH solve token=%llu source=%llu pair_seq=%llu pair=%.6f nav_seq=%llu nav=%.6f sim=%.6f",
          static_cast<unsigned long long>(evaluation.token),
          static_cast<unsigned long long>(paired->key.counter),
          static_cast<unsigned long long>(paired->sequence),paired->capture,
          static_cast<unsigned long long>(navigation->sequence),navigation->capture,now);
      }
      const auto task_static_planning_map=''')
    s = replace_once(s, '        const auto& observation=task_static_work_->evidence;',
        '        const auto& observation=solved_pair?*solved_pair:task_static_work_->evidence;')
    s = replace_once(s, '      if (live_execution_ && live_sent_) return;\n      if(task_static_work_){',
        '''      if (live_execution_ && live_sent_) return;
      const racer_task::TargetObservation* signed_frame=nullptr;
      if(task_static_work_){''')
    s = replace_once(s, '        const auto& evidence=task_static_work_->evidence;',
        '        const auto& static_evaluation=expl_manager_->task_static_evaluation_;')
    s = replace_once(s, '        if(!racer_task::bearingNavigationSupported(evidence.confidence,evidence.support,\n'
                        '           evidence.support_first,evidence.capture)&&',
        '''        if(static_evaluation&&!racer_task::bearingNavigationSupported(
           static_evaluation->evidence.confidence,static_evaluation->evidence.support,
           static_evaluation->evidence.support_first,static_evaluation->evidence.capture)&&''')
    s = replace_once(s, '        const double publish_now=', '''        if(task_fresh_review_){
          signed_frame=taskFreshReviewFrame(racer_ros::Time::now().toSec(),
              static_evaluation?static_evaluation->paired_capture:-1.);
          if(!signed_frame){taskStaticRelease(true,"paired_source_missing_before_signing");break;}
          if(!taskReviewNavigationFrame(*signed_frame,racer_ros::Time::now().toSec())){
            taskStaticRelease(false,"review_navigation_support_expired");break;
          }
          const auto endpoint=curve.evaluateDeBoorT(trajectory_s);
          if(racer_task::staticReviewViewRejection({endpoint[0],endpoint[1],endpoint[2]},
                                                 signed_frame->origin,signed_frame->direction)){
            taskStaticRelease(true,"latest_source_outside_planned_view");break;
          }
        }
        const double publish_now=''')
    s = replace_once(s, '          task_static_metadata_.data.push_back(double(task_static_gate_.map_revision));', '''          task_static_metadata_.data.push_back(double(task_static_gate_.map_revision));
          if(signed_frame){
            task_static_metadata_.data.insert(task_static_metadata_.data.end(),
                {double(signed_frame->key.counter),double(signed_frame->sequence),signed_frame->capture});
            ROS_INFO("STATIC_FRESH signed token=%llu id=%d source=%llu seq=%llu capture=%.6f sim=%.6f",
              static_cast<unsigned long long>(task_static_work_->token),task_static_gate_.proposal,
              static_cast<unsigned long long>(signed_frame->key.counter),
              static_cast<unsigned long long>(signed_frame->sequence),signed_frame->capture,
              racer_ros::Time::now().toSec());
          }''')
    s = replace_once(s, '        task_collision_stop_id_=executed_id_;\n        task_collision_stop_stamp_', '''        task_stop_for_review_=false;
        task_collision_stop_id_=executed_id_;
        task_collision_stop_stamp_''')
    s = replace_once(s, '  request.data={double(task_collision_stop_id_),task_collision_stop_stamp_,now};', '''  request.data={double(task_collision_stop_id_),task_collision_stop_stamp_,now};
  if(task_stop_for_review_){
    request.data.push_back(1.);
    if(task_review_source_retention_)request.data.push_back(task_review_stop_heading_);
  }''')
    s = replace_once(s, '    fd_->avoid_collision_=false;\n    task_collision_stopped_=true;', '''    if(!task_stop_for_review_)fd_->avoid_collision_=false;
    task_collision_stopped_=true;''')
    s = replace_once(s, '    ROS_WARN("COLLISION_STOP released id=%.0f sim=%.6f request=%.6f received=%.6f",d[7],d[5],d[8],received_at);', '''    ROS_WARN("%s released id=%.0f sim=%.6f request=%.6f received=%.6f",
      task_stop_for_review_?"REVIEW_STOP":"COLLISION_STOP",d[7],d[5],d[8],received_at);
    task_stop_for_review_=false;''')
    s = replace_once(s, 'bool FastExplorationFSM::taskStaticEventTick(){', '''const racer_task::TargetRecord* FastExplorationFSM::taskReviewSource() const {
  if(!task_static_work_)return nullptr;
  return racer_task::reviewSource(expl_manager_->task_target_registry_->registry,task_static_work_->evidence);
}

const racer_task::TargetObservation* FastExplorationFSM::taskFreshReviewFrame(double now,double minimum_capture) const {
  if(!task_static_work_)return nullptr;
  return racer_task::freshReviewFrame(expl_manager_->task_target_registry_->registry,task_static_work_->evidence,now,minimum_capture);
}

const racer_task::TargetObservation* FastExplorationFSM::taskReviewNavigationFrame(
    const racer_task::TargetObservation& paired,double now) const {
  return racer_task::reviewNavigationFrame(expl_manager_->task_target_registry_->registry,
    task_static_work_->evidence,paired,now,task_static_gate_.last_started+task_static_budget_.observation_s,
    task_static_weak_);
}

void FastExplorationFSM::taskRequestReviewStop(double now) {
  if(executed_id_<0||executed_done_||!executed_data_||task_collision_stop_id_==executed_id_||
     executed_data_->duration_-executed_progress_<=1.)return;
  task_stop_for_review_=true;
  const auto& bearing=task_static_work_->evidence.direction;
  task_review_stop_heading_=std::atan2(bearing[1],bearing[0]);
  task_collision_stop_id_=executed_id_;
  task_collision_stop_stamp_=now;
  task_collision_stop_last_sent_=-1.;
  taskCollisionStopTick();
  ROS_INFO("REVIEW_STOP requested id=%d sim=%.6f",executed_id_,now);
}

bool FastExplorationFSM::taskStaticEventTick(){''')
    s = replace_once(s, '  if(!task_static_budget_.allows(now,0.)){', '''  if(task_fresh_review_){
   const auto& registry=manager.task_target_registry_->registry;
   const auto& key=task_static_work_->evidence.key;
   if(registry.records.count(key)&&registry.groupConfirmed(key)){
    taskStaticRelease(false,"review_confirmed_before_signing");return true;
   }
   const auto* source=taskReviewSource();
   if(!source||!racer_task::freshReviewCapture(source->latest.capture,now)){
    taskStaticRelease(false,"review_source_lost_while_waiting");return true;
   }
   const bool waiting_for_stop=executed_id_>=0&&!executed_done_;
   if(now-task_static_gate_.last_started>=task_static_budget_.observation_s){
    const auto* paired=taskFreshReviewFrame(now);
    if(waiting_for_stop||!paired||!taskReviewNavigationFrame(*paired,now)){
     taskStaticRelease(false,"review_stop_or_pair_wait_expired");return true;
    }
   }
  }
  if(!task_static_budget_.allows(now,0.)){''')
    s = replace_once(s, ' const bool stopped_after_collision=state_==PLAN_TRAJ&&task_collision_stopped_&&executed_id_<0;',
        ''' const bool stopped_after_collision=state_==PLAN_TRAJ&&task_collision_stopped_&&executed_id_<0;
 const bool stopped_for_fresh_review=task_fresh_review_&&state_==PLAN_TRAJ&&(executed_id_<0||executed_done_);''')
    s = replace_once(s, '&&!waiting_execution&&!stopped_after_collision)return false;',
        '&&!waiting_execution&&!stopped_after_collision&&!stopped_for_fresh_review)return false;')
    s = replace_once(s, 'task_static_events_.start(now,task_static_monitor_.reservedSources());',
        '''task_static_events_.start(now,task_static_monitor_.reservedSources(),task_review_scheduling_priority_?
     &manager.task_target_registry_->registry:nullptr);''')
    s = replace_once(s, ' if(state_==IDLE){\n  // No current exploration goal', ''' if(stopped_for_fresh_review)return false;
 if(state_==IDLE){
  // No current exploration goal''')
    s = replace_once(s, ' if(!useful){taskStaticRelease(false,"native_preview_prefers_exploration");return false;}', ''' if(!useful){taskStaticRelease(false,"native_preview_prefers_exploration");return false;}
 const auto* source=task_fresh_review_?taskReviewSource():nullptr;
 if(source&&racer_task::freshReviewCapture(source->latest.capture,now))taskRequestReviewStop(now);''')
    s = replace_once(s, '  transitState(fd_->static_state_?PLAN_TRAJ:EXEC_TRAJ,"static event proposal retired");',
        '''  const bool awaiting_review_stop=task_fresh_review_&&task_stop_for_review_&&task_collision_stop_id_==executed_id_;
  transitState(fd_->static_state_||awaiting_review_stop?PLAN_TRAJ:EXEC_TRAJ,"static event proposal retired");''')
    s = replace_once(s, ' if(retry)expl_manager_->task_static_events_.retry(token);', ''' const std::string failure(reason);
 if(task_fresh_review_&&(failure=="review_source_lost_while_waiting"||failure=="review_stop_or_pair_wait_expired"||
      failure=="paired_source_missing_before_signing"||failure=="latest_source_outside_planned_view"||
      failure=="review_navigation_support_expired")){
  const auto* source=taskReviewSource();
  expl_manager_->task_static_events_.waitForFreshFrame(token,source?source->latest:task_static_work_->evidence);
 }
 else if(retry)expl_manager_->task_static_events_.retry(token);''')
    p.write_text(s)
    p = core/'src/exploration_manager/fast_exploration_manager.cpp'
    s = replace_once(p.read_text(), '  auto decision=task_bearing_policy_.choose(base,candidates,now);', '''  // A work item carries its single priority admission through preview and solve.
  // A fresh paired frame is independently required before the stopped solve.
  const bool priority=task_static_evaluation_&&task_static_evaluation_->priority&&
    racer_task::freshReviewCapture(std::max(task_static_evaluation_->evidence.capture,
                                         task_static_evaluation_->paired_capture),now);
  auto decision=task_bearing_policy_.choose(base,candidates,now,priority);''')
    p.write_text(s)
