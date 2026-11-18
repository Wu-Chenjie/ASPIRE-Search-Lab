"""Opt-in event re-evaluation through existing native planning and authorization.

Only generated execution-safety builds include this overlay. It preserves the
existing stop-and-plan guard; host contracts do not validate moving C2 handoff.
"""
from pathlib import Path
import shutil
from .optimizer_overlay import replace_once


def apply(core):
    here=Path(__file__).parent
    inc=core/'include/exploration_manager'
    shutil.copyfile(here/'static_replan_gate.hpp',inc/'static_replan_gate.hpp')
    shutil.copyfile(here/'static_mission_budget.hpp',inc/'static_mission_budget.hpp')
    p=core/'include/plan_env/sdf_map.h';s=p.read_text()
    s=replace_once(s,'  enum OCCUPANCY', '''  std::uint64_t taskStaticMapRevision() const { return task_static_map_revision_; }
  std::uint64_t task_static_map_revision_=0; // written only by fused-map export
  enum OCCUPANCY''')
    s='#include <cstdint>\n'+s;p.write_text(s)
    p=core/'src/plan_env/map_ros.cpp';s=p.read_text()
    s=replace_once(s,'  data[2] = static_cast<double>(++execution_safety_map_seq_);',
                   '  data[2] = static_cast<double>(++execution_safety_map_seq_);\n  map_->task_static_map_revision_=execution_safety_map_seq_;')
    p.write_text(s)
    p=inc/'fast_exploration_manager.h';s=p.read_text()
    s=replace_once(s,'  bool task_static_event_audit_=false;',
                   '  bool task_static_event_audit_=false;\n  bool task_static_event_consumer_=false;\n  std::optional<racer_task::StaticGuidanceEvents::Work> task_static_evaluation_;')
    p.write_text(s)
    p=core/'src/exploration_manager/fast_exploration_manager.cpp';s=p.read_text()
    s=replace_once(s,'  ++task_bearing_sequence_;task_bearing_pending_=true;',
                   '  if(task_static_event_consumer_&&!task_static_evaluation_){task_bearing_policy_.original_next=false;return false;}\n  ++task_bearing_sequence_;task_bearing_pending_=true;')
    s=replace_once(s,'  if(policy.persistent)taskRegistryUpdate();', '''  if(policy.persistent)taskRegistryUpdate();
  if(task_static_evaluation_&&!task_static_events_.valid(task_static_evaluation_->token)){
    task_bearing_pending_=false;return false;
  }
  racer_task::ScopedBearingEntity event_scope(policy,ep_->drone_id_-1,
    task_static_evaluation_?task_target_registry_->cueId(task_static_evaluation_->evidence.key):-1,
    bool(task_static_evaluation_));
  // Preserve the qualified event frame across a later weak measurement. This
  // does not rewrite its timestamp, bypass confirmation or change legacy costs.
  if(task_static_evaluation_&&task_static_events_.valid(task_static_evaluation_->token)){
    const auto&o=task_static_evaluation_->evidence;
    policy.weak_event=!racer_task::bearingNavigationSupported(o.confidence,o.support,o.support_first,o.capture);
    const int id=task_target_registry_->cueId(o.key);
    const bool snapshot_loaded=policy.receive({1.,double(ep_->drone_id_-1),double(id),o.capture,o.origin[0],o.origin[1],o.origin[2],
      o.direction[0],o.direction[1],o.direction[2],o.half_angle,o.confidence,double(o.support),0.,
      o.has_position?1.:0.,o.support_first,o.hfov},now,ep_->drone_id_-1);
    if(!snapshot_loaded){task_bearing_pending_=false;return false;}
    if(!policy.event_driven_static)policy.discovery.observe(id,o.first,o.capture,o.origin,o.direction,true,task_target_progress_[o.key]);
  }
''')
    p.write_text(s)
    p=inc/'fast_exploration_fsm.h';s=p.read_text()
    s=replace_once(s,'#include <Eigen/Eigen>','#include <Eigen/Eigen>\n#include <exploration_manager/static_guidance_events.hpp>\n#include <exploration_manager/static_replan_gate.hpp>\n#include <exploration_manager/static_mission_budget.hpp>')
    s=replace_once(s,'  bool task_stop_and_plan_', '''  bool task_static_replan_=false;
  std_msgs::Float64MultiArray task_static_metadata_;
  racer_task::StaticReplanGate task_static_gate_;
  racer_task::StaticMissionBudget task_static_budget_;
  bool task_static_weak_=false;double task_static_weak_trajectory_s_=-1.;
  bool task_static_prior_goal_valid_=false;
  Eigen::Vector3d task_static_prior_goal_;double task_static_prior_yaw_=0.;
  std::optional<racer_task::StaticGuidanceEvents::Work> task_static_work_;
  bool taskStaticEventTick();
  void taskStaticRelease(bool retry,const char* reason);
  bool task_stop_and_plan_''')
    p.write_text(s)
    p=core/'src/exploration_manager/fast_exploration_fsm.cpp';s=p.read_text()
    s=replace_once(s,'  expl_manager_->initialize(nh);', '''  expl_manager_->initialize(nh);
  nh.param("search/static_event_replan",task_static_replan_,false);
  nh.param("search/static_event_min_interval_s",task_static_gate_.min_interval_s,-1.);
  nh.param("search.static_budget.duration_s",task_static_budget_.duration_s,-1.);
  nh.param("search.static_budget.reserve_time_s",task_static_budget_.reserve_time_s,-1.);
  nh.param("search.static_budget.observation_s",task_static_budget_.observation_s,-1.);
  nh.param("search.static_budget.initial_energy_j",task_static_budget_.initial_energy_j,-1.);
  nh.param("search.static_budget.reserve_energy_j",task_static_budget_.reserve_energy_j,-1.);
  nh.param("search.static_budget.power_upper_w",task_static_budget_.power_upper_w,-1.);
  nh.param("search.static_budget.model_id",task_static_budget_.model_id,std::string(""));
  nh.param("search.static_weak_enabled",task_static_weak_,false);
  nh.param("search.static_weak_trajectory_s",task_static_weak_trajectory_s_,-1.);
  nh.param("search.static_weak_extra_cost",expl_manager_->task_bearing_policy_.weak_max_extra_cost,-1.);
  nh.param("search.static_weak_extra_path_m",expl_manager_->task_bearing_policy_.weak_max_extra_path_m,-1.);
  if(task_static_replan_&&task_static_weak_&&(!std::isfinite(task_static_weak_trajectory_s_)||task_static_weak_trajectory_s_<=0.||
     !std::isfinite(expl_manager_->task_bearing_policy_.weak_max_extra_cost)||expl_manager_->task_bearing_policy_.weak_max_extra_cost<=0.||
     !std::isfinite(expl_manager_->task_bearing_policy_.weak_max_extra_path_m)||expl_manager_->task_bearing_policy_.weak_max_extra_path_m<=0.))
    throw std::runtime_error("Single-frame guidance requires explicit low-cost envelopes");
  expl_manager_->task_bearing_policy_.allow_weak=task_static_replan_&&task_static_weak_;
  expl_manager_->task_static_events_.allow_weak=task_static_replan_&&task_static_weak_;
  if(task_static_replan_&&!task_static_budget_.configured())throw std::runtime_error("Static guidance requires explicit mission time and energy envelope");
  if(task_static_replan_&&(!live_execution_||!task_stop_and_plan_||!expl_manager_->task_static_event_audit_||
      !expl_manager_->task_bearing_policy_.enabled||!task_static_gate_.configured()))
    throw std::runtime_error("Static event replan requires authorized stop-and-plan, registry audit, bearing policy and explicit interval");
  expl_manager_->task_static_event_consumer_=task_static_replan_;
  expl_manager_->task_bearing_policy_.event_driven_static=task_static_replan_;''')
    s=replace_once(s,'  fd_->trigger_ = true;', '  fd_->trigger_ = true;\n  if(task_static_replan_)task_static_budget_.start(racer_ros::Time::now().toSec());')
    s=replace_once(s,'  switch (state_) {','  if(taskStaticEventTick())return;\n  switch (state_) {')
    # Preserve original stop-and-plan, recovery and return guards before this point.
    s=replace_once(s,'      proposed_boundary_ = -1.;', '''      const auto task_static_planning_map=expl_manager_->sdf_map_->taskStaticMapRevision();
      proposed_boundary_ = -1.;''')
    s=replace_once(s,'      int res = callExplorationPlanner();', '''      int res = callExplorationPlanner();
      if(task_static_work_){
        if(res!=SUCCEED){taskStaticRelease(false,"native_planning_failed");break;}
        if(!expl_manager_->task_bearing_changed_){
          taskStaticRelease(false,"review_view_not_selected");break;
        }
        const auto& observation=task_static_work_->evidence;
        auto& curve=planner_manager_->local_data_.position_traj_;
        const auto endpoint=curve.evaluateDeBoorT(curve.getTimeSum());
        if(const char* rejection=racer_task::staticReviewViewRejection(
            {endpoint[0],endpoint[1],endpoint[2]},observation.origin,observation.direction)){
          taskStaticRelease(false,rejection);break;
        }
        if(!expl_manager_->task_static_events_.valid(task_static_work_->token)||
           !task_static_gate_.planned(live_counter_+1,executed_id_,proposed_boundary_,task_static_planning_map)){
          taskStaticRelease(true,"planning_snapshot_invalid");break;
        }
        ROS_INFO("STATIC_EVENT planned token=%llu proposal=%d predecessor=%d map=%llu boundary=%.6f",
          static_cast<unsigned long long>(task_static_work_->token),task_static_gate_.proposal,
          task_static_gate_.predecessor,static_cast<unsigned long long>(task_static_gate_.map_revision),proposed_boundary_);
      }''')
    s=replace_once(s,'        boundary_pub_.publish(metadata);',
                   '        if(task_static_work_)task_static_metadata_=metadata;else boundary_pub_.publish(metadata);')
    s=replace_once(s,'      if (live_execution_ && live_sent_) return;', '''      if (live_execution_ && live_sent_) return;
      if(task_static_work_){
        const auto& evidence=task_static_work_->evidence;
        auto& curve=planner_manager_->local_data_.position_traj_;
        const double trajectory_s=curve.getTimeSum();
        if(!racer_task::bearingNavigationSupported(evidence.confidence,evidence.support,
           evidence.support_first,evidence.capture)&&(!task_static_weak_||trajectory_s>task_static_weak_trajectory_s_)){
          taskStaticRelease(false,"weak_evidence_trajectory_budget");break;
        }
        if(const char* rejection=task_static_budget_.rejection(racer_ros::Time::now().toSec(),trajectory_s)){
          taskStaticRelease(false,rejection);break;
        }
        const auto control=curve.getControlPoint();double path_upper=0.;
        for(int i=1;i<control.rows();++i)path_upper+=(control.row(i)-control.row(i-1)).norm();
        if(!expl_manager_->task_static_events_.reserveRoute(task_static_work_->token,
            trajectory_s+task_static_budget_.observation_s,path_upper)){
          taskStaticRelease(false,"component_time_or_distance_budget");break;
        }
        const double publish_now=racer_ros::Time::now().toSec();
        const auto publish_map=expl_manager_->sdf_map_->taskStaticMapRevision();
        if(!task_static_gate_.mayPublish(
             expl_manager_->task_static_events_.valid(task_static_work_->token),
             publish_map,executed_id_,executed_progress_,
             executed_id_<0||executed_done_,task_stop_and_plan_,publish_now,feedback_stamp_)){
          taskStaticRelease(true,"publication_snapshot_invalid");break;
        }
      }''')
    s=replace_once(s,'        bspline_pub_.publish(fd_->newest_traj_);', '''        if(task_static_work_)boundary_pub_.publish(task_static_metadata_);
        bspline_pub_.publish(fd_->newest_traj_);
        if(task_static_work_){
          task_static_gate_.published=true;
          ROS_INFO("STATIC_EVENT submitted token=%llu proposal=%d awaiting_external_authorization",
            static_cast<unsigned long long>(task_static_work_->token),task_static_gate_.proposal);
        }''')
    # Resolve only after the existing feedback callback has applied acceptance/
    # rejection. Never clear or overwrite executor-owned state before it does.
    start=s.index('void FastExplorationFSM::executionFeedback(')
    end=s.index('\nvoid FastExplorationFSM::swarmTrajTimerCallback',start)
    piece=s[start:end];anchor='\n}\n';at=piece.rfind(anchor)
    if at < 0: raise ValueError('Overlay anchor mismatch')
    piece=piece[:at]+'''
  if(task_static_work_&&task_static_gate_.resolved(active,candidate,d[4]<0.)){
    ROS_INFO("STATIC_EVENT resolved token=%llu proposal=%d active=%d rejected=%d",
      static_cast<unsigned long long>(task_static_work_->token),task_static_gate_.proposal,active,int(d[4]<0.));
    expl_manager_->task_static_events_.finish(task_static_work_->token);
    task_static_work_.reset();expl_manager_->task_static_evaluation_.reset();
    task_static_prior_goal_valid_=false;
    task_static_gate_.resetProposal();
  }
'''+piece[at:]
    s=s[:start]+piece+s[end:]
    s=replace_once(s,'void FastExplorationFSM::executionFeedback(',
                   (here/'static_replan_methods.cpp').read_text()+'\nvoid FastExplorationFSM::executionFeedback(')
    p.write_text(s)
