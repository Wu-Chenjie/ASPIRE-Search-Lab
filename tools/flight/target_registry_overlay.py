"""Persistent records and guidance feedback only in generated native RACER copy."""
from pathlib import Path
import shutil
from .optimizer_overlay import replace_once
def apply(core):
    here=Path(__file__).parent;inc=core/'include/exploration_manager'
    for name in ('target_registry.hpp','target_protocol.hpp','target_registry_ros.hpp','static_causal_trace.hpp','discovery_memory.hpp','static_guidance_events.hpp','static_review_budget.hpp','fresh_review.hpp'):
        shutil.copyfile(here/name,inc/name)
    (inc/'review_protocol.hpp').write_text('#pragma once\n#include "task_review_protocol.hpp"\n')
    p=inc/'fast_exploration_manager.h';s=p.read_text()
    s=replace_once(s,'#include <exploration_manager/task_bearing_bias.hpp>','#include <exploration_manager/task_bearing_bias.hpp>\n#include <exploration_manager/target_registry_ros.hpp>\n#include <exploration_manager/static_guidance_events.hpp>')
    s=replace_once(s,'  racer_task::BearingBias task_bearing_policy_;','''  std::unique_ptr<racer_task::TargetRegistryROS> task_target_registry_;
  racer_task::StaticGuidanceEvents task_static_events_;
  bool task_static_event_audit_=false;
  bool task_depth_review_pending_=false;
  bool task_depth_review_has_key_=false;
  double task_depth_review_since_=-1.;
  racer_task::TargetKey task_depth_review_key_{};
  std::set<racer_task::TargetKey> task_depth_review_expired_;
  bool taskDepthReviewPending(double now);
  std::map<racer_task::TargetKey,int> task_target_progress_;
  int task_bearing_selected_cue_=-1;
  void taskRegistryUpdate();
  void taskGuidanceFeedback(int active,bool done,int candidate,bool rejected);
  void taskGuidanceLog();
  racer_task::BearingBias task_bearing_policy_;''');p.write_text(s)
    p=core/'src/exploration_manager/fast_exploration_manager.cpp';s=p.read_text()
    s=replace_once(s,'  if(task_bearing_policy_.enabled && ep_->drone_num_!=1)','''  nh.param("search/target_registry",task_bearing_policy_.persistent,false);
  nh.param("search/static_event_audit",task_static_event_audit_,false);
  if(task_static_event_audit_&&!task_bearing_policy_.persistent)throw std::runtime_error("Static event audit requires target registry");
  if(task_bearing_policy_.persistent){
    if(ep_->drone_num_<1||ep_->drone_num_>2)throw std::runtime_error("Target source cache supports one or two robots only");
    task_target_registry_=std::make_unique<racer_task::TargetRegistryROS>(racer_ros::node,"uav"+std::to_string(ep_->drone_id_-1));
    task_target_registry_->on_update=[this](){
      taskRegistryUpdate();
      taskDepthReviewPending(racer_ros::Time::now().toSec());
    };
  }
  if(task_bearing_policy_.enabled && ep_->drone_num_!=1)''')
    marker='int FastExplorationManager::planExploreMotion('
    s=replace_once(s,marker,(here/'target_registry_methods.cpp').read_text()+'\n'+marker)
    s=replace_once(s,'  return decision.changed;','  taskGuidanceLog();\n  return decision.changed;')
    p.write_text(s)
    p=inc/'fast_exploration_fsm.h';s=p.read_text()
    s=replace_once(s,'  bool task_review_pending_ = false;',
                   '  bool task_depth_exploration_proposal_ = false;\n  bool task_review_pending_ = false;')
    p.write_text(s)
    p=core/'src/exploration_manager/fast_exploration_fsm.cpp';s=p.read_text()
    s=replace_once(s,'      // Inform traj_server the replanning', '''      task_depth_exploration_proposal_=live_execution_&&!fd_->go_back_&&!fd_->avoid_collision_&&
          !task_return_.started()&&!task_review_session_.active()&&!task_review_pending_;
      if(task_depth_exploration_proposal_&&expl_manager_->taskDepthReviewPending(racer_ros::Time::now().toSec()))return;
      // Inform traj_server the replanning''')
    s=replace_once(s,'    case PUB_TRAJ: {','''    case PUB_TRAJ: {
      // The RGB confirmation can arrive between the solve and the first publication.
      if(!live_sent_&&task_depth_exploration_proposal_&&expl_manager_->taskDepthReviewPending(racer_ros::Time::now().toSec())){
        transitState(PLAN_TRAJ,"hold for registered depth");
        return;
      }''')
    s=replace_once(s,'  const int active = int(d[0]), candidate = int(d[3]);','''  const int active = int(d[0]), candidate = int(d[3]);
  expl_manager_->taskGuidanceFeedback(active,d[2]>0.,candidate,d[4]<0.);''')
    p.write_text(s)
