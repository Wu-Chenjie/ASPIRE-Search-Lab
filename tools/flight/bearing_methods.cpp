// Compiled inside the original RACER manager; no fabricated semantic goal.
namespace {
std::string bearingNumber(double x){if(!std::isfinite(x))return "null";std::ostringstream s;s<<std::setprecision(17)<<x;return s.str();}
std::string bearingPoint(const Eigen::Vector3d& p){return "["+bearingNumber(p.x())+","+bearingNumber(p.y())+","+bearingNumber(p.z())+"]";}
racer_task::BearingVec bearingArray(const Eigen::Vector3d& p){return {p.x(),p.y(),p.z()};}
}

void FastExplorationManager::taskBearingReceive(const std_msgs::Float64MultiArrayConstPtr& msg){
  const double now=racer_ros::Time::now().toSec();
  const bool accepted=!task_bearing_policy_.persistent && task_bearing_policy_.receive(msg->data,now,ep_->drone_id_-1);
  std::ostringstream s;s<<"{\"event\":\"cue_receipt\",\"processing_time_s\":"<<bearingNumber(now)
    <<",\"accepted\":"<<(accepted?"true":"false")<<",\"enabled\":"<<(task_bearing_policy_.enabled?"true":"false")<<",\"packet\":[";
  for(size_t i=0;i<msg->data.size();++i){if(i)s<<',';s<<bearingNumber(msg->data[i]);}s<<"]}";
  std_msgs::msg::String row;row.data=s.str();task_bearing_log_.publish(row);
}

bool FastExplorationManager::taskBearingRerank(const Eigen::Vector3d& pos,const Eigen::Vector3d& vel,
    const Eigen::Vector3d& yaw,Eigen::Vector3d& goal,double& goal_yaw){
  const double now=racer_ros::Time::now().toSec();const Eigen::Vector3d original=goal;const double original_yaw=goal_yaw;
  ++task_bearing_sequence_;task_bearing_pending_=true;
  racer_task::BearingCandidate base;base.id=-1;base.point=bearingArray(goal);base.yaw=goal_yaw;
  struct Entry {racer_task::BearingCandidate c;std::string kind,gate;int frontier=-1,trigger=-1;};
  std::vector<Entry> pool;std::vector<racer_task::BearingCandidate> candidates;
  auto cost=[&](const Eigen::Vector3d& p,double y,racer_task::BearingCandidate& c){
    c.point=bearingArray(p);c.yaw=y;c.reachable=false;c.cost=0;
    if(!p.allFinite() || !std::isfinite(y) || !sdf_map_->isInBox(p) || sdf_map_->getOccupancy(p)!=SDFMap::FREE || sdf_map_->getInflateOccupancy(p)!=0)return;
    std::vector<Eigen::Vector3d> path;c.cost=ViewNode::computeCost(pos,p,yaw[0],y,vel,yaw[1],path);
    if(path.size()<2 || !std::isfinite(c.cost) || c.cost<=0)return;
    c.path_length=0.;
    for(size_t i=1;i<path.size();++i){
      double length=(path[i]-path[i-1]).norm();if(!std::isfinite(length))return;
      c.path_length+=length;
      int count=std::max(1,int(std::ceil(length/.05)));
      for(int j=0;j<=count;++j){Eigen::Vector3d q=path[i-1]+double(j)/count*(path[i]-path[i-1]);
        if(!sdf_map_->isInBox(q) || sdf_map_->getOccupancy(q)!=SDFMap::FREE || sdf_map_->getInflateOccupancy(q)!=0)return;
      }
    }c.reachable=true;
  };
  auto& policy=task_bearing_policy_;
  if(policy.persistent)taskRegistryUpdate();
  std::vector<int> evaluating;std::set<int> begun;

  auto explain=[&](const racer_task::BearingCandidate& c){
    std::ostringstream out;out<<'[';bool first=true;
    for(const auto& kv:policy.cues){const auto& cue=kv.second;auto u=policy.explain(c,cue,now);if(!first)out<<',';first=false;
      out<<"{\"cue_id\":"<<cue.id<<",\"capture_s\":"<<bearingNumber(cue.stamp)<<",\"age_s\":"<<bearingNumber(u.age)
       <<",\"heading_angle_rad\":"<<bearingNumber(u.heading_angle)<<",\"heading_alignment\":"<<bearingNumber(u.heading_alignment)
       <<",\"lateral_baseline_m\":"<<bearingNumber(u.lateral_baseline)<<",\"view_diversity\":"<<bearingNumber(u.view_diversity)
       <<",\"confidence\":"<<bearingNumber(cue.confidence)<<",\"age_factor\":"<<bearingNumber(u.age_factor)
       <<",\"coverage\":"<<cue.coverage<<",\"uses\":"<<cue.uses<<",\"utility\":"<<bearingNumber(u.value)<<",\"reason\":\""<<u.reason<<"\"}";
    }out<<']';return out.str();
  };
  const bool score=policy.mayScore(now);
  if(score){
    if(policy.persistent&&!policy.event_driven_static)for(const auto&kv:policy.cues){
      int id=kv.second.id;auto c=policy.discovery.cues.find(id);
      if(c!=policy.discovery.cues.end()&&policy.discovery.available(id,now)&&begun.insert(c->second.group).second&&policy.discovery.begin(id,now))evaluating.push_back(id);
    }
    cost(goal,goal_yaw,base);
    auto add=[&](const Eigen::Vector3d& p,double y,const std::string& kind,int frontier,int trigger){
      Entry e;e.c.id=int(pool.size());e.c.point=bearingArray(p);e.c.yaw=y;e.kind=kind;e.frontier=frontier;e.trigger=trigger;
      e.c.review_view=kind=="bearing_lateral_observation";pool.push_back(e);
    };
    // Bounded observation positions, NOT positions or ranges of a person.
    // Both sides use current measured altitude and the original native path/trajectory interface.
    const racer_task::BearingCue* fresh=nullptr;
    for(const auto& kv:policy.cues){const auto& c=kv.second;
      if((policy.event_driven_static||!policy.persistent||policy.discovery.available(c.id,now)) && now>=c.stamp && (policy.event_driven_static||now-c.stamp<policy.ttl) && (policy.event_driven_static||c.uses<policy.per_cue_limit) && std::hypot(c.direction[0],c.direction[1])>=.2 && (!fresh || c.stamp>fresh->stamp))fresh=&c;
    }
    if(fresh){double h=std::hypot(fresh->direction[0],fresh->direction[1]);
      Eigen::Vector3d lateral(-fresh->direction[1]/h,fresh->direction[0]/h,0.);
      double y=std::atan2(fresh->direction[1],fresh->direction[0]);
      Eigen::Vector3d center=pos;
      if(policy.event_driven_static&&racer_task::staticReviewViewRejection(
          bearingArray(pos),fresh->origin,fresh->direction)){
        center=Eigen::Vector3d(fresh->origin[0],fresh->origin[1],fresh->origin[2]);
      }
      add(center+.6*lateral,y,"bearing_lateral_observation",-1,fresh->id);
      add(center-.6*lateral,y,"bearing_lateral_observation",-1,fresh->id);
    }
    std::vector<int> ids;for(size_t i=0;i<ed_->points_.size();++i)ids.push_back(int(i));
    std::stable_sort(ids.begin(),ids.end(),[&](int a,int b){return (ed_->points_[a]-pos).squaredNorm()<(ed_->points_[b]-pos).squaredNorm();});
    for(int i:ids){if(pool.size()>=66)break;add(ed_->points_[i],ed_->yaws_[i],"native_frontier_viewpoint",i,-1);}
    int evaluated=0;
    for(auto& e:pool){int cue=-1;
      if(policy.utility(e.c,now,cue)<=0){e.gate="zero_observation_utility";continue;}
      if(evaluated>=8){e.gate="path_evaluation_budget";continue;}++evaluated;
      Eigen::Vector3d p(e.c.point[0],e.c.point[1],e.c.point[2]);cost(p,e.c.yaw,e.c);
      e.gate=e.c.reachable?"path_evaluated":"native_path_not_known_free";candidates.push_back(e.c);
    }
  }
  const std::string base_components=explain(base);std::ostringstream pool_log;pool_log<<'[';
  for(size_t i=0;i<pool.size();++i){const auto& e=pool[i];if(i)pool_log<<',';Eigen::Vector3d p(e.c.point[0],e.c.point[1],e.c.point[2]);
    pool_log<<"{\"candidate_id\":"<<e.c.id<<",\"kind\":\""<<e.kind<<"\",\"frontier_id\":"<<e.frontier<<",\"trigger_cue_id\":"<<e.trigger
     <<",\"goal\":"<<bearingPoint(p)<<",\"yaw\":"<<bearingNumber(e.c.yaw)<<",\"gate\":\""<<e.gate<<"\",\"native_cost\":"<<bearingNumber(e.c.cost)
     <<",\"reachable\":"<<(e.c.reachable?"true":"false")<<",\"components\":"<<explain(e.c)<<"}";
  }pool_log<<']';
  int without_discount=-1;double raw_best=base.cost;
  for(const auto& c:candidates)if(c.reachable && policy.admissibleCost(c.cost,base.cost) && c.cost<raw_best-1e-9){raw_best=c.cost;without_discount=c.id;}
  std::set<int> reachable_cues;
  for(int id:evaluating)for(const auto&c:candidates)if(c.reachable&&policy.admissibleCost(c.cost,base.cost)&&policy.explain(c,policy.cues.at({ep_->drone_id_-1,id}),now).value>0)reachable_cues.insert(id);
  auto decision=task_bearing_policy_.choose(base,candidates,now);
  task_bearing_selected_cue_=decision.changed?decision.cue_id:-1;
  if(policy.persistent&&!policy.event_driven_static)for(int id:evaluating){
    policy.discovery.finish(id,now,reachable_cues.count(id));
  }
  std::string chosen_kind="native_frontier_viewpoint";
  if(decision.changed){const auto& e=pool.at(decision.selected);goal=Eigen::Vector3d(e.c.point[0],e.c.point[1],e.c.point[2]);goal_yaw=e.c.yaw;chosen_kind=e.kind;}
  task_bearing_target_=goal;task_bearing_changed_=decision.changed;
  std::ostringstream out;out<<"{\"event\":\"decision\",\"evaluation_mode\":\""<<(task_bearing_preview_?"preview":"planning")<<"\",\"selection_id\":"<<task_bearing_sequence_<<",\"processing_time_s\":"<<bearingNumber(now)
    <<",\"policy_mode\":\""<<(policy.event_driven_static?"static_event":"legacy")<<"\",\"enabled\":"<<(policy.enabled?"true":"false")<<",\"changed\":"<<(decision.changed?"true":"false")
    <<",\"reason\":\""<<decision.reason<<"\",\"goal_kind\":\""<<chosen_kind<<"\",\"human_position\":null,\"baseline_goal\":"<<bearingPoint(original)
    <<",\"position\":"<<bearingPoint(pos)<<",\"baseline_yaw\":"<<bearingNumber(original_yaw)<<",\"chosen_goal\":"<<bearingPoint(goal)<<",\"chosen_yaw\":"<<bearingNumber(goal_yaw)
    <<",\"baseline_native_cost\":"<<bearingNumber(base.cost)<<",\"baseline_reachable\":"<<(score&&base.reachable?"true":"false")
    <<",\"chosen_candidate_id\":"<<decision.selected<<",\"cue_id\":"<<decision.cue_id<<",\"mission_bias_selections\":"<<policy.selections
    <<",\"without_discount_candidate_id\":"<<without_discount<<",\"discount_changes_selection\":"<<((decision.changed && decision.selected!=without_discount)?"true":"false")
    <<",\"native_frontier_pool_size\":"<<ed_->points_.size()<<",\"baseline_components\":"<<base_components<<",\"candidate_pool\":"<<pool_log.str()<<",\"candidates\":[";
  for(size_t i=0;i<decision.scores.size();++i){const auto& c=decision.scores[i];if(i)out<<',';
    Eigen::Vector3d p=original;double y=original_yaw;if(c.id>=0){const auto& q=pool.at(c.id).c;p=Eigen::Vector3d(q.point[0],q.point[1],q.point[2]);y=q.yaw;}
    out<<"{\"candidate_id\":"<<c.id<<",\"goal\":"<<bearingPoint(p)<<",\"yaw\":"<<bearingNumber(y)<<",\"native_cost\":"<<bearingNumber(c.base_cost)
      <<",\"adjusted_cost\":"<<bearingNumber(c.adjusted_cost)<<",\"utility\":"<<bearingNumber(c.utility)<<",\"cue_id\":"<<c.cue_id
      <<",\"reachable\":"<<(c.reachable?"true":"false")<<",\"reason\":\""<<c.reason<<"\"}";
  }out<<"]}";std_msgs::msg::String row;row.data=out.str();task_bearing_log_.publish(row);
  ROS_INFO("BEARING_DECISION selection=%d changed=%d reason=%s",task_bearing_sequence_,int(decision.changed),decision.reason.c_str());
  return decision.changed;
}

void FastExplorationManager::taskBearingPlan(int native_id,int result){
  if(!task_bearing_pending_)return;task_bearing_pending_=false;
  if(task_bearing_policy_.persistent&&!task_bearing_policy_.event_driven_static&&task_bearing_changed_&&task_bearing_selected_cue_>0){
    if(result==SUCCEED)task_bearing_policy_.discovery.planned(native_id,task_bearing_selected_cue_);
    else task_bearing_policy_.discovery.fail(task_bearing_selected_cue_,racer_ros::Time::now().toSec(),"native_plan_failed");
  }
  const std::string endpoint=result==SUCCEED?bearingPoint(planner_manager_->local_data_.position_traj_.evaluateDeBoorT(planner_manager_->local_data_.position_traj_.getTimeSum())):"null";
  std_msgs::msg::String row;row.data="{\"event\":\"plan_result\",\"selection_id\":"+std::to_string(task_bearing_sequence_)+
      ",\"native_plan_id\":"+std::to_string(native_id)+",\"result\":"+std::to_string(result)+
      ",\"processing_time_s\":"+bearingNumber(racer_ros::Time::now().toSec())+
      ",\"native_requested_goal\":"+bearingPoint(ed_->next_pos_)+
      ",\"bearing_choice_changed\":"+(task_bearing_changed_?"true":"false")+
      ",\"bearing_goal_preserved\":"+((ed_->next_pos_-task_bearing_target_).norm()<1e-4?"true":"false")+
      ",\"curve_endpoint\":"+endpoint+"}";task_bearing_log_.publish(row);
}
