void FastExplorationManager::taskRegistryUpdate(){
 if(!task_target_registry_)return;
 auto&registry=task_target_registry_->registry;auto&policy=task_bearing_policy_;
 const double now=racer_ros::Time::now().toSec();
 if(task_static_event_audit_)task_static_events_.sync(registry,now);
 for(const auto&kv:registry.records){
  if(kv.first.robot!=registry.self)continue;
  const auto&r=kv.second;const auto&o=r.latest;int id=task_target_registry_->cueId(kv.first);
  // Confirmation is sticky at the current reversible identity component.
  // Keep registry evidence for counting/passive mapping; remove active guidance.
  if(racer_task::suppressConfirmedGuidance(registry,kv.first,policy,ep_->drone_id_-1,id))continue;
  const bool fresh=now>=r.last&&now-r.last<=policy.discovery.ttl;
  // Record revisions can change on coexistence/visual events; these are not localization gains.
  if(o.has_position)task_target_progress_[kv.first]=1;
  if(o.has_bearing&&(policy.event_driven_static||fresh)){
   policy.receive({1.,double(ep_->drone_id_-1),double(id),r.last,o.origin[0],o.origin[1],o.origin[2],
    o.direction[0],o.direction[1],o.direction[2],o.half_angle,o.confidence,double(o.support),0.,
    o.has_position?1.:0.,o.support_first,o.hfov},now,ep_->drone_id_-1);
   if(!policy.event_driven_static){
    const bool enabled=racer_task::bearingNavigationSupported(o.confidence,o.support,o.support_first,r.last);
    policy.discovery.observe(id,registry.firstDiscovery(kv.first),r.last,o.origin,o.direction,enabled,task_target_progress_[kv.first],now);
   }
  }else{
   auto c=policy.discovery.cues.find(id);if(c!=policy.discovery.cues.end())c->second.enabled=false;
  }
 }
 if(policy.event_driven_static)return; // scheduling identity comes from the reversible inbox
 for(const auto&a:registry.aliases)if(a.a.robot==registry.self&&a.b.robot==registry.self)
  policy.discovery.alias(task_target_registry_->cueId(a.a),task_target_registry_->cueId(a.b));
 for(auto&kv:policy.discovery.groups){auto&g=kv.second;if(!g.blocked)continue;
  double h=std::hypot(g.failure_direction[0],g.failure_direction[1]);if(h<.2)continue;
  Eigen::Vector3d origin(g.failure_origin[0],g.failure_origin[1],g.failure_origin[2]);
  Eigen::Vector3d lateral(-g.failure_direction[1]/h,g.failure_direction[0]/h,0.);
  std::vector<int> free;std::set<std::array<int,3>> sampled;
  Eigen::Vector3d map_origin,map_size;sdf_map_->getRegion(map_origin,map_size);
  const double resolution=sdf_map_->getResolution();
  // Fixed local samples are only online known-free SDF evidence, never truth.
  for(double side:{-.6,0.,.6})for(double x:{-.1,0.,.1})for(double y:{-.1,0.,.1}){
   Eigen::Vector3d q=origin+side*lateral+Eigen::Vector3d(x,y,0.);
   std::array<int,3> cell;for(int axis=0;axis<3;++axis)cell[axis]=int(std::floor((q[axis]-map_origin[axis])/resolution));
   if(!sampled.insert(cell).second)continue;
   free.push_back(sdf_map_->isInBox(q)&&sdf_map_->getOccupancy(q)==SDFMap::FREE&&sdf_map_->getInflateOccupancy(q)==0);
  }
  policy.discovery.mapContext(kv.first,free,now);
 }
}
void FastExplorationManager::taskGuidanceFeedback(int active,bool done,int candidate,bool rejected){
 if(!task_bearing_policy_.persistent||task_bearing_policy_.event_driven_static)return;double now=racer_ros::Time::now().toSec();
 if(done)task_bearing_policy_.discovery.completed(active,now);
 if(rejected)task_bearing_policy_.discovery.rejected(candidate,now);
}
void FastExplorationManager::taskGuidanceLog(){
 if(!task_bearing_policy_.persistent)return;
 if(task_static_event_audit_){
  const auto q=task_static_events_.stats();std_msgs::msg::String event;
  event.data="{\"event\":\"static_event_inbox\",\"mode\":\""+std::string(task_bearing_policy_.event_driven_static?"consumer":"audit_only")+"\",\"entities\":"+std::to_string(q.entities)+
   ",\"pending\":"+std::to_string(q.pending)+",\"active\":"+std::to_string(q.active)+
   ",\"enqueued\":"+std::to_string(q.enqueued)+",\"coalesced\":"+std::to_string(q.coalesced)+
   ",\"invalidated\":"+std::to_string(q.invalidated)+",\"identity_waiting\":"+std::to_string(q.waiting)+
   ",\"readiness_updates\":"+std::to_string(q.readiness_updates)+",\"readiness_checks\":"+std::to_string(q.readiness_checks)+"}";task_bearing_log_.publish(event);
 }
 if(task_bearing_policy_.event_driven_static)return;
 auto&d=task_bearing_policy_.discovery;double now=racer_ros::Time::now().toSec();
 std::ostringstream s;s<<"{\"event\":\"discovery_memory\",\"evaluation_mode\":\""<<(task_bearing_preview_?"preview":"planning")<<"\",\"processing_time_s\":"<<bearingNumber(now)
  <<",\"mission_evaluations\":"<<d.mission_attempts<<",\"groups\":[";
 bool first=true;for(const auto&kv:d.cues){const auto&g=d.groups.at(kv.second.group);if(!first)s<<',';first=false;
  s<<"{\"cue_id\":"<<kv.first<<",\"group\":"<<g.id<<",\"first_s\":"<<bearingNumber(g.first)
   <<",\"epoch_s\":"<<bearingNumber(g.epoch)<<",\"value\":"<<bearingNumber(d.value(kv.first,now))
   <<",\"eligibility\":"<<racer_task::reviewJsonString(d.reason(kv.first,now))<<",\"last_outcome\":"<<racer_task::reviewJsonString(g.reason)
   <<",\"attempts\":"<<g.attempts<<",\"selections\":"<<g.uses<<",\"epoch_updates\":"<<g.updates
   <<",\"material_retries\":"<<g.retries<<",\"blocked\":"<<(g.blocked?"true":"false")<<",\"cooldown_until_s\":"<<bearingNumber(g.cooldown_until)<<"}";
 }s<<"]}";std_msgs::msg::String m;m.data=s.str();task_bearing_log_.publish(m);
}
