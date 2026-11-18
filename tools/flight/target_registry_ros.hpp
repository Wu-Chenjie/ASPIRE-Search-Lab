#pragma once
#include "target_protocol.hpp"
#include "static_causal_trace.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <deque>
#include <functional>
namespace racer_task {
class TargetRegistryROS {
 using String=std_msgs::msg::String;
 rclcpp::Node::SharedPtr node_;
 rclcpp::Publisher<String>::SharedPtr state_pub_,summary_pub_,sync_pub_,event_pub_;
 rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
 rclcpp::Subscription<String>::SharedPtr local_sub_,summary_sub_,sync_sub_;
 rclcpp::TimerBase::SharedPtr timer_;
 std::map<TargetKey,std::uint64_t> pending_;std::deque<TargetKey> order_;
 std::deque<std::string> sync_out_;
 double last_send_=-1,last_sync_request_=-10;
 std::size_t sync_cursor_=0;bool more_=false,prefer_sync_=false;
 std::uint64_t local_receive_id_=0;
 std::uint64_t request_counter_=0;std::string active_request_,boot_;
 std::map<TargetKey,int> cue_ids_;int next_cue_=1;
 void queue(const TargetKey&k){
  if(k.robot!=registry.self)return;
  if(!pending_.count(k))order_.push_back(k);pending_[k]=registry.records.at(k).revision;
 }
 void error(const std::string&reason){
  ++errors;String m;m.data="{\"event\":\"interface_error\",\"reason\":"+reviewJsonString(reason)+"}";event_pub_->publish(m);
 }
 void local(const String::SharedPtr m){
  const double entry=now();const auto callback=++local_receive_id_;
  staticCausalTrace().record(1,entry,callback,0,0,0,m->data.size());
  try{
   auto result=processTargetJSON(registry,m->data,now());raw_tracks=std::max(raw_tracks,result.raw_tracks);
   for(const auto&k:result.accepted){const auto&o=registry.records.at(k).latest;
    const std::uint64_t state=o.identity_input=="pending"?1:o.identity_input=="completed"?2:o.identity_input=="unavailable"?3:0;
    staticCausalTrace().record(2,now(),callback,k.counter,o.sequence,o.capture,state,(o.has_identity_surface?1:0)|(registry.records.at(k).confirmed?2:0));
   }
   for(const auto&k:result.changed)queue(k);
   if(on_update)on_update();publishState();
   staticCausalTrace().record(3,now(),callback,0,0,0,result.accepted.size());
  }catch(const std::exception&e){staticCausalTrace().record(4,now(),callback);error(e.what());}
 }
 void summary(const String::SharedPtr m){
  try{
   auto t=parseTargetTree(m->data);auto robot=t.get<std::string>("robot");
   if(robot==registry.self){++self_echo_messages;return;}
   received_bytes+=m->data.size();++received_messages;
   auto kind=t.get<std::string>("kind");
   if(kind=="target_summary"){
    auto result=processTargetTree(registry,t,now());
    if(result.accepted.empty())++ignored_messages;
    if(on_update)on_update();
   }else if(kind!="sync_done")throw std::runtime_error("unsupported_peer_kind");
   if(t.get<std::string>("sync_for","")==registry.self&&
      t.get<std::string>("sync_request","")==active_request_){
    sync_cursor_=t.get<std::size_t>("sync_after",0);more_=t.get<bool>("sync_more",false);
    if(!more_)sync_cursor_=0;
   }
  }catch(const std::exception&e){error(e.what());}
 }
 void syncRequest(const String::SharedPtr m){
  try{
   auto t=parseTargetTree(m->data);
   if(t.get<int>("schema")!=1||t.get<std::string>("kind")!="sync_request")return;
   auto peer=t.get<std::string>("robot");auto request=t.get<std::string>("request_id");
   auto cursor=targetUnsigned(t,"after_counter");double time=now();
   ++sync_requests_received;sync_request_bytes_received+=m->data.size();
   if(peer==registry.self||registry.sync_ids.count(peer+":"+request)||
     (registry.last_sync.count(peer)&&time-registry.last_sync.at(peer)<1.))return;
   auto page=registry.sync(peer,request,cursor,time);
   std::size_t total=0;for(const auto&kv:registry.records)total+=kv.first.robot==registry.self;
   if(page.empty()&&cursor<total)return;
   std::size_t next=cursor+page.size();bool more=next<total;
   for(std::size_t i=0;i<page.size();++i){
    auto body=targetRecordSummaryJSON(registry.records.at(page[i]));
    if(i+1==page.size()){
     body.pop_back();body+=",\"sync_for\":"+reviewJsonString(peer)+
      ",\"sync_request\":"+reviewJsonString(request)+",\"sync_after\":"+std::to_string(next)+
      ",\"sync_more\":"+(more?"true":"false")+"}";
    }
    if(sync_out_.size()<32)sync_out_.push_back(body);
   }
   if(page.empty()&&sync_out_.size()<32)
    sync_out_.push_back("{\"schema\":1,\"kind\":\"sync_done\",\"robot\":"+
     reviewJsonString(registry.self)+",\"sync_for\":"+reviewJsonString(peer)+
     ",\"sync_request\":"+reviewJsonString(request)+",\"sync_after\":0,\"sync_more\":false}");
  }catch(const std::exception&e){error(e.what());}
 }
 void expireOwn(){
  double time=now();registry.retire(time);
  for(auto&kv:registry.records){
   auto&r=kv.second;
   if(kv.first.robot!=registry.self||time-r.last<=registry.current_ttl||
      (!r.latest.has_position&&!r.latest.has_bearing))continue;
   if(r.latest.has_position)r.latest.localization="expired";
   r.latest.has_position=r.latest.has_bearing=false;
   ++r.revision;r.material_reference=r.latest;queue(kv.first);
   registry.events.push_back({"record","capture_lease_expired",kv.first,{},time});
  }
 }
 bool displayable(const TargetRecord&r)const{
  if(r.latest.key.robot==registry.self)return true;
  for(const auto&kv:registry.records)
   if(kv.first.robot==registry.self&&registry.compatibleFrames(kv.second.latest,r.latest))return true;
  return false;
 }
 void markers(){
  using Marker=visualization_msgs::msg::Marker;
  visualization_msgs::msg::MarkerArray out;Marker clear;clear.action=Marker::DELETEALL;out.markers.push_back(clear);
  double time=now();
  for(const auto&key:registry.roots()){
   auto*point=registry.currentPosition(key,time);const TargetRecord*bearing=nullptr;
   for(const auto&k:registry.component(key)){const auto&r=registry.records.at(k);
    if(r.latest.has_bearing&&time>=r.last&&time-r.last<=2.&&displayable(r)&&(!bearing||r.last>bearing->last))bearing=&r;}
   if(point&&!displayable(*point))point=nullptr;
   if(!point&&!bearing)continue;
   Marker mark;mark.header.frame_id=point?point->latest.frame:bearing->latest.frame;mark.header.stamp=node_->now();
   mark.id=cueId(key);mark.action=Marker::ADD;mark.pose.orientation.w=1.;mark.color.a=.9;
   TargetVec anchor{};
   if(point){
    mark.ns="persistent_visible_surface";mark.type=Marker::SPHERE;anchor=point->latest.position;
    mark.pose.position.x=anchor[0];mark.pose.position.y=anchor[1];mark.pose.position.z=anchor[2];
    mark.scale.x=mark.scale.y=mark.scale.z=.2;mark.color.b=1.;mark.color.g=.7;
   }else{
    mark.ns="persistent_bearing_no_range";mark.type=Marker::LINE_LIST;mark.scale.x=.025;mark.color.g=1.;
    anchor=bearing->latest.origin;
    for(int i=0;i<2;++i){geometry_msgs::msg::Point p;
     p.x=anchor[0]+i*2.*bearing->latest.direction[0];p.y=anchor[1]+i*2.*bearing->latest.direction[1];
     p.z=anchor[2]+i*2.*bearing->latest.direction[2];mark.points.push_back(p);}
   }
   out.markers.push_back(mark);Marker label;label.header=mark.header;label.id=mark.id;
   label.ns="persistent_identity_status";label.type=Marker::TEXT_VIEW_FACING;label.action=Marker::ADD;
   label.pose.orientation.w=1.;label.pose.position.x=anchor[0];label.pose.position.y=anchor[1];label.pose.position.z=anchor[2]+.2;
   label.scale.z=.12;label.color.a=label.color.r=label.color.g=label.color.b=1.;
   label.text="H"+std::to_string(mark.id)+(registry.groupConfirmed(key)?" visual-confirmed":" candidate")+
    (point?" / visible-surface":" / ray display only; no range");out.markers.push_back(label);
  }
  markers_pub_->publish(out);
 }
 void publishState(){
  for(const auto&e:registry.events)if(e.kind=="record")queue(e.key);
  auto body=targetRegistryJSON(registry,now(),raw_tracks);body.pop_back();
  body+=",\"communication\":{\"sent_messages\":"+std::to_string(sent_messages)+
   ",\"sent_payload_bytes\":"+std::to_string(sent_bytes)+
   ",\"received_foreign_messages\":"+std::to_string(received_messages)+
   ",\"received_foreign_payload_bytes\":"+std::to_string(received_bytes)+
   ",\"ignored_messages\":"+std::to_string(ignored_messages)+
   ",\"self_echo_messages\":"+std::to_string(self_echo_messages)+
   ",\"pending_records\":"+std::to_string(pending_.size())+
   ",\"sync_queue\":"+std::to_string(sync_out_.size())+
   ",\"sync_requests_sent\":"+std::to_string(sync_requests_sent)+
   ",\"sync_request_bytes_sent\":"+std::to_string(sync_request_bytes_sent)+
   ",\"sync_requests_received_including_echo\":"+std::to_string(sync_requests_received)+
   ",\"sync_request_bytes_received_including_echo\":"+std::to_string(sync_request_bytes_received)+
   ",\"network_overhead_included\":false},\"interface_errors\":"+std::to_string(errors)+"}";
  String state;state.data=body;state_pub_->publish(state);
  if(!registry.events.empty()){
   String events;events.data="{\"events\":"+targetEventsJSON(registry.events)+"}";event_pub_->publish(events);
   registry.events.clear();
  }
  markers();
 }
 void tick(){
  double time=now();expireOwn();if(on_update)on_update();
  if(time-last_send_>=.2&&(!pending_.empty()||!sync_out_.empty())){
   std::string body;bool take_sync=!sync_out_.empty()&&(pending_.empty()||prefer_sync_);
   prefer_sync_=!prefer_sync_;
   if(take_sync){body=std::move(sync_out_.front());sync_out_.pop_front();}
   else{
    auto key=order_.front();order_.pop_front();pending_.erase(key);
    body=targetRecordSummaryJSON(registry.records.at(key));
   }
   String message;message.data=body;summary_pub_->publish(message);
   ++sent_messages;sent_bytes+=body.size();last_send_=time;
  }
  if(time-last_sync_request_>=(more_?1.:5.)){
   active_request_=boot_+"-"+std::to_string(++request_counter_);
   String request;request.data="{\"schema\":1,\"kind\":\"sync_request\",\"robot\":"+
    reviewJsonString(registry.self)+",\"request_id\":"+reviewJsonString(active_request_)+
    ",\"after_counter\":"+reviewJsonString(std::to_string(sync_cursor_))+"}";
   sync_pub_->publish(request);++sync_requests_sent;sync_request_bytes_sent+=request.data.size();last_sync_request_=time;more_=false;
  }
  publishState();
 }
public:
 TargetRegistry registry;std::function<void()>on_update;std::size_t raw_tracks=0;
 std::uint64_t sent_messages=0,sent_bytes=0,received_messages=0,received_bytes=0;
 std::uint64_t ignored_messages=0,self_echo_messages=0,errors=0;
 std::uint64_t sync_requests_sent=0,sync_request_bytes_sent=0,sync_requests_received=0,sync_request_bytes_received=0;
 explicit TargetRegistryROS(rclcpp::Node::SharedPtr node,const std::string&self):
  node_(std::move(node)),registry(self){
  staticCausalTrace(); // initialize/prefault before callbacks
  const auto alignment_json=node_->declare_parameter<std::string>("search.peer_alignment_json","");
  if(!alignment_json.empty()){
   auto alignment=readTargetAlignment(parseTargetTree(alignment_json));
   if(alignment.robot==self)throw std::runtime_error("peer_alignment_cannot_reference_self");
   registry.peer_alignments[alignment.robot]=alignment;
  }
  boot_=std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
  std::string prefix="/"+self+"/search/";
  state_pub_=node_->create_publisher<String>(prefix+"person_records",10);
  event_pub_=node_->create_publisher<String>(prefix+"target_registry_events",20);
  markers_pub_=node_->create_publisher<visualization_msgs::msg::MarkerArray>(prefix+"targets",10);
  summary_pub_=node_->create_publisher<String>("/search/target_summary",20);
  sync_pub_=node_->create_publisher<String>("/search/target_sync_request",20);
  local_sub_=node_->create_subscription<String>(prefix+"target_observations",20,[this](String::SharedPtr m){local(m);});
  summary_sub_=node_->create_subscription<String>("/search/target_summary",50,[this](String::SharedPtr m){summary(m);});
  sync_sub_=node_->create_subscription<String>("/search/target_sync_request",20,[this](String::SharedPtr m){syncRequest(m);});
  timer_=node_->create_wall_timer(std::chrono::milliseconds(200),[this](){tick();});
 }
 double now()const{return node_->now().seconds();}
 int cueId(const TargetKey&k){auto it=cue_ids_.find(k);if(it==cue_ids_.end())it=cue_ids_.emplace(k,next_cue_++).first;return it->second;}
};
}

