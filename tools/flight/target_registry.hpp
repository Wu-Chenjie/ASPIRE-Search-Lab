#pragma once
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>
namespace racer_task {
using TargetVec=std::array<double,3>;
inline double targetDistance(const TargetVec&a,const TargetVec&b){double v=0;for(int i=0;i<3;++i)v+=(a[i]-b[i])*(a[i]-b[i]);return std::sqrt(v);}
inline bool targetFinite(const TargetVec&v){return std::all_of(v.begin(),v.end(),[](double x){return std::isfinite(x);});}
struct TargetKey {
 std::string robot,session;std::uint64_t counter=0;
 bool operator<(const TargetKey&b)const{return std::tie(robot,session,counter)<std::tie(b.robot,b.session,b.counter);}
 bool operator==(const TargetKey&b)const{return robot==b.robot&&session==b.session&&counter==b.counter;}
 bool operator!=(const TargetKey&b)const{return !(*this==b);}
};
struct TargetObservation {
 TargetKey key;std::vector<std::uint64_t> exclusions;std::uint64_t sequence=0,revision=0;double first=0,capture=0;
 std::string frame="map",frame_version,visual="person_candidate",localization="unknown";
 bool shared_aligned=false,has_position=false,has_bearing=false;
 TargetVec position{},origin{},direction{};double radius=.15,half_angle=.1,confidence=0;
 // Local capture support only. Never creates a position/anchor or confirmation.
 bool has_identity_surface=false;TargetVec identity_surface{};double identity_radius=.15;
 std::string identity_input="unknown";double identity_deadline=0;
 int support=0;double support_first=0,hfov=.637;std::string support_policy="legacy_rgb";
 std::string clock_domain="mission",map_epoch,alignment_id,observation_id;
 bool alignment_verified=false; // internal only; never accepted from the wire
 bool has_surface_quality=false;int support_pixels=0;double valid_depth_fraction=0,component_fraction=0,depth_span_m=0,rgb_depth_skew_s=0;
 std::string calibration_id;
};
struct TargetAlignment {
 std::string id,robot,source_version,target_version,source_clock="mission",target_clock="mission";
 std::array<double,9> rotation{1,0,0,0,1,0,0,0,1};TargetVec translation{};
 double clock_offset_s=0,clock_error_s=0,position_error_m=0,rotation_error_rad=0,valid_from=0,valid_until=0;
 TargetVec apply(const TargetVec&p)const{
  TargetVec q=translation;for(int i=0;i<3;++i)for(int j=0;j<3;++j)q[i]+=rotation[i*3+j]*p[j];return q;
 }
 bool valid(double now)const{
  if(id.empty()||robot.empty()||source_version.empty()||target_version.empty()||source_clock.empty()||target_clock.empty()||
     !targetFinite(translation)||!std::isfinite(clock_offset_s)||!std::isfinite(clock_error_s)||clock_error_s<0||clock_error_s>.02||
     !std::isfinite(position_error_m)||position_error_m<0||!std::isfinite(rotation_error_rad)||rotation_error_rad<0||
     !std::isfinite(valid_from)||!std::isfinite(valid_until)||valid_until<=valid_from||now<valid_from||now>valid_until)return false;
  for(double x:rotation)if(!std::isfinite(x))return false;
  for(int i=0;i<3;++i)for(int j=0;j<3;++j){double dot=0;for(int k=0;k<3;++k)dot+=rotation[i*3+k]*rotation[j*3+k];
   if(std::abs(dot-(i==j?1.:0.))>1e-6)return false;}
  auto&r=rotation;double determinant=r[0]*(r[4]*r[8]-r[5]*r[7])-r[1]*(r[3]*r[8]-r[5]*r[6])+r[2]*(r[3]*r[7]-r[4]*r[6]);
  return std::abs(determinant-1)<1e-6;
 }
};
struct TargetVisibility {
 TargetKey key;double capture=0,anchor_capture=0;TargetVec center{},extent{};
 std::string frame_version,state,reason,scope,model_id;
};
struct TargetRecord {
 std::set<std::uint64_t> exclusions;TargetObservation latest,material_reference;std::uint64_t revision=0;double first=0,last=0;
 bool confirmed=false,has_anchor=false;TargetVec anchor{};double anchor_time=0,anchor_radius=.15;
 std::string identity="unresolved",association_reason="no_qualified_position";
 std::vector<TargetObservation> history;
 std::string visibility="unknown";
 double visibility_capture=-1.,visibility_anchor_capture=-1.;
 std::uint64_t visible_region_misses=0,visibility_revision=0;
};
struct TargetAlias {TargetKey a,b;std::uint64_t a_revision=0,b_revision=0;std::string frame_version;double distance=0;bool surface_sequence=false;};
struct TargetUpdate {bool accepted=false,material=false;std::string reason;};
struct TargetBatchResult {
 std::vector<TargetKey> accepted,changed;std::string reason;std::size_t raw_tracks=0;
 bool peer=false;
};
struct TargetStats {std::size_t detections=0,records=0,scans=0,edges=0,ambiguous=0,linked=0,unmatched=0;double elapsed_us=0;};
struct TargetEvent {std::string kind,reason;TargetKey key,other;double time=0;};
// Each robot keeps source records and reversible local correspondences.
class TargetRegistry {
public:
 std::string self="uav0";std::size_t per_robot_limit=128,total_limit=256;
 double surface_margin=.35,max_gate=.8,match_cost_limit=.6,current_ttl=2.;
 std::map<TargetKey,TargetRecord> records;std::vector<TargetAlias> aliases;
 // records is the mission ledger, not the admission cache. It retains source
 // keys, revisions, confirmations, anchors and relationship endpoints. Only
 // recent sources consume active capacity; archived histories release storage.
 std::set<TargetKey> active_sources;
 std::map<std::string,TargetAlignment> peer_alignments;
 std::map<std::string,std::pair<TargetKey,double>> observation_origins;
 std::vector<TargetEvent> events;TargetStats stats;
 std::set<std::pair<TargetKey,TargetKey>> cannot_link;
 std::map<std::string,double> last_sync;std::set<std::string> sync_ids;std::vector<std::string> sync_order;
 std::uint64_t evidence_epoch=0; // accepted input/topology changes, not timer/duplicate packets
 explicit TargetRegistry(std::string own="uav0"):self(std::move(own)){}
 static std::pair<TargetKey,TargetKey> pair(TargetKey a,TargetKey b){if(b<a)std::swap(a,b);return {a,b};}
 std::set<TargetKey> component(const TargetKey&k)const{
  std::set<TargetKey> out{k};bool changed=true;
  while(changed){changed=false;for(const auto&e:aliases)if(out.count(e.a)||out.count(e.b)){
   changed|=out.insert(e.a).second;changed|=out.insert(e.b).second;}}
  return out;
 }
 TargetKey representative(const TargetKey&k)const{return *component(k).begin();}
 bool compatibleFrames(const TargetObservation&a,const TargetObservation&b)const{
  return a.frame==b.frame&&a.frame_version==b.frame_version&&!a.frame_version.empty()&&
   a.clock_domain==b.clock_domain&&(a.map_epoch.empty()?a.frame_version:a.map_epoch)==(b.map_epoch.empty()?b.frame_version:b.map_epoch)&&
   (a.key.robot==b.key.robot||((a.key.robot==self||a.alignment_verified)&&(b.key.robot==self||b.alignment_verified)));
 }
 bool blocked(const std::set<TargetKey>&a,const std::set<TargetKey>&b)const{
  for(const auto&x:a)for(const auto&y:b)if(x!=y&&cannot_link.count(pair(x,y)))return true;
  return false;
 }
 bool valid(const TargetObservation&o,double now)const{
  if(o.exclusions.size()>128)return false;
  std::set<std::uint64_t> unique;for(auto c:o.exclusions)if(c==0||c>1000000000||c==o.key.counter||!unique.insert(c).second)return false;
  if(o.key.robot.empty()||o.key.robot.size()>48||o.key.session.empty()||o.key.session.size()>64||
    o.key.counter==0||o.key.counter>1000000000||o.sequence==0)return false;
  if(!std::isfinite(now)||!std::isfinite(o.first)||!std::isfinite(o.capture)||
    o.first>o.capture||o.capture>now+.05||o.first<0)return false;
  if(o.frame.size()>64||o.frame_version.empty()||o.frame_version.size()>96||
    o.clock_domain.empty()||o.clock_domain.size()>96||o.observation_id.size()>256||o.map_epoch.size()>96||(!o.map_epoch.empty()&&o.map_epoch!=o.frame_version)||
    !std::isfinite(o.confidence)||o.confidence<0||o.confidence>1)return false;
  if(o.visual!="person_candidate"&&o.visual!="visually_confirmed")return false;
  if(o.has_surface_quality&&(o.support_pixels<1||o.calibration_id.empty()||o.calibration_id.size()>160||
     !std::isfinite(o.valid_depth_fraction)||o.valid_depth_fraction<0||o.valid_depth_fraction>1||
     !std::isfinite(o.component_fraction)||o.component_fraction<0||o.component_fraction>1||
     !std::isfinite(o.depth_span_m)||o.depth_span_m<0||!std::isfinite(o.rgb_depth_skew_s)||o.rgb_depth_skew_s<0||o.rgb_depth_skew_s>.05))return false;
  if(o.localization!="unknown"&&o.localization!="unreliable"&&o.localization!="reliable"&&o.localization!="expired")return false;
  if((o.localization=="reliable")!=o.has_position)return false;
  if(o.has_position&&(!targetFinite(o.position)||
    !std::isfinite(o.radius)||o.radius<.05||o.radius>.4))return false;
  if(o.identity_input!="unknown"&&o.identity_input!="pending"&&o.identity_input!="completed"&&o.identity_input!="unavailable")return false;
  if(o.identity_input!="unknown"&&(!std::isfinite(o.identity_deadline)||o.identity_deadline<=o.capture||o.identity_deadline>o.capture+.75+1e-9))return false;
  if(o.identity_input=="pending"&&(o.has_identity_surface||o.has_position))return false;
  if(o.has_identity_surface&&(!targetFinite(o.identity_surface)||!std::isfinite(o.identity_radius)||
    o.identity_radius<.05||o.identity_radius>.4))return false;
  if(o.has_bearing&&(!targetFinite(o.origin)||!targetFinite(o.direction)||
    std::abs(targetDistance(o.direction,{0,0,0})-1)>1e-5||!std::isfinite(o.half_angle)||
    o.half_angle<.01||o.half_angle>1.5))return false;
  return true;
 }
 bool capacity(const TargetKey&k)const{
  if(!supportedRobotDomain(k.robot))return false;
  if(active_sources.count(k))return true;
  if(active_sources.size()>=total_limit)return false;
  std::size_t count=0;
  for(auto it=active_sources.lower_bound({k.robot,{},0});it!=active_sources.end()&&it->robot==k.robot;++it)
   ++count;
  return count<per_robot_limit;
 }
 void retire(double now){
  bool invalidated=false;
  for(auto it=records.begin();it!=records.end();){
   const auto&robot=it->first.robot;
   const auto end=records.lower_bound(afterRobot(robot));
   if(robot==self){it=end;continue;}
   auto proof=peer_alignments.find(robot);
   const bool aligned=proof!=peer_alignments.end()&&proof->second.valid(now);
   for(;it!=end;++it){
    auto&r=it->second;
    if(r.latest.alignment_verified&&(!aligned||proof->second.id!=r.latest.alignment_id)){
     r.has_anchor=false;r.latest.has_position=false;r.latest.has_bearing=false;r.latest.localization="expired";
     r.latest.alignment_verified=false;r.history.clear();invalidated=true;++evidence_epoch;
    }
   }
  }
  if(invalidated)revalidate(now);
  for(auto it=active_sources.begin();it!=active_sources.end();){
   auto&r=records.at(*it);
   if(now-r.last<=current_ttl){++it;continue;}
   r.history.clear();r.history.shrink_to_fit();
   it=active_sources.erase(it);++evidence_epoch;
  }
 }
 bool material(const TargetRecord&r,const TargetObservation&o)const{
  const auto&l=r.material_reference;
  return (o.visual=="visually_confirmed"&&!r.confirmed)||l.localization!=o.localization||
   l.has_position!=o.has_position||l.frame!=o.frame||l.frame_version!=o.frame_version||
   l.shared_aligned!=o.shared_aligned||
   l.clock_domain!=o.clock_domain||l.calibration_id!=o.calibration_id||
   (o.has_position&&(targetDistance(l.position,o.position)>=.2||o.radius<l.radius*.8));
 }
 TargetUpdate ingest(TargetObservation o,double now,bool peer=false){
  TargetUpdate out;
  const double source_capture=o.capture;
  if(peer&&o.key.robot!=self){
   if(!valid(o,o.capture)){out.reason="invalid_source_observation_contract";return out;}
   auto proof=peer_alignments.find(o.key.robot);
   if(proof==peer_alignments.end()||!proof->second.valid(now)){out.reason="peer_alignment_or_clock_proof_required";return out;}
   const auto&a=proof->second;
   if(o.frame!="map"||o.frame_version!=a.source_version||o.clock_domain!=a.source_clock||(!o.map_epoch.empty()&&o.map_epoch!=a.source_version)){
    out.reason="peer_coordinate_or_clock_epoch_mismatch";return out;}
   if(o.has_position){
    o.radius+=a.position_error_m+a.rotation_error_rad*targetDistance(o.position,{0,0,0});
    o.position=a.apply(o.position);
    if(o.radius>.4){o.has_position=false;o.localization="unreliable";}
   }
   // Foreign rays cannot authorize local navigation. Original source keys and
   // sequence/revision remain unchanged, so forwarded evidence is idempotent.
   o.has_bearing=false;o.first+=a.clock_offset_s;o.capture+=a.clock_offset_s;
   o.frame_version=a.target_version;o.map_epoch=a.target_version;o.clock_domain=a.target_clock;
   o.alignment_verified=true;o.alignment_id=a.id;
  }
  if(!valid(o,now)||(peer&&o.revision==0)){out.reason="invalid_observation_contract";return out;}
  bool retired=false;
  return ingestValidated(std::move(o),now,peer,source_capture,retired);
 }
 TargetBatchResult ingestLocalFrame(std::vector<TargetObservation> observations,double now){
  if(observations.size()>128)throw std::runtime_error("too_many_frame_observations");
  std::set<TargetKey> seen;
  for(const auto&o:observations)
   if(o.key.robot!=self||!valid(o,now)||!seen.insert(o.key).second)
    throw std::runtime_error("invalid_local_frame_batch");
  for(const auto&o:observations)
   if(o.key.session!=observations.front().key.session||o.capture!=observations.front().capture)
    throw std::runtime_error("mixed_camera_frame_batch");

  TargetBatchResult result;
  std::vector<TargetKey> coexist_keys;
  bool retired=false;
  for(auto&o:observations){
   const auto key=o.key;
   const double capture=o.capture;
   auto update=ingestValidated(std::move(o),now,false,capture,retired);
   if(update.accepted)result.accepted.push_back(key);
   else events.push_back({"observation_rejected",update.reason,key,{},now});
   if(update.material)result.changed.push_back(key);
   auto it=records.find(key);
   if(it!=records.end()&&it->second.latest.capture==capture)coexist_keys.push_back(key);
  }
  if(!retired)retire(now);
  associateCurrent(coexist_keys,now);
  result.reason="local_batch_processed";
  return result;
 }
private:
 // TargetKey sorts by robot first. Skip all ledger entries of the same robot.
 static TargetKey afterRobot(const std::string& robot){return {robot+'\0',{},0};}
 bool supportedRobotDomain(const std::string& robot)const{
  const std::string* peer=robot==self?nullptr:&robot;
  for(auto it=records.begin();it!=records.end();it=records.lower_bound(afterRobot(it->first.robot))){
   const auto&name=it->first.robot;
   if(name==self)continue;
   if(peer&&name!=*peer)return false;
   peer=&name;
  }
  return true;
 }
 TargetUpdate ingestValidated(TargetObservation o,double now,bool peer,double source_capture,bool& retired){
  TargetUpdate out;
  if(!o.observation_id.empty()){
   auto original=observation_origins.find(o.observation_id);
   if(original!=observation_origins.end()&&(original->second.first!=o.key||original->second.second!=source_capture)){
    out.reason="observation_origin_conflict";return out;
   }
  }
  if(peer&&(o.has_identity_surface||o.identity_input!="unknown")){out.reason="local_identity_surface_not_peer_authority";return out;}
  if(peer&&o.key.robot==self){out.reason="own_source_echo_ignored";return out;}
  // Preserve the first eligible observation's retirement point. Earlier
  // origin conflicts still report before retirement events.
  if(!retired){retire(now);retired=true;}
  const bool current=now-o.capture<=current_ttl;
  if(current&&!capacity(o.key)){out.reason="bounded_source_cache_full";return out;}
  auto it=records.find(o.key);
  if(it==records.end()){
   // Even archived peer facts obey the two-robot protocol boundary.
   if(!current&&!supportedRobotDomain(o.key.robot)){out.reason="unsupported_robot_domain";return out;}
   TargetRecord r;r.exclusions.insert(o.exclusions.begin(),o.exclusions.end());r.first=o.first;r.last=o.capture;r.latest=o;r.material_reference=o;
   r.confirmed=o.visual=="visually_confirmed";r.revision=peer?o.revision:1;
   if(o.has_position){r.has_anchor=true;r.anchor=o.position;r.anchor_radius=o.radius;r.anchor_time=o.capture;}
   if(current)r.history.push_back(o);records.emplace(o.key,r);if(current)active_sources.insert(o.key);
   if(!o.observation_id.empty())observation_origins[o.observation_id]={o.key,source_capture};
   ++evidence_epoch;out.accepted=out.material=true;
   out.reason="new_source_record";events.push_back({"record",out.reason,o.key,{},now});return out;
  }
  auto&r=it->second;
  const bool new_alignment=peer&&(!r.latest.alignment_verified||o.alignment_id!=r.latest.alignment_id);
  if(peer?(o.revision<r.revision||(o.revision==r.revision&&!new_alignment)):o.sequence<=r.latest.sequence){
   out.reason=(peer?o.revision==r.revision:o.sequence==r.latest.sequence)?"duplicate_ignored":"out_of_order_ignored";
   return out;
  }
  const bool realigned=peer&&o.alignment_id!=r.latest.alignment_id;
  if(!realigned&&(o.capture<r.last||o.first!=r.first)){
   out.reason="capture_regression_or_first_epoch_changed";return out;
  }
  // Identity-input readiness is monotone within an exact capture/frame. Still
  // accept newer visual confirmation/bearing; never let a late pending packet
  // erase completed range evidence or prolong its deadline.
  if(o.capture==r.latest.capture&&compatibleFrames(o,r.latest)){
   if(r.latest.identity_input!="unknown"){
    if(o.identity_input!="unknown")o.identity_deadline=std::min(o.identity_deadline,r.latest.identity_deadline);
    if((r.latest.identity_input=="completed"||r.latest.identity_input=="unavailable")&&
       (o.identity_input=="pending"||o.identity_input=="unknown")){
     o.identity_input=r.latest.identity_input;o.identity_deadline=r.latest.identity_deadline;
     o.has_identity_surface=r.latest.has_identity_surface;o.identity_surface=r.latest.identity_surface;o.identity_radius=r.latest.identity_radius;
    }
   }
  }
  auto exclusions=r.exclusions;exclusions.insert(o.exclusions.begin(),o.exclusions.end());
  if(exclusions.size()>128){out.reason="bounded_exclusion_cache_full";return out;}
  out.material=material(r,o)||exclusions!=r.exclusions;r.exclusions=std::move(exclusions);
  if(out.material||peer)r.material_reference=o;
  if(realigned){r.first=o.first;r.history.clear();}
  if(realigned||o.frame!=r.latest.frame||o.frame_version!=r.latest.frame_version||o.clock_domain!=r.latest.clock_domain)r.has_anchor=false;
  r.latest=o;r.last=o.capture;r.confirmed|=o.visual=="visually_confirmed";
  if(r.confirmed)r.latest.visual="visually_confirmed";
  if(peer)r.revision=o.revision;else if(out.material)++r.revision;
  if(o.has_position){r.has_anchor=true;r.anchor=o.position;r.anchor_radius=o.radius;r.anchor_time=o.capture;}
  if(current){
   active_sources.insert(o.key);
   if(!r.history.empty()&&r.history.back().capture==o.capture)r.history.back()=o;else r.history.push_back(o);if(r.history.size()>32)r.history.erase(r.history.begin());
  }
  ++evidence_epoch;out.accepted=true;out.reason=out.material?"material_state_change":"latest_observation_only";
  if(!o.observation_id.empty())observation_origins[o.observation_id]={o.key,source_capture};
  if(out.material)events.push_back({"record",out.reason,o.key,{},now});
  return out;
 }
public:
 void coexist(const std::vector<TargetKey>&keys,double now){
  for(std::size_t i=0;i<keys.size();++i)for(std::size_t j=i+1;j<keys.size();++j){
   if(keys[i].robot!=keys[j].robot||keys[i].session!=keys[j].session)continue;
   if(std::abs(records.at(keys[i]).latest.capture-records.at(keys[j]).latest.capture)>1e-6)continue;
   if(cannot_link.insert(pair(keys[i],keys[j])).second)++evidence_epoch;
   for(const auto&v:std::vector<std::pair<TargetKey,TargetKey>>{{keys[i],keys[j]},{keys[j],keys[i]}}){
    auto&r=records.at(v.first);if(r.exclusions.insert(v.second.counter).second&&v.first.robot==self){
     ++r.revision;r.latest.exclusions={r.exclusions.begin(),r.exclusions.end()};r.material_reference=r.latest;
     events.push_back({"record","new_camera_coexistence_constraint",v.first,v.second,now});
    }
   }
   if(!aliases.empty()&&component(keys[i]).count(keys[j])){
    auto size=aliases.size();
    aliases.erase(std::remove_if(aliases.begin(),aliases.end(),[&](const TargetAlias&e){
     return e.a==keys[j]||e.b==keys[j];}),aliases.end());
    if(size!=aliases.size())++evidence_epoch;
    if(size!=aliases.size())events.push_back({"unlink","same_camera_coexistence",keys[j],keys[i],now});
   }
  }
 }
 void revalidate(double now){
  aliases.erase(std::remove_if(aliases.begin(),aliases.end(),[&](TargetAlias&e){
   const auto&a=records.at(e.a);const auto&b=records.at(e.b);
   bool bad=!compatibleFrames(a.latest,b.latest);
   if(!bad&&a.latest.has_position&&b.latest.has_position)
    bad=targetDistance(a.latest.position,b.latest.position)>std::min(max_gate,a.latest.radius+b.latest.radius+surface_margin);
   if(!bad){
    const auto group=component(e.a);
    bad=blocked(group,group);
    if(!bad&&e.surface_sequence&&a.latest.has_identity_surface)
     bad=std::none_of(group.begin(),group.end(),[&](const TargetKey&k){
      const auto&anchor=records.at(k);
      return k!=e.a&&anchor.has_anchor&&compatibleFrames(a.latest,anchor.latest)&&
       targetDistance(a.latest.identity_surface,anchor.anchor)<=std::min(max_gate,a.latest.identity_radius+anchor.anchor_radius+surface_margin);
     });
   }
   if(bad)++evidence_epoch;
   if(bad)events.push_back({"unlink","frame_coexistence_or_geometry_conflict",e.a,e.b,now});else{e.a_revision=a.revision;e.b_revision=b.revision;e.frame_version=a.latest.frame_version;}return bad;
  }),aliases.end());
 }
 const TargetRecord*anchorFor(const std::set<TargetKey>&g,const TargetObservation&o,double*cost=nullptr)const{
  const TargetRecord*best=nullptr;double best_distance=INFINITY;
  for(const auto&k:g){const auto&r=records.at(k);
   // A component can retain several compatible measured surface anchors.
   if(!r.has_anchor||!compatibleFrames(o,r.latest))continue;
   const double distance=targetDistance(o.position,r.anchor);
   if(distance<=std::min(max_gate,o.radius+r.anchor_radius+surface_margin)&&(!best||distance<best_distance)){
    best=&r;best_distance=distance;
   }
  }
  if(cost)*cost=best_distance;
  return best;
 }
 // Multiple actual range measurements must agree with ONE historical robust
 // anchor. Reuse navigation's 3 captures/.4s/2s support contract; uncertainty
 // overlap is stricter than the existing spatial gate (no surface margin).
 // No bearing-only merge and no new position/anchor is inferred here.
 const TargetRecord*identityAnchorFor(const std::set<TargetKey>&g,const TargetRecord&r,double&cost)const{
  const auto&o=r.latest;if(!o.has_identity_surface||o.confidence<.50)return nullptr;
  cost=INFINITY;
  const auto first_sample=r.history.rbegin();
  const auto past_support=std::find_if(first_sample,r.history.rend(),[&](const TargetObservation&s){
   return o.capture-s.capture>2.||!s.has_identity_surface||s.confidence<.50||!compatibleFrames(s,o);
  });
  if(past_support-first_sample<3||o.capture-(past_support-1)->capture+1e-9<.4)return nullptr;
  const TargetRecord*best=nullptr;
  for(const auto&k:g){const auto&a=records.at(k);
   if(!a.has_anchor||!compatibleFrames(o,a.latest))continue;
   int count=0;double first=o.capture,worst=0;
   for(auto i=first_sample;i!=past_support;++i){const auto&s=*i;
    double distance=targetDistance(s.identity_surface,a.anchor);
    if(!(distance<match_cost_limit)||distance>std::min(max_gate,s.identity_radius+a.anchor_radius))break;
    ++count;first=s.capture;worst=std::max(worst,distance);
   }
   if(count>=3&&o.capture-first+1e-9>=.4&&worst<cost){best=&a;cost=worst;}
  }
  return best;
 }
 // Scheduling hint ONLY: two preceding actual surfaces may justify waiting
 // for the third exact capture. No alias/anchor/confirmation is created here.
 // Compute once per input epoch, never on every FSM tick. All compatible
 // components compete, including unconfirmed anchors and other recent sources.
 std::map<TargetKey,TargetKey> identityWaitTargets(double now,std::size_t&checks)const{
  checks=0;
  bool pending=false;for(const auto&kv:records){const auto&o=kv.second.latest;
   if(o.key.robot==self&&o.identity_input=="pending"&&now>=o.capture&&now<o.identity_deadline&&kv.second.history.size()>=3){pending=true;break;}
  }
  if(!pending)return {}; // common path: no spatial work for ordinary/legacy input
  std::map<TargetKey,std::set<TargetKey>> groups;
  std::map<TargetKey,TargetKey> roots_by_key;
  for(const auto&kv:records){if(roots_by_key.count(kv.first))continue;
   auto g=component(kv.first);auto root=*g.begin();groups[root]=g;
   for(const auto&k:g)roots_by_key[k]=root;
  }
  std::map<TargetKey,std::vector<TargetKey>> choices,competitors;
  std::set<TargetKey> eligible;
  for(const auto&kv:records){const auto&k=kv.first;const auto&r=kv.second;const auto&o=r.latest;
   if(k.robot!=self||groups.at(roots_by_key.at(k)).size()!=1||r.confirmed||now<o.capture||now-o.capture>current_ttl)continue;
   // Expired input cannot compete as a current observation; its historical
   // anchor/history remain intact. Legacy packets keep the original TTL.
   if(o.identity_input!="unknown"&&now>=o.identity_deadline)continue;
   bool pending=o.identity_input=="pending"&&now<o.identity_deadline;
   std::vector<const TargetObservation*> samples;
   for(auto i=r.history.rbegin();i!=r.history.rend();++i){
    if(pending&&i->capture==o.capture)continue;
    if(o.capture-i->capture>current_ttl||!i->has_identity_surface||i->confidence<.50||!compatibleFrames(*i,o))break;
    samples.push_back(&*i);if(samples.size()==2)break;
   }
   if(samples.empty())continue;
   const bool ready=pending&&samples.size()==2&&o.confidence>=.50&&o.capture-samples.back()->capture+1e-9>=.4;
   if(ready)eligible.insert(k);
   for(const auto&group:groups){if(group.second.count(k)||blocked({k},group.second))continue;
    bool matches=false;
    for(const auto&other:group.second){const auto&a=records.at(other);
     if(!a.has_anchor||!compatibleFrames(o,a.latest))continue;
     bool fits=true;for(const auto*sample:samples){++checks;
      double d=targetDistance(sample->identity_surface,a.anchor);
      if(d>=match_cost_limit||d>std::min(max_gate,sample->identity_radius+a.anchor_radius)){fits=false;break;}
     }
     if(fits){matches=true;break;}
    }
    if(matches){choices[k].push_back(group.first);competitors[group.first].push_back(k);}
   }
  }
  std::map<TargetKey,TargetKey> result;
  for(const auto&k:eligible){const auto&v=choices[k];
   if(v.size()==1&&competitors[v.front()].size()==1&&groupConfirmed(v.front()))result[k]=v.front();
  }
  return result;
 }
 void associate(const std::vector<TargetKey>&keys,double now){
  retire(now);
  associateCurrent(keys,now);
 }
private:
 void associateCurrent(const std::vector<TargetKey>&keys,double now){
  const auto start=std::chrono::steady_clock::now();stats={};
  stats.detections=keys.size();stats.records=records.size();
  for(const auto&kv:records)for(auto counter:kv.second.exclusions){
   TargetKey other{kv.first.robot,kv.first.session,counter};
   if(records.count(other))cannot_link.insert(pair(kv.first,other));
  }
  coexist(keys,now);revalidate(now);
  struct Choice{TargetKey to;double cost=0;bool surface=false;std::size_t count=0;};
  std::map<TargetKey,Choice> from;
  std::map<TargetKey,std::size_t> to;
  std::set<TargetKey>active(keys.begin(),keys.end());
  // Sparse geometric proposals over retained anchors. Frame/version are part
  // of the cell key; compatibility and cannot-link still gate every edge.
  std::map<TargetKey,std::set<TargetKey>> groups;std::map<TargetKey,TargetKey> root_of;
  for(const auto&kv:records)if(!root_of.count(kv.first)){
   auto g=component(kv.first);auto root=*g.begin();groups[root]=g;for(const auto&member:g)root_of[member]=root;
  }
  using Cell=std::tuple<std::string,std::string,double,double,double>;
  std::map<Cell,std::set<TargetKey>> grid;
  const double cell_size=std::max(max_gate,.001);
  for(const auto&kv:records)if(kv.second.has_anchor){const auto&r=kv.second;
   grid[{r.latest.frame,r.latest.frame_version,std::floor(r.anchor[0]/cell_size),std::floor(r.anchor[1]/cell_size),std::floor(r.anchor[2]/cell_size)}].insert(root_of.at(kv.first));
  }
  for(const auto&k:keys){auto&r=records.at(k);
   if(groups.at(root_of.at(k)).size()>1){r.identity="associated";r.association_reason="existing_reversible_component";continue;}
   if((!r.latest.has_position&&!r.latest.has_identity_surface)||now<r.latest.capture||now-r.latest.capture>current_ttl){
    r.identity="unresolved";r.association_reason="no_current_reliable_position";continue;}
   std::set<TargetKey>candidates;auto point=r.latest.has_position?r.latest.position:r.latest.identity_surface;
   for(auto&coordinate:point)coordinate=std::floor(coordinate/cell_size);
   for(int x=-1;x<=1;++x)for(int y=-1;y<=1;++y)for(int z=-1;z<=1;++z){
    auto cell=grid.find({r.latest.frame,r.latest.frame_version,point[0]+x,point[1]+y,point[2]+z});
    if(cell!=grid.end())candidates.insert(cell->second.begin(),cell->second.end());
   }
   for(const auto&root:candidates){const auto&g=groups.at(root);
    if(g.count(k))continue;++stats.scans;
    if(blocked({k},g))continue;
    if(g.size()==1&&active.count(root)&&!(root<k))continue;
    double distance=INFINITY;
    const bool surface=!r.latest.has_position;
    if(surface){
     if(!identityAnchorFor(g,r,distance))continue;
    }else if(!anchorFor(g,r.latest,&distance)||!(distance<match_cost_limit))continue;
    // Only selectable edges can compete, even when they cannot eventually link.
    auto&choice=from[k];
    if(choice.count==0)choice={root,distance,surface,0};
    ++choice.count;++to[root];++stats.edges;
   }
  }
  for(const auto&k:keys){auto&r=records.at(k);if(component(k).size()>1)continue;
   auto it=from.find(k);if(it==from.end()){r.association_reason="no_compatible_spatial_candidate";++stats.unmatched;continue;}
   const auto&e=it->second;
   if(e.count!=1||to.at(e.to)!=1){
    r.identity="ambiguous";r.association_reason="competing_or_unmatched_cost_preferred";++stats.ambiguous;continue;}
   const auto ga=component(k),gb=component(e.to);
   if(blocked(ga,gb)){r.identity="ambiguous";r.association_reason="coexistence_conflict";++stats.ambiguous;continue;}
   ++evidence_epoch;aliases.push_back({k,e.to,r.revision,records.at(e.to).revision,r.latest.frame_version,e.cost,e.surface});
   r.identity="associated";r.association_reason=e.surface?"unique_registered_surface_sequence_to_anchor":"unique_spatial_component_with_surface_margin";
   events.push_back({"link",r.association_reason,k,e.to,now});++stats.linked;
  }
  stats.elapsed_us=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count();
 }
public:
 bool observeVisibility(const TargetVisibility&v,double now){
  auto it=records.find(v.key);
  if(it==records.end()||v.key.robot!=self||!std::isfinite(now)||!std::isfinite(v.capture)||
     v.capture<0||now<v.capture||now-v.capture>current_ttl||!std::isfinite(v.anchor_capture)||
     v.scope!="visible_surface_region"||v.model_id.empty()||v.model_id.size()>128||
     v.reason.size()>128||!targetFinite(v.center)||
     (v.state!="clear"&&v.state!="occluded"&&v.state!="unknown"&&v.state!="outside"))return false;
  auto&r=it->second;
  if(!r.has_anchor||v.capture<r.last||v.capture<=r.visibility_capture||v.frame_version!=r.latest.frame_version||
     std::abs(v.anchor_capture-r.anchor_time)>1e-8||targetDistance(v.center,r.anchor)>1e-6)return false;
  for(double extent:v.extent)if(extent<r.anchor_radius||!std::isfinite(extent))return false;
  const auto group=component(v.key);
  const bool detected=std::any_of(group.begin(),group.end(),[&](const TargetKey&k){
   return std::abs(records.at(k).last-v.capture)<=1e-8;
  });
  const std::string state=detected?"observed":v.state;
  if(state!=r.visibility)++r.visibility_revision;
  r.visibility=state;r.visibility_capture=v.capture;r.visibility_anchor_capture=v.anchor_capture;
  if(detected)r.visible_region_misses=0;else if(v.state=="clear")++r.visible_region_misses;
  // Region misses never clear confirmed presence, identity or static anchors.
  return true;
 }
 std::vector<TargetKey>roots()const{
  std::set<TargetKey>r;for(const auto&kv:records)r.insert(representative(kv.first));return {r.begin(),r.end()};
 }
 const TargetRecord*currentPosition(const TargetKey&k,double now)const{
  const TargetRecord*best=nullptr;for(const auto&i:component(k)){const auto&r=records.at(i);
   if(r.latest.has_position&&now>=r.last&&now-r.last<=current_ttl&&(!best||r.last>best->last))best=&r;}return best;
 }
 bool groupConfirmed(const TargetKey&k)const{for(const auto&i:component(k))if(records.at(i).confirmed)return true;return false;}
 // A static person's qualified map position survives the live observation lease.
 // Resolve aliases on each update so an unrelated surface cannot finish the review.
 const TargetRecord* pendingDepthReview()const{
  for(const auto& [key,record]:records){
   if(key.robot!=self||!record.confirmed||record.has_anchor)continue;
   const auto members=component(key);
   if(std::none_of(members.begin(),members.end(),[&](const TargetKey& member){
     return records.at(member).has_anchor;
   }))return &record;
  }
  return nullptr;
 }
 double firstDiscovery(const TargetKey&k)const{
  double first=INFINITY;for(const auto&i:component(k))first=std::min(first,records.at(i).first);return first;
 }
 std::vector<TargetKey>sync(const std::string&peer,const std::string&request,std::uint64_t after,double now){
  std::vector<TargetKey>out;if(peer==self||peer.empty()||request.empty()||request.size()>96)return out;
  std::string id=peer+":"+request;
  if(last_sync.size()>=1&&!last_sync.count(peer))return out;
  if(sync_ids.count(id)||(last_sync.count(peer)&&now-last_sync.at(peer)<1.))return out;
  last_sync[peer]=now;sync_ids.insert(id);sync_order.push_back(id);
  if(sync_order.size()>64){sync_ids.erase(sync_order.front());sync_order.erase(sync_order.begin());}
  std::size_t index=0;
  for(const auto&kv:records)if(kv.first.robot==self){if(index++<after)continue;
   out.push_back(kv.first);if(out.size()==8)break;}
  return out;
 }
};

// Read-only count uncertainty over accepted source/alias/cannot-link evidence.
// Bounds concern observed confirmed hypotheses, conditional on evidence validity.
// They never certify ground-truth people or include unseen people.
struct TargetCountBounds {
 std::size_t lower=0,upper=0,pair_checks=0;bool pair_budget_exhausted=false,inconsistent=false;
 std::vector<TargetKey>witness;
};
inline TargetCountBounds targetCountBounds(const TargetRegistry&r){
 TargetCountBounds out;std::vector<TargetKey>roots;std::vector<std::set<TargetKey>>groups;
 for(const auto&k:r.roots())if(r.groupConfirmed(k)){roots.push_back(k);groups.push_back(r.component(k));}
 out.upper=roots.size();if(roots.empty())return out;
 for(const auto&g:groups)if(r.blocked(g,g)){out.inconsistent=true;return out;}
 const std::size_t n=roots.size(),budget=16384;
 std::vector<std::vector<bool>>adj(n,std::vector<bool>(n,false));std::vector<std::size_t>degree(n,0),order;
 for(std::size_t i=0;i<n;++i){order.push_back(i);for(std::size_t j=i+1;j<n;++j){
  if(out.pair_checks>=budget){out.pair_budget_exhausted=true;continue;}
  ++out.pair_checks;if(r.blocked(groups[i],groups[j])){adj[i][j]=adj[j][i]=true;++degree[i];++degree[j];}
 }}
 std::stable_sort(order.begin(),order.end(),[&](std::size_t a,std::size_t b){return degree[a]>degree[b];});
 std::vector<std::size_t>clique;
 for(auto i:order)if(std::all_of(clique.begin(),clique.end(),[&](std::size_t j){return adj[i][j];})){clique.push_back(i);out.witness.push_back(roots[i]);}
 out.lower=clique.size();return out;
}
}
