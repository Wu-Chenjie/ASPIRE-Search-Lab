#pragma once
#include "target_registry.hpp"
namespace racer_task {
// Component membership is supplied by the native registry. Reservations retain
// their identity through merges/splits and cannot be refunded by cancellation.
class StaticReviewBudget {
public:
 enum class Purpose {ConfirmPresence,DisambiguateIdentity,ImprovePosition};
 struct Reservation {std::set<TargetKey> members;TargetKey source;std::string version;double time,seconds,distance;};
 std::size_t component_limit=3,mission_limit=32;
 double cooldown_s=.25,component_seconds=30.,component_distance_m=6.;
 std::vector<Reservation> reservations;
 void reconcile(const std::set<TargetKey>&members){
  for(auto&r:reservations)if(overlap(r.members,members))r.members.insert(members.begin(),members.end());
 }
 std::string eligible(const std::set<TargetKey>&members,const TargetKey&source,const std::string&version,
                      double now,double capture,double ttl,Purpose purpose,bool confirmed,
                      bool identity_ambiguous=false,double seconds=0.,double distance=0.)const{
  if(!std::isfinite(now)||!std::isfinite(capture)||now<capture||now-capture>ttl)return "expired_evidence";
  if(!std::isfinite(seconds)||!std::isfinite(distance)||seconds<0||distance<0)return "invalid_estimate";
  if(purpose==Purpose::ImprovePosition)return "precision_only_review_disabled";
  if(confirmed&&purpose==Purpose::ConfirmPresence)return "presence_already_confirmed";
  if(purpose==Purpose::DisambiguateIdentity&&!identity_ambiguous)return "no_identity_question";
  if(reservations.size()>=mission_limit)return "mission_attempt_limit";
  std::size_t count=0;double used_seconds=0,used_distance=0,last=-INFINITY;
  for(const auto&r:reservations)if(overlap(r.members,members)){
   ++count;used_seconds+=r.seconds;used_distance+=r.distance;last=std::max(last,r.time);
   if(r.source==source&&r.version==version)return "evidence_already_reserved";
  }
  if(count>=component_limit)return "component_attempt_limit";
  if(now<last+cooldown_s)return "component_cooldown";
  if(used_seconds+seconds>component_seconds||used_distance+distance>component_distance_m)return "component_route_budget";
  return {};
 }
 void reserve(const std::set<TargetKey>&members,const TargetKey&source,const std::string&version,
              double now,double seconds=0.,double distance=0.){
  reservations.push_back({members,source,version,now,seconds,distance});
 }
 bool route(const TargetKey&source,const std::string&version,double seconds,double distance){
  if(!std::isfinite(seconds)||!std::isfinite(distance)||seconds<0||distance<0)return false;
  for(auto&r:reservations)if(r.source==source&&r.version==version){
   double used_seconds=0,used_distance=0;
   for(const auto&other:reservations)if(&other!=&r&&overlap(r.members,other.members)){
    used_seconds+=other.seconds;used_distance+=other.distance;
   }
   seconds=std::max(seconds,r.seconds);distance=std::max(distance,r.distance);
   if(used_seconds+seconds>component_seconds||used_distance+distance>component_distance_m)return false;
   r.seconds=seconds;r.distance=distance;return true;
  }
  return false;
 }
 static bool overlap(const std::set<TargetKey>&a,const std::set<TargetKey>&b){
  const bool a_is_smaller=a.size()<=b.size();
  const auto& smaller=a_is_smaller?a:b;
  const auto& larger=a_is_smaller?b:a;
  for(const auto&k:smaller)if(larger.count(k))return true;
  return false;
 }
};
}
