#pragma once
#include "static_guidance_events.hpp"
#include "static_cancellation.hpp"
#include <map>
namespace racer_task {
// Accepted execution owns no solver slot. Keep unresolved identities bounded
// and separate, including an old task whose successor is already executing.
class StaticExecutionMonitor {
 struct Entry {StaticGuidanceEvents::Work work;StaticCancellation cancel;std::optional<double> observation_until;};
 std::map<std::uint64_t,Entry> entries_;
 int active_=-1;
 void collectCancelled(){
  for(auto i=entries_.begin();i!=entries_.end();)
   if(i->second.cancel.requested&&i->second.cancel.released(active_,false,false))i=entries_.erase(i);else ++i;
 }
public:
 bool room()const{return entries_.size()<256;}
 std::size_t size()const{return entries_.size();}
 std::set<TargetKey> reservedSources()const{
  std::set<TargetKey> sources;
  for(const auto& item:entries_)
   sources.insert(item.second.work.members.begin(),item.second.work.members.end());
  return sources;
 }
 bool accept(const StaticGuidanceEvents::Work& work,const StaticCancellation& cancel,
             int active,int applied,bool rejected){
  if(cancel.identity.size()!=6||active!=applied||active!=cancel.identity[3]||rejected||
     cancel.identity[2]!=double(work.token))return false;
  auto prior=entries_.find(work.token);
  if(prior!=entries_.end())return prior->second.cancel.identity==cancel.identity;
  if(!room())return false;
  entries_.emplace(work.token,Entry{work,cancel,{}});return true;
 }
 void feedback(int active,bool done,double now=0.,double observation_s=0.){
  active_=active;
  for(auto& item:entries_){auto& entry=item.second;
   if(active==entry.cancel.identity[3]&&done&&!entry.observation_until)
    entry.observation_until=now+observation_s;
  }
  collectCancelled();
 }
 bool acknowledge(const std::vector<double>& data){
  bool accepted=false;
  for(auto& item:entries_)accepted=item.second.cancel.acknowledge(data)||accepted;
  collectCancelled();return accepted;
 }
 // Process any confirmation in the inbox before closing a normally observed view.
 // The window starts once at certified completion and cannot be refreshed by feedback.
 template<class Send> bool tick(const StaticGuidanceEvents& inbox,double now,double interval,Send send){
  bool holding=false;
  for(auto i=entries_.begin();i!=entries_.end();){
   auto& entry=i->second;
   const bool active=active_==entry.cancel.identity[3];
   if(!active||entry.cancel.requested||!inbox.refreshExecuting(entry.work)){
    if(entry.cancel.due(now,interval))send(entry.cancel.identity);
    holding|=active&&entry.observation_until.has_value();
   }else if(entry.observation_until){
    if(now>=*entry.observation_until){i=entries_.erase(i);continue;}
    holding=true;
   }
   ++i;
  }
  return holding;
 }
};
}
