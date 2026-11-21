#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <string>
#include <vector>
namespace racer_task {
using DiscoveryVec=std::array<double,3>;
inline double discoveryDistance(const DiscoveryVec&a,const DiscoveryVec&b){double s=0;for(int i=0;i<3;++i)s+=(a[i]-b[i])*(a[i]-b[i]);return std::sqrt(s);}
inline double discoveryAngle(const DiscoveryVec&a,const DiscoveryVec&b){double s=0;for(int i=0;i<3;++i)s+=a[i]*b[i];return std::acos(std::clamp(s,-1.,1.));}
struct DiscoveryCue {
 int group=-1;double capture=0;bool enabled=true;DiscoveryVec origin{},direction{};bool committed=false;
};
struct DiscoveryGroup {
 int id=0,attempts=0,uses=0,updates=0,retries=0,serial=0,blocked_serial=0,progress=0;
 double first=0,epoch=0,cooldown_until=0;bool blocked=false,evaluating=false;
 DiscoveryVec anchor{},direction{},failure_origin{},failure_direction{};
 std::string reason="new_discovery";std::vector<int> failure_context;
};
struct DiscoveryPlan {int cue=-1,progress=0;bool closed=false;};
class DiscoveryMemory {
public:
 double ttl=5.,window=40.,cooldown=4.;int max_attempts=4,mission_attempt_limit=16;
 int mission_attempts=0;std::map<int,DiscoveryCue> cues;std::map<int,DiscoveryGroup> groups;
 std::map<int,DiscoveryPlan> plans;
 double value(int cue,double now)const{
  auto i=cues.find(cue);return i==cues.end()?0.:decay(groups.at(i->second.group),now);
 }
 bool material(DiscoveryGroup&g,double capture,const DiscoveryVec&o,const DiscoveryVec&d,int progress){
  bool changed=progress>g.progress ||
   (discoveryDistance(o,g.anchor)>=.5&&discoveryAngle(d,g.direction)>=5.*3.141592653589793/180.);
  g.progress=std::max(g.progress,progress);
  if(changed){
   ++g.serial;g.anchor=o;g.direction=d;g.reason="material_evidence";
   if(g.updates<1&&capture-g.epoch>=5.){++g.updates;g.epoch=capture;}
   return true;
  }return false;
 }
 bool observe(int id,double first,double capture,const DiscoveryVec&o,const DiscoveryVec&d,
              bool enabled,int progress){
  return observe(id,first,capture,o,d,enabled,progress,capture);
 }
 bool observe(int id,double first,double capture,const DiscoveryVec&o,const DiscoveryVec&d,
              bool enabled,int progress,double now){
  if(id<1||!std::isfinite(first)||first<0||first>capture||!std::isfinite(capture)||now<capture)return false;
  auto old=cues.find(id);int group=-1;bool committed=false;
  if(old!=cues.end()){
   if(capture<old->second.capture)return false;
   group=old->second.group;committed=old->second.committed;
  }else{
   if(now-capture>ttl)return false;
   if(groups.count(id))group=id;
   else{
    double best=.75;
    for(const auto&kv:groups){
     double distance=discoveryDistance(o,kv.second.anchor);
     if(distance<=best&&discoveryAngle(d,kv.second.direction)<=.20){best=distance;group=kv.first;}
    }
   }
   if(cues.size()>=64&&!reclaimUnused(now,group))return false;
   if(group<0){group=id;DiscoveryGroup g;g.id=id;g.first=g.epoch=first;g.anchor=o;g.direction=d;
    g.failure_origin=o;g.failure_direction=d;g.progress=progress;groups[group]=g;}
  }
  auto&g=groups.at(group);
  // Aliases may only move first discovery earlier, never reset it to a new detection.
  if(first<g.first){g.first=first;if(g.updates==0)g.epoch=first;}
  material(g,capture,o,d,progress);
  cues[id]={group,capture,enabled,o,d,committed};return true;
 }
 const char* rejection(int id,double now,double* age_value=nullptr)const{
  auto it=cues.find(id);if(it==cues.end())return "unknown_discovery";
  const auto&c=it->second;const auto&g=groups.at(c.group);
  if(!c.enabled)return "navigation_revoked";
  if(now<c.capture||now-c.capture>ttl)return "capture_expired";
  const double age_weight=decay(g,now);
  if(age_weight<=0)return "discovery_value_expired";
  if(g.uses>=4)return "persistent_selection_budget_exhausted";
  if((g.attempts>=max_attempts||mission_attempts>=mission_attempt_limit)&&!g.evaluating)return "evaluation_budget_exhausted";
  if(g.blocked){
   if(now<g.cooldown_until)return "failure_cooldown";
   if(g.serial<=g.blocked_serial)return "awaiting_material_change";
   if(g.retries>=3)return "retry_budget_exhausted";
  }
  if(age_value)*age_value=age_weight;
  return nullptr;
 }
 std::string reason(int id,double now)const{
  const char* why=rejection(id,now);return why?why:"eligible";
 }
 bool available(int id,double now)const{return !rejection(id,now);}
 bool begin(int id,double now){
  if(!available(id,now))return false;
  auto& cue=cues.at(id);cue.committed=true;auto&g=groups.at(cue.group);
  if(g.blocked){++g.retries;g.blocked=false;g.reason="material_retry";}
  ++g.attempts;++mission_attempts;g.evaluating=true;return true;
 }
 void fail(int id,double now,const std::string&why){
  auto it=cues.find(id);if(it==cues.end())return;
  it->second.committed=true;auto&g=groups.at(it->second.group);
  g.blocked=true;g.cooldown_until=std::max(g.cooldown_until,now+cooldown);
  g.blocked_serial=g.serial;g.failure_origin=it->second.origin;g.failure_direction=it->second.direction;
  g.failure_context.clear();g.reason=why;g.evaluating=false;
 }
 void finish(int id,double now,bool reachable){
  auto it=cues.find(id);if(it==cues.end())return;
  groups.at(it->second.group).evaluating=false;if(!reachable)fail(id,now,"native_candidates_unreachable");
 }
 void selected(int id){
  auto i=cues.find(id);if(i==cues.end())return;
  i->second.committed=true;++groups.at(i->second.group).uses;
 }
 void planned(int plan,int cue){auto i=cues.find(cue);if(i!=cues.end())plans[plan]={cue,groups.at(i->second.group).progress,false};if(plans.size()>64)plans.erase(plans.begin());}
 void rejected(int plan,double now){auto i=plans.find(plan);if(i!=plans.end()&&!i->second.closed){i->second.closed=true;fail(i->second.cue,now,"execution_authorization_rejected");}}
 void completed(int plan,double now){
  auto it=plans.find(plan);if(it==plans.end()||it->second.closed)return;auto&p=it->second;p.closed=true;
  auto c=cues.find(p.cue);if(c==cues.end())return;auto&g=groups.at(c->second.group);
  if(g.progress<=p.progress)fail(p.cue,now,"completed_observation_without_localization_progress");
 }
 void mapContext(int group,const std::vector<int>&free,double now){
  auto i=groups.find(group);if(i==groups.end()||!i->second.blocked)return;auto&g=i->second;
  if(g.failure_context.empty()){g.failure_context=free;return;}
  if(free.size()!=g.failure_context.size())return;int opened=0;
  for(std::size_t j=0;j<free.size();++j)opened+=free[j]>0&&g.failure_context[j]<=0;
  if(opened>=6){
   ++g.serial;g.failure_context=free;g.reason="material_map_reachability_change";
   if(g.updates<1&&now-g.epoch>=5.){++g.updates;g.epoch=now;}
  }
 }
 void alias(int a,int b){
  auto ia=cues.find(a),ib=cues.find(b);if(ia==cues.end()||ib==cues.end()||ia->second.group==ib->second.group)return;
  int ga=ia->second.group,gb=ib->second.group;auto&x=groups.at(ga);auto&y=groups.at(gb);
  y.first=std::min(x.first,y.first);y.epoch=std::min(x.epoch,y.epoch);
  y.attempts+=x.attempts;y.uses+=x.uses;y.updates=std::min(1,x.updates+y.updates);y.retries+=x.retries;
  y.blocked|=x.blocked;y.cooldown_until=std::max(x.cooldown_until,y.cooldown_until);
  if(x.blocked){y.blocked_serial=y.serial;y.reason="aliased_failure_memory";}
  for(auto&kv:cues)if(kv.second.group==ga)kv.second.group=gb;
  groups.erase(ga);
 }
private:
 double decay(const DiscoveryGroup&g,double now)const{
  return now>=g.epoch?std::clamp(1.-(now-g.epoch)/window,0.,1.):0.;
 }
 bool reclaimUnused(double now,int incoming_group){
  auto oldest=cues.end();
  for(auto it=cues.begin();it!=cues.end();++it){
   const auto& cue=it->second;
   if(cue.committed||now-cue.capture<=ttl)continue;
   if(std::any_of(plans.begin(),plans.end(),[&](const auto& p){
    return !p.second.closed&&p.second.cue==it->first;
   }))continue;
   if(oldest==cues.end()||cue.capture<oldest->second.capture)oldest=it;
  }
  if(oldest==cues.end())return false;
  const int group=oldest->second.group;cues.erase(oldest);
  // Re-associating this observation can still use the original discovery epoch.
  if(group!=incoming_group&&std::none_of(cues.begin(),cues.end(),[&](const auto& c){
   return c.second.group==group;
  }))groups.erase(group);
  return true;
 }
};
}

