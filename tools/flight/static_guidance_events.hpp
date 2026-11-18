#pragma once
#include "target_registry.hpp"
#include "static_causal_trace.hpp"
#include "static_review_budget.hpp"
#include "fresh_review.hpp"
#if __has_include("task_bearing_bias.hpp")
#include "task_bearing_bias.hpp"
#else
#include "bearing_bias.hpp"
#endif
#include <algorithm>
#include <mutex>
#include <optional>
#include <stdexcept>

namespace racer_task {
inline bool suppressConfirmedGuidance(const TargetRegistry& registry,const TargetKey& key,
                                     BearingBias& policy,int robot,int cue_id){
 if(!registry.groupConfirmed(key))return false;
 policy.cues.erase({robot,cue_id});
 auto cue=policy.discovery.cues.find(cue_id);
 if(cue!=policy.discovery.cues.end())cue->second.enabled=false;
 return true;
}

// Scope legacy utility evaluation to the one admitted entity snapshot. Other
// cues are restored even if native scoring throws; their budgets are untouched.
class ScopedBearingEntity {
 BearingBias& policy_;std::map<std::pair<int,int>,BearingCue> saved_;
 std::pair<int,int> key_;bool enabled_,original_,snapshot_;
public:
 ScopedBearingEntity(BearingBias& policy,int robot,int id,bool enabled):policy_(policy),key_{robot,id},enabled_(enabled),original_(policy.cues.count(key_)),snapshot_(policy.event_driven_static){
  if(enabled)for(auto i=policy_.cues.begin();i!=policy_.cues.end();){
   // A newer low-confidence cache entry must not veto the qualified snapshot.
   // Static scoring restores the entire live cache on exit, including this ID.
   if(snapshot_||i->first!=key_){saved_.insert(*i);i=policy_.cues.erase(i);}else ++i;
  }
 }
 ~ScopedBearingEntity(){
  if(enabled_&&(snapshot_||!original_))policy_.cues.erase(key_);
  for(const auto&kv:saved_)policy_.cues.emplace(kv);
 }
 ScopedBearingEntity(const ScopedBearingEntity&)=delete;
 ScopedBearingEntity& operator=(const ScopedBearingEntity&)=delete;
};

// Scheduling identity is a snapshot of the registry's REVERSIBLE components.
// This is an event inbox, not an existence filter or a trajectory authorizer.
// The manager owns one instance. Registry access remains on its ROS callback
// thread; all inbox operations are serialized for future worker consumers.
class StaticGuidanceEvents {
public:
 bool allow_weak=false; // configured once before callbacks, never per frame
 StaticReviewBudget review_budget;
 struct Work {
  std::uint64_t token=0,generation=0;
  TargetKey entity;
  TargetObservation evidence;
  std::set<TargetKey> members;
  std::string budget_version;
  std::uint64_t source_episode=0,visibility_revision=0;
  bool extended_identity=false,priority=false;
  double paired_capture=-1.; // Used on the manager's stopped-solve copy.
 };
 struct Stats {std::size_t entities=0,pending=0,active=0;std::uint64_t enqueued=0,coalesced=0,invalidated=0;std::size_t waiting=0;std::uint64_t readiness_updates=0,readiness_checks=0;};
 explicit StaticGuidanceEvents(std::size_t limit=256,std::size_t concurrent=1)
   :limit_(limit),concurrent_(concurrent){
  if(!limit||!concurrent||concurrent>limit)throw std::invalid_argument("invalid inbox limits");
 }
 // No timestamp-only expiry and no negative observations are synthesized here.
 void sync(const TargetRegistry&r,double now){
  if(!std::isfinite(now))return;
  std::lock_guard<std::mutex> lock(mutex_);
  last_sync_=now;
  if(wait_epoch_!=r.evidence_epoch){
   std::size_t checks=0;wait_targets_=r.identityWaitTargets(now,checks);
   wait_epoch_=r.evidence_epoch;++readiness_updates_;readiness_checks_+=checks;
  }
  // Build connected components once per callback, reusing source IDs. Do not
  // invoke spatial association, or rescan every component for each source.
  std::map<TargetKey,std::vector<TargetKey>> adjacent;
  for(const auto&kv:r.records)adjacent[kv.first];
  for(const auto&e:r.aliases)if(adjacent.count(e.a)&&adjacent.count(e.b)){
   adjacent[e.a].push_back(e.b);adjacent[e.b].push_back(e.a);
  }
  std::set<TargetKey> visited;
  std::map<TargetKey,State> next;
  for(const auto&kv:adjacent){
   if(visited.count(kv.first))continue;
   std::set<TargetKey> members;std::vector<TargetKey> todo{kv.first};
   while(!todo.empty()){
    auto k=todo.back();todo.pop_back();if(!visited.insert(k).second)continue;
    members.insert(k);for(const auto&other:adjacent.at(k))todo.push_back(other);
   }
   const auto root=*members.begin();
   review_budget.reconcile(members);
   bool recent=false,busy=StaticReviewBudget::overlap(executing_sources_,members);
   for(const auto&k:members)recent|=now-r.records.at(k).last<=r.current_ttl;
   for(const auto&w:active_)busy|=StaticReviewBudget::overlap(w.second.members,members);
   if(!recent&&!busy)continue; // ledger history must not occupy all inbox slots
   if(next.size()>=limit_&&!busy)continue;
   State s;s.members=members;
   auto old=states_.find(root);
   const bool same=old!=states_.end()&&old->second.members==members;
   if(same)s=old->second;
   else s.generation=++serial_;
   s.confirmed=false;
   for(const auto&k:members)s.confirmed|=r.records.at(k).confirmed;
   // Carry a queued qualification through a merge/split only into the new
   // component containing its source. Never clone one event across children.
   if(!same)for(const auto&entry:states_){const auto&prior=entry.second;
    if(prior.pending&&members.count(prior.pending->key)&&
       (!s.pending||prior.pending->capture>s.pending->capture))s.pending=prior.pending;
   }
   if(!same)for(const auto&entry:active_){const auto&w=entry.second;
    if(members.count(w.evidence.key)&&(!s.pending||w.evidence.capture>s.pending->capture))s.pending=w.evidence;
   }
   // Revalidate inherited geometry even if its source sequence did not change.
   if(s.pending){const auto&current=r.records.at(s.pending->key).latest;
    if(current.frame!=s.pending->frame||current.frame_version!=s.pending->frame_version){
     s.pending.reset();s.generation=++serial_;++invalidated_;
    }
   }
   if(!same)for(const auto&entry:states_){const auto&prior=entry.second;
    if(prior.remembered&&members.count(prior.remembered->key)&&
       (!s.remembered||prior.remembered->capture>s.remembered->capture))s.remembered=prior.remembered;
   }
   if(s.remembered){const auto&latest=r.records.at(s.remembered->key).latest;
    if(latest.frame!=s.remembered->frame||latest.frame_version!=s.remembered->frame_version)s.remembered.reset();
   }
   for(const auto&k:members){const auto&o=r.records.at(k).latest;
    if(k.robot!=r.self)continue;
    auto&seen=seen_[k];
    if(o.sequence<=seen.sequence)continue;
    int tier=0;
    if(o.has_bearing&&now>=o.capture&&now-o.capture<=r.current_ttl){
      if(bearingNavigationSupported(o.confidence,o.support,o.support_first,o.capture))tier=2;
      else if(allow_weak&&bearingWeakSupported(o.confidence,o.support,o.support_first,o.capture))tier=1;
    }
    const bool supported=tier>0;
    // A new qualified capture after a measurement gap is reacquisition;
    // elapsed time alone never creates an event or erases the hypothesis.
    const bool frame_changed=seen.sequence&&(seen.frame!=o.frame||seen.frame_version!=o.frame_version);
    const bool new_episode=frame_changed||!seen.sequence||o.capture-seen.qualified_capture>r.current_ttl;
    // Retain the best admitted tier through brief weak/missing captures. A
    // confidence oscillation is not an upgrade and must not invalidate a solve.
    const bool rising=supported&&(new_episode||tier>seen.tier);
    const bool refreshed=s.awaiting_refresh&&s.awaiting_refresh->key==k&&
      o.capture>s.awaiting_refresh->capture&&o.identity_input=="completed"&&
      r.compatibleFrames(o,*s.awaiting_refresh)&&tier==2;
    if(refreshed)s.awaiting_refresh.reset();
    // Frame changes invalidate active results even when there is no queue.
    if(frame_changed){
     s.generation=++serial_;++invalidated_;
    }
    // Frame changes invalidate old geometric snapshots even on a weak frame.
    if(s.pending&&s.pending->key==k&&
       (s.pending->frame!=o.frame||s.pending->frame_version!=o.frame_version)){
      s.pending.reset();s.generation=++serial_;++invalidated_;
    }
    if(supported&&(!s.remembered||tier==2||
       !bearingNavigationSupported(s.remembered->confidence,s.remembered->support,s.remembered->support_first,s.remembered->capture)))s.remembered=o;
    if((rising||refreshed)&&!s.confirmed){
     if(new_episode){s.wait_deadline=0;s.wait_closed=false;}
     if(s.pending)++coalesced_;else ++enqueued_;
     const bool old_strong=s.pending&&bearingNavigationSupported(s.pending->confidence,s.pending->support,s.pending->support_first,s.pending->capture);
     if(!old_strong||tier==2){s.pending=o;s.retry_reservation.reset();}
     s.generation=++serial_;
    }
    seen={o.sequence,new_episode?tier:std::max(seen.tier,tier),o.frame,o.frame_version,
          supported?o.capture:(frame_changed?-std::numeric_limits<double>::infinity():seen.qualified_capture),
          rising||frame_changed?o.sequence:seen.episode,seen.priority_episode};
   }
   // A reversible split can expose an unconfirmed source previously suppressed
   // by a confirmed neighbour. Topology itself is a material scheduling event.
   if(!same&&!s.confirmed&&!s.pending){
    for(const auto&k:members){const auto&o=r.records.at(k).latest;
    if(k.robot==r.self&&o.has_bearing&&now>=o.capture&&now-o.capture<=r.current_ttl&&
       (bearingNavigationSupported(o.confidence,o.support,o.support_first,o.capture)||(allow_weak&&bearingWeakSupported(o.confidence,o.support,o.support_first,o.capture)))){
     if(!s.pending||o.capture>s.pending->capture)s.pending=o;
    }
   }
    if(s.pending)++enqueued_;
   }
   std::string visibility="unknown";double visibility_capture=-1.;std::uint64_t visibility_revision=0;
   for(const auto&k:members){const auto&r0=r.records.at(k);
    if(r0.has_anchor&&r0.visibility_anchor_capture==r0.anchor_time&&r0.visibility_capture>visibility_capture){
     visibility=r0.visibility;visibility_capture=r0.visibility_capture;visibility_revision=r0.visibility_revision;
    }
   }
   if(same&&visibility!=s.visibility&&(visibility=="clear"||visibility=="occluded")&&s.remembered&&!s.confirmed){
    if(s.pending)++coalesced_;else ++enqueued_;
    s.pending=s.remembered;s.retry_reservation.reset();s.generation=++serial_;
   }
   s.visibility=visibility;
   s.visibility_revision=visibility_revision;
   if(s.confirmed){
    if(s.pending||!same||!old->second.confirmed){s.generation=++serial_;++invalidated_;}
    s.pending.reset();
   }
   if(s.retry_reservation&&(!s.pending||!isRetry(s))){
    s.pending.reset();s.retry_reservation.reset();
   }
   const double previous_wait=s.wait_until;
   s.wait_until=0;
   if(s.pending&&!s.confirmed&&!s.wait_closed){
    const auto&current=r.records.at(s.pending->key).latest;
    const bool exact=current.capture==s.pending->capture&&current.frame==s.pending->frame&&current.frame_version==s.pending->frame_version;
    if(exact&&wait_targets_.count(current.key)&&current.identity_input=="pending"){
     if(!s.wait_deadline)s.wait_deadline=current.identity_deadline;
     s.wait_until=std::min(s.wait_deadline,current.identity_deadline);
     if(now>=s.wait_until){s.wait_until=0;s.wait_closed=true;}
    }else if(s.wait_deadline)s.wait_closed=true; // resolved/replaced; never restart same episode
   }
   if(s.wait_until!=previous_wait){
    const auto&o=s.pending?*s.pending:r.records.at(root).latest;
    staticCausalTrace().record(s.wait_until?10:12,now,0,o.key.counter,o.sequence,o.capture,
      StaticCausalTrace::stamp(s.wait_until?s.wait_until:previous_wait),s.confirmed?1:now>=previous_wait?3:2);
   }
   next.emplace(root,std::move(s));
  }
  if(staticCausalTrace().enabled())for(const auto&prior:states_){
   const auto&s=prior.second;if(!s.wait_until||!s.pending)continue;
   auto current=next.find(prior.first);
   if(current==next.end()||current->second.members!=s.members){const auto&o=*s.pending;
    staticCausalTrace().record(12,now,0,o.key.counter,o.sequence,o.capture,StaticCausalTrace::stamp(s.wait_until),r.groupConfirmed(o.key)?1:2);
   }
  }
  states_=std::move(next);
 }
 // Called by a single global consumer, not once per entity/ROS callback.
 // A work item retains the qualified frame, even if a later weak frame arrives.
 // An accepted trajectory releases the solver, while its unresolved task
 // still owns its source identities. Overlapping components stay queued.
 std::optional<Work> start(double now=std::numeric_limits<double>::quiet_NaN(),
                           const std::set<TargetKey>& reserved_sources={},
                           const TargetRegistry* fresh_registry=nullptr){
  std::lock_guard<std::mutex> lock(mutex_);
  if(std::isnan(now))now=last_sync_;
  if(active_.size()>=concurrent_)return {};
  auto best=states_.end();
  for(auto it=states_.begin();it!=states_.end();++it){
   auto&s=it->second;if(!s.pending||s.confirmed)continue;
   if(fresh_registry){
    const auto* source=reviewSource(*fresh_registry,*s.pending);
    const auto* latest=source?&source->latest:nullptr;
    const bool fresh=latest&&freshReviewCapture(latest->capture,now);
    if(fresh&&!freshReviewCapture(s.pending->capture,now)&&latest->has_bearing&&
       bearingNavigationSupported(latest->confidence,latest->support,latest->support_first,latest->capture)){
     s.pending=*latest;
     s.retry_reservation.reset();
    }
    if(!fresh||!freshReviewCapture(s.pending->capture,now)){
     const auto& expired=*s.pending;
     staticCausalTrace().record(13,now,0,expired.key.counter,expired.sequence,expired.capture,
                               latest?StaticCausalTrace::stamp(latest->capture):0);
     if(latest)s.awaiting_refresh=*latest;
     s.pending.reset();s.retry_reservation.reset();continue;
    }
   }
   if(now<s.pending->capture||now-s.pending->capture>2.){
    s.pending.reset();s.retry_reservation.reset();continue;
   }
   if(std::any_of(s.members.begin(),s.members.end(),[&](const TargetKey& key){
     return reserved_sources.count(key)!=0;
   }))continue;
   if(!isRetry(s)){
    auto reason=review_budget.eligible(s.members,s.pending->key,budgetVersion(*s.pending,s.visibility_revision),now,s.pending->capture,2.,StaticReviewBudget::Purpose::ConfirmPresence,s.confirmed);
    if(reason=="component_cooldown")continue;
    if(!reason.empty()){s.pending.reset();continue;}
   }
   if(s.wait_until&&now<s.wait_until)continue;
   if(s.wait_until){staticCausalTrace().record(12,now,0,s.pending->key.counter,s.pending->sequence,s.pending->capture,StaticCausalTrace::stamp(s.wait_until),3);s.wait_until=0;s.wait_closed=true;}
   bool busy=false;for(const auto&w:active_)for(const auto&k:w.second.members)
    if(s.members.count(k)){busy=true;break;}
   if(busy)continue;
   if(best==states_.end()||s.pending->capture<best->second.pending->capture)best=it;
  }
  if(best==states_.end())return {};
  auto&s=best->second;Work w{++token_,s.generation,best->first,*s.pending,s.members};
  w.budget_version=budgetVersion(w.evidence,s.visibility_revision);
  auto& seen=seen_.at(w.evidence.key);
  w.source_episode=seen.episode;
  w.visibility_revision=s.visibility_revision;
  if(!isRetry(s)){
   review_budget.reserve(s.members,w.evidence.key,w.budget_version,now);
   if(fresh_registry&&seen.priority_episode!=seen.episode&&bearingNavigationSupported(
      w.evidence.confidence,w.evidence.support,w.evidence.support_first,w.evidence.capture)){
    w.priority=true;
    seen.priority_episode=seen.episode;
   }
  }
  s.retry_reservation.reset();
  staticCausalTrace().record(11,now,0,w.evidence.key.counter,w.evidence.sequence,w.evidence.capture,w.token,w.generation);
  s.pending.reset();s.wait_closed=true;active_.emplace(w.token,w);return w;
 }
 bool valid(std::uint64_t token)const{
  std::lock_guard<std::mutex> lock(mutex_);return validUnlocked(token);
 }
 // A handed-off trajectory can still own a task after its solver slot is free.
 // Keep its component generation available until execution settles.
 void setExecutingSources(std::set<TargetKey> sources){
  std::lock_guard<std::mutex> lock(mutex_);
  executing_sources_=std::move(sources);
 }
 bool reserveRoute(std::uint64_t token,double seconds,double distance){
  std::lock_guard<std::mutex> lock(mutex_);
  if(!validUnlocked(token))return false;
  const auto&w=active_.at(token);return review_budget.route(w.evidence.key,w.budget_version,seconds,distance);
 }
 // Execution monitoring does not own a solver slot. The copied identity and
 // generation remain checkable after finish(), without creating a new request.
 bool current(const Work&w)const{
  std::lock_guard<std::mutex> lock(mutex_);
  auto s=states_.find(w.entity);
  return s!=states_.end()&&!s->second.confirmed&&s->second.generation==w.generation&&s->second.members==w.members;
 }
 // An executing route keeps its source when the registry adds aliases. A
 // solver snapshot is still strict. Splits, frame changes and confirmation
 // invalidate the execution even if the old source remains in the registry.
 bool refreshExecuting(Work& w)const{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto seen=seen_.find(w.evidence.key);
  if(seen==seen_.end()||seen->second.frame!=w.evidence.frame||
     seen->second.frame_version!=w.evidence.frame_version||seen->second.episode!=w.source_episode)return false;
  auto state=states_.find(w.entity);
  if(state!=states_.end()&&!state->second.confirmed&&
     state->second.members==w.members){
    return state->second.generation==w.generation||
           (w.extended_identity&&state->second.visibility_revision==w.visibility_revision);
   }
  for(const auto& [root,s]:states_){
   if(s.confirmed||s.members.size()<=w.members.size()||
      !std::includes(s.members.begin(),s.members.end(),w.members.begin(),w.members.end()))continue;
   for(const auto& key:w.members){
    const auto previous=seen_.find(key);
    if(previous==seen_.end()||previous->second.frame!=w.evidence.frame||
       previous->second.frame_version!=w.evidence.frame_version)return false;
   }
   w.entity=root;
   w.generation=s.generation;
   w.members=s.members;
   w.visibility_revision=s.visibility_revision;
   w.extended_identity=true;
   return true;
  }
  return false;
 }
 // Retry only a still-current task. Preserve an already newer queued event.
 void retry(std::uint64_t token){
  std::lock_guard<std::mutex> lock(mutex_);
  auto a=active_.find(token);if(a==active_.end())return;
  if(validUnlocked(token)){
   const auto&w=a->second;
   auto&s=states_.at(w.entity);
   const auto reservation=std::make_pair(w.evidence.key,w.budget_version);
   // A deferral can use its existing reservation once for the same evidence.
   if(!s.pending&&w.budget_version==budgetVersion(w.evidence,s.visibility_revision)&&retried_.insert(reservation).second){
    s.pending=w.evidence;
    s.retry_reservation=reservation;
   }
  }
  active_.erase(a);
 }
 // A lost route can only be reconsidered with new paired, qualifying input.
 // The next solve still spends the normal component and mission budgets.
 void waitForFreshFrame(std::uint64_t token,const TargetObservation& last){
  std::lock_guard<std::mutex> lock(mutex_);
  if(validUnlocked(token))states_.at(active_.at(token).entity).awaiting_refresh=last;
  active_.erase(token);
 }
 // Releases a slot even when stale. A true result validates ONLY the inbox
 // snapshot: map, current visibility, dynamics and handoff still require checks.
 bool finish(std::uint64_t token){
  std::lock_guard<std::mutex> lock(mutex_);
  auto a=active_.find(token);if(a==active_.end())return false;
  const auto w=a->second;active_.erase(a);auto s=states_.find(w.entity);
  return s!=states_.end()&&!s->second.confirmed&&s->second.generation==w.generation&&s->second.members==w.members;
 }
 Stats stats()const{
  std::lock_guard<std::mutex> lock(mutex_);std::size_t pending=0,waiting=0;
  for(const auto&kv:states_){pending+=bool(kv.second.pending);waiting+=bool(kv.second.wait_until);}
  return {states_.size(),pending,active_.size(),enqueued_,coalesced_,invalidated_,waiting,readiness_updates_,readiness_checks_};
 }
private:
 bool validUnlocked(std::uint64_t token)const{
  auto a=active_.find(token);if(a==active_.end())return false;
  auto s=states_.find(a->second.entity);
  return s!=states_.end()&&!s->second.confirmed&&s->second.generation==a->second.generation&&s->second.members==a->second.members;
 }
 using ReservationKey=std::pair<TargetKey,std::string>;
 struct State {
  std::set<TargetKey> members;
  std::uint64_t generation=0,visibility_revision=0;
  bool confirmed=false;
  std::optional<TargetObservation> pending,remembered,awaiting_refresh;
  std::optional<ReservationKey> retry_reservation;
  std::string visibility="unknown";
  double wait_until=0,wait_deadline=0;
  bool wait_closed=false;
 };
 static std::string budgetVersion(const TargetObservation&o,std::uint64_t visibility_revision){
  return std::to_string(o.sequence)+"/"+std::to_string(visibility_revision);
 }
 static bool isRetry(const State&s){
  return s.retry_reservation&&s.retry_reservation->first==s.pending->key&&
         s.retry_reservation->second==budgetVersion(*s.pending,s.visibility_revision);
 }
 struct Seen {std::uint64_t sequence=0;int tier=0;std::string frame,frame_version;double qualified_capture=-std::numeric_limits<double>::infinity();std::uint64_t episode=0,priority_episode=0;};
 mutable std::mutex mutex_;
 std::size_t limit_,concurrent_;double last_sync_=0;
 std::uint64_t serial_=0,token_=0,enqueued_=0,coalesced_=0,invalidated_=0;
 std::map<TargetKey,State> states_;
 std::uint64_t wait_epoch_=std::numeric_limits<std::uint64_t>::max(),readiness_updates_=0,readiness_checks_=0;
 std::map<TargetKey,TargetKey> wait_targets_;
 std::map<TargetKey,Seen> seen_;
 std::map<std::uint64_t,Work> active_;
 std::set<TargetKey> executing_sources_;
 std::set<ReservationKey> retried_;
};
}
