#pragma once
// Pure bounded angular utility. This is not a human-position estimator.
#include <array>
#include "discovery_memory.hpp"
#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include <vector>
#include <limits>

namespace racer_task {
using BearingVec=std::array<double,3>;
// One native navigation contract for packets and registry records. The 1 ns
// tolerance covers decimal capture timestamp arithmetic, not stale captures.
inline bool bearingSupportEvidence(double confidence,double hits,double first,double capture){
  return confidence>=.5&&confidence<=1.&&hits>=1&&hits<=1e6&&std::floor(hits)==hits&&
    std::isfinite(first)&&std::isfinite(capture);
}
inline bool bearingMultiFrame(double hits,double first,double capture){
  return hits>=3&&first<=capture-.4+1e-9;
}
inline bool bearingWeakFrame(double first,double capture){return first>=0&&first<=capture;}
inline bool bearingNavigationSupported(double confidence,double hits,double first,double capture){
  return bearingSupportEvidence(confidence,hits,first,capture)&&bearingMultiFrame(hits,first,capture);
}
inline bool bearingWeakSupported(double confidence,double hits,double first,double capture){
  return bearingSupportEvidence(confidence,hits,first,capture)&&bearingWeakFrame(first,capture);
}
inline double bdDot(const BearingVec&a,const BearingVec&b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
inline BearingVec bdSub(const BearingVec&a,const BearingVec&b){return {a[0]-b[0],a[1]-b[1],a[2]-b[2]};}
inline double bdNorm(const BearingVec&a){return std::sqrt(bdDot(a,a));}
struct BearingCue {int source=0,id=0,coverage=0,uses=0;double stamp=0,confidence=0,hfov=0;BearingVec origin{},direction{};};
struct BearingCandidate {int id=-1;BearingVec point{};double yaw=0,cost=0;bool reachable=true;double path_length=std::numeric_limits<double>::quiet_NaN();bool review_view=false;};
struct BearingScore {int id=-1,cue_id=-1;double base_cost=0,adjusted_cost=0,utility=0;bool reachable=false;std::string reason;};
struct BearingDecision {int selected=-1,cue_id=-1;bool changed=false;std::string reason;std::vector<BearingScore> scores;};
struct BearingUtility {double age=0,heading_angle=0,heading_alignment=0,lateral_baseline=0,view_diversity=0,age_factor=0,value=0;std::string reason;};

// Bound changes from the admitted camera view without estimating target range.
inline const char* staticReviewViewRejection(const BearingVec& point,const BearingVec& origin,
                                            const BearingVec& direction){
  const auto delta=bdSub(point,origin);
  const double horizontal=std::hypot(direction[0],direction[1]);
  if(!std::isfinite(bdNorm(delta))||!std::isfinite(horizontal)||horizontal<.2)
    return "invalid_capture_view_geometry";
  const double forward=(delta[0]*direction[0]+delta[1]*direction[1])/horizontal;
  if(forward<-.35)return "review_view_behind_capture";
  const double sideways=(delta[0]*direction[1]-delta[1]*direction[0])/horizontal;
  if(std::hypot(sideways,delta[2])>1.)return "review_view_far_from_capture_bearing";
  return nullptr;
}

class BearingBias {
public:
  bool enabled=false,persistent=false,event_driven_static=false;
  bool allow_weak=false,weak_event=false;
  double weak_max_extra_cost=-1.,weak_max_extra_path_m=-1.;
  DiscoveryMemory discovery;
  double ttl=20.,discount=.5,max_cost_ratio=1.5,active_budget_s=60.;
  int per_cue_limit=2,mission_limit=4;
  int selections=0;double active_since=-1.;bool original_next=false;
  std::map<std::pair<int,int>,BearingCue> cues;
  bool receive(const std::vector<double>& d,double now,int own_source=0){
    if(d.size()!=17 || !std::isfinite(now))return false;
    // The scalar bounds below also exclude nonfinite values. Only vector
    // coordinates have no scalar bounds.
    for(std::size_t i=4;i<=9;++i)if(!std::isfinite(d[i]))return false;
    if(d[0]!=1 || d[1]!=own_source || d[2]<1 || d[2]>1e9 || std::floor(d[2])!=d[2] ||
       now-d[3]<0 || (!event_driven_static&&now-d[3]>ttl) || !(d[10]>=.01&&d[10]<=1.5) ||
       !bearingSupportEvidence(d[11],d[12],d[15],d[3]) ||
       !(bearingMultiFrame(d[12],d[15],d[3])||(event_driven_static&&allow_weak&&bearingWeakFrame(d[15],d[3]))) ||
       d[13]<0 || d[13]>1e6 || std::floor(d[13])!=d[13] || (d[14]!=0. && d[14]!=1.) ||
       !(d[16]>0.&&d[16]<1.57))return false;
    BearingCue c;c.source=int(d[1]);c.id=int(d[2]);c.stamp=d[3];c.origin={d[4],d[5],d[6]};c.direction={d[7],d[8],d[9]};
    if(std::abs(bdNorm(c.direction)-1.)>1e-5)return false;
    c.confidence=d[11];c.coverage=int(d[13]);c.hfov=d[16];
    auto key=std::make_pair(c.source,c.id);auto old=cues.find(key);
    if(old!=cues.end()){
      if(c.stamp<old->second.stamp)return false;
      c.uses=old->second.uses;c.coverage=std::max(c.coverage,old->second.coverage);
    }else if(cues.size()>=64){
      // Keep spent cue IDs so delayed refreshes cannot refill their budget.
      if(!event_driven_static){
        const double retention=persistent?std::min(ttl,discovery.ttl):ttl;
        for(auto it=cues.begin();it!=cues.end();){
          if(!it->second.uses&&now-it->second.stamp>retention)it=cues.erase(it);
          else ++it;
        }
      }
      if(cues.size()>=64)return false;
    }
    cues[key]=c;return true;
  }
  // Static mode means retained qualified evidence, not a fresh RGB frame.
  // Its source/frame/confirmation validity is owned by the admitted inbox work.
  bool hasFresh(double now)const{
    for(const auto& kv:cues)if(now>=kv.second.stamp && (event_driven_static||now-kv.second.stamp<=ttl) && (event_driven_static||!persistent||discovery.available(kv.second.id,now)))return true;
    return false;
  }
  BearingUtility explain(const BearingCandidate& candidate,const BearingCue& c,double now)const{
    BearingUtility out;out.age=now-c.stamp;out.reason="stale";
    if(out.age<0 || (!event_driven_static&&out.age>=ttl))return out;
    double discovery_value=0.;
    if(!event_driven_static&&persistent){
      if(const char* rejection=discovery.rejection(c.id,now,&discovery_value)){out.reason=rejection;return out;}
    }
    if(!event_driven_static&&c.uses>=per_cue_limit){out.reason="cue_budget_exhausted";return out;}
    const double horizontal=std::hypot(c.direction[0],c.direction[1]);
    if(horizontal<.2){out.reason="near_vertical_bearing_unsupported";return out;}
    if(event_driven_static){
      if(const char* rejection=staticReviewViewRejection(candidate.point,c.origin,c.direction)){
        out.reason=rejection;return out;
      }
    }
    const double cosine=std::clamp((std::cos(candidate.yaw)*c.direction[0]+std::sin(candidate.yaw)*c.direction[1])/horizontal,-1.,1.);
    out.heading_angle=std::acos(cosine);
    const double half_fov_cosine=std::cos(c.hfov);
    out.heading_alignment=std::clamp((cosine-half_fov_cosine)/(1.-half_fov_cosine),0.,1.);
    auto delta=bdSub(candidate.point,c.origin);
    out.lateral_baseline=std::abs(delta[0]*c.direction[1]-delta[1]*c.direction[0])/horizontal;
    out.view_diversity=.5+.5*std::min(1.,out.lateral_baseline/.6);
    out.age_factor=event_driven_static?1.:(persistent?discovery_value:1.-out.age/ttl);
    out.value=c.confidence*out.heading_alignment*out.view_diversity*out.age_factor/(1.+c.coverage+(event_driven_static?0:c.uses));
    out.reason=out.value>0?"directional_observation_opportunity":"camera_heading_outside_bearing_attention";
    return out;
  }
  double utility(const BearingCandidate& candidate,double now,int& cue_id)const{
    double best=0.;cue_id=-1;
    for(const auto& kv:cues){const auto& c=kv.second;
      double u=explain(candidate,c,now).value;
      if(u>best){best=u;cue_id=c.id;}
    }return std::clamp(best,0.,1.);
  }
  bool mayScore(double now)const{
    return enabled&&hasFresh(now)&&(event_driven_static||(!original_next&&selections<mission_limit&&
      (active_since<0||now-active_since<active_budget_s)));
  }
  bool admissibleCost(double candidate,double baseline)const{
    return std::isfinite(candidate)&&candidate>0&&std::isfinite(baseline)&&baseline>0&&
      (event_driven_static||candidate<=max_cost_ratio*baseline);
  }
  BearingDecision choose(const BearingCandidate& baseline,const std::vector<BearingCandidate>& candidates,double now,bool prioritize_review=false){
    BearingDecision out;out.selected=baseline.id;out.reason="no_fresh_evidence";
    if(!enabled){out.reason="disabled";return out;}
    if(!hasFresh(now))return out;
    if(event_driven_static&&(!std::isfinite(discount)||discount<0.||discount>=1.)){
      out.reason="invalid_nonnegative_cost_discount";return out;
    }
    if(active_since<0)active_since=now;
    if(!event_driven_static&&(selections>=mission_limit || now-active_since>=active_budget_s)){out.reason="mission_bias_budget_exhausted";return out;}
    if(!event_driven_static&&original_next){original_next=false;out.reason="forced_original_exploration";return out;}
    const bool baseline_ready=baseline.reachable&&std::isfinite(baseline.cost)&&baseline.cost>0;
    const bool priority=event_driven_static&&prioritize_review&&!weak_event;
    if(!baseline_ready&&!priority){out.reason="baseline_not_reachable_no_override";return out;}
    double best=std::numeric_limits<double>::infinity();bool picked_priority=false;
    if(baseline_ready){
      int id=-1;double base_utility=utility(baseline,now,id);best=baseline.cost*(1.-discount*base_utility);
      out.scores.push_back({baseline.id,id,baseline.cost,best,base_utility,true,"original_native_goal"});
    }
    const double detour_limit=event_driven_static?std::numeric_limits<double>::infinity():max_cost_ratio*baseline.cost;
    const bool weak=event_driven_static&&weak_event;
    const bool weak_envelope=weak&&allow_weak&&std::isfinite(weak_max_extra_cost)&&weak_max_extra_cost>0&&
      std::isfinite(weak_max_extra_path_m)&&weak_max_extra_path_m>0&&
      std::isfinite(baseline.path_length)&&baseline.path_length>=0;
    for(const auto& c:candidates){
      BearingScore s;s.id=c.id;s.base_cost=c.cost;s.adjusted_cost=c.cost;s.reachable=c.reachable;
      if(!c.reachable || !std::isfinite(c.cost) || c.cost<=0){s.reason="native_path_not_known_free";out.scores.push_back(s);continue;}
      const bool priority_view=priority&&c.review_view;
      if(!priority_view&&(!baseline_ready||!(c.cost<=detour_limit))){s.reason="bounded_detour_cost_rejected";out.scores.push_back(s);continue;}
      if(weak&&(!weak_envelope||!std::isfinite(c.path_length)||c.path_length<0||
         c.cost-baseline.cost>weak_max_extra_cost||
         c.path_length-baseline.path_length>weak_max_extra_path_m)){
        s.reason="weak_evidence_low_cost_envelope";out.scores.push_back(s);continue;
      }
      s.utility=utility(c,now,s.cue_id);s.adjusted_cost=c.cost*(1.-discount*s.utility);s.reason=s.utility>0?"angular_utility":"outside_evidence";
      if(s.utility>0&&(priority_view!=picked_priority?priority_view:s.adjusted_cost<best-1e-9)){
        picked_priority=priority_view;best=s.adjusted_cost;out.selected=c.id;out.cue_id=s.cue_id;out.changed=true;
      }
      out.scores.push_back(s);
    }
    if(picked_priority)out.reason="fresh_review_priority";
    else if(out.changed)out.reason="bounded_bearing_cost_preferred";
    else out.reason=baseline_ready?"original_native_goal_kept":"baseline_not_reachable_no_override";
    if(out.changed){if(persistent&&!event_driven_static)discovery.selected(out.cue_id);++selections;original_next=true;for(auto& kv:cues)if(kv.second.id==out.cue_id){++kv.second.uses;break;}}
    return out;
  }
};
}
