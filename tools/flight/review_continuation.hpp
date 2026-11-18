#pragma once
// Scheduling and correlation only. Native RACER and the executor retain all
// geometric, dynamics, map freshness, stopping and authorization decisions.
#include <array>
#include <cmath>
#include <cstdint>
#include <string>

namespace racer_task {
class ReviewContinuation {
public:
  enum class Phase { idle, queued, planning, proposed, executing, holding, terminal };
  using Point = std::array<double,3>;
  int max_segments=3;
  double max_duration_s=60., max_path_m=6., observation_hold_s=5.;
  Phase phase=Phase::idle;
  std::int64_t request_id=0, last_request_id=0;
  int plan_id=-1, segment_index=0;
  Point goal{};
  double received=-1., path_used=0., last_pose_stamp=-1., hold_started=-1.;
  Point last_pose{};
  std::string exit_state, exit_reason;
  static double distance(const Point& a,const Point& b) {
    double d=0.;for(int i=0;i<3;++i)d+=(a[i]-b[i])*(a[i]-b[i]);return std::sqrt(d);
  }
  bool active() const {return phase!=Phase::idle && phase!=Phase::terminal;}
  bool validConfig() const {
    return max_segments>0 && max_segments<=20 && std::isfinite(max_duration_s) &&
        max_duration_s>0. && max_duration_s<=300. && std::isfinite(max_path_m) &&
        max_path_m>0. && max_path_m<=30. && std::isfinite(observation_hold_s) &&
        observation_hold_s>=0. && observation_hold_s<=30.;
  }
  bool begin(std::int64_t id,const Point& p,double now) {
    if(active() || id<=last_request_id || id<=0 || !validConfig() || !std::isfinite(now))return false;
    for(double v:p)if(!std::isfinite(v))return false;
    request_id=last_request_id=id;goal=p;received=now;path_used=0.;
    segment_index=0;plan_id=-1;last_pose_stamp=-1.;hold_started=-1.;exit_state.clear();exit_reason.clear();
    phase=Phase::queued;return true;
  }
  bool requestExit(const std::string& state,const std::string& reason) {
    if(!active() || !exit_state.empty())return false;
    exit_state=state;exit_reason=reason;return true;
  }
  void bounds(double now) {
    if(!active() || !exit_state.empty())return;
    if(!std::isfinite(now) || now<received)requestExit("invalid","clock_reversed_or_invalid");
    else if(now-received>=max_duration_s)requestExit("timeout","request_duration_budget");
    else if(path_used>=max_path_m)requestExit("budget_exhausted","actual_path_budget");
  }
  void observe(const Point& p,double stamp,bool fresh) {
    if((phase!=Phase::executing && phase!=Phase::holding) || !fresh || !std::isfinite(stamp) || stamp<=last_pose_stamp)return;
    for(double v:p)if(!std::isfinite(v))return;
    if(last_pose_stamp>=0.)path_used+=distance(p,last_pose);
    last_pose=p;last_pose_stamp=stamp;
  }
  bool attempt(double now) {
    bounds(now);
    if(phase!=Phase::queued || !exit_state.empty())return false;
    if(segment_index>=max_segments) {requestExit("budget_exhausted","segment_budget");return false;}
    ++segment_index;phase=Phase::planning;return true;
  }
  bool proposed(int id,double length) {
    if(phase!=Phase::planning || !exit_state.empty())return false;
    if(!std::isfinite(length) || length<=0.) {requestExit("invalid","invalid_native_path");return false;}
    if(length+path_used>max_path_m) {requestExit("budget_exhausted","proposed_path_exceeds_remaining_budget");return false;}
    plan_id=id;phase=Phase::proposed;return true;
  }
  bool activated(int id,const Point& p,double stamp) {
    if(phase!=Phase::proposed || id!=plan_id)return false;
    phase=Phase::executing;last_pose=p;last_pose_stamp=stamp;return true;
  }
  // A matching completion is consumed only once; caller supplies fresh odom
  // AND fresh authorized-executor completion. No planned/reference endpoint.
  bool complete(int id,bool done,const Point& actual,bool fresh) {
    if(phase!=Phase::executing || id!=plan_id || !done || !fresh)return false;
    if(!exit_state.empty()) {phase=Phase::terminal;return true;}
    if(distance(actual,goal)<=.25) {exit_state="arrived";exit_reason="fresh_actual_pose_and_execution_complete";phase=Phase::terminal;}
    else phase=Phase::queued;
    return true;
  }
  // Called only after emitting a real arrived result. A finite observation
  // window keeps the already completed executor hover; it issues no command.
  bool startObservation(double now) {
    if(phase!=Phase::terminal || exit_state!="arrived" || !std::isfinite(now) ||
        now<received || now>=received+max_duration_s || observation_hold_s<=0.)return false;
    phase=Phase::holding;hold_started=now;exit_state.clear();exit_reason.clear();return true;
  }
  bool waitingObservation(double now) {
    if(phase!=Phase::holding)return false;
    bounds(now);
    if(exit_state.empty() && now-hold_started>=observation_hold_s)
      requestExit("released","observation_hold_elapsed");
    return exit_state.empty();
  }
  void finishExit() {if(active() && !exit_state.empty())phase=Phase::terminal;}
};
} // namespace racer_task
