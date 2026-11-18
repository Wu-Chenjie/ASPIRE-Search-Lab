#pragma once
// Return mission accounting only; every motion still needs native planning and
// external measured-map / dynamics authorization. No scene or person truth.
#include "task_recovery_policy.hpp"
namespace racer_task {
class ReturnPolicy {
public:
  enum class State { inactive, planning_home, planning_temporary, executing_home,
    executing_temporary, waiting_retry, waiting_no_goal, returned, retired };
private:
  State state_=State::inactive;
  Point home_{};
  int attempts_=0, proposal_=-1, active_=-1;
  bool retry_home_=false;
  double next_=0.;
public:
  State state() const{return state_;}
  bool started() const{return state_!=State::inactive;}
  bool terminal() const{return state_==State::returned || state_==State::retired;}
  bool executing() const{return state_==State::executing_home || state_==State::executing_temporary;}
  const Point& home() const{return home_;}
  int attempts() const{return attempts_;}
  int active() const{return active_;}
  const char* name() const {
    switch(state_) {
      case State::inactive:return "inactive";
      case State::planning_home:return "planning_home";
      case State::planning_temporary:return "planning_temporary";
      case State::executing_home:return "executing_home";
      case State::executing_temporary:return "executing_temporary";
      case State::waiting_retry:return "waiting_return_retry";
      case State::waiting_no_goal:return "waiting_return_candidates";
      case State::returned:return "returned";
      case State::retired:return "execution_lost_latched";
    }
    return "invalid";
  }
  void begin(const Point& home) {
    if(started())return; // A repeated trigger must not move home or reset scheduling.
    for(double x:home)if(!std::isfinite(x)){state_=State::retired;return;}
    home_=home;state_=State::planning_home;
  }
  bool mayPlan(double now) const {
    return started() && !terminal() && !executing() && proposal_<0 && std::isfinite(now) && now>=next_;
  }
  bool mayUseTemporary() const{return !retry_home_;}
  bool attempt(double now,bool temporary) {
    if(!mayPlan(now))return false;
    if(attempts_<std::numeric_limits<int>::max())++attempts_;
    next_=now+planningRetryDelay(attempts_);
    retry_home_=false;
    state_=temporary?State::planning_temporary:State::planning_home;return true;
  }
  void proposed(int id) {
    if(proposal_<0 && (state_==State::planning_home || state_==State::planning_temporary))proposal_=id;
  }
  bool awaiting(int id) const{return !terminal() && id>=0 && id==proposal_;}
  bool activated(int id) {
    if(!awaiting(id))return false;
    active_=id;proposal_=-1;
    state_=state_==State::planning_temporary?State::executing_temporary:State::executing_home;
    return true;
  }
  // An accepted safe prefix may stop short of home. It is progress, not success.
  bool observe(int id,bool done,const Point& actual,bool fresh) {
    if(!executing() || id!=active_ || !done || !fresh)return false;
    for(double x:actual)if(!std::isfinite(x))return false;
    if(state_==State::executing_home && distance(actual,home_)<=.15)state_=State::returned;
    else{state_=State::planning_home;retry_home_=true;}
    return true;
  }
  void failed(double now) {
    if(!started() || terminal() || executing())return;
    next_=std::max(next_,now+planningRetryDelay(attempts_));
    state_=State::waiting_retry;
  }
  bool rejected(int id,double now) {
    if(id<0 || id!=proposal_ || terminal())return false;
    proposal_=-1;failed(now);return true;
  }
  void noGoal(double now) {
    if(!started() || terminal() || executing())return;
    next_=now+5.;
    retry_home_=true;
    state_=State::waiting_no_goal;
  }
  bool stopped(int id,double now) {
    if(!executing() || id!=active_)return false;
    state_=State::planning_home;
    retry_home_=true;
    failed(now);
    return true;
  }
  void retire(){if(started())state_=State::retired;}
};
}
