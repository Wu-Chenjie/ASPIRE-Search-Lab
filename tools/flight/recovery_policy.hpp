#pragma once
// Rate-limited scheduling only. The caller must still validate the measured map,
// invoke native RACER and obtain the unchanged external execution authorization.
#include <array>
#include <vector>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
namespace racer_task {
using Point = std::array<double,3>;
// The native idle-to-home interval, in simulation seconds.
constexpr double kNoProgressSeconds = 100.;
inline double distance(const Point& a,const Point& b) {
  double s=0;for(int i=0;i<3;++i)s+=(a[i]-b[i])*(a[i]-b[i]);return std::sqrt(s);
}
inline double planningRetryDelay(int attempts) {
  if (attempts <= 0) return .5;
  return attempts >= 8 ? 2. : .5;
}
// Shared review arbitration: current observed-free endpoint is necessary;
// the full native path and external authorization remain separate gates.
template<class MeasuredFree>
bool reviewReady(bool enabled,bool pending,bool returning,double now,double stamp,
                 MeasuredFree measured_free) {
  const double age=now-stamp;
  return enabled && pending && !returning && std::isfinite(age) && age>=0. && age<20. && measured_free();
}
class RecoveryPolicy {
  struct Bad {Point p;double until;};
  std::vector<Bad> bad_;
  bool active_=false;
  int failures_=0, attempts_=0;
  double next_=0;
  double first_failure_=-1.;
  void remember(const Point& p,double now) {
    bad_.erase(std::remove_if(bad_.begin(),bad_.end(),[now](const Bad& b){return b.until<=now;}),bad_.end());
    for(auto& b:bad_)if(distance(p,b.p)<.5){b.until=now+15.;return;}
    bad_.push_back({p,now+15.});if(bad_.size()>64)bad_.erase(bad_.begin());
  }
public:
  bool active() const{return active_;}
  int attempts() const{return attempts_;}
  int failures() const{return failures_;}
  bool stalled(double now) const {
    return active_ && now-first_failure_>=kNoProgressSeconds;
  }
  void failed(const Point& p,double now) {
    remember(p,now);
    if(failures_==0)first_failure_=now;
    if(failures_<std::numeric_limits<int>::max())++failures_;
    if(failures_>=3)active_=true;
    if(attempts_>0)next_=std::max(next_,now+planningRetryDelay(attempts_));
  }
  bool ready(double now) {
    if(!active_ || !std::isfinite(now) || now<next_)return false;
    // Reserve the next check even if the caller defers planning after refresh.
    next_=now+.5;return true;
  }
  bool eligible(const Point& p,const Point& root,double now,double min_distance=.5) const {
    for(double v:p)if(!std::isfinite(v))return false;
    if(distance(p,root)<=min_distance)return false;
    for(const auto& b:bad_)if(b.until>now && distance(p,b.p)<.5)return false;
    return true;
  }
  void attempted(const Point& p,double now) {
    if(attempts_<std::numeric_limits<int>::max())++attempts_;
    remember(p,now);
    next_=now+planningRetryDelay(attempts_);
  }
  void hold(double now){next_=now+5.;}
  // A proposal is insufficient: only the actual external activation resets.
  void activated(){active_=false;failures_=0;attempts_=0;next_=0;first_failure_=-1.;bad_.clear();}
};
// Shared by the native FSM and its integration fixture. Geometry comes only
// from the caller's measured map; this function never invents candidate goals.
template<class MeasuredFree>
int selectGoal(const RecoveryPolicy& policy,const std::vector<Point>& points,
               const std::vector<double>& yaws,const Point& root,double now,
               MeasuredFree measured_free) {
  int selected=-1;double best=INFINITY;
  for(size_t i=0;i<points.size() && i<yaws.size();++i) {
    if(!std::isfinite(yaws[i]) || !policy.eligible(points[i],root,now) || !measured_free(points[i]))continue;
    const double cost=distance(points[i],root);
    if(cost<best){best=cost;selected=int(i);}
  }
  return selected;
}
}
