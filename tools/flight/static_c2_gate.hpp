#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
namespace racer_task {
// Schedules a proposal only; external admission/transition certify actual C2.
struct StaticC2Gate {
 bool enabled=false;
 double lead=-1.,boundary=-1.,start_progress=0.,started=0.,planning_wall=0.;
 int predecessor=-1,proposal=-1;
 std::uint64_t map=0;
 void reset(){boundary=-1.;predecessor=-1;proposal=-1;map=0;planning_wall=0.;}
 bool prepare(int active,double progress,double duration,double now,double stamp,std::uint64_t revision){
  reset();
  if(!enabled||active<0||!revision||!std::isfinite(lead)||lead<=0.||
     !std::isfinite(progress)||progress<0.||!std::isfinite(duration)||
     !std::isfinite(now)||!std::isfinite(stamp)||now<stamp||now-stamp>.5)return false;
  const double next_boundary=progress+lead;
  if(next_boundary>=duration)return false;
  predecessor=active;start_progress=progress;boundary=next_boundary;started=now;map=revision;return true;
 }
 bool current(int active,double progress,double now,double stamp,std::uint64_t revision)const{
  const double feedback_age=now-stamp;
  if(boundary<0.||active!=predecessor||revision!=map||!revision||
     !std::isfinite(progress)||progress<start_progress||!std::isfinite(now)||now<started||
     !std::isfinite(stamp)||now<stamp||feedback_age>.5||
     !std::isfinite(planning_wall)||planning_wall<0.)return false;
  // Original run contract requires simulation time not faster than wall time.
  // Use the larger observed elapsed bound; do not clip an expired lead window.
  const double upper=std::max(progress+feedback_age,start_progress+std::max(now-started,planning_wall));
  return upper+.05<boundary;
 }
 bool owns(int id)const{return enabled&&proposal>=0&&proposal==id&&boundary>=0.;}
};
}
