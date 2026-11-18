#pragma once
#include <cmath>
#include <string>
namespace racer_task {
// Admission for additional static-guidance work. Energy is a declared mission
// envelope, NOT measured battery telemetry or a replacement for emergency RTL.
// Without a validated external power/initial-energy envelope no detour is allowed.
struct StaticMissionBudget {
 double duration_s=-1.,reserve_time_s=-1.,observation_s=-1.;
 double initial_energy_j=-1.,reserve_energy_j=-1.,power_upper_w=-1.;
 std::string model_id;
 double started_s=-1.;
 bool configured()const{
  return !model_id.empty()&&model_id.size()<=128&&std::isfinite(duration_s)&&duration_s>0&&
   std::isfinite(reserve_time_s)&&reserve_time_s>=0&&reserve_time_s<duration_s&&
   std::isfinite(observation_s)&&observation_s>=0&&
   std::isfinite(initial_energy_j)&&initial_energy_j>0&&std::isfinite(reserve_energy_j)&&
   reserve_energy_j>=0&&reserve_energy_j<initial_energy_j&&std::isfinite(power_upper_w)&&power_upper_w>0;
 }
 bool start(double now){
  if(started_s>=0||!configured()||!std::isfinite(now)||now<0)return false;
  started_s=now;return true; // accepted mission trigger, never first target event
 }
 const char* rejection(double now,double trajectory_s)const{
  if(!configured())return "budget_unconfigured";
  if(started_s<0)return "mission_not_started";
  if(!std::isfinite(now)||now<started_s||!std::isfinite(trajectory_s)||trajectory_s<0)
   return "budget_clock_or_duration_invalid";
  const double spent_s=now-started_s+trajectory_s+observation_s;
  if(spent_s+reserve_time_s>duration_s)return "mission_time_reserve";
  // Watts * seconds = joules. Use total mission elapsed, not per-entity resets.
  if(spent_s*power_upper_w+reserve_energy_j>initial_energy_j)
   return "mission_energy_reserve";
  return nullptr;
 }
 const char* reason(double now,double duration)const{
  const char* why=rejection(now,duration);return why?why:"allowed_under_declared_envelope";
 }
 bool allows(double now,double duration)const{return !rejection(now,duration);}
};
}
