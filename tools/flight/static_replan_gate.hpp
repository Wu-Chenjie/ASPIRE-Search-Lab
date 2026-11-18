#pragma once
#include <cmath>
#include <cstdint>
#include <limits>
namespace racer_task {
// Proposal scheduling only. External native execution authorization still owns
// collision, dynamics, C2 continuity, arrival and the original safety budgets.
struct StaticReplanGate {
 double min_interval_s=-1.; // deployment must supply an independently validated value
 double last_started=-std::numeric_limits<double>::infinity();
 int proposal=-1,predecessor=-1;
 std::uint64_t map_revision=0;
 double boundary=-1.;
 bool published=false;
 bool configured()const{return std::isfinite(min_interval_s)&&min_interval_s>0.;}
 bool ready(double now,double feedback_stamp,bool competing)const{
  return configured()&&std::isfinite(now)&&std::isfinite(feedback_stamp)&&
    now>=feedback_stamp&&now-feedback_stamp<=.5&&!competing&&
    now-last_started>=min_interval_s;
 }
 void started(double now){last_started=now;resetProposal();}
 bool planned(int id,int active,double at,std::uint64_t map){
  resetProposal();
  if(id<0||active< -1||!map||!std::isfinite(at)||(at<0.&&at!=-1.)||(at>=0.&&active<0))return false;
  proposal=id;predecessor=active;boundary=at;map_revision=map;published=false;return true;
 }
 bool mayPublish(bool event_current,std::uint64_t map,int active,double progress,
                 bool stopped,bool stop_and_plan,double now,double feedback_stamp)const{
  if(proposal<0||published||!event_current||map!=map_revision||!map||active!=predecessor||
     !std::isfinite(now)||!std::isfinite(feedback_stamp)||now<feedback_stamp||now-feedback_stamp>.5||
     !std::isfinite(progress)||progress<0.)return false;
  // Preserve the established stop-and-plan restriction. A moving proposal is
  // only schedulable when that restriction was already disabled independently;
  // this class never toggles it or certifies moving continuity.
  if(stop_and_plan&&!stopped)return false;
  if(boundary<0.)return stopped;
  return !stopped&&active>=0&&progress<boundary;
 }
 bool resolved(int active,int candidate,bool rejected)const{
  return published&&proposal>=0&&(active==proposal||(candidate==proposal&&rejected));
 }
 void resetProposal(){proposal=-1;predecessor=-1;boundary=-1.;map_revision=0;published=false;}
};
}
