#pragma once
#include <cmath>
#include <limits>
#include <vector>
namespace racer_task {
// Session/task/proposal/map version identify an immutable published transaction.
// No motion authority: a terminal receipt still needs inactive native feedback.
struct StaticCancellation {
 double session=0.,last_request=-std::numeric_limits<double>::infinity();
 std::vector<double> identity;
 bool requested=false,terminal=false;
 bool enabled()const{return exact(session)&&session>0.;}
 static bool exact(double x){return std::isfinite(x)&&x>=0.&&x<9007199254740992.&&std::floor(x)==x;}
 void reset(){identity.clear();requested=false;terminal=false;last_request=-std::numeric_limits<double>::infinity();}
 bool bind(double task,double proposal,double map){
  reset();if(!enabled()||!exact(task)||task<=0.||!exact(proposal)||proposal>2147483647.||!exact(map)||map<=0.)return false;
  identity={1.,session,task,proposal,map,1.};return true;
 }
 bool due(double now,double interval){
  if(identity.empty()||terminal||!std::isfinite(now)||!std::isfinite(interval)||interval<=0.||
     now<last_request||now-last_request<interval)return false;
  requested=true;last_request=now;return true;
 }
 bool acknowledge(const std::vector<double>& d){
  if(!requested||d.size()!=7||identity.size()!=6)return false;
  for(unsigned i=0;i<6;++i)if(d[i]!=identity[i])return false;
  if(d[6]!=1.&&d[6]!=2.&&d[6]!=3.&&d[6]!=4.)return false;
  terminal=terminal||d[6]==2.||d[6]==3.||d[6]==4.;return true;
 }
 bool released(int active,bool observed_done,bool rejected)const{
  if(identity.empty())return false;
  const int proposal=static_cast<int>(identity[3]);
  if(requested)return terminal&&active!=proposal;
  return (active==proposal&&observed_done)||rejected;
 }
};
}
