#include "target_registry.hpp"
#include <cassert>
#include <iostream>
using namespace racer_task;
TargetObservation obs(int i,std::string robot="uav0") {TargetObservation o;o.key={robot,"session",(unsigned)i};o.sequence=1;o.frame_version="frame";o.visual="visually_confirmed";o.confidence=.9;return o;}
int main(){
 TargetRegistry r;for(int i=1;i<=4;i++)assert(r.ingest(obs(i),0).accepted);
 auto a=targetCountBounds(r);assert(a.lower==1&&a.upper==4&&r.aliases.empty());
 r.coexist({obs(1).key,obs(2).key},0);auto before=r.aliases.size();a=targetCountBounds(r);assert(a.lower==2&&a.upper==4&&a.witness.size()==2&&r.aliases.size()==before);
 r.aliases.push_back({obs(3).key,obs(2).key});r.aliases.push_back({obs(4).key,obs(2).key});a=targetCountBounds(r);assert(a.lower==2&&a.upper==2&&r.aliases.size()==2);
 r.aliases.push_back({obs(1).key,obs(2).key});a=targetCountBounds(r);assert(a.inconsistent);
 TargetRegistry empty;a=targetCountBounds(empty);assert(a.lower==0&&a.upper==0);
 TargetRegistry rays;for(int i=1;i<=3;i++){auto o=obs(i);o.has_bearing=true;o.direction={1,0,0};assert(rays.ingest(o,0).accepted);}for(int i=1;i<=3;i++)rays.associate({obs(i).key},0);a=targetCountBounds(rays);assert(rays.aliases.empty()&&a.lower==1&&a.upper==3);
 TargetRegistry bounded;for(int j=0;j<2;j++)for(int i=1;i<=128;i++)assert(bounded.ingest(obs(i,j?"uav1":"uav0"),0).accepted);a=targetCountBounds(bounded);assert(a.upper==256&&a.pair_checks==16384&&a.pair_budget_exhausted);
 std::cout<<"PASS_CONDITIONAL_IDENTITY_BOUNDS"<<std::endl;
}
