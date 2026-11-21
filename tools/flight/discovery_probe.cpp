#include "discovery_memory.hpp"
#include "bearing_bias.hpp"
#include <cassert>
#include <iostream>
using namespace racer_task;
int main(){
 DiscoveryMemory m;DiscoveryVec o{0,0,1.5},d{1,0,0};
 assert(m.observe(1,1,1,o,d,true,0));assert(m.value(1,1)==1.);
 for(int t=2;t<=41;++t){assert(m.observe(1,1,t,o,d,true,0));assert(m.observe(t,t,t,o,d,true,0));}
 assert(m.groups.size()==1&&m.groups.begin()->second.first==1);
 assert(m.value(1,41)==0&&m.value(41,41)==0&&!m.begin(1,41));
 DiscoveryMemory n;n.observe(1,1,1,o,d,true,0);assert(n.begin(1,1));n.fail(1,2,"unreachable");
 assert(!n.begin(1,5));
 n.observe(1,1,7,o,d,true,0);assert(n.reason(1,7)=="awaiting_material_change");
 DiscoveryVec shifted{0,.6,1.5},turned{std::cos(.1),std::sin(.1),0};
 n.observe(99,7,7,shifted,turned,true,0);assert(n.groups.size()==1);
 assert(n.begin(99,7));assert(n.groups.begin()->second.retries==1);
 n.fail(99,8,"unreachable2");n.observe(100,13,13,o,d,true,0);assert(n.begin(100,13));
 n.fail(100,14,"unreachable3");n.observe(101,19,19,shifted,turned,true,0);assert(n.begin(101,19));
 n.fail(101,20,"unreachable4");n.observe(102,25,25,o,d,true,0);assert(!n.begin(102,25));
 DiscoveryMemory maps;maps.observe(1,1,1,o,d,true,0);maps.begin(1,1);maps.fail(1,2,"unreachable");
 maps.mapContext(1,std::vector<int>(12,0),2);maps.observe(1,1,7,o,d,true,0);
 auto free=std::vector<int>(12,0);std::fill(free.begin(),free.begin()+6,1);
 maps.mapContext(1,free,7);assert(maps.begin(1,7));
 maps.fail(1,8,"still_unreachable");maps.observe(2,9,9,o,d,true,0);assert(!maps.begin(2,9));
 DiscoveryMemory progress;progress.observe(1,1,1,o,d,true,0);progress.begin(1,1);
 progress.selected(1);progress.planned(10,1);progress.finish(1,1,true);progress.completed(10,2);
 assert(progress.groups.begin()->second.blocked);
 progress.observe(1,1,3,o,d,false,0);assert(progress.reason(1,3)=="navigation_revoked");
 DiscoveryMemory alias;alias.observe(1,1,1,o,d,true,0);
 alias.observe(2,50,50,{3,0,1.5},d,true,0);alias.alias(1,2);
 assert(alias.value(2,50)==0&&alias.groups.begin()->second.first==1);
 BearingBias policy;policy.enabled=policy.persistent=true;
 std::vector<double> packet{1,0,1,10,0,0,1.5,1,0,0,.2,.9,5,0,0,9,.637};
 assert(policy.receive(packet,10));policy.discovery.observe(1,10,10,o,d,true,0);
 BearingCandidate base{-1,{1.2,0,1.5},3.141592653589793,10,true},view{1,{0,.6,1.5},0,11,true};
 assert(policy.choose(base,{view},10).changed);
 packet[2]=2;packet[3]=50;packet[15]=49;assert(policy.receive(packet,50));
 policy.discovery.observe(2,50,50,o,d,true,0);policy.original_next=false;
 assert(policy.discovery.value(2,50)==0.&&!policy.hasFresh(50));
 auto expired=policy.choose(base,{view},50);assert(!expired.changed&&expired.selected==base.id);
 std::cout<<"PASS_DISCOVERY fixed_epoch decay_zero ID_churn cooldown material_map one_retry no_progress revoke alias\n";
}

