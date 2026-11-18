#include "target_registry.hpp"
#include "discovery_memory.hpp"
#include <cassert>
#include <iostream>
using namespace racer_task;
TargetObservation sample(int id,double time,TargetVec p){
 TargetObservation o;o.key={"uav0","session-a",std::uint64_t(id)};
 o.first=o.capture=time;o.sequence=o.revision=1;o.frame_version="aligned-v1";
 o.shared_aligned=true;o.has_position=true;o.localization="reliable";
 o.visual="visually_confirmed";o.position=p;o.radius=.35;o.confidence=.9;return o;
}
void add(TargetRegistry&r,TargetObservation o){assert(r.ingest(o,o.capture).accepted);}
void cost_filter(){
 // Three-source minimum: one selectable edge and one geometric-only edge.
 TargetRegistry r;auto a=sample(1,1,{0,0,1}),b=sample(2,1,{.75,0,1});
 add(r,a);add(r,b);r.associate({a.key,b.key},1);
 auto c=sample(3,10,{.05,0,1});add(r,c);r.associate({c.key},10);
 assert(r.component(c.key).count(a.key)&&!r.component(c.key).count(b.key));
 assert(r.roots().size()==2&&r.stats.edges==1&&r.stats.ambiguous==0);
 // Exactly .6 loses to unmatched; geometric gate is an independent limit.
 TargetRegistry boundary;add(boundary,a);auto d=sample(4,10,{.6,0,1});
 add(boundary,d);boundary.associate({d.key},10);assert(boundary.roots().size()==2);
 TargetRegistry geometry;a.radius=.05;add(geometry,a);d.position={.5,0,1};d.radius=.05;
 add(geometry,d);geometry.associate({d.key},10);assert(geometry.roots().size()==2);
}
void reverse_filter(){
 TargetRegistry r;auto a=sample(1,1,{0,0,1});add(r,a);
 auto near=sample(2,10,{.1,0,1}),far=sample(3,10,{.65,0,1});
 add(r,near);add(r,far);r.associate({far.key,near.key},10);
 assert(r.component(near.key).count(a.key));assert(r.component(far.key).size()==1);
 assert(r.stats.edges==1&&r.stats.linked==1);
}
void genuine_competition(){
 TargetRegistry r;auto a=sample(1,1,{0,0,1}),b=sample(2,1,{.2,0,1});
 add(r,a);add(r,b);r.associate({a.key,b.key},1);
 auto c=sample(3,10,{.05,0,1});add(r,c);r.associate({c.key},10);
 assert(r.roots().size()==3&&r.records.at(c.key).identity=="ambiguous");
 TargetRegistry reverse;add(reverse,a);
 auto x=sample(2,10,{.05,0,1}),y=sample(3,10,{.1,0,1});
 add(reverse,x);add(reverse,y);reverse.associate({x.key,y.key},10);
 assert(reverse.roots().size()==3&&reverse.stats.ambiguous==2);
}
void coexistence_scope(){
 // Close people in the same camera capture are never spatially collapsed.
 TargetRegistry same;auto a=sample(1,1,{0,0,1}),b=sample(2,1,{.02,0,1});
 add(same,a);add(same,b);same.associate({a.key,b.key},1);
 assert(same.roots().size()==2&&same.blocked({a.key},{b.key}));
 // Merely sharing an associate call does not imply simultaneous visibility.
 TargetRegistry times;b=sample(2,2,{.02,0,1});add(times,a);add(times,b);
 times.associate({b.key,a.key},2);assert(times.roots().size()==1&&times.cannot_link.empty());
 TargetRegistry robots;b=sample(2,1,{.02,0,1});b.key.robot="uav1";
 TargetAlignment proof;proof.id="synthetic-alignment";proof.robot="uav1";proof.source_version=proof.target_version="aligned-v1";proof.valid_until=100;
 robots.peer_alignments["uav1"]=proof;
 add(robots,a);assert(robots.ingest(b,1,true).accepted);robots.associate({a.key,b.key},1);
 assert(robots.roots().size()==1&&robots.cannot_link.empty());
 TargetRegistry sessions;b=sample(2,1,{.02,0,1});b.key.session="session-b";
 add(sessions,a);add(sessions,b);sessions.associate({a.key,b.key},1);
 assert(sessions.roots().size()==1&&sessions.cannot_link.empty());
 // A delayed source summary still carries a valid historical exclusion.
 TargetRegistry peer("uav1");proof.robot="uav0";peer.peer_alignments["uav0"]=proof;
 a.exclusions={2};assert(peer.ingest(a,1,true).accepted);
 peer.associate({a.key},1);b=sample(2,2,{.02,0,1});
 assert(peer.ingest(b,2,true).accepted);peer.associate({b.key},2);
 assert(peer.roots().size()==2&&peer.blocked({a.key},{b.key}));
}
void lifecycle_and_no_xyz(){
 TargetRegistry r;auto a=sample(1,1,{0,0,1});add(r,a);r.associate({a.key},1);
 auto c=sample(2,10,{.05,0,1});c.has_position=false;c.localization="unknown";
 c.has_bearing=true;c.origin={-1,0,1};c.direction={1,0,0};add(r,c);r.associate({c.key},10);
 assert(r.roots().size()==2&&r.records.at(c.key).identity=="unresolved");
 assert(r.currentPosition(a.key,10)==nullptr&&r.records.at(a.key).has_anchor);
 // An unresolved source is reconsidered when qualified evidence arrives.
 c.sequence=2;c.capture=11;c.has_position=true;c.localization="reliable";
 assert(r.ingest(c,11).accepted);r.associate({c.key},11);
 assert(r.roots().size()==1&&r.firstDiscovery(c.key)==1);
 c.sequence=3;c.capture=14;c.has_position=false;c.has_bearing=false;c.localization="expired";
 assert(r.ingest(c,14).accepted);r.associate({c.key},14);
 auto d=sample(3,30,{.02,0,1});add(r,d);r.associate({d.key},30);
 assert(r.roots().size()==1&&r.component(d.key).size()==3);
 assert(r.currentPosition(d.key,33)==nullptr&&r.records.at(c.key).has_anchor);
 // RGB-only evidence remains unresolved even with a compatible bearing.
 auto no_xyz=sample(4,40,{0,0,1});no_xyz.has_position=false;no_xyz.localization="unreliable";
 no_xyz.has_bearing=true;no_xyz.origin={-1,0,1};no_xyz.direction={1,0,0};
 add(r,no_xyz);r.associate({no_xyz.key},40);assert(r.component(no_xyz.key).size()==1);
 assert(r.records.at(no_xyz.key).identity=="unresolved"&&!r.records.at(no_xyz.key).has_anchor);
}
void messages_and_false_positive(){
 TargetRegistry r;auto a=sample(1,1,{0,0,1});add(r,a);r.associate({a.key},1);
 auto b=sample(2,10,{.05,0,1});add(r,b);r.associate({b.key},10);
 auto aliases=r.aliases.size(),history=r.records.at(b.key).history.size();
 assert(!r.ingest(b,10).accepted);b.sequence=2;b.capture=11;assert(r.ingest(b,11).accepted);
 auto old=b;old.sequence=1;old.capture=10;assert(!r.ingest(old,11).accepted);
 r.associate({b.key},11);assert(r.aliases.size()==aliases&&r.records.at(b.key).history.size()==history+1);
 auto false_candidate=sample(3,12,{10,0,1});false_candidate.visual="person_candidate";
 add(r,false_candidate);r.associate({false_candidate.key},12);
 assert(!r.groupConfirmed(false_candidate.key)&&targetCountBounds(r).upper==1);
 // This association change cannot cure an upstream false visual confirmation.
 false_candidate.sequence=2;false_candidate.capture=13;false_candidate.visual="visually_confirmed";
 assert(r.ingest(false_candidate,13).accepted);r.associate({false_candidate.key},13);
 assert(targetCountBounds(r).upper==2);
}
void budget_no_reset(){
 TargetRegistry r;DiscoveryMemory m;auto a=sample(1,1,{0,0,1});add(r,a);
 r.associate({a.key},1);m.observe(1,1,1,{0,0,1},{1,0,0},true,0);
 assert(m.begin(1,1));m.finish(1,1,true);m.selected(1);m.selected(1);m.fail(1,2,"fixed_failure");
 auto b=sample(2,10,{.05,0,1});add(r,b);r.associate({b.key},10);
 // Camera displacement/rotation can create a separate discovery cue group.
 m.observe(2,r.firstDiscovery(b.key),10,{3,0,1},{0,1,0},true,0);
 assert(m.groups.size()==2);for(const auto&e:r.aliases)m.alias(e.a.counter,e.b.counter);
 auto&g=m.groups.at(m.cues.at(2).group);
 assert(m.groups.size()==1&&g.first==1&&g.epoch<=1&&g.uses==2&&g.attempts==1&&g.blocked);
 assert(!m.begin(2,10)&&m.mission_attempts==1);
}
int main(){cost_filter();reverse_filter();genuine_competition();coexistence_scope();
 lifecycle_and_no_xyz();messages_and_false_positive();budget_no_reset();
 std::cout<<"PASS_ELIGIBLE_EDGES forward reverse exact_cost geometry close_people coexistence_scope delayed_summary lifecycle no_XYZ ordering false_positive budget\n";
}
