"""Synthetic scalar contracts only: not calibrated costs or search outcomes."""
from test_native_static_events import compile_run


def test_static_mode_has_no_age_or_count_ban_and_compares_all_safe_costs(tmp_path):
    compile_run(tmp_path,r'''#include "static_guidance_events.hpp"
#include <cassert>
using namespace racer_task;
int main(){
 BearingBias legacy;legacy.enabled=true;
 std::vector<double> packet={1.,0.,1.,1.4,0.,0.,0.,1.,0.,0.,.1,1.,3.,0.,0.,1.,.6};
 assert(legacy.receive(packet,1.4));
 BearingCandidate base;base.id=-1;base.yaw=3.141592653589793;base.cost=1.;
 BearingCandidate detour;detour.id=0;detour.point={0,.6,0};detour.yaw=0.;detour.cost=1.6;
 auto rejected=legacy.choose(base,{detour},1.4);assert(!rejected.changed);
 assert(rejected.scores.back().reason=="bounded_detour_cost_rejected");
 BearingBias stat=legacy;stat.event_driven_static=true;stat.persistent=true;
 // No legacy discovery records are needed: identity/versions belong to inbox.
 assert(stat.discovery.cues.empty());
 stat.selections=1000;stat.active_since=0.;stat.original_next=true;
 stat.cues.at({0,1}).uses=1000;
 assert(stat.mayScore(1000.));assert(stat.hasFresh(1000.));
 assert(stat.explain(detour,stat.cues.at({0,1}),1000.).value==1.);
 auto selected=stat.choose(base,{detour},1000.);
 assert(selected.changed&&selected.selected==0);
 for(const auto&s:selected.scores)assert(s.adjusted_cost>=0.);
 assert(!legacy.hasFresh(1000.)); // legacy behavior unchanged for comparison
 // Full comparison still rejects an expensive detour and gives exploration
 // credit when its view already supplies the same target evidence.
 detour.cost=3.;assert(!stat.choose(base,{detour},1001.).changed);
 detour.cost=1.6;base.yaw=0.;base.point={0,.6,0};
 assert(!stat.choose(base,{detour},1002.).changed);
 detour.cost=.5;detour.reachable=false;assert(!stat.choose(base,{detour},1003.).changed);
 // Actual evidence timestamp remains untouched; clocks cannot precede capture.
 assert(stat.receive(packet,2000.));assert(stat.cues.at({0,1}).stamp==1.4);
 assert(!stat.receive(packet,1.));
 stat.discount=1.1;assert(stat.choose(base,{detour},2000.).reason=="invalid_nonnegative_cost_discount");
 // A full legacy cache cannot prevent temporary scoring of an admitted ID.
 BearingBias cache;cache.event_driven_static=true;
 for(int i=1;i<=64;++i){packet[2]=i;assert(cache.receive(packet,2000.));}
 {ScopedBearingEntity only(cache,0,65,true);packet[2]=65;assert(cache.receive(packet,2000.));
  assert(cache.cues.size()==1&&cache.cues.count({0,65}));}
 assert(cache.cues.size()==64&&!cache.cues.count({0,65}));
}
''')


def test_retained_strong_snapshot_replaces_newer_weak_cache_only_while_scoring(tmp_path):
    compile_run(tmp_path,r'''#include "static_guidance_events.hpp"
#include <cassert>
using namespace racer_task;
int main(){
 BearingBias p;p.event_driven_static=p.allow_weak=true;
 std::vector<double> weak={1.,0.,1.,2.,0.,0.,0.,0.,1.,0.,.1,.55,1.,0.,0.,2.,.6};
 std::vector<double> strong={1.,0.,1.,1.4,0.,0.,0.,1.,0.,0.,.1,.8,3.,0.,0.,1.,.6};
 assert(p.receive(weak,2.));auto before=p.cues.at({0,1});
 {ScopedBearingEntity snapshot(p,0,1,true);
  assert(p.receive(strong,2.));
  assert(p.cues.size()==1&&p.cues.at({0,1}).stamp==1.4);
  assert(p.cues.at({0,1}).direction[0]==1.);
 }
 assert(p.cues.at({0,1}).stamp==before.stamp&&p.cues.at({0,1}).direction==before.direction);
}
''')
