"""Compile and execute the same pure C++ policy used inside native RACER."""
from pathlib import Path
import shutil
import subprocess
import pytest

def test_fresh_review_requires_latest_same_source_rgbd(tmp_path):
    compiler=shutil.which('g++') or shutil.which('clang++')
    if not compiler:pytest.skip('C++ compiler unavailable')
    header=Path(__file__).resolve().parents[1]/'tools/flight'
    source=tmp_path/'fresh.cpp';binary=tmp_path/'fresh'
    source.write_text(r'''
#include "fresh_review.hpp"
#include "static_guidance_events.hpp"
#include <cassert>
int main(){
 using namespace racer_task;
 TargetRegistry registry("uav0"); StaticGuidanceEvents inbox;
 TargetObservation original; original.key={"uav0","local-session",25};
 original.first=9.;original.capture=10.;original.sequence=1;original.frame_version="local-v1";
 original.has_bearing=true;original.direction={0.,1.,0.};original.identity_input="completed";
 original.identity_deadline=10.75;original.support=3;original.support_first=9.5;original.confidence=.8;
 assert(registry.ingest(original,10.1).accepted);inbox.sync(registry,10.1);
 auto work=inbox.start(10.1);assert(work);
 assert(freshReviewFrame(registry,original,10.74)->sequence==1);
 assert(!freshReviewFrame(registry,original,10.751));
 inbox.waitForFreshFrame(work->token,original);assert(!inbox.start(10.751));
 auto newer=original;newer.capture=10.8;newer.sequence=2;newer.identity_deadline=11.55;
 auto other=newer;other.key.session="other-session";
 assert(registry.ingest(other,10.9).accepted);
 assert(!freshReviewFrame(registry,original,10.9));
 newer.identity_input="pending";assert(registry.ingest(newer,10.91).accepted);
 inbox.sync(registry,10.91);assert(!freshReviewFrame(registry,original,10.91));
 assert(freshReviewCapture(reviewSource(registry,original)->latest.capture,10.91));
 newer.identity_input="completed";newer.sequence=3;
 assert(registry.ingest(newer,10.93).accepted);inbox.sync(registry,10.93);
 const auto* fresh=freshReviewFrame(registry,original,10.93);
 assert(fresh&&fresh->key==original.key&&fresh->sequence==3);
 work=inbox.start(10.93,{other.key});assert(work&&work->evidence.key==original.key&&work->evidence.sequence==3);
 auto epoch=newer;epoch.frame_version="local-v2";epoch.sequence=4;
 assert(registry.ingest(epoch,11.).accepted);assert(!freshReviewFrame(registry,original,11.));

 TargetRegistry stopped;
 assert(stopped.ingest(original,10.1).accepted);
 auto low_pair=original;
 low_pair.sequence=2; low_pair.capture=11.5; low_pair.identity_deadline=12.25;
 low_pair.confidence=.44; low_pair.support=0; low_pair.support_first=11.5;
 assert(stopped.ingest(low_pair,11.55).accepted);
 const auto* paired=freshReviewFrame(stopped,original,11.55);
 assert(paired&&paired->sequence==2&&paired->support==0);
 assert(reviewNavigationFrame(stopped,original,*paired,11.55,12.,false)==&original);
 assert(!reviewNavigationFrame(stopped,original,*paired,12.01,12.,false));
 assert(!reviewNavigationFrame(stopped,original,*paired,12.251,13.,false));
 assert(!freshReviewFrame(stopped,original,11.55,11.6));
 auto wrong_pair=*paired;wrong_pair.key.session="other";
 assert(!reviewNavigationFrame(stopped,original,wrong_pair,11.55,12.,false));
 wrong_pair=*paired;wrong_pair.frame_version="other-map";
 assert(!reviewNavigationFrame(stopped,original,wrong_pair,11.55,12.,false));
 auto strong_pair=*paired;
 strong_pair.confidence=.8;strong_pair.support=3;strong_pair.support_first=11.;
 assert(reviewNavigationFrame(stopped,original,strong_pair,12.01,12.,false)==&strong_pair);

 TargetRegistry scheduled;
 StaticGuidanceEvents queue;
 auto stale=original;stale.key.counter=13;
 assert(scheduled.ingest(stale,10.05).accepted);queue.sync(scheduled,10.05);
 auto current=original;current.key.counter=14;current.capture=11.;current.identity_deadline=11.75;
 assert(scheduled.ingest(current,11.05).accepted);queue.sync(scheduled,11.05);
 const auto first=queue.start(11.05,{},&scheduled);
 assert(first&&first->evidence.key==current.key&&first->priority);
 assert(queue.review_budget.reservations.size()==1&&queue.stats().pending==0);
 queue.retry(first->token);
 const auto retry=queue.start(11.06,{},&scheduled);
 assert(retry&&retry->evidence.key==current.key&&!retry->priority);
 queue.finish(retry->token);

 auto refreshed=stale;refreshed.sequence=2;refreshed.capture=11.1;refreshed.identity_deadline=11.85;
 assert(scheduled.ingest(refreshed,11.15).accepted);queue.sync(scheduled,11.15);
 const auto recovered=queue.start(11.15,{},&scheduled);
 assert(recovered&&recovered->evidence.key==stale.key&&recovered->priority&&recovered->evidence.sequence==2);
}
''')
    subprocess.run([compiler,'-std=c++17','-O2','-I',str(header),str(source),'-o',str(binary)],check=True,capture_output=True,text=True)
    subprocess.run([str(binary)],check=True,capture_output=True,text=True)

def test_native_bounded_bearing_policy(tmp_path):
    compiler=shutil.which('g++') or shutil.which('clang++')
    if not compiler:pytest.skip('C++ compiler unavailable')
    header=Path(__file__).resolve().parents[1]/'tools/flight'
    code=r'''
#include "bearing_bias.hpp"
#include <cassert>
#include <iostream>
int main(){
 using namespace racer_task;
 BearingCandidate baseline{-1,{0,2,1.5},3.141592653589793,10,true};
 BearingCandidate toward{1,{0,.6,1.5},0,12,true};
 BearingCandidate away{2,{0,-.6,1.5},3.141592653589793,7,true};
 std::vector<double> packet{1,0,1,10,0,0,1.5,1,0,0,.4,.9,3,0,0,9,.9};
 BearingBias p;p.enabled=true;
 BearingBias weak;weak.event_driven_static=weak.allow_weak=true;
 auto single=packet;single[12]=1.;single[15]=single[3];
 assert(weak.receive(single,10.)&&!p.receive(single,10.));
 for(std::size_t i=0;i<packet.size();++i){
  for(double bad:{std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()}){
   auto malformed=packet;malformed[i]=bad;
   assert(!p.receive(malformed,10.)&&!weak.receive(malformed,10.));
  }
 }
 BearingBias persistent;persistent.persistent=true;
 assert(persistent.receive(packet,10.));
 assert(persistent.discovery.observe(1,9.,10.,{0,0,1.5},{1,0,0},true,0));
 const auto& stored=persistent.cues.at({0,1});
 const auto weighted=persistent.explain(toward,stored,10.);
 assert(weighted.age_factor==persistent.discovery.value(1,10.)&&weighted.value>0.);
 auto boundary=toward;boundary.yaw=stored.hfov;
 const auto edge=persistent.explain(boundary,stored,10.);
 assert(edge.heading_alignment==0.&&std::abs(edge.heading_angle-stored.hfov)<1e-12);
 boundary.yaw=stored.hfov-.001;
 assert(persistent.explain(boundary,stored,10.).heading_alignment>0.);
 boundary.yaw=stored.hfov+.001;
 assert(persistent.explain(boundary,stored,10.).heading_alignment==0.);
 auto vertical=stored;vertical.direction={0.,0.,1.};
 assert(persistent.explain(toward,vertical,10.).reason=="near_vertical_bearing_unsupported");
 persistent.discovery.fail(1,10.,"fixture");
 const auto cooled=persistent.explain(toward,stored,10.);
 assert(cooled.reason=="failure_cooldown"&&cooled.age_factor==0.&&cooled.value==0.);
 assert(!p.choose(baseline,{toward,away},10).changed); // exact no-cue original
 assert(p.receive(packet,10));
 auto d=p.choose(baseline,{toward,away},10);assert(d.changed && d.selected==1);
 assert(d.scores[1].adjusted_cost>=.5*d.scores[1].base_cost);
 assert(d.scores[1].base_cost>baseline.cost && d.scores[1].adjusted_cost<baseline.cost); // discount is pivotal
 assert(bdDot(toward.point,{1,0,0})==0); // observation point is outside the old target-pixel cone
 BearingBias already;already.enabled=true;assert(already.receive(packet,10));auto sameview=baseline;sameview.yaw=0;
 auto kept=already.choose(sameview,{toward},10);assert(!kept.changed && kept.scores[0].utility>0 && kept.scores[0].adjusted_cost<sameview.cost);
 assert(!p.choose(baseline,{toward},10.1).changed); // forced original after bias
 assert(p.choose(baseline,{toward},31).reason=="no_fresh_evidence");
 BearingBias off;assert(off.receive(packet,10));assert(!off.choose(baseline,{toward},10).changed);
 BearingBias blocked;blocked.enabled=true;assert(blocked.receive(packet,10));toward.reachable=false;
 assert(!blocked.choose(baseline,{toward},10).changed);toward.reachable=true;
 auto same=packet;same[3]=10.1;assert(p.receive(same,10.1)); // refresh cannot reset uses
 assert(p.cues.begin()->second.uses==1);
 same[13]=3;assert(p.receive(same,10.1));assert(!p.choose(baseline,{toward},10.1).changed); // covered sector decay
 BearingBias budget;budget.enabled=true;budget.mission_limit=1;assert(budget.receive(packet,10));
 assert(budget.choose(baseline,{toward},10).changed);
 assert(budget.choose(baseline,{toward},10.2).reason=="mission_bias_budget_exhausted");
 auto fresh=packet;fresh[2]=2;fresh[3]=11;fresh[15]=10;
 assert(budget.receive(fresh,11));assert(!budget.choose(baseline,{toward},11).changed);
 BearingBias timeout;timeout.enabled=true;assert(timeout.receive(packet,10));toward.reachable=false;
 assert(!timeout.choose(baseline,{toward},10).changed);toward.reachable=true;
 fresh[3]=71;fresh[15]=70;assert(timeout.receive(fresh,71));
 assert(timeout.choose(baseline,{toward},71).reason=="mission_bias_budget_exhausted");
 packet[7]=2;assert(!budget.receive(packet,10));
 BearingBias review;review.enabled=true;review.event_driven_static=true;
 packet[7]=1;assert(review.receive(packet,10));
 const auto& capture=review.cues.begin()->second;
 BearingCandidate retreat{3,{-1.5,3.3,1.6},0,5,true};
 BearingCandidate far_side{4,{0,1.1,1.5},0,6,true};
 BearingCandidate nearby{5,{-.12,.59,1.5},0,7,true};
 assert(review.explain(retreat,capture,10).reason=="review_view_behind_capture");
 assert(review.explain(far_side,capture,10).reason=="review_view_far_from_capture_bearing");
 assert(review.choose(baseline,{retreat,far_side,nearby},10).selected==nearby.id);
 nearby.review_view=true;
 auto lower_cost=nearby;lower_cost.id=6;lower_cost.cost=.5;lower_cost.review_view=false;
 auto low_baseline=nearby;low_baseline.id=-1;low_baseline.cost=1.;
 assert(!review.choose(low_baseline,{nearby},10).changed);
 auto priority=review.choose(low_baseline,{nearby,lower_cost},10,true);
 assert(priority.selected==nearby.id&&priority.reason=="fresh_review_priority");
 low_baseline.reachable=false;
 assert(!review.choose(low_baseline,{nearby},10).changed);
 assert(review.choose(low_baseline,{nearby,lower_cost},10,true).selected==nearby.id);
 nearby.reachable=false;
 assert(!review.choose(low_baseline,{nearby,lower_cost},10,true).changed);
 nearby.reachable=true;nearby.point=retreat.point;
 assert(!review.choose(low_baseline,{nearby},10,true).changed);
 assert(p.explain(retreat,p.cues.begin()->second,10.2).value>0); // legacy bearing bias
 std::cout<<"PASS_NATIVE_BEARING no_evidence directional unreachable expired disabled coverage budget invalid\n";
}
'''
    source=tmp_path/'probe.cpp';source.write_text(code);binary=tmp_path/'probe'
    subprocess.run([compiler,'-std=c++17','-O2','-I',str(header),str(source),'-o',str(binary)],check=True,capture_output=True,text=True)
    result=subprocess.run([str(binary)],check=True,capture_output=True,text=True)
    assert 'PASS_NATIVE_BEARING' in result.stdout


def test_expired_native_cues_release_space_without_refilling_mission_budgets(tmp_path):
    from test_native_static_events import compile_run

    compile_run(tmp_path, r"""
#include "bearing_bias.hpp"
#include <cassert>
using namespace racer_task;
std::vector<double> packet(int id,double capture){
 return {1.,0.,double(id),capture,0.,0.,1.5,1.,0.,0.,.4,.9,3.,0.,0.,capture-1.,.9};
}
int main(){
 BearingBias p;p.enabled=true;p.mission_limit=3;
 BearingCandidate base{-1,{0,2,1.5},3.141592653589793,10,true};
 BearingCandidate toward{1,{0,.6,1.5},0,12,true};
 assert(p.receive(packet(1,10),10));
 assert(p.choose(base,{toward},10).changed);
 assert(p.choose(base,{toward},10.1).reason=="forced_original_exploration");
 assert(p.choose(base,{toward},10.2).changed);
 for(int id=2;id<=64;++id)assert(p.receive(packet(id,10),10));
 assert(!p.receive(packet(65,11),11));
 assert(p.receive(packet(65,31),31));
 assert(p.cues.size()==2&&p.cues.at({0,1}).uses==2);
 assert(!p.receive(packet(1,10),31));
 assert(p.receive(packet(1,31),31));
 assert(p.explain(toward,p.cues.at({0,1}),31).reason=="cue_budget_exhausted");
 assert(p.receive(packet(64,30.9),31)); // New-to-cache ID arrived out of order.
 assert(p.choose(base,{toward},31).reason=="forced_original_exploration");
 auto picked=p.choose(base,{toward},31);assert(picked.changed&&picked.cue_id==65);
 assert(p.choose(base,{toward},31).reason=="mission_bias_budget_exhausted");
 BearingBias stat;stat.event_driven_static=true;
 for(int id=1;id<=64;++id)assert(stat.receive(packet(id,10),10));
 assert(!stat.receive(packet(65,31),31)&&stat.hasFresh(31));
 assert(stat.receive(packet(1,31),31)&&stat.cues.size()==64);
}
""")


def test_registry_update_keeps_expired_history_out_of_native_guidance_admission(tmp_path):
    from test_native_static_events import HEADERS, compile_run

    manager_update = (HEADERS/'target_registry_methods.cpp').read_text().split(
        'void FastExplorationManager::taskGuidanceFeedback', 1)[0]
    compile_run(tmp_path, r"""
#include "static_guidance_events.hpp"
#include <cassert>
namespace racer_ros {
 struct Time { inline static double current=10.; static Time now(){return {};} double toSec(){return current;} };
}
namespace Eigen {
 struct Vector3d {
  double v[3]{};Vector3d(double x=0.,double y=0.,double z=0.):v{x,y,z}{}
  double operator[](int i)const{return v[i];}
  Vector3d operator+(const Vector3d& rhs)const{return {v[0]+rhs[0],v[1]+rhs[1],v[2]+rhs[2]};}
  friend Vector3d operator*(double s,const Vector3d& a){return {s*a.v[0],s*a.v[1],s*a.v[2]};}
 };
}
struct SDFMap {
 enum {FREE};void getRegion(Eigen::Vector3d&,Eigen::Vector3d&)const{}
 double getResolution()const{return 1.;}bool isInBox(const Eigen::Vector3d&)const{return true;}
 int getOccupancy(const Eigen::Vector3d&)const{return FREE;}
 int getInflateOccupancy(const Eigen::Vector3d&)const{return 0;}
};
struct Ledger {
 racer_task::TargetRegistry registry;
 int cueId(const racer_task::TargetKey& key){return int(key.counter);}
};
struct FastExplorationManager {
 Ledger* task_target_registry_;
 racer_task::BearingBias task_bearing_policy_;
 bool task_static_event_audit_=false;
 racer_task::StaticGuidanceEvents task_static_events_;
 std::map<racer_task::TargetKey,int> task_target_progress_;
 struct Params {int drone_id_=1;} params;
 Params* ep_=&params;
 SDFMap map;SDFMap* sdf_map_=&map;
 void taskRegistryUpdate();
};
""" + manager_update + r"""
int main(){
 using namespace racer_task;
 Ledger ledger;FastExplorationManager manager;manager.task_target_registry_=&ledger;
 auto& p=manager.task_bearing_policy_;p.enabled=p.persistent=true;
 auto record=[&](int id,double capture){
  TargetObservation o;o.key={"uav0","session",std::uint64_t(id)};o.capture=capture;
  o.first=id<=64?9.:capture-1.;o.has_bearing=true;o.origin={double(id)*2.,0.,1.5};
  o.direction={1.,0.,0.};o.support=3;o.support_first=capture-1.;o.confidence=.9;
  auto& r=ledger.registry.records[o.key];r.latest=o;r.first=o.first;r.last=capture;
 };
 for(int id=1;id<=64;++id)record(id,10.);
 manager.taskRegistryUpdate();assert(p.cues.size()==64&&p.discovery.cues.size()==64);
 assert(p.discovery.begin(1,10.));
 BearingCandidate base{-1,{0,2,1.5},3.141592653589793,10,true};
 BearingCandidate toward{1,{0,.6,1.5},0,12,true};
 auto picked=p.choose(base,{toward},10.);assert(picked.changed&&picked.cue_id==1);
 p.discovery.finish(1,10.,true);p.discovery.planned(88,1);
 racer_ros::Time::current=13.;record(65,13.);
 manager.taskRegistryUpdate();
 assert(p.cues.count({0,65})&&p.discovery.available(65,13.)&&p.hasFresh(13.));
 assert(p.cues.at({0,1}).uses==1&&p.discovery.groups.at(1).uses==1&&p.discovery.mission_attempts==1);
 assert(p.discovery.plans.at(88).cue==1&&!p.discovery.plans.at(88).closed);
 const auto remaining=p.discovery.cues.size();
 manager.taskRegistryUpdate(); // Registry continues to represent all 65 sources.
 assert(ledger.registry.records.size()==65&&p.discovery.cues.size()==remaining);
 assert(!p.cues.count({0,2})&&!p.discovery.cues.count(2));
 assert(!p.discovery.observe(2,9.,10.,{4,0,1.5},{1,0,0},true,0,13.));
 p.discovery.completed(88,13.);
 assert(p.discovery.plans.at(88).closed&&p.discovery.groups.at(1).blocked);
 assert(p.discovery.groups.at(1).reason=="completed_observation_without_localization_progress");
 assert(p.discovery.groups.at(1).attempts==1&&p.discovery.groups.at(1).retries==0);
}
""")
