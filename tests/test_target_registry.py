from pathlib import Path
import shutil,subprocess,pytest

def test_cpp_target_registry_identity_and_source_protocol(tmp_path):
    compiler=shutil.which('c++')
    if not compiler:pytest.skip('C++17 compiler unavailable')
    root=Path(__file__).resolve().parents[1];header=root/'tools/flight';binary=tmp_path/'registry-probe'
    subprocess.run([compiler,'-std=c++17','-O2','-I',str(header),str(header/'target_registry_probe.cpp'),'-o',str(binary)],check=True,capture_output=True,text=True)
    result=subprocess.run([str(binary)],check=True,capture_output=True,text=True)
    assert 'PASS_TARGET_REGISTRY' in result.stdout


def test_cpp_local_frame_keeps_validation_capacity_and_event_order(tmp_path):
    from test_native_static_events import compile_run

    compile_run(tmp_path, r'''#include "target_registry.hpp"
#include <cassert>
using namespace racer_task;

TargetObservation observation(int id,double capture,std::string robot="uav0"){
 TargetObservation o;
 o.key={robot,"session",std::uint64_t(id)};
 o.sequence=o.revision=1;
 o.first=o.capture=capture;
 o.frame_version="map-v1";
 o.has_position=true;o.localization="reliable";o.position={4.,0.,1.};
 o.observation_id=robot+"-"+std::to_string(id);
 return o;
}

TargetBatchResult sequential(TargetRegistry&r,const std::vector<TargetObservation>&obs,double now){
 TargetBatchResult result;
 std::vector<TargetKey> coexist;
 for(const auto&o:obs){
  auto u=r.ingest(o,now);
  if(u.accepted)result.accepted.push_back(o.key);
  else r.events.push_back({"observation_rejected",u.reason,o.key,{},now});
  if(u.material)result.changed.push_back(o.key);
  auto i=r.records.find(o.key);
  if(i!=r.records.end()&&i->second.latest.capture==o.capture)coexist.push_back(o.key);
 }
 r.associate(coexist,now);
 return result;
}

void equivalent(const TargetRegistry&a,const TargetRegistry&b){
 assert(a.active_sources==b.active_sources&&a.evidence_epoch==b.evidence_epoch);
 assert(a.roots()==b.roots()&&a.cannot_link==b.cannot_link);
 assert(a.observation_origins==b.observation_origins&&a.events.size()==b.events.size());
 assert(a.stats.detections==b.stats.detections&&a.stats.edges==b.stats.edges);
 for(std::size_t i=0;i<a.events.size();++i){
  const auto&x=a.events[i];const auto&y=b.events[i];
  assert(std::tie(x.kind,x.reason,x.key,x.other,x.time)==std::tie(y.kind,y.reason,y.key,y.other,y.time));
 }
 for(const auto&[key,left]:a.records){
  const auto&right=b.records.at(key);
  assert(left.latest.sequence==right.latest.sequence&&left.exclusions==right.exclusions);
  assert(left.history.size()==right.history.size()&&left.revision==right.revision);
  assert(left.has_anchor==right.has_anchor&&left.association_reason==right.association_reason);
 }
}

int main(){
 TargetRegistry r;
 auto local=observation(1,1.);
 auto remote=observation(1,1.,"uav1");
 TargetAlignment proof;
 proof.id="alignment";proof.robot="uav1";
 proof.source_version=proof.target_version="map-v1";proof.valid_until=2.5;
 r.peer_alignments["uav1"]=proof;
 assert(r.ingest(local,1.).accepted&&r.ingest(remote,1.,true).accepted);
 r.associate({remote.key},1.);
 assert(r.aliases.size()==1);
 const auto epoch=r.evidence_epoch;
 const auto event_count=r.events.size();
 auto valid=observation(2,4.);
 auto invalid=observation(3,4.);
 invalid.identity_input="pending";
 invalid.identity_deadline=4.5; // Pending cannot describe a measured position.
 auto rejects_atomically=[&](const std::vector<TargetObservation>&frame){
  bool rejected=false;
  try{r.ingestLocalFrame(frame,4.);}catch(const std::runtime_error&){rejected=true;}
  assert(rejected&&r.evidence_epoch==epoch&&r.events.size()==event_count);
  assert(r.records.size()==2&&r.aliases.size()==1);
 };
 rejects_atomically({valid,invalid});
 rejects_atomically({valid,valid});
 rejects_atomically({valid,observation(3,3.)});
 rejects_atomically({valid,observation(3,4.,"uav1")});
 invalid=observation(3,4.);invalid.key.session="other-session";
 rejects_atomically({valid,invalid});
 assert(!r.ingest(observation(3,4.1),4.).accepted);

 auto conflict=observation(3,4.);conflict.observation_id=local.observation_id;
 auto reference=r;
 auto expected=sequential(reference,{conflict,valid},4.);
 auto actual=r.ingestLocalFrame({conflict,valid},4.);
 assert(actual.accepted==expected.accepted&&actual.changed==expected.changed);
 assert(r.events.at(event_count).reason=="observation_origin_conflict");
 assert(r.events.at(event_count+1).kind=="unlink");
 equivalent(r,reference);
 assert(r.active_sources.size()==1&&r.records.size()==3);

 auto duplicate=r.ingestLocalFrame({valid},4.);
 assert(duplicate.accepted.empty()&&r.events.back().reason=="duplicate_ignored");
 auto later=valid;later.sequence=3;later.capture=4.1;later.observation_id="later";
 assert(r.ingestLocalFrame({later},4.1).accepted.size()==1);
 auto earlier=later;earlier.sequence=4;earlier.capture=4.05;earlier.observation_id="earlier";
 assert(r.ingestLocalFrame({earlier},4.1).accepted.empty());
 assert(r.events.back().reason=="capture_regression_or_first_epoch_changed");
 r.ingestLocalFrame({},7.);
 assert(r.active_sources.empty()&&r.records.size()==3);
 assert(r.ingest(observation(1,7.,"uav2"),7.).reason=="bounded_source_cache_full");
 assert(r.ingest(observation(2,1.,"uav2"),7.).reason=="unsupported_robot_domain");

 TargetRegistry bounded;bounded.per_robot_limit=2;bounded.total_limit=3;
 assert(bounded.ingest(observation(1,4.,"uav1"),4.).accepted);
 auto a=observation(1,4.),b=observation(2,4.),c=observation(3,4.);
 const auto put=bounded.ingestLocalFrame({a,b,c},4.);
 assert((put.accepted==std::vector<TargetKey>{a.key,b.key}));
 assert(bounded.events[3].reason=="bounded_source_cache_full");
 auto fresh=a;fresh.sequence=2;
 assert(bounded.ingestLocalFrame({fresh},4.).accepted.size()==1);
 assert(bounded.ingest(observation(2,4.,"uav1"),4.).reason=="bounded_source_cache_full");
 auto archived=observation(4,1.);
 assert(bounded.ingestLocalFrame({archived},4.).accepted.size()==1);
 assert(bounded.active_sources.size()==3&&bounded.records.size()==4);
 bounded.total_limit=4;
 assert(bounded.ingestLocalFrame({c},4.).accepted.empty()); // Per-robot, not global.
 bounded.ingestLocalFrame({},7.);
 assert(bounded.ingestLocalFrame({c},7.).accepted.size()==1); // Old, still admissible history.

 TargetRegistry foreign;foreign.peer_alignments["uav1"]=proof;
 auto malformed=remote;malformed.first=2.;
 assert(foreign.ingest(malformed,1.,true).reason=="invalid_source_observation_contract");
 foreign.peer_alignments["uav1"].clock_offset_s=.1;
 assert(foreign.ingest(remote,1.,true).reason=="invalid_observation_contract");
 assert(foreign.records.empty());
}
''')

def test_cpp_only_selectable_edges_compete_and_visibility_requires_finite_coverage(tmp_path):
    from test_native_static_events import compile_run

    compile_run(tmp_path, r'''#include "target_registry.hpp"
#include <cassert>
#include <limits>
using namespace racer_task;

TargetObservation measured(int id,double stamp,double x){
 TargetObservation o;
 o.key={"uav0","same-camera",std::uint64_t(id)};
 o.sequence=1;o.first=o.capture=stamp;
 o.frame_version="survey";
 o.position={x,0.,1.};o.radius=.4;
 o.has_position=true;o.localization="reliable";
 return o;
}
int main(){
 TargetRegistry r;
 const auto near=measured(1,1.,.25),at_cost_limit=measured(2,1.,.6),current=measured(3,2.,0.);
 r.ingestLocalFrame({near,at_cost_limit},1.);
 r.ingestLocalFrame({current},2.);
 assert(r.stats.edges==1&&r.stats.linked==1);
 assert(r.component(current.key).count(near.key)&&!r.component(current.key).count(at_cost_limit.key));

 TargetVisibility scan;
 scan.key=current.key;scan.capture=2.25;scan.anchor_capture=2.;
 scan.center=current.position;scan.extent={.4,.4,std::numeric_limits<double>::quiet_NaN()};
 scan.frame_version=current.frame_version;scan.state="clear";scan.scope="visible_surface_region";
 scan.model_id="survey-test";
 assert(!r.observeVisibility(scan,scan.capture));
 scan.extent[2]=.39;assert(!r.observeVisibility(scan,scan.capture));
 scan.extent[2]=.4;assert(r.observeVisibility(scan,scan.capture));
 assert(r.records.at(current.key).visible_region_misses==1);

 auto changed=current;
 changed.sequence=2;changed.capture=2.5;changed.position[0]=3.;
 r.ingestLocalFrame({changed},2.5);
 assert(r.component(current.key).size()==1);
 assert(r.roots().size()==3);
}
''')
