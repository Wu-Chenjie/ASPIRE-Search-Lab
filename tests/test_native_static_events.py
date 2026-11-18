"""Host C++ contracts; no ROS, planner, control, flight or GPU claims."""
import json
import re
import shutil
import subprocess
from pathlib import Path
import pytest
from test_navigation_support import feed
from person_search.presence import PresenceTracker
from person_search.registry_evidence import RegistryEvidence

ROOT=Path(__file__).resolve().parents[1]
HEADERS=ROOT/'tools/flight'


def compile_run(tmp_path, source, stdin=''):
    compiler=shutil.which('c++')
    if not compiler: pytest.skip('C++ compiler unavailable')
    cpp=tmp_path/'probe.cpp';cpp.write_text(source)
    binary=tmp_path/'probe'
    result=subprocess.run([compiler,'-std=c++17','-pthread','-O2','-I',str(HEADERS),str(cpp),'-o',str(binary)],capture_output=True,text=True)
    assert result.returncode==0,result.stderr
    return subprocess.run([str(binary)],input=stdin,check=True,capture_output=True,text=True)


def test_actual_producer_fields_are_in_native_strict_allowlist():
    t=PresenceTracker();wire=RegistryEvidence(session='contract')
    for stamp in (1.,1.2,1.4):packet=feed(t,wire,.6,stamp)
    source=(HEADERS/'target_protocol.hpp').read_text()
    block=source.split('static const std::set<std::string> fields{')[1].split('};')[0]
    fields=set(re.findall(r'"([a-z_]+)"',block))
    assert set(packet['observations'][0])<=fields
    assert 'unsupported_navigation_support_policy' in source


def test_real_json_parser_accepts_python_packet_when_boost_available(tmp_path):
    compiler=shutil.which('c++')
    if not compiler:pytest.skip('C++ compiler unavailable')
    probe=subprocess.run([compiler,'-std=c++17','-x','c++','-fsyntax-only','-'],input='#include <boost/property_tree/json_parser.hpp>\n',text=True,capture_output=True)
    if probe.returncode:pytest.skip('Existing environment lacks Boost headers; real JSON parser unverified')
    t=PresenceTracker();wire=RegistryEvidence(session='contract')
    for stamp in (1.,1.2,1.4):packet=feed(t,wire,.6,stamp)
    # Synthetic surface only for the protocol contract, not a measured person.
    row=packet['observations'][0]
    row.update(has_position=True,localization_state='reliable',px=0.,py=0.,pz=3.,radius_m=.1)
    import numpy as np
    from types import SimpleNamespace
    from person_search.static_hypothesis import VisibilityConfig
    from person_search.visibility_evidence import visibility_batch
    wire.last_qualified[1]=dict(xyz=[0.,0.,3.],radius_m=.1,capture_time_s=1.4)
    visibility=visibility_batch(wire,SimpleNamespace(frames={}),2_000_000_000,
        np.full((100,100),5.),np.array([[100.,0,50],[0,100.,50],[0,0,1.]]),
        np.eye(4),2.1,VisibilityConfig(.5,8.,.5,9,.05,1000),'synthetic-contract')
    compile_run(tmp_path,'''#include "target_protocol.hpp"
#include "static_guidance_events.hpp"
#include <cassert>
#include <iostream>
int main(){std::string text;std::getline(std::cin,text);racer_task::TargetRegistry r;
 racer_task::processTargetJSON(r,text,1.41);assert(r.records.size()==1);
 racer_task::StaticGuidanceEvents q;q.sync(r,1.41);assert(q.stats().pending==1);
 auto w=q.start();assert(w&&w->evidence.support==3);assert(q.finish(w->token));
 std::getline(std::cin,text);racer_task::processTargetJSON(r,text,2.1);
 assert(r.records.begin()->second.visibility=="clear");
 assert(r.records.begin()->second.visible_region_misses==1);
 auto output=racer_task::parseTargetTree(racer_task::targetRegistryJSON(r,2.1,0));
 assert(output.get_child("source_records").size()==1);
}
''',json.dumps(packet)+'\n'+json.dumps(visibility))


def test_native_inbox_identity_versions_and_resource_contract(tmp_path):
    compile_run(tmp_path,'''#include "static_guidance_events.hpp"
#include <cassert>
#include <thread>
#include <atomic>
using namespace racer_task;
TargetObservation obs(int id,double at=1.4){TargetObservation o;
 o.key={"uav0","s",std::uint64_t(id)};o.sequence=1;o.first=1.;o.capture=at;
 o.frame_version="map-v1";o.has_bearing=true;o.direction={1,0,0};o.confidence=.6;
 o.support=3;o.support_first=1.;return o;}
int main(){
 TargetRegistry r;StaticGuidanceEvents q;auto a=obs(1);
 assert(r.ingest(a,1.4).accepted);q.sync(r,1.4);assert(q.stats().pending==1);
 for(int i=0;i<100;++i)q.sync(r,1.4);
 assert(q.stats().enqueued==1);
 // A weak later frame cannot erase the pending qualification.
 a.sequence++;a.capture=1.5;a.confidence=.2;assert(r.ingest(a,1.5).accepted);q.sync(r,1.5);
 auto w=q.start();assert(w&&w->evidence.capture==1.4);assert(!q.start());
 // A brief weak frame followed by the same qualification is not a new task.
 a.sequence++;a.capture=1.6;a.confidence=.6;r.ingest(a,1.6);q.sync(r,1.6);
 assert(q.stats().pending==0&&q.valid(w->token));
 a.sequence++;a.capture=1.7;a.frame_version="map-v2";a.confidence=.2;r.ingest(a,1.7);q.sync(r,1.7);
 assert(!q.finish(w->token));assert(!q.start());
 // No-frame time does not erase identity or create work; an actual later
 // qualified capture reopens an event independent of old identity age.
 TargetRegistry retained;StaticGuidanceEvents reacquire;auto old=obs(50);
 retained.ingest(old,1.4);reacquire.sync(retained,1.4);auto initial=reacquire.start();
 assert(initial&&reacquire.finish(initial->token));reacquire.sync(retained,1000.);
 assert(reacquire.stats().entities==0&&!reacquire.start());
 assert(retained.records.size()==1); // facts survive working-cache retirement
 old.sequence++;old.capture=1000.;old.support_first=999.6;
 retained.ingest(old,1000.);reacquire.sync(retained,1000.);
 auto reacquired=reacquire.start();assert(reacquired&&reacquired->evidence.first==1.);
 assert(reacquire.finish(reacquired->token));
 // Confirm without 3D location, suppress all later requests, retain identity.
 a.sequence++;a.capture=1.8;a.confidence=.9;a.visual="visually_confirmed";r.ingest(a,1.8);q.sync(r,1.8);
 assert(!q.start());assert(q.stats().entities==1);q.sync(r,10000.);assert(r.records.size()==1);
 // Merge while a worker owns one source, then undo the merge. Neither
 // shared-resource slots nor the surviving qualification may disappear.
 TargetRegistry x;StaticGuidanceEvents inbox(256,1);
 inbox.review_budget.cooldown_s=0;inbox.review_budget.component_limit=16; // topology-only test
 auto b=obs(2),c=obs(3);
 x.ingest(b,1.4);x.ingest(c,1.4);inbox.sync(x,1.4);auto first=inbox.start();assert(first);
 x.aliases.push_back({b.key,c.key,1,1,"map-v1",0.});inbox.sync(x,1.4);
 assert(inbox.stats().entities==1&&inbox.stats().pending==1);assert(!inbox.start());
 assert(!inbox.finish(first->token));auto merged=inbox.start();assert(merged);
 x.aliases.clear();inbox.sync(x,1.4);assert(!inbox.finish(merged->token));
 assert(inbox.stats().entities==2&&inbox.stats().pending==2);
 // Both raw evidence versions were already consumed before the split.
 // Topology changes cannot mint another budget for the same observations.
 auto split=inbox.start();assert(!split);
 assert(!inbox.start());
 // The same helper used by the ROS manager removes confirmed navigation,
 // including an unconfirmed source aliased to a confirmed source.
 BearingBias policy;policy.enabled=true;policy.persistent=true;
 TargetRegistry confirmed;auto ca=obs(40),cb=obs(41);ca.visual="visually_confirmed";
 confirmed.ingest(ca,1.4);confirmed.ingest(cb,1.4);
 confirmed.aliases.push_back({ca.key,cb.key,1,1,"map-v1",0.});
 policy.receive({1.,0.,41.,1.4,0.,0.,0.,1.,0.,0.,.1,.6,3.,0.,0.,1.,.6},1.4);
 policy.discovery.observe(41,1.,1.4,{0,0,0},{1,0,0},true,0);
 assert(suppressConfirmedGuidance(confirmed,cb.key,policy,0,41));
 assert(policy.cues.empty()&&!policy.discovery.cues.at(41).enabled);
 assert(confirmed.records.size()==2&&confirmed.groupConfirmed(cb.key));
 // Undo a confirmed alias: an actually unconfirmed source can be evaluated
 // again, without inventing a new RGB frame or retaining a permanent union.
 TargetRegistry z;StaticGuidanceEvents reversible;auto d=obs(30),e=obs(31);
 d.visual="visually_confirmed";z.ingest(d,1.4);z.ingest(e,1.4);
 z.aliases.push_back({d.key,e.key,1,1,"map-v1",0.});reversible.sync(z,1.4);
 assert(!reversible.start());z.aliases.clear();reversible.sync(z,1.4);
 auto unlinked=reversible.start();assert(unlinked&&unlinked->entity==e.key);
 assert(reversible.finish(unlinked->token));
 // Many callers share the same global cap; confirmation invalidates running.
 TargetRegistry y;StaticGuidanceEvents global(256,1);
 for(int i=10;i<20;++i)y.ingest(obs(i),1.4);global.sync(y,1.4);
 std::atomic<int> starts{0};std::atomic<std::uint64_t> token{0};std::vector<std::thread> threads;
 for(int i=0;i<16;++i)threads.emplace_back([&]{auto job=global.start();if(job){++starts;token=job->token;}});
 for(auto&t:threads)t.join();assert(starts==1&&global.stats().active==1);
 for(auto&kv:y.records)kv.second.confirmed=true;global.sync(y,1.4);
 assert(global.stats().pending==0&&!global.finish(token));assert(!global.finish(token));
}
''')


def test_confirmation_stops_navigation_without_localization():
    t=PresenceTracker();wire=RegistryEvidence(session='confirmed')
    for stamp in (1.,1.2,1.4,1.6,1.8):packet=feed(t,wire,.9,stamp)
    assert packet['observations'][0]['visual_state']=='visually_confirmed'
    assert not packet['observations'][0]['has_position']
    assert not t.active_bearings(1.81)
    assert packet['observations'][0]['has_bearing']  # passive map evidence retained


def test_short_dropouts_do_not_repeat_work_or_downgrade_pending_strength(tmp_path):
    compile_run(tmp_path,r'''#include "static_guidance_events.hpp"
#include <cassert>
using namespace racer_task;
int main(){TargetRegistry r;StaticGuidanceEvents q;q.allow_weak=true;
 TargetObservation o;o.key={"uav0","jitter",1};o.sequence=1;o.first=o.support_first=1.;
 o.capture=1.4;o.frame_version="map";o.has_bearing=true;o.direction={1,0,0};o.confidence=.8;o.support=3;
 r.ingest(o,1.4);q.sync(r,1.4);
 o.sequence++;o.capture=1.5;o.confidence=.1;r.ingest(o,1.5);q.sync(r,1.5);
 o.sequence++;o.capture=1.6;o.confidence=.6;o.support=1;o.support_first=1.6;
 r.ingest(o,1.6);q.sync(r,1.6);
 auto w=q.start();assert(w&&w->evidence.support==3&&w->evidence.capture==1.4);
 for(int i=0;i<30;++i){o.sequence++;o.capture+=.1;o.confidence=i%2?.8:.1;
  o.support=3;o.support_first=o.capture-.4;r.ingest(o,o.capture);q.sync(r,o.capture);
  assert(q.valid(w->token)&&q.stats().pending==0);
 }
 assert(q.finish(w->token));q.sync(r,100.);assert(!q.start());
 o.sequence++;o.capture=100.;o.support_first=99.6;o.confidence=.8;
 r.ingest(o,100.);q.sync(r,100.);auto next=q.start();assert(next&&q.finish(next->token));
}
''')


def test_generated_overlay_headers_compile_in_actual_layout(tmp_path):
    output=tmp_path/'native'
    subprocess.run(['python3',str(ROOT/'tools/search/build_native.py'),str(output),
                    '--execution-safety','--consistent-inflation'],check=True,capture_output=True,text=True)
    core=output/'src/racer_core'
    manager=(core/'src/exploration_manager/fast_exploration_manager.cpp').read_text()
    assert 'task_static_events_.sync(registry,now)' in manager
    assert 'suppressConfirmedGuidance(registry,kv.first,policy,ep_->drone_id_-1,id)' in manager
    assert 'audit_only' in manager
    # Build the actual generated header; catches source-vs-overlay include names.
    compiler=shutil.which('c++')
    if not compiler:pytest.skip('C++ compiler unavailable')
    source=tmp_path/'generated.cpp'
    source.write_text('#include "static_guidance_events.hpp"\nint main(){racer_task::StaticGuidanceEvents q;return q.stats().pending;}\n')
    subprocess.run([compiler,'-std=c++17','-pthread','-I',str(core/'include/exploration_manager'),
                    str(source),'-o',str(tmp_path/'generated')],check=True,capture_output=True,text=True)
    subprocess.run([str(tmp_path/'generated')],check=True)
