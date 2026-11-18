"""Independent identity range support: no new localization or confirmation."""
import json
import shutil
import subprocess
import numpy as np
import pytest
from test_native_static_events import compile_run
from test_registry_evidence import PresenceTracker,RegistryEvidence,sample,K,T,localize_surfaces


def test_capture_identity_support_does_not_bypass_localization_or_visual_gates():
    t=PresenceTracker();w=RegistryEvidence(session='unit');d,depth=sample();depth[:]=2.
    rows,_=localize_surfaces([d],depth,K,T)
    for s in (10.,10.264,10.528):
        ns=round(s*1e9);t.observe_rgb([d],ns,320,240,s+.01)
        w.capture(t,ns,rows,s+.02);p=w.batch(t,ns,s+.02)['observations'][0]
        assert p['identity_surface_policy']=='registered_surface_anchor_v1'
        assert not p['has_position'] and p['visual_state']=='person_candidate'
        assert 'px' not in p
    w.reject(t,ns,10.55,'missing_capture_tf')
    assert 'identity_surface_policy' not in w.batch(t,ns,10.55)['observations'][0]
    # Awaiting/mismatched/stale captures never borrow an old range measurement.
    ns=10_792_000_000;t.observe_rgb([d],ns,320,240,10.8)
    w.capture(t,ns,None,10.8)
    assert 'identity_surface_policy' not in w.batch(t,ns,10.8)['observations'][0]
    w.capture(t,ns,rows,10.82)
    assert 'identity_surface_policy' not in w.batch(t,ns,11.6)['observations'][0]


CPP=r'''#include "static_execution_monitor.hpp"
#include <cassert>
using namespace racer_task;
TargetObservation anchor(int id=1,TargetVec p={4,0,1},bool confirmed=true){
 TargetObservation o;o.key={"uav0","s",std::uint64_t(id)};o.sequence=1;o.first=o.capture=1;
 o.frame_version="v1";o.has_position=true;o.localization="reliable";o.position=p;o.radius=.2;
 o.confidence=.9;o.visual=confirmed?"visually_confirmed":"person_candidate";return o;
}
TargetObservation patch(int id,double at,TargetVec p={4.05,0,1}){
 auto o=anchor(id);o.first=10;o.capture=at;o.sequence=1+std::uint64_t(std::round((at-10)*1000));
 o.has_position=false;o.localization="unreliable";o.visual="person_candidate";
 o.has_identity_surface=true;o.identity_surface=p;o.identity_radius=.15;
 o.has_bearing=true;o.origin={0,0,1};o.direction={1,0,0};o.support=3;o.support_first=9.5;return o;
}
void put(TargetRegistry&r,TargetObservation o){assert(r.ingest(o,o.capture).accepted);r.associate({o.key},o.capture);}
void three(TargetRegistry&r,int id,TargetVec p={4.05,0,1}){for(double at:{10.,10.2,10.4})put(r,patch(id,at,p));}
int main(){
 TargetRegistry r;put(r,anchor());auto p=patch(2,10.);put(r,p);
 assert(r.roots().size()==2); // single frame cannot merge
 assert(!r.ingest(p,10).accepted);r.associate({p.key},10);assert(r.roots().size()==2);
 put(r,patch(2,10.2));assert(r.roots().size()==2);put(r,patch(2,10.4));
 assert(r.roots().size()==1&&r.groupConfirmed(p.key));
 assert(!r.records.at(p.key).confirmed&&!r.records.at(p.key).has_anchor&&!r.records.at(p.key).latest.has_position);
 assert(r.currentPosition(p.key,10.4)==nullptr); // old anchor stays history
 StaticGuidanceEvents q;q.sync(r,10.4);assert(!q.start()); // no repeated confirmed-person detour
 // Missing/occluded measurements do not erase an existing correspondence.
 auto missing=patch(2,10.6);missing.has_identity_surface=false;missing.has_bearing=false;put(r,missing);
 assert(r.roots().size()==1&&r.groupConfirmed(p.key));
 // A real contradictory range reverses the new alias.
 put(r,patch(2,10.8,{6,0,1}));assert(r.roots().size()==2);
 // Different people along the same ray cannot merge from bearing alone/range mismatch.
 TargetRegistry depth;put(depth,anchor());three(depth,2,{6,0,1});assert(depth.roots().size()==2);
 TargetRegistry rays;put(rays,anchor());for(double at:{10.,10.2,10.4}){auto o=patch(2,at);o.has_identity_surface=false;put(rays,o);}assert(rays.roots().size()==2);
 // Two known neighbours compete; no nearest-only or greedy merge.
 TargetRegistry neighbours;auto a=anchor(),b=anchor(2,{4.1,0,1});neighbours.ingest(a,1);neighbours.ingest(b,1);neighbours.associate({a.key,b.key},1);
 three(neighbours,3);assert(neighbours.roots().size()==3&&neighbours.stats.ambiguous==1);
 // Co-observed identities remain distinct even when spatially compatible.
 TargetRegistry coexist;put(coexist,anchor());three(coexist,2);
 a=anchor();a.sequence=2;a.capture=10.6;b=patch(2,10.6);
 coexist.ingest(a,10.6);coexist.ingest(b,10.6);coexist.associate({a.key,b.key},10.6);assert(coexist.roots().size()==2);
 // New captures must independently support the same anchor/frame and time window.
 for(int mode=0;mode<4;++mode){TargetRegistry bad;put(bad,anchor());
  for(double at:{10.,10.2,10.4}){auto o=patch(2,at);
   if(mode==0)o.frame_version="other";
   if(mode==1&&at==10.2)o.has_identity_surface=false;
   if(mode==2&&at==10.2)o.identity_surface={6,0,1};
   if(mode==3&&at==10.2)o.confidence=.49;
   put(bad,o);
  }assert(bad.roots().size()==2);
 }
 TargetRegistry stale;put(stale,anchor());put(stale,patch(2,10.));put(stale,patch(2,10.2));
 p=patch(2,10.4);stale.ingest(p,13);stale.associate({p.key},13);assert(stale.roots().size()==2);
 auto old=p;old.sequence=1;assert(!stale.ingest(old,13).accepted);
 // Actual inbox + execution monitor: confirmation after an accepted task must
 // invalidate work, request exact cancellation, retain until receipt AND inactive.
 TargetRegistry live;put(live,anchor(1,{4,0,1},false));three(live,2);
 StaticGuidanceEvents inbox;inbox.sync(live,10.4);auto work=inbox.start();assert(work&&!inbox.start());
 StaticCancellation cancel;cancel.session=123;assert(cancel.bind(work->token,42,7));
 StaticExecutionMonitor monitor;assert(monitor.accept(*work,cancel,42,42,false));
 monitor.feedback(42,false);assert(inbox.finish(work->token));assert(inbox.stats().active==0);
 a=anchor();a.sequence=2;a.capture=10.5;put(live,a);inbox.sync(live,10.5);
 assert(!inbox.current(*work)&&!inbox.start());int sent=0;
 monitor.tick(inbox,10.5,.5,[&](auto id){assert(id==cancel.identity);++sent;});assert(sent==1);
 monitor.tick(inbox,10.6,.5,[&](auto){++sent;});assert(sent==1);
 auto receipt=cancel.identity;receipt.push_back(3);auto wrong=receipt;wrong[2]++;
 assert(!monitor.acknowledge(wrong));assert(monitor.acknowledge(receipt));assert(monitor.size()==1);
 monitor.feedback(-1,false);assert(monitor.size()==0);assert(!monitor.acknowledge(receipt));
 // A unique new alias can add evidence to an executing review. The queued
 // solver snapshot remains strict, and neither a split nor a confirmation
 // may let the existing execution continue as if its identity were current.
 for(bool split:{false,true}){
  TargetRegistry tracked;
  auto source=anchor(2,{4,0,1},false);source.has_bearing=true;
  source.direction={1,0,0};source.support=3;source.support_first=.5;
  put(tracked,source);
  StaticGuidanceEvents events;events.sync(tracked,1.);
  auto task=events.start();assert(task);
  assert(cancel.bind(task->token,42,7));
  StaticExecutionMonitor execution;assert(execution.accept(*task,cancel,42,42,false));
  execution.feedback(42,false);
  assert(events.finish(task->token));
  events.setExecutingSources(execution.reservedSources());
  auto new_source=anchor(1,{4.05,0,1},false);
  new_source.first=new_source.capture=2.;put(tracked,new_source);
  assert(tracked.component(source.key).count(new_source.key));
  events.sync(tracked,2.);assert(!events.current(*task));
  int cancels=0;auto send=[&](auto id){assert(id==cancel.identity);++cancels;};
  execution.tick(events,2.,.5,send);
  assert(cancels==0&&execution.reservedSources().count(new_source.key));
  events.setExecutingSources(execution.reservedSources());
  execution.feedback(42,true,2.1,1.);
  new_source.sequence=2;new_source.capture=2.3;
  if(split)new_source.position={6,0,1};else new_source.visual="visually_confirmed";
  put(tracked,new_source);events.sync(tracked,2.3);
  assert(execution.tick(events,2.3,.5,send)&&cancels==1);
 }
 // Delayed old candidate state cannot undo confirmation/recreate a task.
 a.sequence=3;a.capture=10.6;a.visual="person_candidate";put(live,a);inbox.sync(live,10.6);
 assert(live.groupConfirmed(p.key)&&!inbox.start());
}
'''

def test_native_surface_correspondence_and_confirmed_active_cancellation(tmp_path):
    compile_run(tmp_path,CPP)


def test_surface_protocol_is_local_only_and_strict(tmp_path):
    compiler=shutil.which('c++')
    if not compiler:pytest.skip('C++ unavailable')
    p=subprocess.run([compiler,'-std=c++17','-x','c++','-fsyntax-only','-'],input='#include <boost/property_tree/json_parser.hpp>\n',text=True,capture_output=True)
    if p.returncode:pytest.skip('Existing environment lacks Boost headers')
    source=CPP.split('int main(){')[0].replace('#include "static_execution_monitor.hpp"','#include "target_protocol.hpp"')+r'''
int main(){auto o=patch(2,10.);auto text=targetObservationJSON(o,1,"source_record");
 auto parsed=readTargetObservation(parseTargetTree(text));assert(parsed.has_identity_surface);
 assert(parsed.identity_surface==o.identity_surface);TargetRegistry r;
 assert(!r.ingest(parsed,10,true).accepted);
 auto summary=readTargetObservation(parseTargetTree(targetObservationJSON(o,1)));
 assert(!summary.has_identity_surface);
 auto t=parseTargetTree(text);t.erase("identity_surface_policy");bool rejected=false;
 try{readTargetObservation(t);}catch(...){rejected=true;}assert(rejected);
 t=parseTargetTree(text);t.put("identity_surface_policy","untrusted");rejected=false;
 try{readTargetObservation(t);}catch(...){rejected=true;}assert(rejected);
 o.identity_radius=.401;assert(!r.valid(o,10));o.identity_radius=.15;o.identity_surface[0]=NAN;assert(!r.valid(o,10));
}
'''
    compile_run(tmp_path,source)
