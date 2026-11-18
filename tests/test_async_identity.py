"""Readiness scheduling contracts. Synthetic components, not flight evidence."""
import json
from pathlib import Path
import pytest
from test_native_static_events import compile_run
from test_identity_surface import sample, K, T
from person_search.presence import PresenceTracker
from person_search.registry_evidence import RegistryEvidence
from person_search.surface import localize_surfaces


def test_producer_pending_complete_unavailable_monotone_and_confirmation():
    t=PresenceTracker();wire=RegistryEvidence(session='async');d,depth=sample();depth[:]=2.
    rows,_=localize_surfaces([d],depth,K,T)
    for s in (10.,10.264,10.528,10.792,11.056):
        ns=round(s*1e9);t.observe_rgb([d],ns,320,240,s+.01)
        wire.capture(t,ns,None,s+.02)
        packet=wire.batch(t,ns,s+.02)['observations'][0]
        assert packet['identity_input_state']=='pending'
        assert packet['identity_input_deadline_s']==pytest.approx(s+.75)
    assert packet['visual_state']=='visually_confirmed'  # geometry never gates RGB
    wire.capture(t,ns,rows,s+.03)
    p=wire.batch(t,ns,s+.03)['observations'][0]
    assert p['identity_input_state']=='completed' and 'ix' in p
    wire.capture(t,ns,None,s+.04)
    assert wire.batch(t,ns,s+.04) is None
    wire.reject(t,ns,s+.05,'capture_evicted')
    p=wire.batch(t,ns,s+.05)['observations'][0]
    assert p['identity_input_state']=='unavailable' and 'ix' not in p
    wire.capture(t,ns,None,s+.06)
    assert wire.batch(t,ns,s+.06) is None


CPP=r'''#include "static_guidance_events.hpp"
#include <cassert>
using namespace racer_task;
TargetObservation anchor(int id=1,double x=4,bool confirmed=true){TargetObservation o;
 o.key={"uav0","s",std::uint64_t(id)};o.sequence=1;o.first=o.capture=1;o.frame_version="v";
 o.has_position=true;o.localization="reliable";o.position={x,0,1};o.radius=.2;o.confidence=.9;
 o.visual=confirmed?"visually_confirmed":"person_candidate";return o;}
TargetObservation sample(int id,double t,int seq,bool pending=false){auto o=anchor(id);
 o.first=10;o.capture=t;o.sequence=seq;o.has_position=false;o.localization="unknown";o.visual="person_candidate";
 o.has_bearing=true;o.direction={1,0,0};o.origin={0,0,1};o.support=seq>=3?3:1;o.support_first=10;
 o.has_identity_surface=!pending;o.identity_surface={4.05,0,1};o.identity_radius=.15;
 o.identity_input=pending?"pending":"completed";o.identity_deadline=t+.75;return o;}
void put(TargetRegistry&r,StaticGuidanceEvents&q,TargetObservation o,double now=-1){
 if(now<0)now=o.capture;assert(r.ingest(o,now).accepted);r.associate({o.key},now);q.sync(r,now);}
void setup(TargetRegistry&r,StaticGuidanceEvents&q){put(r,q,anchor());put(r,q,sample(2,10,1));put(r,q,sample(2,10.264,2));put(r,q,sample(2,10.528,3,true));}
int main(){
 // Real failure mechanism: navigation qualifies before exact-capture geometry.
 TargetRegistry r;StaticGuidanceEvents q;setup(r,q);auto p=sample(2,10.528,3,true);
 assert(r.roots().size()==2&&!r.groupConfirmed(p.key));assert(q.stats().waiting==1&&!q.start(10.54));
 assert(!q.start()); // no-arg compatibility must not bypass readiness
 auto stats=q.stats();for(int i=0;i<1000;++i){assert(!r.ingest(p,10.54).accepted);r.associate({p.key},10.54);q.sync(r,10.54);assert(!q.start(10.54));}
 assert(q.stats().readiness_updates==stats.readiness_updates&&q.stats().readiness_checks==stats.readiness_checks);
 auto ready=sample(2,10.528,4);put(r,q,ready,10.56);
 assert(r.groupConfirmed(p.key)&&!q.start(10.56));assert(!r.records.at(p.key).confirmed&&!r.records.at(p.key).has_anchor);
 // A late higher sequence pending cannot regress exact-frame completion; visual
 // confirmation still progresses and no duplicate/late packet creates a task.
 p.sequence=5;p.visual="visually_confirmed";put(r,q,p,10.57);
 assert(r.records.at(p.key).latest.identity_input=="completed");assert(r.records.at(p.key).latest.has_identity_surface);
 assert(r.records.at(p.key).confirmed&&!q.start(10.57));
 assert(!r.ingest(ready,10.57).accepted);
 // No geometry ever arrives: release the ORIGINAL brief qualification at the
 // original capture deadline, even with no further registry callback.
 TargetRegistry never;StaticGuidanceEvents nq;setup(never,nq);assert(!nq.start(11.277));
 auto w=nq.start(11.278);assert(w&&w->evidence.capture==10.528&&!nq.start(11.278));nq.retry(w->token);
 w=nq.start(11.279);assert(w);assert(nq.finish(w->token));assert(!nq.start(50));
 // Resending/new pending capture cannot extend the same qualification wait.
 TargetRegistry more;StaticGuidanceEvents mq;setup(more,mq);auto next=sample(2,10.792,4,true);put(more,mq,next);
 w=mq.start(10.8);assert(w&&w->evidence.capture==10.528);mq.finish(w->token);
 next=sample(2,11.056,5,true);put(more,mq,next);assert(!mq.start(11.06));
 // Explicit unavailable/invalid surface ends waiting immediately as unknown.
 for(int mode=0;mode<3;++mode){TargetRegistry bad;StaticGuidanceEvents bq;setup(bad,bq);
 auto o=sample(2,10.528,4);if(mode==0){o.has_identity_surface=false;o.identity_input="unavailable";}
 if(mode==1)o.identity_surface={6,0,1};if(mode==2){o.has_identity_surface=false;}
 put(bad,bq,o,10.55);w=bq.start(10.55);assert(w&&!bad.groupConfirmed(o.key));bq.finish(w->token);
 }
 // Wrong frame invalidates old geometry; cannot resolve as the old person.
 TargetRegistry frame;StaticGuidanceEvents fq;setup(frame,fq);auto wrong=sample(2,10.528,4);wrong.frame_version="other";put(frame,fq,wrong,10.55);
 assert(frame.roots().size()==2);w=fq.start(10.55);assert(w&&w->evidence.frame_version=="other");fq.finish(w->token);
 // Old capture, even with newer sequence, does not release/resolve current wait.
 TargetRegistry old;StaticGuidanceEvents oq;setup(old,oq);auto late=sample(2,10.264,10);assert(!old.ingest(late,10.55).accepted);oq.sync(old,10.55);assert(!oq.start(10.55));
 // A genuine new weak cue never waits for depth/confirmation/localization.
 TargetRegistry fresh;StaticGuidanceEvents weak;weak.allow_weak=true;put(fresh,weak,anchor());auto only=sample(2,10,1,true);put(fresh,weak,only);
 w=weak.start(10.01);assert(w);weak.finish(w->token);
 // Legacy unknown or no confirmed anchor: original qualified event remains usable.
 for(int mode=0;mode<3;++mode){TargetRegistry unknown;StaticGuidanceEvents uq;
 if(mode!=1)put(unknown,uq,anchor(1,4,mode!=2));
 put(unknown,uq,sample(2,10,1));put(unknown,uq,sample(2,10.264,2));auto o=sample(2,10.528,3,true);
 if(mode==0){o.identity_input="unknown";o.identity_deadline=0;}put(unknown,uq,o);
 w=uq.start(10.54);assert(w&&w->evidence.key.counter==2);uq.finish(w->token);}
 // Neighbour anchors and contemporaneous unresolved sources compete, never
 // silently choose nearest. Coexistence exclusions prevent waiting as well.
 for(int mode=0;mode<3;++mode){TargetRegistry near;StaticGuidanceEvents iq;put(near,iq,anchor());
 if(mode==0){auto neighbour=anchor(3,4.1,false);neighbour.exclusions={1};put(near,iq,neighbour);}
 if(mode==1){auto o=sample(3,10.4,1);put(near,iq,o);}
 put(near,iq,sample(2,10,1));put(near,iq,sample(2,10.264,2));auto o=sample(2,10.528,3,true);
 if(mode==2)o.exclusions={1};put(near,iq,o);w=iq.start(10.54);assert(w&&w->evidence.key.counter==2);iq.finish(w->token);}
 // Two simultaneous pending sources for one anchor are both unknown; neither
 // blocks the other with a forged identity, and one shared slot still applies.
 TargetRegistry pair;StaticGuidanceEvents pq;put(pair,pq,anchor());
 for(int id:{2,3}){put(pair,pq,sample(id,10,1));put(pair,pq,sample(id,10.264,2));put(pair,pq,sample(id,10.528,3,true));}
 w=pq.start(10.54);assert(w&&!pq.start(10.54));pq.finish(w->token);w=pq.start(10.54);assert(w);pq.finish(w->token);
 // Input cache eviction leaves an unknown result without losing a short window.
 TargetRegistry evict;StaticGuidanceEvents eq;setup(evict,eq);auto gone=sample(2,10.792,4,true);gone.has_bearing=false;put(evict,eq,gone);
 w=eq.start(10.8);assert(w&&w->evidence.capture==10.528);eq.finish(w->token);
 // Queue capacity does not restart wait or duplicate an active computation.
 TargetRegistry bounded;StaticGuidanceEvents bq(2,1);setup(bounded,bq);auto other=sample(3,10.6,3,true);put(bounded,bq,other);
 assert(bq.stats().entities==2);w=bq.start(11.278);assert(w&&!bq.start(11.278));bq.finish(w->token);assert(!bq.start(20));
 // Waiting is per entity: a new independent short-lived target can use the
 // single slot immediately while the possible confirmed repeat is deferred.
 TargetRegistry fair;StaticGuidanceEvents fairq;setup(fair,fairq);auto independent=sample(3,10.55,3,true);
 independent.first=10.;independent.support_first=10.;put(fair,fairq,independent);
 w=fairq.start(10.56);assert(w&&w->evidence.key.counter==3);fairq.finish(w->token);assert(!fairq.start(10.56));
 // Same-capture retransmission cannot lengthen an already shorter validity.
 TargetRegistry bound;StaticGuidanceEvents limitq;put(bound,limitq,anchor());put(bound,limitq,sample(2,10,1));put(bound,limitq,sample(2,10.264,2));
 auto shortp=sample(2,10.528,3,true);shortp.identity_deadline=10.6;put(bound,limitq,shortp);
 shortp.sequence=4;shortp.identity_deadline=11.278;put(bound,limitq,shortp,10.55);
 assert(bound.records.at(shortp.key).latest.identity_deadline==10.6);assert(!limitq.start(10.599));w=limitq.start(10.6);assert(w);limitq.finish(w->token);
 // Reappearance of associated confirmed source stays suppressed; a reversible
 // contradictory surface can become a fresh unconfirmed event without waiting.
 auto reappear=sample(2,14,6);reappear.first=10;put(r,q,reappear);assert(!q.start(14));
 TargetRegistry split;StaticGuidanceEvents sq;setup(split,sq);put(split,sq,sample(2,10.528,4),10.56);
 auto changed=sample(2,14,5);changed.identity_surface={6,0,1};put(split,sq,changed);w=sq.start(14);assert(w);sq.finish(w->token);
}
'''

def test_native_async_identity_admission_boundaries(tmp_path):
    compile_run(tmp_path,CPP)


def test_strict_async_protocol_when_boost_available(tmp_path):
    import shutil,subprocess
    compiler=shutil.which('c++')
    p=subprocess.run([compiler,'-std=c++17','-x','c++','-fsyntax-only','-'],input='#include <boost/property_tree/json_parser.hpp>\n',text=True,capture_output=True)
    if p.returncode:pytest.skip('Boost unavailable on this host')
    compile_run(tmp_path,r'''#include "target_protocol.hpp"
#include <cassert>
using namespace racer_task;
int main(){TargetObservation o;o.key={"uav0","s",1};o.sequence=1;o.first=o.capture=10;o.frame_version="v";
 o.identity_input="pending";o.identity_deadline=10.75;TargetRegistry r;
 auto text=targetObservationJSON(o,1,"source_record");auto parsed=readTargetObservation(parseTargetTree(text));assert(r.valid(parsed,10));
 assert(parsed.identity_input=="pending");assert(!r.ingest(parsed,10,true).accepted);
 auto peer=readTargetObservation(parseTargetTree(targetObservationJSON(o,1)));assert(peer.identity_input=="unknown");
 for(int mode=0;mode<7;++mode){auto t=parseTargetTree(text);
 if(mode==0)t.put("identity_input_state","made_up");
 if(mode==1)t.erase("identity_input_deadline_s");
 if(mode==2)t.put("identity_input_deadline_s",11.);
 if(mode==3)t.put("identity_input_deadline_s",10.);
 if(mode==4)t.erase("identity_input_state");
 if(mode==5)t.put("identity_input_state","unknown");
 if(mode==6){t.put("identity_surface_policy","registered_surface_anchor_v1");t.put("ix",4);t.put("iy",0);t.put("iz",1);t.put("identity_radius_m",.1);}
 bool rejected=false;try{auto x=readTargetObservation(t);rejected=!r.valid(x,10);}catch(...){rejected=true;}assert(rejected);
 }
 o.identity_input="completed";o.has_identity_surface=true;o.identity_surface={4,0,1};o.identity_radius=.15;
 assert(r.ingest(o,10).accepted);auto regress=o;regress.sequence=2;regress.identity_input="pending";regress.has_identity_surface=false;regress.visual="visually_confirmed";
 assert(r.ingest(regress,10.1).accepted);assert(r.records.at(o.key).confirmed);assert(r.records.at(o.key).latest.has_identity_surface);
}
''')
