#include "target_registry.hpp"
#include <cassert>
#include <iostream>
using namespace racer_task;
TargetObservation obs(int id,double time,TargetVec p={4,0,1},std::string robot="uav0"){
 TargetObservation o;o.key={robot,"session-a",std::uint64_t(id)};o.sequence=1;o.revision=1;
 o.first=o.capture=time;o.frame_version="shared-v1";o.shared_aligned=true;
 o.visual="visually_confirmed";o.localization="reliable";o.has_position=true;o.position=p;
 o.confidence=.95;o.support=5;o.support_first=time-.8;return o;
}
int main(){
 TargetRegistry one;auto a=obs(1,1);assert(one.ingest(a,1).accepted);one.associate({a.key},1);
 auto b=obs(2,10,{4.3,0,1});assert(one.ingest(b,10).accepted);one.associate({b.key},10);
 assert(one.records.size()==2&&one.roots().size()==1&&one.firstDiscovery(b.key)==1);
 assert(one.currentPosition(b.key,10)->latest.position==b.position);
 assert(one.currentPosition(b.key,13)==nullptr&&one.groupConfirmed(b.key));
 auto count=one.records.at(b.key).history.size();assert(!one.ingest(b,10).accepted);
 assert(one.records.at(b.key).history.size()==count);
 b.sequence=2;b.capture=11;b.first=11;assert(!one.ingest(b,11).accepted);
 TargetRegistry two;auto x=obs(1,1),y=obs(2,1,{4.2,0,1});
 two.ingest(x,1);two.ingest(y,1);two.associate({x.key,y.key},1);
 assert(two.roots().size()==2&&two.cannot_link.count(TargetRegistry::pair(x.key,y.key)));
 auto x2=obs(3,10,{4.05,0,1}),y2=obs(4,10,{4.15,0,1});
 two.ingest(x2,10);two.ingest(y2,10);two.associate({x2.key,y2.key},10);
 assert(two.roots().size()==4&&two.stats.ambiguous==2);
 TargetRegistry rays;auto r=obs(1,1);r.has_position=false;r.localization="unknown";
 rays.ingest(r,1);rays.associate({r.key},1);assert(rays.groupConfirmed(r.key));
 assert(rays.currentPosition(r.key,1)==nullptr&&rays.records.at(r.key).identity=="unresolved");
 assert(rays.pendingDepthReview()->latest.key==r.key);
 assert(!rays.pendingDepthReview({r.key}));
 auto unrelated=obs(2,1);rays.ingest(unrelated,1);rays.associate({r.key,unrelated.key},1);
 assert(rays.pendingDepthReview()->latest.key==r.key);
 r.sequence++;r.capture=2.;r.has_identity_surface=true;r.identity_surface={4.,0.,1.};
 assert(rays.ingest(r,2.).accepted&&rays.pendingDepthReview());
 r.sequence++;r.capture=3.;r.has_position=true;r.localization="reliable";r.position=r.identity_surface;
 assert(rays.ingest(r,3.).accepted&&!rays.pendingDepthReview());
 rays.retire(6.);assert(!rays.currentPosition(r.key,6.)&&!rays.pendingDepthReview());
 TargetRegistry peers;peers.ingest(a,1);
 TargetAlignment proof;proof.id="synthetic-alignment";proof.robot="uav1";proof.source_version=proof.target_version="shared-v1";proof.valid_until=100;
 peers.peer_alignments["uav1"]=proof;
 auto remote=obs(1,1,{4.1,0,1},"uav1");assert(peers.ingest(remote,1,true).accepted);
 peers.associate({remote.key},1);assert(peers.roots().size()==1);
 assert(!peers.ingest(remote,1,true).accepted);
 assert(peers.records.at(a.key).anchor_radius==.15);
 assert(!peers.ingest(a,1,true).accepted);
 remote.revision=2;remote.sequence=2;remote.capture=2;remote.shared_aligned=false;peers.peer_alignments["uav1"].valid_until=1.5;
 peers.ingest(remote,2,true);peers.associate({remote.key},2);assert(peers.roots().size()==2);
 auto previous=remote;previous.revision=1;assert(!peers.ingest(previous,2,true).accepted);
 TargetRegistry frames;frames.ingest(a,1);auto changed=a;changed.sequence=2;changed.capture=2;
 changed.frame_version="unrelated";changed.has_position=false;changed.localization="unknown";
 frames.ingest(changed,2);assert(!frames.records.at(a.key).has_anchor);
 assert(frames.pendingDepthReview()->latest.key==a.key);
 for(int i=1;i<=10;++i){auto z=obs(i,1,{double(i),0,1});if(i!=1)one.ingest(z,1);}
 auto page=one.sync("uav1","request-1",0,20);assert(page.size()==8);
 assert(one.sync("uav1","request-1",0,21).empty());
 assert(one.sync("uav1","request-2",0,20.5).empty());
 auto next=one.sync("uav1","request-3",page.size(),22);assert(next.size()==2);
 
 TargetRegistry rx("uav1");proof.robot="uav0";rx.peer_alignments["uav0"]=proof;
 auto sx=two.records.at(x.key).latest,sy=two.records.at(y.key).latest;
 sx.revision=two.records.at(x.key).revision;sy.revision=two.records.at(y.key).revision;
 assert(rx.ingest(sx,1,true).accepted);rx.associate({sx.key},1);
 assert(rx.ingest(sy,1,true).accepted);rx.associate({sy.key},1);
 assert(rx.roots().size()==2&&rx.cannot_link.count(TargetRegistry::pair(x.key,y.key)));
 TargetRegistry sessions;auto session_a=obs(1,1),session_b=obs(1,2);
 session_b.key.session="session-b";sessions.ingest(session_a,1);sessions.ingest(session_b,2);
 assert(sessions.sync("uav1","page-a",0,20).size()==2);
 assert(sessions.sync("uav1","page-b",1,22).size()==1);
 // A component's latest limb surface must not hide retained torso anchors.
 TargetRegistry parts;auto torso=obs(1,1,{4,0,1.1});torso.radius=.35;
 parts.ingest(torso,1);parts.associate({torso.key},1);
 auto retire=torso;retire.sequence=2;retire.capture=4;retire.has_position=false;retire.localization="expired";parts.ingest(retire,4);
 auto foot=obs(2,10,{4.1,0,1.1});foot.radius=.35;parts.ingest(foot,10);parts.associate({foot.key},10);
 assert(parts.roots().size()==1);
 foot.sequence=2;foot.capture=11;foot.position={4.1,-.25,.34};parts.ingest(foot,11);parts.associate({foot.key},11);
 foot.sequence=3;foot.capture=14;foot.has_position=false;foot.localization="expired";parts.ingest(foot,14);
 auto reentry=obs(3,20,{4.05,0,1.1});reentry.radius=.35;parts.ingest(reentry,20);parts.associate({reentry.key},20);
 assert(parts.roots().size()==1&&parts.component(reentry.key).size()==3);
 assert(parts.currentPosition(reentry.key,23)==nullptr); // History stays history.
 // A nearby competing component still blocks a forced identity decision.
 TargetRegistry competing;auto aa=obs(1,1),bb=obs(2,1,{4.1,0,1});
 competing.ingest(aa,1);competing.ingest(bb,1);competing.associate({aa.key,bb.key},1);
 auto cc=obs(3,10,{4.05,0,1});competing.ingest(cc,10);competing.associate({cc.key},10);
 assert(competing.roots().size()==3&&competing.records.at(cc.key).identity=="ambiguous");
 std::cout<<"PASS_TARGET_REGISTRY reentry coexistence ambiguity RGB_only provenance duplicate order sync frame_invalidation\n";
}

