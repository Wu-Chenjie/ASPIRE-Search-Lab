"""Regression cases for information preservation, not detector benchmarks."""
import sys
from pathlib import Path
import subprocess
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from person_search.presence import PresenceTracker,PresenceConfig,association_view
from person_search.segmentation import PersonMask
from person_search.surface import localize_surfaces

def put(t,s,ds):return t.observe_rgb(ds,round(s*1e9),160,120,s)
def detection(x=40):
    m=np.zeros((120,160),bool);m[40:80,x:x+40]=True
    return PersonMask((x,40,40,40),.95,m)

def test_disjoint_masks_survive_identical_boxes_and_localize_by_source_index():
    a,b=detection(),detection(80)
    a.box=b.box=(40,40,80,40)
    t=PresenceTracker();assert len(put(t,1,[a,b]))==2
    assert {r['id'] for r in put(t,1.2,[a,b])}=={1,2}
    # Duplicate of one mask is removed without losing the other.
    assert len(put(PresenceTracker(),1,[a,a,b]))==2

def test_lost_recovery_does_not_fill_confirmation_gap_or_force_ambiguity():
    t=PresenceTracker();d=detection()
    for s in (1,1.2,1.4,1.6):put(t,s,[d])
    put(t,1.8,[]);row=put(t,2,[d])[0]
    assert row['id']==1 and row['visual_state']=='person_candidate' and row['rgb_support_captures']==1
    before=t.at(1).captures;put(t,2,[d]);assert t.at(1).captures==before
    put(t,2.2,[]);rows=put(t,2.4,[detection(25),detection(55)])
    assert {r['id'] for r in rows}=={2,3}

def test_retirement_keeps_facts_and_never_reuses_source_ids():
    t=PresenceTracker(PresenceConfig(max_tracks=2))
    for s in (1,1.2,1.4,1.6,1.8):put(t,s,[detection()])
    for i in range(1,140):
        row=put(t,2+i*5,[detection()])[0]
        assert row['id']==i+1
    report=t.report(697)
    assert len(t.tracks)==1 and report['counts']['archived_confirmed_records']==1
    archived=t.confirmed_history[1]
    assert archived['semantic_state']=='confirmed' and archived['current_surface'] is None
    assert archived['lifecycle_state']=='archived'

def test_surface_gate_is_orientation_independent_and_keeps_quality_factors():
    mask=np.zeros((120,160),bool);mask[40:80,40:120]=True
    d=PersonMask((40,40,80,40),.95,mask);z=np.full(mask.shape,2.)
    k=[[200,0,80],[0,200,60],[0,0,1]]
    for transform in (np.diag([1.,-1.,-1.,1.]),np.eye(4)):
        rows,support=localize_surfaces([d],z,k,transform)
        assert rows[0]['xyz'] is not None and support.any()
        q=rows[0]['quality'];assert q['covariance_calibrated'] is False
        assert np.linalg.eigvalsh(q['common_covariance_m2']).min()>=.05**2-1e-12

def test_native_active_capacity_is_not_lifetime_capacity(tmp_path):
    source=tmp_path/'probe.cpp';binary=tmp_path/'probe'
    source.write_text('''#include "target_registry.hpp"
#include <cassert>
using namespace racer_task;
int main(){TargetRegistry r; r.per_robot_limit=2;
for(int i=1;i<=140;++i){TargetObservation o;o.key={"uav0","s",(unsigned)i};o.sequence=1;
o.first=o.capture=i*3.;o.frame_version="m";o.visual="visually_confirmed";
assert(r.ingest(o,o.capture).accepted);assert(r.active_sources.size()==1);}
assert(r.records.size()==140);assert(r.groupConfirmed({"uav0","s",1}));
assert(r.records.begin()->second.history.empty());
auto old=r.records.begin()->second.latest;assert(!r.ingest(old,500.).accepted);
}''')
    subprocess.run(['c++','-std=c++17','-I',str(ROOT/'tools/flight'),str(source),'-o',str(binary)],check=True,capture_output=True)
    subprocess.run([str(binary)],check=True)
