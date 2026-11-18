import sys
from pathlib import Path
from types import SimpleNamespace
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from person_search.registry_evidence import RegistryEvidence
from person_search.static_hypothesis import VisibilityConfig
from person_search.visibility_evidence import visibility_batch,visibility_ros_arguments
from test_native_static_events import compile_run


def test_weak_qualification_upgrade_and_actual_cost_envelopes(tmp_path):
    compile_run(tmp_path,'''#include "static_guidance_events.hpp"
#include <cassert>
using namespace racer_task;
int main(){TargetRegistry r;StaticGuidanceEvents q;q.allow_weak=true;
 TargetObservation o;o.key={"uav0","single",1};o.sequence=1;o.first=o.capture=o.support_first=1.;
 o.frame_version="map";o.has_bearing=true;o.direction={1,0,0};o.confidence=1.;o.support=1;
 r.ingest(o,1.);q.sync(r,1.);auto weak=q.start();assert(weak&&weak->evidence.support==1);
 o.sequence++;o.capture=1.4;o.support=3;r.ingest(o,1.4);q.sync(r,1.4);
 assert(q.stats().pending==1&&!q.finish(weak->token));auto strong=q.start();assert(strong&&strong->evidence.support==3);
 BearingBias p;p.enabled=p.event_driven_static=p.allow_weak=p.weak_event=true;
 p.weak_max_extra_cost=.1;p.weak_max_extra_path_m=.1;
 assert(p.receive({1.,0.,1.,1.,0.,0.,0.,1.,0.,0.,.1,1.,1.,0.,0.,1.,.6},1.));
 BearingCandidate b;b.cost=1.;b.yaw=3.141592653589793;b.path_length=1.;
 BearingCandidate c;c.id=0;c.cost=1.05;c.path_length=1.05;c.point={0,.6,0};
 assert(p.choose(b,{c},1.).changed);c.cost=1.6;assert(!p.choose(b,{c},1.).changed);
 c.cost=1.05;c.path_length=1.2;assert(!p.choose(b,{c},1.).changed);
 p.weak_event=false;c.cost=1.6;assert(p.choose(b,{c},1.).changed);
}
''')


def test_capture_aligned_surface_visibility_is_bounded_and_not_body_absence():
    e=RegistryEvidence(session='vis');e.last_qualified[1]=dict(xyz=[0.,0.,3.],radius_m=.1,capture_time_s=1.)
    tracker=SimpleNamespace(frames={2_000_000_000:[]})
    k=np.array([[100.,0,50],[0,100.,50],[0,0,1.]])
    config=VisibilityConfig(.5,8.,.5,9,.05,1000)
    image=np.full((100,100),5.)
    batch=visibility_batch(e,tracker,2_000_000_000,image,k,np.eye(4),2.1,config,'test-calibration')
    row=batch['regions'][0]
    assert row['state']=='clear' and row['scope']=='visible_surface_region'
    assert row['anchor_capture_s']==1. and not row['detected_source']
    assert visibility_batch(e,tracker,2_000_000_000,image,k,np.eye(4),2.1,config,'test-calibration') is None
    image[50,50]=1.
    row=visibility_batch(e,tracker,3_000_000_000,image,k,np.eye(4),3.1,config,'test-calibration')['regions'][0]
    assert row['state']=='occluded'
    image[50,50]=0.
    assert visibility_batch(e,tracker,4_000_000_000,image,k,np.eye(4),4.1,config,'test-calibration')['regions'][0]['state']=='unknown'


def test_native_visibility_preserves_static_identity_and_reopens_real_events(tmp_path):
    compile_run(tmp_path,'''#include "static_guidance_events.hpp"
#include <cassert>
using namespace racer_task;
int main(){TargetRegistry r;StaticGuidanceEvents q;
 TargetObservation o;o.key={"uav0","vis",1};o.sequence=1;o.first=1.;o.capture=1.4;o.support_first=1.;
 o.frame_version="map";o.has_bearing=true;o.direction={1,0,0};o.confidence=.8;o.support=3;
 o.has_position=true;o.localization="reliable";o.position={0,0,3};o.radius=.1;
 assert(r.ingest(o,1.4).accepted);q.sync(r,1.4);auto initial=q.start();assert(initial&&q.finish(initial->token));
 TargetVisibility v;v.key=o.key;v.capture=2.;v.anchor_capture=1.4;v.frame_version="map";
 v.center={0,0,3};v.extent={.1,.1,.1};v.state="occluded";v.scope="visible_surface_region";v.model_id="test";
 assert(r.observeVisibility(v,2.));q.sync(r,2.);auto occluded=q.start();assert(occluded&&q.finish(occluded->token));
 assert(r.records.at(o.key).visible_region_misses==0&&r.records.at(o.key).has_anchor);
 v.capture=100.;v.state="unknown";assert(r.observeVisibility(v,100.));q.sync(r,100.);assert(!q.start());
 v.capture=101.;v.state="clear";assert(r.observeVisibility(v,101.));q.sync(r,101.);
 auto clear=q.start();assert(!clear); // old bearing is not fresh evidence
 assert(r.records.at(o.key).visible_region_misses==1&&r.records.size()==1);
 assert(!r.observeVisibility(v,101.));v.capture=102.;v.anchor_capture=0.;assert(!r.observeVisibility(v,102.));
 v.anchor_capture=1.4;v.scope="whole_person_absent";assert(!r.observeVisibility(v,102.));
 o.sequence++;o.capture=103.;o.visual="visually_confirmed";r.ingest(o,103.);q.sync(r,103.);
 v.scope="visible_surface_region";v.anchor_capture=103.;v.capture=104.;v.state="clear";
 assert(r.observeVisibility(v,104.));q.sync(r,104.);assert(!q.start()&&r.records.at(o.key).confirmed);
}
''')


def test_visibility_round_robin_and_frame_budget():
    evidence = RegistryEvidence(session='many')
    for ident in range(100):
        evidence.last_qualified[ident] = dict(xyz=[0., 0., 3.], radius_m=.1, capture_time_s=1.)
    config = VisibilityConfig(.5, 8., .5, 9, .05, 100)
    k = np.array([[100., 0, 50], [0, 100., 50], [0, 0, 1.]])
    image = np.full((100, 100), 5.)
    tracker = SimpleNamespace(frames={})
    checked, listed = set(), set()
    for n in range(2, 102):
        rows = visibility_batch(evidence, tracker, n*1_000_000_000, image,
                                k, np.eye(4), n+.1, config, 'test')['regions']
        assert len({row['counter'] for row in rows}) == len(rows) == 64
        assert sum(row['checked_pixels'] for row in rows) == 81
        assert rows[0]['state'] == 'clear'
        assert all(row['state'] == 'unknown' and row['checked_pixels'] == 0 for row in rows[1:])
        checked.add(rows[0]['counter'])
        listed.update(row['counter'] for row in rows)
        if n == 3:
            assert len(listed) == 100
    assert checked == listed


def test_two_readable_visibility_anchors_alternate_under_a_shared_budget():
    evidence = RegistryEvidence(session='two')
    for ident in (1, 2):
        evidence.last_qualified[ident] = dict(xyz=[0., 0., 3.], radius_m=.1, capture_time_s=1.)
    config = VisibilityConfig(.5, 8., .5, 9, .05, 100)
    k = np.array([[100., 0, 50], [0, 100., 50], [0, 0, 1.]])
    image = np.full((100, 100), 5.)
    tracker = SimpleNamespace(frames={})
    for n, ident in ((2, '1'), (3, '2'), (4, '1')):
        rows = visibility_batch(evidence, tracker, n*1_000_000_000, image,
                                k, np.eye(4), n+.1, config, 'test')['regions']
        assert len(rows) == 2
        assert (rows[0]['counter'], rows[0]['state'], rows[0]['checked_pixels']) == (ident, 'clear', 81)
        assert (rows[1]['state'], rows[1]['checked_pixels']) == ('unknown', 0)


def test_visibility_runtime_requires_explicit_calibration_and_ros_strings():
    import pytest,yaml,json
    assert visibility_ros_arguments({})==[]
    config=dict(min_range_m=.5,max_range_m=8.,max_capture_age_s=.5,min_pixels=9,depth_margin_m=.05,max_pixels=100)
    runtime=dict(confirmation_mode='registry',static_visibility_config=config,static_visibility_model_id='test-calibration')
    args=visibility_ros_arguments(runtime)
    assert json.loads(yaml.safe_load(args[1].split(':=',1)[1]))==config
    assert yaml.safe_load(args[3].split(':=',1)[1])=='test-calibration'
    for field,value in [('static_visibility_model_id',''),('confirmation_mode','legacy')]:
        with pytest.raises(ValueError):visibility_ros_arguments(dict(runtime,**{field:value}))


def test_oversized_visibility_projection_leaves_budget_for_another_anchor():
    evidence = RegistryEvidence(session='budget')
    evidence.last_qualified[1] = dict(xyz=[0.,0.,3.], radius_m=.4, capture_time_s=1.)
    evidence.last_qualified[2] = dict(xyz=[0.,0.,3.], radius_m=.1, capture_time_s=1.)
    config = VisibilityConfig(.5, 8., .5, 9, .05, 100)
    k = np.array([[100.,0,50],[0,100.,50],[0,0,1.]])
    tracker = SimpleNamespace(frames={})
    image = np.full((100, 100), 5.)
    for n in (2, 3, 4):
        batch = visibility_batch(evidence, tracker, n*1_000_000_000,
                                 image, k, np.eye(4), n+.1, config, 'test')
        rows = {row['counter']: row for row in batch['regions']}
        oversized, fitting = rows['1'], rows['2']
        assert oversized['reason'] == 'visibility_compute_budget' and oversized['checked_pixels'] == 0
        assert fitting['state'] == 'clear' and fitting['checked_pixels'] == 81

def test_visibility_batch_validates_capture_transform_once(monkeypatch):
    from person_search import static_hypothesis
    evidence=RegistryEvidence(session='shared-capture')
    for ident in range(4):
        evidence.last_qualified[ident]=dict(xyz=[0.,0.,3.],radius_m=.1,capture_time_s=1.)
    tracker=SimpleNamespace(frames={})
    k=[[100.,0,50],[0,100.,50],[0,0,1.]]
    config=VisibilityConfig(.5,8.,.5,9,.05,1000)
    validate=static_hypothesis.validate_transform
    checked=[]
    def counted(transform):
        checked.append(transform)
        return validate(transform)
    monkeypatch.setattr(static_hypothesis,'validate_transform',counted)
    batch=visibility_batch(evidence,tracker,2_000_000_000,np.full((100,100),5.),k,
                           np.eye(4),2.1,config,'test-calibration')
    assert [row['state'] for row in batch['regions']]==['clear']*4
    assert len(checked)==1
