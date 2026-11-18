"""Independence and evidence-lifetime tests, not detector accuracy claims."""
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'src'),str(ROOT/'src/annual_planner/src')]
import numpy as np
import pytest
from person_search.presence import PresenceTracker
from person_search.segmentation import PersonMask
from person_search.node import confirmation_options
from person_search.bearing import bearing_packet

def detection(conf=.9,box=(10,10,30,80)):
    mask=np.zeros((120,160),bool);x,y,w,h=map(int,box);mask[y:y+h,x:x+w]=True
    return PersonMask(box,conf,mask)

def rgb(t,s,ds=None):return t.observe_rgb([detection()] if ds is None else ds,round(s*1e9),160,120,s+.01)

def surface(box=(10,10,30,80)):
    return dict(box=box,xyz=[1.,2.,1.],localization_state='localized_surface',quality=dict(depth_pixels=300,valid_depth_fraction=1.,depth_span_m=.2,boundary_valid_fraction=1.,boundary_near_fraction=0.,foreground_occlusion_fraction=0.,mask_box_fraction=.413,border_clipped=True))

def confirm(t):
    for s in np.arange(10,11,.2):rgb(t,float(s))

def test_visual_confirmation_ignores_mask_depth_transform_and_only_fires_once():
    t=PresenceTracker()
    for s in np.arange(10,12,.2):
        rgb(t,float(s));t.reject_localization(round(s*1e9),s+.01,'missing_depth');t.reject_bearing(round(s*1e9),s+.01,'missing_tf')
    assert t.tracks[0].visual_state=='visually_confirmed'
    assert t.tracks[0].position is None
    assert sum(e['event']=='visual_confirmed' for e in t.events)==1
    assert t.report(11.81)['counts']['independent_true_people'] is None

def test_duplicate_capture_duplicate_boxes_and_external_id_changes_do_not_add_people():
    t=PresenceTracker()
    for i,s in enumerate(np.arange(10,11,.2)):
        d=detection();d.detector_id=i
        rgb(t,float(s),[d,detection(.8,(11,10,30,80))]);rgb(t,float(s),[d])
    assert len(t.tracks)==1 and t.tracks[0].captures==5
    assert t.suppressed_duplicates==5
    assert sum(e['event']=='visual_confirmed' for e in t.events)==1


@pytest.mark.parametrize('fields',[{},dict(class_id=0,mask=None)])
def test_legacy_box_only_detections_preserve_tracking_and_duplicate_suppression(fields):
    from types import SimpleNamespace
    tracker=PresenceTracker()
    box=(10,10,30,80)
    first=SimpleNamespace(box=box,confidence=.9,**fields)
    duplicate=SimpleNamespace(box=box,confidence=.8,**fields)
    assert len(rgb(tracker,10.,[first,duplicate]))==1
    assert tracker.suppressed_duplicates==1
    assert len(rgb(tracker,10.2,[first]))==1
    assert tracker.tracks[0].captures==2
    mixed=PresenceTracker()
    assert len(rgb(mixed,10.,[first,detection(.8,box)]))==2
    assert mixed.suppressed_duplicates==0


def test_one_box_low_confidence_and_empty_misses_never_form_confirmation():
    t=PresenceTracker();rgb(t,10);rgb(t,10.2,[])
    for s in (10.4,10.6,10.8,11.,11.2):rgb(t,s,[detection(.4)])
    assert all(x.visual_state=='person_candidate' for x in t.tracks)
    assert not any(e['event']=='visual_confirmed' for e in t.events)

def test_localization_recovers_then_loss_clears_current_point_not_visual_history():
    t=PresenceTracker()
    for s in np.arange(10,11,.2):
        rgb(t,float(s));t.localize(round(s*1e9),[surface()],s+.01)
    track=t.tracks[0];assert track.visual_state=='visually_confirmed' and track.localization_state=='reliable'
    good=track.last_reliable_location.copy()
    rgb(t,11.);t.reject_localization(11000000000,11.01,'ambiguous_depth_span')
    assert track.localization_state=='unreliable' and track.position is None
    assert track.last_reliable_location==good and good['history_only'] is True
    assert track.visual_state=='visually_confirmed' and len(track.rgb_hits)>=5
    for s in np.arange(11.2,12.2,.2):
        rgb(t,float(s));t.localize(round(s*1e9),[surface()],s+.01)
    assert track.localization_state=='reliable'
    t.expire(15);assert track.position is None and track.localization_state=='expired'
    assert track.visual_state=='visually_confirmed'

def test_wrong_or_stale_transform_does_not_create_current_xyz():
    t=PresenceTracker();confirm(t)
    t.localize(10800000000,[surface()],10.81,frame='wrong')
    assert t.tracks[0].localization_state=='unreliable' and t.tracks[0].position is None
    rgb(t,11.);t.localize(11000000000,[surface()],13.)
    assert t.tracks[0].localization_state=='expired' and t.tracks[0].visual_state=='visually_confirmed'
    before=t.tracks[0].captures;rgb(t,12.);t.observe_rgb([detection()],13000000000,160,120,15.)
    assert t.tracks[0].captures==before+1

def test_confirmed_unlocalized_keeps_passive_bearing_without_active_guidance():
    t=PresenceTracker();confirm(t)
    transform=np.eye(4);k=[100.,0,80.,0,100.,60.,0,0,1.]
    t.update_bearing([],k,transform,160,120,10800000000,10.81,{})
    assert not t.active_bearings(10.81)
    row=t.report(10.81)['tracks'][0]
    assert row['visual_state']=='visually_confirmed' and row['position'] is None
    assert row['bearing_state']=='fresh' and row['bearing']['range_m'] is None
    t.reject_bearing(10800000000,10.82,'wrong_tf')
    assert not t.active_bearings(10.83) and t.tracks[0].visual_state=='visually_confirmed'

def test_capture_arrival_and_processing_times_are_distinct():
    t=PresenceTracker();confirm(t)
    t.update_bearing([], [100.,0,80.,0,100.,60.,0,0,1.],np.eye(4),160,120,10800000000,10.9,dict(token='a',arrived=True,time=10.75,observation_proof=dict(token='a',completed_at=10.7)))
    row=t.report(10.95)['tracks'][0]
    assert row['capture_time_s']==pytest.approx(10.8)
    assert row['processing_time_s']==pytest.approx(10.81)
    assert row['last_observation_event_time']==10.7 and row['last_observation_processing_time']==10.9
    assert row['published_processing_time_s']==10.95

def test_explicit_mode_preserves_baselines_and_rejects_legacy_xyz_review():
    assert confirmation_options('hybrid',False,False)==('hybrid',False,True)
    assert confirmation_options('decoupled',False,False)==('decoupled',False,False)
    assert confirmation_options('decoupled',True,True)==('decoupled',False,False)


def test_capture_transform_parent_child_and_stamp_are_checked():
    from types import SimpleNamespace as N
    from person_search.presence_adapter import capture_world
    ns=10000000000
    transform=N(translation=N(x=0.,y=0.,z=1.),rotation=N(x=0.,y=0.,z=0.,w=1.))
    world=N(header=N(frame_id='map',stamp=N(sec=10,nanosec=0)),child_frame_id='optical',transform=transform)
    node=N(require_pose=False,fresh_capture=lambda _:True,buffer=N(lookup_transform=lambda *a:world))
    Time=lambda **kw:kw
    assert capture_world(node,'optical',ns,None,Time).shape==(4,4)
    world.header.frame_id='wrong'
    with pytest.raises(ValueError,match='frame_mismatch'):capture_world(node,'optical',ns,None,Time)
    world.header.frame_id='map';world.header.stamp.sec=9
    with pytest.raises(ValueError,match='time_mismatch'):capture_world(node,'optical',ns,None,Time)
    world.header.stamp.sec=0
    assert capture_world(node,'optical',ns,None,Time).shape==(4,4)


def test_independent_reporting_never_turns_visual_track_ids_into_true_people():
    from tools.flight.presence_audit import audit_presence
    t=PresenceTracker();confirm(t);r=t.report(10.81)
    telemetry=[dict(kind='tracks',data=r),*[dict(kind='presence_event',data=e) for e in t.events]]
    positive=audit_presence(telemetry,dict(person=[4,0,0]))
    assert positive['visually_confirmed_local_track_records']==1
    assert positive['confirmed_people'] is None and positive['evaluator_ground_truth_people_total']==1
    empty=audit_presence(telemetry,dict(person=None))
    assert empty['false_confirmed_tracks']==1 and empty['confirmed_people'] is None


def test_future_latest_capture_does_not_starve_available_valid_evidence():
    from person_search.node import newest_bracketed_capture
    assert newest_bracketed_capture([10_000_000_000,11_000_000_000],None,10_200_000_000,.75)==10_000_000_000
    assert newest_bracketed_capture([10_000_000_000,11_000_000_000],None,10_900_000_000,.75) is None
    assert newest_bracketed_capture([11_000_000_000],None,11_100_000_000,.75)==11_000_000_000
