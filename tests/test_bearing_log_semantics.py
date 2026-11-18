import sys
from pathlib import Path
from types import SimpleNamespace
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from person_search.active_confirmation import ActiveConfirmationTracker,ReviewTrack
from person_search.confirmation_quality import QUALITY_PROXY_VERSION

def good():
    return SimpleNamespace(semantic=True,xyz=np.array([4.,0.,1.]),pixels=np.ones((20,20),bool),confidence=.9,box=(0,0,20,20),
        quality=dict(quality_proxy_version=QUALITY_PROXY_VERSION,mask_pixels=400,mask_box_fraction=.8,border_clipped=False,
            valid_depth_fraction=1.,depth_span_m=.2,foreground_occlusion_fraction=0.,boundary_near_fraction=0.,boundary_valid_fraction=1.))

def test_localization_rejection_is_not_logged_as_missing_semantic_detection():
    tracker=ActiveConfirmationTracker(mode='hybrid');tracker.update([good()],[0,0,1.5],1.)
    tracker.update([], [0,0,1.5],1.2,capture_context=dict(semantic_candidate_count=1,localization_rejections=['ambiguous_depth_span'],capture_time_s=1.2,processing_time_s=1.3))
    t=tracker.tracks[0];assert t.rejection_reasons==['no_localized_match_semantic_candidate_retained']
    event=tracker.events[-1];assert event['reason']=='direct_sequence_reset' and event['before_frames']==1 and event['after_frames']==0
    assert event['capture_context']['localization_rejections']==['ambiguous_depth_span']

def test_native_arrival_event_cutoff_is_separate_from_observation_deadline():
    c=good();tracker=ActiveConfirmationTracker(mode='hybrid',require_request_status=True)
    t=ReviewTrack(1,c.xyz.copy(),-.2,-.2);t.state='reviewing';t.request_time=-.2;t.request_id='1'
    t.request_base=[0,0,1.5];t.request_goal=[0,0,1.5];t.request_origin=[-1,0,1.5]
    t.native_status=dict(state='arrived',native_plan_id=8,request_id='1',time=0.)
    tracker.tracks=[t]
    tracker.advance(.2,dict(time=.2,ready=True,native_plan_id=8,token='fixture',arrived=True,
        observation_proof=dict(token='fixture',native_plan_id=8),position_stamp=.19,actual_position=[0,0,1.5],reference=[0,0,1.5]))
    assert t.arrival_event_time==0. and t.completed_time==.2
    for stamp in [.1,.6,1.]:tracker.update([c],[0,0,1.5],stamp)
    assert tracker.events[-1]['confirmation_path']=='active_review'
