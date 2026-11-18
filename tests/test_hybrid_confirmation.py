"""Logic regressions only; quality proxies are not a detector accuracy claim."""
from copy import deepcopy
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
import numpy as np
import pytest
from person_search.active_confirmation import ActiveConfirmationTracker
from person_search.confirmation_quality import ConfirmationQuality, SemanticCandidateMemory
from person_search.segmentation import LocalizedPerson, PersonMask, localize_masks
from person_search.node import confirmation_options


def candidate(**changes):
    q=dict(mask_pixels=400,mask_box_fraction=.7,border_clipped=False,
           foreground_occlusion_fraction=0.,boundary_near_fraction=0.,boundary_valid_fraction=1.,
           valid_depth_fraction=.98,depth_span_m=.12)
    q.update(changes)
    return LocalizedPerson(np.array([4.,0.,1.]),np.ones((20,20),bool),(3,3,20,20),True,.9,4.,.3,q)


def execution(now, **changes):
    return dict(dict(time=now,ready=True,arrived=True,token='baseline',reference=[0,0,1.5]),**changes)


def tracker(**kwargs):
    t=ActiveConfirmationTracker(mode='hybrid',**kwargs);t.advance(1.,execution(1.));return t


def captures(t, times=(1.,1.5,2.), c=None, origin=(0,0,1.5)):
    for now in times:t.update([c if c is not None else candidate()],origin,now)
    return t.tracks[0]


def test_reliable_direct_confirmation_without_motion_or_review():
    t=tracker();c=captures(t)
    assert c.state=='confirmed' and c.attempts==0 and len(c.semantic_views)==1
    event=t.events[-1]
    assert event['reason']=='reliable_direct_confirmed' and not event['during_review']
    assert event['capture_times']==[1.,1.5,2.] and len(event['evidence'])==3
    assert event['quality_thresholds']['min_confidence']==.7
    assert event['position_representation']=='visible_surface_not_body_centre'


def test_confirmed_position_and_quality_are_frozen_despite_later_bad_candidates():
    t=tracker();c=captures(t);original=c.xyz.copy();quality=deepcopy(c.quality_evidence)
    low=candidate(valid_depth_fraction=.3);low.xyz[0]=4.6;low.confidence=.4
    for now in np.arange(3.,13.,.5):t.update([low],[0,0,1.5],float(now))
    assert len(t.tracks)==1 and c.state=='confirmed' and c.confirmed_time==2.
    np.testing.assert_array_equal(c.xyz,original)
    assert c.quality_evidence==quality and c.reason=='reliable_direct_confirmed'
    assert t.events[-1]['reason']=='confirmed_position_frozen'
    assert not t.events[-1]['accepted_into_confirmed_position']
    assert t.events[-1]['observed_xyz'][0]==4.6
    assert 'confidence_gate' in t.events[-1]['observed_rejections']


def test_confirmation_point_comes_only_from_qualified_capture_sequence():
    t=tracker();uncertain=candidate(valid_depth_fraction=.3);uncertain.xyz[0]=4.6
    c=captures(t,times=(1.,1.4,1.8),c=uncertain)
    captures(t,times=(2.2,2.7,3.2))
    assert c.state=='confirmed'
    np.testing.assert_allclose(c.xyz,[4.,0.,1.])
    assert t.events[-1]['confirmed_position']==[4.,0.,1.]


def test_explicit_confirmation_mode_never_implicitly_enables_review():
    assert confirmation_options('hybrid',True,False)==('hybrid',False,True)
    assert confirmation_options('strict',False,False)==('strict',False,True)
    assert confirmation_options('',True,False)==('strict',True,False)
    with pytest.raises(ValueError):confirmation_options('anything',False,False)


@pytest.mark.parametrize('quality,reason',[
    ({'border_clipped':True},'border_clipped_or_unknown'),
    ({'mask_pixels':60},'mask_pixels_gate'),
    ({'mask_box_fraction':.2},'mask_box_fraction_gate'),
    ({'foreground_occlusion_fraction':.25},'foreground_occlusion_fraction_gate'),
    ({'boundary_near_fraction':.25},'boundary_near_fraction_gate'),
    ({'boundary_valid_fraction':.3},'boundary_valid_fraction_gate'),
    ({'valid_depth_fraction':.5},'valid_depth_fraction_gate'),
    ({'depth_span_m':.6},'depth_span_m_gate')])
def test_high_semantic_confidence_does_not_override_quality(quality,reason):
    t=tracker();c=captures(t,c=candidate(**quality))
    assert c.state=='tentative' and reason in c.rejection_reasons
    assert t.eligible(c,2.,execution(2.)) # uncertain, localized candidates can ask for review


def test_low_score_and_geometry_alone_cannot_confirm():
    t=tracker();c=candidate();c.confidence=.4;track=captures(t,c=c)
    assert track.state=='tentative' and 'confidence_gate' in track.rejection_reasons
    c=candidate();c.semantic=False
    empty=tracker();empty.update([c],[0,0,1.5],1.);assert not empty.tracks


def test_same_frame_or_short_burst_do_not_confirm_and_pre_ready_is_labelled():
    t=tracker()
    c=captures(t,times=(1.,1.,1.));assert len(c.direct_hits)==1
    captures(t,times=(1.1,1.2));assert c.state=='tentative'
    assert not t.eligible(c,1.2,execution(1.2)) # allow good sequence to accumulate
    not_ready=ActiveConfirmationTracker(mode='hybrid');c=captures(not_ready)
    assert c.state=='confirmed' and not_ready.events[-1]['is_before_ready']
    assert c.attempts==0 # Passive perception during startup is not a flight/review claim.


def test_flight_fault_blocks_new_direct_candidates_and_rejected_capture_order():
    t=tracker();t.advance(1.,dict(execution(1.),flight_session_fault='fault'))
    c=captures(t)
    assert c.state=='tentative' and 'flight_session_fault' in c.rejection_reasons
    t=tracker();t.reject_capture(3.,'invalid_depth');c=captures(t)
    assert c.state=='tentative' and 'capture_precedes_newer_rejection' in c.rejection_reasons


def test_missing_observation_or_depth_breaks_continuity_but_does_not_deny_person():
    t=tracker();c=captures(t,times=(1.,1.4))
    t.update([],[0,0,1.5],1.6)
    captures(t,times=(1.8,2.2));assert c.state=='tentative' and len(c.direct_hits)==2
    t.reject_capture(2.4,'invalid_depth')
    assert c.state=='tentative' and not c.direct_hits and c.rejection_reasons==['invalid_depth']
    captures(t,times=(2.6,3.,3.5));assert c.state=='confirmed'


def test_large_capture_gap_and_unstable_position_restart_sequence():
    t=tracker();c=captures(t,times=(1.,1.5,5.))
    assert c.state=='tentative' and len(c.direct_hits)==1
    wrong=candidate();wrong.xyz[0]+=.6
    captures(t,times=(5.5,),c=wrong)
    assert c.state=='tentative' and 'position_stability_gate' in c.rejection_reasons


def test_semantics_survive_invalid_depth_and_same_capture_is_not_new_hit():
    mask=np.zeros((100,100),bool);mask[25:70,35:65]=True
    d=PersonMask((35,25,30,45),.9,mask)
    memory=SemanticCandidateMemory()
    rows,localized=localize_masks([d],None,None,None)
    memory.update(rows,1.);memory.update(deepcopy(rows),1.)
    assert not localized and memory.candidates[0]['capture_count']==1
    assert memory.candidates[0]['semantic_state']=='person_candidate'
    assert memory.candidates[0]['localization_state']=='unlocalized' and memory.candidates[0]['xyz'] is None
    assert memory.candidates[0]['visibility_state']=='unknown'
    visible=deepcopy(rows);visible[0]['visibility_state']='no_near_foreground_evidence';memory.update(visible,1.1)
    assert memory.candidates[0]['visibility_state']=='no_near_foreground_evidence'
    memory.update([],2.)
    assert memory.candidates[0]['semantic_state']=='not_currently_observed_unknown'
    assert memory.candidates[0]['visibility_state']=='unknown'


def test_measured_foreground_inside_box_blocks_clear_proxy():
    mask=np.zeros((100,100),bool);mask[20:80,30:70]=True
    mask[40:65,30:70]=False
    d=PersonMask((30,20,40,60),.95,mask)
    depth=np.full((100,100),3.,float);depth[40:65,30:70]=1.
    k=[100,0,50,0,100,50,0,0,1]
    tf=np.array([[0,0,1,0],[-1,0,0,0],[0,-1,0,2],[0,0,0,1.]])
    rows,observations=localize_masks([d],depth,k,tf)
    assert len(observations)==1 and rows[0]['quality']['foreground_occlusion_fraction']>.4
    assert 'foreground_occlusion_fraction_gate' in ConfirmationQuality().evaluate(observations[0])[0]


def review_tracker():
    t=tracker(require_request_status=True)
    c=captures(t,c=candidate(foreground_occlusion_fraction=.3))
    assert t.review(c,[0,0,1.5],[0,1,1.5],2.,execution(2.),request_id='2000000000')
    return t,c


def status(c, now=3., state='arrived', plan_id=7):
    return dict(request_id=c.request_id,time=now,state=state,goal=c.request_base,native_plan_id=plan_id)


def matched_execution(c, now=3.):
    return execution(now,token='real-token',native_plan_id=7,reference=c.request_base,
                     actual_position=c.request_base,position_stamp=now,
                     observation_proof=dict(token='real-token',native_plan_id=7,completed_at=now))


def test_completion_requires_same_request_plan_proof_and_actual_position():
    t,c=review_tracker()
    assert not t.review_status(dict(status(c),request_id='wrong'),3.)
    assert not t.review_status(dict(status(c),goal=[8,8,8]),3.)
    assert not t.review_status(status(c),5.) # stale arrival acknowledgement
    assert t.review_status(status(c),3.)
    for bad in [dict(matched_execution(c),native_plan_id=8),
                dict(matched_execution(c),actual_position=[0,0,1.5]),
                dict(matched_execution(c),position_stamp=1.),
                dict(matched_execution(c),observation_proof={'token':'unrelated','native_plan_id':7})]:
        t.advance(3.,bad);assert c.state=='reviewing'
    t.advance(3.,matched_execution(c));assert c.state=='observing' and c.completed_token=='real-token'
    captures(t,times=(3.2,3.7,4.2),origin=(0,1,1.5))
    assert c.state=='confirmed' and c.reason=='active_multiview_confirmed'
    assert t.events[-1]['post_capture_times']==[3.2,3.7,4.2]


def test_reliable_evidence_during_review_can_confirm_and_cancel_future_segments():
    t,c=review_tracker()
    captures(t,times=(2.2,2.7,3.2),origin=(0,.4,1.5))
    assert c.state=='confirmed' and c.reason=='reliable_direct_confirmed'
    assert t.events[-1]['during_review'] and t.events[-1]['confirmation_path']=='direct'
    assert t.cancellations==[dict(request_id='2000000000',reason='reliable_direct_confirmed')]


def test_native_failure_and_bounded_timeout_remain_unknown_not_no_person():
    t,c=review_tracker();assert t.review_status(status(c,state='unreachable'),3.)
    assert c.state=='tentative' and c.reason=='native_review_unreachable'
    assert t.cancellations[-1]['request_id']=='2000000000'
    t,c=review_tracker();t.advance(28.,execution(28.))
    assert c.state=='tentative' and c.reason=='no_matched_execution_before_timeout'


@pytest.mark.parametrize('close_by',['ttl','attempts'])
def test_exhausted_review_budget_allows_only_new_passive_confirmation(close_by):
    t,c=review_tracker();t.advance(2.1,execution(2.1));eligible_since=c.eligible_since
    if close_by=='ttl':
        t.advance(92.,execution(92.));closed=92.
    else:
        t.max_attempts=1;t.advance(28.,execution(28.));closed=28.
    assert c.state=='unresolved' and c.review_closed_time==closed
    attempts=c.attempts;reason=c.review_closed_reason
    # No closed track can ask again, even when recent low-quality evidence
    # refreshes its historical semantic cue.
    captures(t,times=(closed+.1,closed+.6),c=candidate(foreground_occlusion_fraction=.3))
    assert not t.eligible(c,closed+.6,execution(closed+.6))
    assert not t.review(c,[0,0,1.5],[0,1,1.5],closed+.6,execution(closed+.6))
    captures(t,times=(closed+1.,closed+1.5,closed+2.))
    assert c.state=='confirmed' and c.reason=='reliable_direct_confirmed'
    assert c.attempts==attempts and c.eligible_since==eligible_since
    assert c.review_closed_time==closed and c.review_closed_reason==reason
    assert t.events[-1]['after_review_closed'] and not t.events[-1]['during_review']
    assert t.events[-1]['capture_times']==[closed+1.,closed+1.5,closed+2.]
    assert not t.eligible(c,closed+2.,execution(closed+2.))


@pytest.mark.parametrize('interruption',['missing','invalid','low_quality'])
def test_bad_new_frames_still_break_passive_evidence_after_review_closes(interruption):
    t,c=review_tracker();t.advance(92.,execution(92.))
    captures(t,times=(92.1,92.6))
    if interruption=='missing':t.update([],[0,0,1.5],92.8)
    elif interruption=='invalid':t.reject_capture(92.8,'no_fresh_bracketed_capture')
    else:captures(t,times=(92.8,),c=candidate(valid_depth_fraction=.3))
    assert c.state=='unresolved' and not c.direct_hits
    captures(t,times=(93.,93.5));assert c.state=='unresolved'
    captures(t,times=(94.,));assert c.state=='confirmed'
    assert t.events[-1]['capture_times']==[93.,93.5,94.]


def test_review_closure_discards_preclosure_sequence_and_fault_still_blocks():
    t=tracker();c=captures(t,times=(1.,1.5))
    t.advance(92.,execution(92.));assert c.state=='unresolved' and not c.direct_hits
    captures(t,times=(91.6,91.9));assert not c.direct_hits
    assert 'capture_not_after_review_closure' in c.rejection_reasons
    t.advance(92.1,dict(execution(92.1),flight_session_fault='latched_fault'))
    captures(t,times=(92.2,92.7,93.2))
    assert c.state=='unresolved' and 'flight_session_fault' in c.rejection_reasons


@pytest.mark.parametrize('kwargs',[{'min_frames':1},{'min_span_s':0.},{'min_confidence':1.1},{'max_depth_span_m':float('nan')}])
def test_invalid_quality_configuration_fails_closed(kwargs):
    with pytest.raises(ValueError):ConfirmationQuality(**kwargs)


def test_direct_confirmation_can_reach_a_configured_minimum_above_thirty_frames():
    t = tracker(quality=ConfirmationQuality(min_frames=33))
    track = captures(t, times=[1. + i*.04 for i in range(33)])
    assert track.state == 'confirmed'
    assert len(t.events[-1]['capture_times']) == 33

def test_exhausted_native_review_budget_does_not_request_retry():
    t,c=review_tracker()
    assert c.attempts<t.max_attempts
    assert t.review_status(status(c,state='budget_exhausted'),3.)
    assert c.state=='unresolved' and c.review_closed_time==3.
    assert not t.eligible(c,7.,execution(7.))
    assert t.cancellations[-1]['reason']=='native_review_budget_exhausted'
