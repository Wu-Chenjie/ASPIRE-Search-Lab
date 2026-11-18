"""Safety/evidence contracts for optional active review; no detector accuracy claims."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
import numpy as np
from person_search.active_confirmation import ActiveConfirmationTracker
from person_search.perception import Candidate
from person_search.policy import ObservedVolume,choose_view


def candidate(x=4.):
    return Candidate(np.array([x,0.,1.]),np.ones((5,5),bool),(0,0,5,5),True,.4,4.,.1)


def execution(now,token='baseline',arrived=True,reference=None):
    return dict(time=now,ready=True,arrived=arrived,token=token,
                observation_proof={'capture':now},reference=reference or [0,0,1.5])


def prepared():
    t=ActiveConfirmationTracker()
    for now,z in [(1.,.3),(2.,.9),(3.,1.5)]:t.update([candidate()],[0,0,z],now)
    t.advance(40.,execution(40.))
    for now in (40.,40.5,41.):t.update([candidate()],[0,0,1.5],now)
    return t,t.tracks[0]


def request(t,track,now=41.):
    assert t.review(track,[0,0,1.5],[0,1.,1.5],now,execution(now))


def complete(t,track,now=43.):
    t.advance(now-1,execution(now-1,'review',False))
    t.advance(now,execution(now,'review',True,track.request_base))
    assert track.state=='observing'


def test_vertical_startup_does_not_confirm_or_expire_before_ready():
    t,track=prepared()
    assert track.state=='tentative' and len(track.semantic_views)==1
    assert track.eligible_since==40.
    assert t.eligible(track,41.,execution(41.))
    t.advance(131.,execution(131.))
    assert track.state=='unresolved' and track.reason=='review_budget_expired'


def test_confirmation_requires_executed_endpoint_then_new_capture_evidence():
    t,track=prepared();request(t,track)
    # Existing arrived packet, and an unrelated completed route, are insufficient.
    t.advance(41.1,execution(41.1))
    assert track.state=='reviewing'
    t.advance(41.2,execution(41.2,'other',False))
    t.advance(41.3,execution(41.3,'other',True,[5,5,1.5]))
    assert track.state=='reviewing'
    complete(t,track)
    t.update([candidate()],[0,1,1.5],42.9) # old capture even if processed after completion
    t.update([candidate()],[0,0,1.5],43.1) # no actual horizontal displacement
    t.update([candidate()],[0,2,1.5],43.2) # wrong actual endpoint
    assert track.state=='observing' and not track.post_hits
    t.update([candidate()],[0,1,1.5],43.3)
    t.update([candidate()],[0,1,1.5],43.31)
    assert track.state=='observing'
    t.update([candidate()],[0,1,1.5],43.8)
    assert track.state=='confirmed' and track.completed_token=='review'


def test_missing_new_depth_or_negative_frames_do_not_confirm_or_deny_person():
    t,track=prepared();request(t,track);complete(t,track)
    for now in (44.,45.,46.):t.update([],[0,1,1.5],now)
    t.advance(52.,execution(52.))
    assert track.state=='tentative' and track.reason=='no_fresh_matched_visual_confirmation'
    assert track.attempts==1
    empty=ActiveConfirmationTracker();empty.advance(1.,execution(1.))
    empty.update([],[0,0,1.5],1.)
    assert empty.tracks==[]


def test_finite_retries_and_fault_do_not_turn_uncertainty_into_no_person():
    t,track=prepared();t.max_attempts=2;request(t,track)
    t.advance(67.,execution(67.))
    assert track.state=='tentative' and track.attempts==1
    for now in (70.,70.5,71.):t.update([candidate()],[0,0,1.5],now)
    request(t,track,71.);t.advance(97.,execution(97.))
    assert track.state=='unresolved'
    t.update([candidate()],[0,1,1.5],98.)
    assert track.state=='unresolved'
    t2,other=prepared();request(t2,other)
    t2.advance(42.,dict(execution(42.),flight_session_fault='tracking_fault'))
    assert other.state=='unresolved' and other.reason=='flight_fault'


def test_no_goal_in_unknown_space_and_no_stale_or_single_hit_trigger():
    t,track=prepared()
    assert choose_view(track,np.array([0.,0.,1.5]),ObservedVolume(),min_horizontal=.8) is None
    assert not t.review(track,[0,0,1.5],[0,.3,1.5],41.,execution(41.))
    assert not t.eligible(track,72.,execution(72.))
    assert not t.eligible(track,41.,execution(40.))
    t2=ActiveConfirmationTracker();t2.advance(1.,execution(1.));t2.update([candidate()],[0,0,1.5],1.)
    assert not t2.eligible(t2.tracks[0],1.,execution(1.))


def test_persistent_cue_queues_while_moving_until_next_safe_stop():
    t=ActiveConfirmationTracker()
    for now in (10.,10.5,11.):
        t.advance(now,execution(now,arrived=False));t.update([candidate()],[0,0,1.5],now)
    track=t.tracks[0]
    assert track.qualified_capture_time==11.
    assert not t.eligible(track,11.,execution(11.,arrived=False))
    # No fresh detections at the later stop. A bounded historical cue can ask
    # for a new observation, but cannot itself confirm anything.
    assert t.review(track,[0,0,1.5],[0,1,1.5],20.,execution(20.))
    event=t.events[-1]
    assert event['cue_age_s']==9. and event['qualified_capture_time']==11.
    assert 'historical' in event['uncertainty'] and track.state=='reviewing'
    complete(t,track,22.)
    t.update([],[0,1,1.5],23.)
    assert track.state=='observing'


def test_old_qualified_cue_expires_and_sparse_hits_cannot_refresh_it():
    t,track=prepared()
    qualified=track.qualified_capture_time
    assert not t.review(track,[0,0,1.5],[0,1,1.5],qualified+30.01,execution(qualified+30.01))
    t.update([candidate()],[0,0,1.5],qualified+31.)
    assert track.qualified_capture_time==qualified
    assert not t.eligible(track,qualified+31.,execution(qualified+31.))


def test_publication_rechecks_current_execution_and_uses_one_fresh_timestamp():
    from types import SimpleNamespace
    from person_search.node import review_request_snapshot
    t,track=prepared()
    old_now=41.
    latest=execution(41.7)
    # A report that arrived during CPU view selection is in the future relative
    # to old_now. The publication snapshot must use the current clock instead.
    assert not t.eligible(track,old_now,latest)
    stamp=SimpleNamespace(nanoseconds=41_800_000_000)
    context=review_request_snapshot(lambda:latest,lambda:stamp)
    assert context is not None and context[0] is stamp
    request_stamp,report=context
    assert t.review(track,[0,0,1.5],[0,1,1.5],request_stamp.nanoseconds*1e-9,report)
    assert track.request_time==request_stamp.nanoseconds*1e-9
    for bad in [execution(41.7,arrived=False),execution(41.),execution(42.),
                dict(execution(41.7),flight_session_fault='stop')]:
        assert review_request_snapshot(lambda:bad,lambda:stamp) is None
        retry,eligible=prepared()
        _,report=review_request_snapshot(lambda:bad,lambda:stamp,validate_execution=False)
        assert not retry.review(eligible,[0,0,1.5],[0,1,1.5],stamp.nanoseconds*1e-9,report)
        assert eligible.state=='tentative' and eligible.attempts==0


def test_same_frame_overlap_suppresses_lower_score_without_extra_time_evidence():
    from person_search.active_confirmation import suppress_same_frame_duplicates
    low=candidate();high=candidate();low.confidence=.3;high.confidence=.8
    low.xyz=high.xyz+np.array([.005,0,.02])
    kept,suppressed=suppress_same_frame_duplicates([low,high])
    assert len(kept)==1 and kept[0] is high
    assert suppressed[0]['source_confidence']==.3 and suppressed[0]['kept_confidence']==.8
    t=ActiveConfirmationTracker();t.update([low,high],[0,0,1.5],1.)
    assert len(t.tracks)==1 and t.tracks[0].hits==[1.] and t.tracks[0].observations==1
    t.update([low,high],[0,0,1.5],1.)
    assert t.tracks[0].hits==[1.] and t.tracks[0].observations==1
    assert not t.eligible(t.tracks[0],1.,execution(1.))


def test_nearby_people_with_independent_masks_or_different_surfaces_are_preserved():
    from person_search.active_confirmation import suppress_same_frame_duplicates
    left=candidate();right=candidate();right.xyz=left.xyz+np.array([.02,0,0])
    left.pixels=np.zeros((20,20),bool);left.pixels[:,:8]=True
    right.pixels=np.zeros((20,20),bool);right.pixels[:,12:]=True
    kept,suppressed=suppress_same_frame_duplicates([left,right])
    assert len(kept)==2 and suppressed==[]
    right.pixels=left.pixels.copy();right.xyz=left.xyz+np.array([.3,0,0])
    kept,suppressed=suppress_same_frame_duplicates([left,right])
    assert len(kept)==2 and suppressed==[]


def test_actual_saved_duplicate_capture(monkeypatch):
    import os,pytest
    from person_search import active_confirmation as module
    from person_search.segmentation import PersonMask,localize_masks
    folder=os.environ.get('SEARCH_DUPLICATE_CAPTURE')
    if not folder:pytest.skip('Set SEARCH_DUPLICATE_CAPTURE to actual 73.756 s RGB-D evidence')
    folder=Path(folder);stamp=73_756_000_000
    with np.load(folder/f'{stamp}-rgb-masks.npz') as m,np.load(folder/f'{stamp}-geometry.npz') as g:
        detections=[PersonMask(tuple(box),float(score),mask) for box,score,mask in zip(m['boxes'],m['scores'],m['masks'])]
        _,observations=localize_masks(detections,g['depth'],g['k'],g['camera_to_map'])
        origin=g['camera_to_map'][:3,3].copy()
    assert len(observations)==2
    suppress=module.suppress_same_frame_duplicates
    monkeypatch.setattr(module,'suppress_same_frame_duplicates',lambda observations:(observations,[]))
    original=module.ActiveConfirmationTracker();original.update(observations,origin,stamp*1e-9)
    assert len(original.tracks)==2
    monkeypatch.setattr(module,'suppress_same_frame_duplicates',suppress)
    fixed=module.ActiveConfirmationTracker();fixed.update(observations,origin,stamp*1e-9)
    assert len(fixed.tracks)==1 and fixed.tracks[0].hits==[stamp*1e-9]
    kept=max(observations,key=lambda c:c.confidence)
    np.testing.assert_allclose(fixed.tracks[0].xyz,kept.xyz)
    assert fixed.events[0]['suppressed_count']==1


def test_duplicate_instance_cannot_steal_post_arrival_confirmation():
    t,track=prepared();request(t,track);complete(t,track)
    high=candidate();high.confidence=.8
    low=candidate();low.confidence=.4;low.xyz=high.xyz+np.array([.005,0,.02])
    t.update([high,low],[0,1,1.5],43.3)
    assert len(t.tracks)==1 and track.post_hits==[43.3]
    t.update([low],[0,1,1.5],43.9)
    assert track.state=='confirmed' and track.post_hits==[43.3,43.9]
    assert track.completed_token=='review' and len(t.tracks)==1
