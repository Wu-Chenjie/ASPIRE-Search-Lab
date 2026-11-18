"""Conservative static-person RGB identity during camera rotation; no depth/truth."""
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'src')]
import numpy as np
from scipy.spatial.transform import Rotation
from person_search.presence import PresenceTracker,association_view,rotated_box
from person_search.segmentation import PersonMask

def view(yaw=0):
    t=np.eye(4);t[:3,:3]=Rotation.from_euler('y',yaw).as_matrix()
    return association_view([[200,0,160],[0,200,120],[0,0,1]],t,320,240)

def d(box,confidence=.9):return PersonMask(tuple(box),confidence,np.zeros((240,320),bool))
def put(t,seconds,boxes,v=None):return t.observe_rgb([d(b) for b in boxes],round(seconds*1e9),320,240,seconds+.01,view=v)

def test_camera_rotation_does_not_assign_previous_neighbour_image_box():
    t=PresenceTracker();a,b=(80,80,35,90),(125,80,35,90)
    put(t,10,[a,b],view());current=rotated_box(b,view(),view(.21))
    # Only the second person remains visible/detected; raw overlap points to1.
    from person_search.confirmation_quality import box_iou
    assert box_iou(a,current)>.3 and box_iou(b,current)<.3
    rows=put(t,10.2,[current],view(.21))
    assert rows[0]['id']==2 and t.tracks[0].last_capture_s==10

def test_competing_boxes_do_not_greedily_reuse_source_identity():
    t=PresenceTracker();put(t,10,[(80,80,40,90)])
    rows=put(t,10.2,[(65,80,40,90),(95,80,40,90)])
    assert {r['id'] for r in rows}=={2,3}
    assert t.tracks[0].last_capture_s==10

def test_short_rgb_miss_recovers_unique_source_and_keeps_confirmed_history():
    t=PresenceTracker()
    for i in range(5):put(t,10+i*.2,[(80,80,40,90)])
    assert t.tracks[0].visual_state=='visually_confirmed'
    put(t,11,[]);rows=put(t,11.2,[(80,80,40,90)])
    assert rows[0]['id']==1 and rows[0]['visual_state']=='visually_confirmed'
    assert t.tracks[0].visual_state=='visually_confirmed'

def test_paired_depth_recovers_source_after_translation_and_rejects_conflicting_range():
    first=view();second=view();second['origin'][0]=.18
    def masked(x):
        mask=np.zeros((240,320),bool);mask[80:150,x:x+42]=True
        return PersonMask((x,80,42,70),.9,mask)
    for range_m,same in ((3.,True),(2.,False)):
        tracker=PresenceTracker()
        tracker.observe_rgb([masked(100)],10_000_000_000,320,240,10.01,view=first,
                            paired=(np.full((240,320),3.),first))
        tracker.observe_rgb([],10_250_000_000,320,240,10.26,view=first)
        rows=tracker.observe_rgb([masked(88)],10_500_000_000,320,240,10.51,view=second,
                                 paired=(np.full((240,320),range_m),second))
        assert rows[0]['id']==(1 if same else 2)
        assert rows[0]['visual_state']=='person_candidate'


def test_missing_pose_still_confirms_rgb_and_geometry_transition_starts_new_source():
    t=PresenceTracker()
    for i in range(5):put(t,10+i*.2,[(80,80,40,90)])
    assert t.tracks[0].visual_state=='visually_confirmed'
    rows=put(t,11,[(80,80,40,90)],view())
    assert rows[0]['id']==2 and rows[0]['position'] is None

def test_rotated_out_of_view_or_behind_camera_cannot_match():
    assert rotated_box((80,80,35,90),view(),view(3.14)) is None
    assert rotated_box((80,80,35,90),None,view()) is None


def test_causal_rgb_calibration_tolerates_separate_callbacks_without_using_future_info():
    from types import SimpleNamespace as N
    from person_search.presence_adapter import rgb_capture_intrinsics
    info=lambda frame:N(header=N(frame_id=frame),width=320,height=240,k=list(range(9)))
    k,stamp=rgb_capture_intrinsics({10_000_000_000:info('optical'),10_300_000_000:info('optical')},10_200_000_000,'optical',320,240,.75)
    assert stamp==10_000_000_000 and k==list(range(9))


def test_causal_rgb_calibration_rejects_old_other_camera_or_changed_image_shape():
    import pytest
    from types import SimpleNamespace as N
    from person_search.presence_adapter import rgb_capture_intrinsics
    info=N(header=N(frame_id='optical'),width=320,height=240,k=list(range(9)))
    for ns,frame,width in [(11_000_000_000,'optical',320),(10_200_000_000,'other',320),(10_200_000_000,'optical',160)]:
        with pytest.raises(ValueError,match='missing_causal_rgb_calibration'):
            rgb_capture_intrinsics({10_000_000_000:info},ns,frame,width,240,.75)

def test_rotation_association_reuses_frame_calibration(monkeypatch):
    inverse=np.linalg.inv
    calls=[]
    def counted(k):
        calls.append(k)
        return inverse(k)
    monkeypatch.setattr(np.linalg,'inv',counted)
    tracker=PresenceTracker()
    boxes=[(60,80,35,90),(160,80,35,90),(230,80,35,90)]
    first=view()
    put(tracker,10,boxes,first)
    second=view()
    rows=put(tracker,10.2,boxes,second)
    assert [row['id'] for row in rows]==[1,2,3]
    assert len(calls)==2
    # Callers holding the old calibration dictionary can still project a box.
    for calibration in (first, second):
        calibration.pop('map_to_pixel')
        calibration.pop('pixel_to_map')
    np.testing.assert_allclose(rotated_box(boxes[0],first,second),boxes[0])
