import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
import numpy as np
from person_search.segmentation import PersonMask,localize_masks

K=[80,0,50,0,80,50,0,0,1]
T=np.array([[0,0,1,0],[-1,0,0,0],[0,-1,0,2],[0,0,0,1.]])


def person_mask():
    mask=np.zeros((100,100),bool);mask[25:70,35:65]=True
    return PersonMask((35,25,30,45),.9,mask)


def test_invalid_depth_keeps_semantics_without_a_map_position():
    d=person_mask()
    for depth in [None,np.full((100,100),np.nan),np.full((100,100),np.inf),np.zeros((100,100))]:
        semantic,positions=localize_masks([d],depth,K,T)
        assert len(semantic)==1 and semantic[0]['state']=='unlocalized'
        assert semantic[0]['xyz'] is None and not positions


def test_partial_mask_excludes_occluder_at_box_center():
    d=person_mask();d.mask[40:58,42:59]=False
    depth=np.full((100,100),4.);depth[40:58,42:59]=2.
    semantic,positions=localize_masks([d],depth,K,T)
    assert semantic[0]['state']=='localized' and len(positions)==1
    assert positions[0].depth_m==4. and positions[0].xyz[0]==4.
    assert not positions[0].pixels[50,50]


def test_seven_metre_merged_surface_does_not_remove_2d_candidate():
    d=person_mask();depth=np.broadcast_to(np.linspace(.3,7.9,100)[:,None],(100,100)).copy()
    semantic,positions=localize_masks([d],depth,K,T)
    assert len(semantic)==1 and semantic[0]['depth_reason']=='ambiguous_depth_span'
    assert not positions


def test_background_component_cannot_supply_mask_depth():
    d=person_mask();depth=np.full((100,100),4.);depth[d.mask]=np.nan
    semantic,positions=localize_masks([d],depth,K,T)
    assert semantic[0]['xyz'] is None and not positions


def test_no_detections_produces_no_person_from_geometry():
    assert localize_masks([],np.full((100,100),4.),K,T)==([],[])


def test_capture_age_rejects_stale_and_future_poses():
    from person_search.node import capture_age_valid
    assert capture_age_valid(10_000_000_000,10_200_000_000,.75)
    assert not capture_age_valid(10_000_000_000,11_000_000_000,.75)
    assert not capture_age_valid(11_000_000_000,10_000_000_000,.75)


def test_newest_image_waits_for_bracket_without_starving_previous_pair():
    from person_search.node import newest_bracketed_capture
    from annual_runtime.pose_history import PoseHistory
    h=PoseHistory()
    h.add(10.,[0,0,1],[0,0,0,1]);h.add(10.02,[.01,0,1],[0,0,0,1])
    assert newest_bracketed_capture([10_010_000_000,10_050_000_000],h,10_060_000_000,.75)==10_010_000_000
    assert newest_bracketed_capture([10_010_000_000],h,11_000_000_000,.75) is None


def test_capture_age_uses_inclusive_nanosecond_boundaries():
    from person_search.node import capture_age_valid
    stamp=1_700_000_000_000_000_000
    assert capture_age_valid(stamp,stamp+750_000_000,.75)
    assert not capture_age_valid(stamp,stamp+750_000_001,.75)
    assert capture_age_valid(stamp,stamp-50_000_000,.75)
    assert not capture_age_valid(stamp,stamp-50_000_001,.75)
