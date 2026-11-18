import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
import numpy as np
import pytest
from person_search.static_hypothesis import (
    StaticHypothesis,Visibility,VisibilityConfig,depth_visibility)

CONFIG=VisibilityConfig(.5,8.,.5,9,.05)
K=np.array([[100.,0,50],[0,100.,50],[0,0,1]])


def check(depth,**kwargs):
    inputs=dict(center=(0,0,3.),half_extent=(.1,.1,.1),depth=depth,k=K,
                camera_to_map=np.eye(4),capture_s=1.,now=1.1,config=CONFIG)
    inputs.update(kwargs)
    return depth_visibility(**inputs)


def test_only_observed_clear_depth_can_support_a_miss():
    image=np.full((100,100),5.)
    assert check(image).state=='clear'
    image[50,50]=1.
    assert check(image).state=='occluded'
    image[50,50]=3.
    assert check(image).state=='unknown'
    for value in (0.,float('nan'),float('inf')):
        image[50,50]=value
        assert check(image).reason=='unknown_depth_pixels'


def test_range_fov_capture_sensor_and_budget_are_required():
    image=np.full((100,100),5.)
    assert check(image,center=(2,0,3)).state=='outside'
    assert check(image,center=(0,0,9)).state=='outside'
    assert check(image,center=(0,0,-3)).state=='outside'
    assert check(image,now=2.).state=='unknown'
    assert check(image,now=.9).state=='unknown'
    assert check(image,sensor_valid=False).state=='unknown'
    assert check(None).state=='unknown'
    assert check(image,config=VisibilityConfig(.5,8.,.5,9,.05,20)).reason=='visibility_compute_budget'


def test_transform_is_applied_at_capture_and_invalid_geometry_is_unknown():
    image=np.full((100,100),5.)
    transform=np.eye(4);transform[0,3]=2.
    assert check(image,center=(2,0,3),camera_to_map=transform).state=='clear'
    transform[0,0]=2
    assert check(image,camera_to_map=transform).state=='unknown'
    assert check(image,k=np.zeros((3,3))).state=='unknown'
    assert check(image,half_extent=(0,.1,.1)).state=='unknown'


def test_static_identity_and_location_survive_no_frames_occlusion_and_unknown():
    h=StaticHypothesis('canonical-A',(0,0,3),(.1,.1,.1),1.,1.)
    for stamp,state in ((2.,'occluded'),(1000.,'unknown'),(1001.,'outside')):
        assert h.observe(stamp,detected=False,
                         visibility=Visibility(state,'fixture',stamp))=='uninformative_miss'
    assert h.center==(0,0,3) and h.last_seen_s==1.
    assert h.effective_misses==0 and h.active_search_allowed
    clear=Visibility('clear','fixture',1002.)
    assert h.observe(1002.,detected=False,visibility=clear)=='effective_miss'
    assert h.observe(1002.,detected=False,visibility=clear)=='duplicate_or_old'
    assert h.effective_misses==1 and h.active_search_allowed
    assert h.observe(1003.,detected=True)=='positive'
    assert h.effective_misses==0 and h.entity=='canonical-A'
    h.confirm()
    assert not h.active_search_allowed
    assert h.observe(1004.,detected=True)=='positive'  # passive refinement allowed
    assert h.confirmed and not h.active_search_allowed


def test_visibility_from_other_capture_cannot_be_reused():
    h=StaticHypothesis('A',(0,0,3),(.1,.1,.1),1.,1.)
    assert h.observe(2.,detected=False,visibility=Visibility('clear','old',1.))=='uninformative_miss'
    assert h.effective_misses==0
    with pytest.raises(ValueError):
        VisibilityConfig(.5,.4,.5,9,.05)


def test_capture_aligned_occlusion_lifecycle_without_synthetic_negative_evidence():
    h=StaticHypothesis('retained',(0,0,3),(.1,.1,.1),1.,1.)
    image=np.full((100,100),5.)
    image[50,50]=1.  # observed foreground blocking the retained static volume
    visibility=check(image,capture_s=2.,now=2.1)
    assert visibility.state=='occluded'
    assert h.observe(2.,detected=False,visibility=visibility)=='uninformative_miss'
    image[50,50]=0.  # unknown depth after occluder departure is still not clear
    visibility=check(image,capture_s=1000.,now=1000.1)
    assert h.observe(1000.,detected=False,visibility=visibility)=='uninformative_miss'
    assert h.center==(0,0,3) and h.effective_misses==0
    image[50,50]=5.
    visibility=check(image,capture_s=1001.,now=1001.1)
    assert h.observe(1001.,detected=False,visibility=visibility)=='effective_miss'
    assert h.active_search_allowed and h.effective_misses==1
    assert h.observe(1002.,detected=True)=='positive'  # actual reacquisition
    h.confirm()
    assert h.observe(1003.,detected=True)=='positive'
    assert not h.active_search_allowed and h.entity=='retained'
