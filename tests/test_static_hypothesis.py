import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
import numpy as np
import pytest
from person_search.static_hypothesis import (
    Visibility,VisibilityConfig,depth_visibility)

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
    with pytest.raises(ValueError):
        VisibilityConfig(.5,.4,.5,9,.05)
