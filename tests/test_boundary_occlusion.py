"""Mask-boundary depth is a conservative veto, never a visibility oracle."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
import numpy as np
from scipy.ndimage import binary_dilation
from person_search.segmentation import PersonMask,localize_masks,boundary_depth_quality
from person_search.confirmation_quality import ConfirmationQuality,QUALITY_PROXY_VERSION


def fixture():
    mask=np.zeros((240,320),bool);mask[70:170,140:180]=True
    depth=np.full(mask.shape,5.);depth[mask]=3.
    return mask,depth


def localize(mask,depth):
    tf=np.array([[0,0,1,0],[-1,0,0,0],[0,-1,0,2],[0,0,0,1.]])
    return localize_masks([PersonMask((140,70,40,100),.95,mask)],depth,
                          [200,0,160,0,200,120,0,0,1],tf)


def test_outside_box_foreground_veto_does_not_change_person_position():
    mask,depth=fixture();_,clear=localize(mask,depth)
    depth[:,:140]=1.;rows,blocked=localize(mask,depth)
    assert rows[0]['state']=='localized' and len(blocked)==1
    assert blocked[0].quality['foreground_occlusion_fraction']==0.
    assert blocked[0].quality['boundary_near_fraction']>.3
    assert rows[0]['visibility_state']=='near_foreground_evidence'
    assert 'boundary_near_fraction_gate' in ConfirmationQuality().evaluate(blocked[0])[0]
    np.testing.assert_array_equal(clear[0].xyz,blocked[0].xyz)
    assert rows[0]['quality']['quality_proxy_version']==QUALITY_PROXY_VERSION


def test_distant_background_same_depth_neighbor_and_one_noise_pixel():
    mask,depth=fixture()
    assert boundary_depth_quality(mask,depth)['boundary_near_fraction']==0.
    assert boundary_depth_quality(mask,depth)['visibility_state']=='no_near_foreground_evidence'
    depth[:,:140]=3.
    assert boundary_depth_quality(mask,depth)['boundary_near_fraction']==0.
    depth[100,139]=1.
    assert 0<boundary_depth_quality(mask,depth)['boundary_near_fraction']<.08


def test_unknown_ring_is_not_clear_and_keeps_localized_semantic_candidate():
    mask,depth=fixture();depth[binary_dilation(mask,iterations=2)&~mask]=np.nan
    rows,observations=localize(mask,depth)
    assert len(observations)==1 and rows[0]['semantic_state']=='person_candidate'
    assert observations[0].quality['boundary_valid_fraction']==0.
    assert rows[0]['visibility_state']=='unknown'
    assert observations[0].quality['boundary_near_fraction'] is None
    reasons,_=ConfirmationQuality().evaluate(observations[0])
    assert 'boundary_near_fraction_unavailable' in reasons and 'boundary_valid_fraction_gate' in reasons


def test_resolution_scaling_retains_boundary_decision():
    mask,depth=fixture();depth[:,:140]=1.
    one=boundary_depth_quality(mask,depth)
    two=boundary_depth_quality(np.repeat(np.repeat(mask,2,0),2,1),np.repeat(np.repeat(depth,2,0),2,1))
    assert (one['boundary_ring_radius_px'],two['boundary_ring_radius_px'])==(2,4)
    assert abs(one['boundary_near_fraction']-two['boundary_near_fraction'])<.01


def test_near_floor_can_conservatively_veto_even_without_body_occlusion():
    mask,depth=fixture();depth[170:172,:]=2.7
    # Document the expected false-rejection mechanism rather than exempting
    # the bottom edge using an unverified standing-person assumption.
    assert boundary_depth_quality(mask,depth)['boundary_near_fraction']>.08


def test_local_surface_reference_avoids_global_depth_slope_artifact():
    mask,_=fixture();depth=np.broadcast_to(np.linspace(2.,4.4,240)[:,None],mask.shape).copy()
    assert boundary_depth_quality(mask,depth)['boundary_near_fraction']==0.


def test_occluder_beyond_ring_is_not_detected_and_no_full_visibility_is_claimed():
    mask,depth=fixture();depth[:,:135]=1.
    assert boundary_depth_quality(mask,depth)['boundary_near_fraction']==0.
