"""Surface support and lifetime tests, no body-centre accuracy claims."""
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'src'),str(ROOT/'src/annual_planner/src')]
import numpy as np
from person_search.segmentation import PersonMask,localize_masks
from person_search.surface import localize_surfaces,SurfaceFusion
K=[160.,0,160.,0,160.,120.,0,0,1.]
T=np.array([[0.,0.,1.,0.],[-1.,0.,0.,0.],[0.,-1.,0.,1.5],[0.,0.,0.,1.]])
def sample():
 m=np.zeros((240,320),bool);m[80:160,150:190]=True
 return PersonMask((148.,78.,44.,84.),.95,m),np.full((240,320),4.,float)
def test_mask_background_is_excluded_by_depth_connectivity_not_averaged():
 d,z=sample();z[:,178:]=7.
 old,_=localize_masks([d],z,K,T);new,m=localize_surfaces([d],z,K,T)
 assert old[0]['depth_reason']=='ambiguous_depth_span'
 assert new[0]['quality']['valid'] and np.all(z[m[0]]==4.)
 assert new[0]['xyz'][0]==4. and new[0]['quality']['raw_depth_span_m']==3.
def test_ambiguous_components_and_missing_depth_do_not_invent_range():
 d,z=sample();z[:,170:]=6.;row,s=localize_surfaces([d],z,K,T)
 assert row[0]['xyz'] is None and row[0]['depth_reason']=='ambiguous_depth_components' and not s.any()
 row,s=localize_surfaces([d],None,None,None);assert row[0]['xyz'] is None
def test_recover_loss_covariance_and_expiry_do_not_reuse_historical_xyz():
 d,z=sample();row,_=localize_surfaces([d],z,K,T);f=SurfaceFusion()
 for t in (1.,1.4,1.8):f.observe(row[0],t,t+.01)
 assert f.current and f.state=='reliable' and f.current['frame']=='map'
 assert np.linalg.eigvalsh(f.current['covariance_m2']).min()>0
 f.observe(None,2.,2.01);assert f.current is None and f.last_reliable['history_only']
 for t in (2.4,2.8,3.2):f.observe(row[0],t,t+.01)
 assert f.current;f.expire(5.3);assert f.state=='expired' and f.current is None
def test_weak_identity_never_enters_3d_fusion():
 d,z=sample();row,_=localize_surfaces([d],z,K,T);f=SurfaceFusion()
 for t in (1.,1.4,1.8):f.observe(row[0],t,t+.01,association_reliable=False)
 assert f.current is None and not f.samples and f.state=='unknown'


def test_stale_processing_result_clears_current_but_not_previous_qualified_support():
 d,z=sample();row,_=localize_surfaces([d],z,K,T);f=SurfaceFusion()
 # Exact successful capture times from Stage31's failed ROS recovery sequence.
 for t in (12.856918901,13.059492512,13.259966741):f.observe(row[0],t,t+.01)
 assert f.current is None and len(f.samples)==3
 f.reject('stale_after_inference',preserve_qualified=True)
 assert f.current is None and len(f.samples)==3
 f.observe(row[0],14.484345041,14.50)
 assert f.current and f.current['capture_time_s']==14.484345041
 assert f.current['quality']['qualified_frames']==4
 assert .8<=f.current['quality']['span_s']<=2.
 # A later fresh frame cannot recycle expired support.
 f.reject('stale_after_inference',preserve_qualified=True)
 f.observe(row[0],17.,17.01)
 assert f.current is None and len(f.samples)==1
 # Actual invalid surfaces and identity conflicts still clear evidence.
 f.reject('ambiguous_depth_components')
 assert not f.samples and f.current is None


def test_compact_patch_localizes_broad_connected_surface_without_widening_span():
 d,z=sample()
 # Dominant compact visible patch plus a minority sloping limb. Still one
 # continuous component; the representative must retain the original span gate.
 for y in range(140,160):z[y,:]=4.+(y-140)*.04
 rows,support=localize_surfaces([d],z,K,T);r=rows[0]
 assert r['quality']['component_input_depth_span_m']>.35
 assert r['quality']['compact_surface_fraction']>=.8
 assert r['quality']['depth_span_m']<=.35 and r['quality']['valid']
 assert np.ptp(z[support[0]])<=.35+1e-6

def test_broad_continuous_surface_without_a_dominant_compact_patch_is_rejected():
 d,z=sample()
 for y in range(80,160):z[y,:]=4.+(y-80)*.02
 rows,support=localize_surfaces([d],z,K,T)
 assert rows[0]['xyz'] is None and rows[0]['depth_reason']=='component_depth_span_rejected'
 assert not support.any()


def test_unmeasurable_broad_surface_does_not_erase_bounded_qualified_evidence():
 d,z=sample();rows,_=localize_surfaces([d],z,K,T);f=SurfaceFusion()
 for t in (1.,1.2,1.4):f.observe(rows[0],t,t+.01)
 assert not f.current
 f.observe(dict(localization_state='unlocalized',quality={},depth_reason='component_depth_span_rejected'),1.6,1.61)
 assert f.current is None and len(f.samples)==3
 f.observe(rows[0],1.9,1.91)
 assert f.current and f.current['quality']['span_s']>=.8
 assert f.current['quality']['qualified_frames']==4
 # A contradictory qualified position still clears the whole sequence.
 bad=dict(rows[0],xyz=[100.,0.,1.])
 f.observe(bad,2.,2.01)
 assert not f.current and not f.samples and f.reasons==['surface_sequence_dispersion']


def test_same_depth_uncertain_mask_edge_does_not_enter_surface_support():
    from person_search.surface import SurfaceConfig
    d,z=sample()
    # A one-pixel uncertain overhang survives the former 2px inset. Its depth
    # differs by only2cm, so depth connectivity cannot identify it as background.
    trusted=np.zeros_like(d.mask);trusted[83:157,153:187]=True
    uncertain=d.mask & ~trusted;z[uncertain]=4.02
    old,old_support=localize_surfaces([d],z,K,T,SurfaceConfig(mask_border_px=2))
    new,new_support=localize_surfaces([d],z,K,T)
    assert old[0]['xyz'] is not None and (old_support[0]&uncertain).any()
    assert new[0]['xyz'] is not None and not (new_support[0]&uncertain).any()
    assert new[0]['quality']['thresholds']['mask_border_px']==3


def test_thin_mask_rejects_range_instead_of_widening_support_to_meet_pixel_gate():
    from person_search.surface import SurfaceConfig
    mask=np.zeros((240,320),bool);mask[80:160,150:157]=True
    d=PersonMask((150.,80.,7.,80.),.95,mask);z=np.full((240,320),4.)
    old,_=localize_surfaces([d],z,K,T,SurfaceConfig(mask_border_px=2))
    new,support=localize_surfaces([d],z,K,T)
    assert old[0]['xyz'] is not None
    assert new[0]['xyz'] is None and new[0]['depth_reason']=='insufficient_valid_component_support'
    assert not support.any()
