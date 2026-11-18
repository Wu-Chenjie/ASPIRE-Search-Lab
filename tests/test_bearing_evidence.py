import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"src"))
import numpy as np
import pytest
from person_search.bearing import BearingMemory,bearing_geometry,bearing_packet
from person_search.segmentation import PersonMask,localize_masks

K=[160.,0,160.,0,160.,120.,0,0,1.]
T=np.array([[0.,0.,1.,0.],[-1.,0.,0.,0.],[0.,-1.,0.,1.5],[0.,0.,0.,1.]])

def detection(conf=.9):
    m=np.zeros((240,320),bool);m[70:170,140:180]=True
    return PersonMask((140.,70.,40.,100.),conf,m)

def test_rgb_bearing_survives_missing_and_invalid_depth_without_pseudo_range():
    d=detection();rows,loc=localize_masks([d],None,None,None)
    assert not loc and rows[0]['xyz'] is None
    memory=BearingMemory()
    for ns in [1_000_000_000,1_200_000_000,1_600_000_000]:memory.update([d],K,T,320,240,ns,ns/1e9)
    report=memory.active(1.6)[0]
    assert report['persistent'] and report['support_captures']==3
    assert report['range_m'] is None and report['position'] is None
    assert report['direction']==pytest.approx([1,0,0])
    assert report['origin']==[0,0,1.5] and len(bearing_packet(report))==17
    memory.mark_localization(1_600_000_000,rows)
    assert memory.active(1.6)[0]['localization_state']=='unlocalized'
    assert memory.active(22.)==[]

def test_repeated_capture_is_not_three_people_or_three_supports_and_no_area_gate():
    memory=BearingMemory();d=detection()
    for _ in range(3):memory.update([d],K,T,320,240,1_000_000_000,1.)
    assert len(memory.tracks)==1 and memory.active(1.)[0]['support_captures']==1
    assert not memory.active(1.)[0]['persistent']
    # Thin local evidence can seed a bearing without the stage25 area ratio.
    d.mask[:]=False;d.mask[100:140,150:152]=True
    memory.update([d],K,T,320,240,1_600_000_000,1.6)
    assert len(memory.tracks)==1 and memory.tracks[0].last_capture_s==1.6

def test_basic_rgb_and_calibration_gates_remain():
    memory=BearingMemory();memory.update([detection(.49)],K,T,320,240,1_000_000_000,1.)
    assert not memory.tracks
    bad=T.copy();bad[0,0]=2
    with pytest.raises(ValueError):bearing_geometry((140,70,40,100),K,bad,320,240)

def test_observation_coverage_counts_a_completed_token_once_and_new_depth_is_unknown():
    m=BearingMemory();d=detection();execution=dict(token='done1',arrived=True,time=2.,observation_proof={'token':'done1','completed_at':1.5})
    m.update([d],K,T,320,240,2_000_000_000,2.,execution)
    m.update([d],K,T,320,240,2_200_000_000,2.2,execution)
    assert m.active(2.2)[0]['observation_coverage']==1
    m.mark_localization(2_200_000_000,[dict(box=d.box,localization_state='localized_surface',depth_reason=None,xyz=[4,0,1])])
    assert m.active(2.2)[0]['localized_surface_xyz']==[4,0,1]
    m.update([d],K,T,320,240,2_600_000_000,2.6)
    assert m.active(2.6)[0]['localized_surface_xyz'] is None


def test_expired_bearing_cues_make_capacity_available_without_reusing_ids():
    memory = BearingMemory()
    d = detection()
    reports = memory.update([d]*64, K, T, 320, 240, 1_000_000_000, 1.)
    assert [r['cue_id'] for r in reports] == list(range(1, 65))
    reports = memory.update([d], K, T, 320, 240, 22_000_000_000, 22.)
    assert [r['cue_id'] for r in reports] == [65]
    assert len(memory.tracks) == 1

def test_bearing_association_support_and_expiry_include_capture_boundaries():
    d=detection()
    memory=BearingMemory()
    for ns in (13_000_000_000,14_600_000_000,15_000_000_000):
        result=memory.update([d],K,T,320,240,ns,ns*1e-9)
    assert result[0]['support_captures']==3 and result[0]['persistent']
    assert len(memory.active(35.))==1
    assert memory.active(35.000001)==[]
