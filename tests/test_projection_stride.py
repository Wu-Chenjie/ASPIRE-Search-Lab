"""Projection stride is the sampling density knob for narrow obstacle coverage."""
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'src'),str(ROOT/'src/annual_planner/src')]
import numpy as np
import pytest
from annual_runtime.depth_projection import project_depth

K=np.array([[320.,0.,160.],[0.,320.,120.],[0.,0.,1.]])

def frame(value=1.):
    return np.full((240,320),value,np.float32)

def test_finer_stride_projects_every_pixel_it_samples():
    coarse,_=project_depth(frame(),K,stride=8)
    fine,_=project_depth(frame(),K,stride=4)
    assert len(coarse)==30*40 and len(fine)==60*80


def test_narrow_obstacle_missed_at_stride_8_is_kept_at_stride_4():
    image=frame(0.)
    image[:,4:6]=1.
    missed,_=project_depth(image,K,stride=8)
    kept,hits=project_depth(image,K,stride=4)
    assert len(missed)==0
    assert len(kept)==60 and hits.all()
    # The narrow strip keeps its measured lateral offset instead of being clamped away.
    assert np.allclose(kept[:,1],(160-4)/320) and np.allclose(kept[:,0],1.)


@pytest.mark.parametrize('stride',[0,-4,2.5,None])
def test_invalid_stride_is_refused(stride):
    with pytest.raises(ValueError):
        project_depth(frame(),K,stride=stride)
