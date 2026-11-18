"""Reject unsafe RGB-pixel/depth indexing when time or geometry changes."""
from types import SimpleNamespace as S
import copy
import numpy as np
import pytest
from annual_runtime.rgbd_alignment import require_registered_pair


def pair():
    header=lambda f:S(stamp=S(sec=3,nanosec=100),frame_id=f)
    def info(f):return S(header=header(f),width=320,height=240,k=[216.,0,160,0,216.,120,0,0,1],
        r=[1,0,0,0,1,0,0,0,1],p=[216.,0,160,0,0,216.,120,0,0,0,1,0],d=[0.]*5,distortion_model='plumb_bob',
        binning_x=0,binning_y=0,roi=S(x_offset=0,y_offset=0,width=0,height=0,do_rectify=False))
    rgb=S(header=header('rgb'),width=320,height=240,encoding='rgb8')
    depth=S(header=header('depth'),width=320,height=240,encoding='32FC1')
    tf=S(header=header('rgb'),child_frame_id='depth',transform=S(translation=S(x=0.,y=0.,z=0.),rotation=S(x=0.,y=0.,z=0.,w=1.)))
    return rgb,depth,info('rgb'),info('depth'),tf


def test_matching_registered_pair_accepts_identity_and_equivalent_quaternion():
    items=pair();assert require_registered_pair(*items)
    items[-1].transform.rotation.w=-1
    assert require_registered_pair(*items)
    items[-1].header.stamp=S(sec=0,nanosec=0)
    assert require_registered_pair(*items)


@pytest.mark.parametrize('fault',['time','tf_time','intrinsic','baseline','rotation','frame','info_frame','resolution','distortion','nan','roi','binning'])
def test_misalignment_cannot_silently_index_depth(fault):
    rgb,depth,ri,di,tf=pair()
    if fault=='time':depth.header.stamp.nanosec+=1
    if fault=='tf_time':tf.header.stamp.nanosec+=1
    if fault=='intrinsic':ri.k[0]+=1
    if fault=='baseline':tf.transform.translation.x=.01
    if fault=='rotation':tf.transform.rotation.z=.1
    if fault=='frame':tf.child_frame_id='wrong'
    if fault=='info_frame':ri.header.frame_id='wrong'
    if fault=='resolution':rgb.width=640
    if fault=='distortion':ri.d[0]=di.d[0]=.1
    if fault=='nan':ri.k[0]=di.k[0]=np.nan
    if fault=='roi':ri.roi.x_offset=1
    if fault=='binning':di.binning_y=2
    with pytest.raises(ValueError):require_registered_pair(rgb,depth,ri,di,tf)
