import numpy as np
import pytest
from tools.flight.stage23_metrics import rotation_rate_stats


def attitude(yaw,times,signs=None):
    q=np.column_stack([np.cos(np.array(yaw)/2),np.zeros(len(yaw)),np.zeros(len(yaw)),np.sin(np.array(yaw)/2)])
    if signs is not None:q*=np.array(signs)[:,None]
    return {'timestamp':np.array(times)*1e6,**{f'q[{i}]':q[:,i] for i in range(4)}}


def test_rotation_rate_does_not_turn_yaw_wrap_or_quaternion_sign_into_spin():
    # A slow 0.2 rad/s rotation crosses +pi and flips quaternion sign.
    data=attitude([np.pi-.001,-np.pi+.001,-np.pi+.003],[0,.01,.02],[1,-1,1])
    r=rotation_rate_stats(data,[[0,.02]])
    assert r['norm']['samples']==2
    assert r['norm']['maximum']==pytest.approx(.2,abs=1e-7)


def test_rotation_rate_refuses_long_gaps_duplicate_time_and_invalid_quaternions():
    data=attitude([0,.01,.5,.6,.7],[0,.01,.5,.5,.51])
    for i in range(4):data[f'q[{i}]'][-1]=0
    r=rotation_rate_stats(data,[[0,1]])
    assert r['norm']['samples']==1
    assert r['norm']['maximum']==pytest.approx(1.)
    assert rotation_rate_stats(None,[[0,1]]) is None
