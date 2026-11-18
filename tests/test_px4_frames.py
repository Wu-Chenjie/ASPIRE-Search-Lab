import math
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from annual_runtime.px4_frames import vector_to_ned, yaw_to_ned, attitude_to_enu, trajectory_to_ned


def test_enu_ned_world_origin_velocity_acceleration_and_yaw_rate():
    result=trajectory_to_ned([5,9,2],[.4,-.2,.3],[.1,.2,-.1],0.,.5,np.array([2,6,.24]))
    assert np.allclose(result['position'],[3,3,-1.76])
    assert np.allclose(result['velocity'],[-.2,.4,-.3])
    assert np.allclose(result['acceleration'],[.2,.1,.1])
    assert np.isclose(result['yaw'],math.pi/2) and result['yawspeed']==-.5
    assert np.allclose(vector_to_ned(vector_to_ned([1,2,3])),[1,2,3])


def test_north_facing_px4_attitude_and_nonzero_roll_pitch():
    q=attitude_to_enu([1,0,0,0]); r=Rotation.from_quat(q)
    assert np.allclose(r.apply([1,0,0]),[0,1,0])
    assert np.allclose(r.apply([0,0,1]),[0,0,1])
    assert np.isclose(yaw_to_ned(math.pi/2),0)
    assert np.isclose(yaw_to_ned(-math.pi),-math.pi/2)
    qpx4=Rotation.from_euler('xyz',[.2,-.1,.6]).as_quat()
    from annual_runtime.px4_frames import ENU_NED, FLU_FRD
    qenu=attitude_to_enu(qpx4[[3,0,1,2]])
    assert np.allclose(Rotation.from_quat(qenu).as_matrix(), ENU_NED@Rotation.from_quat(qpx4).as_matrix()@FLU_FRD)


@pytest.mark.parametrize('value',[[1,2],[np.nan,1,2],[np.inf,1,2]])
def test_reject_invalid_setpoint_vectors(value):
    with pytest.raises(ValueError): vector_to_ned(value)


def test_reject_invalid_attitude():
    with pytest.raises(ValueError): attitude_to_enu([0,0,0,0])
