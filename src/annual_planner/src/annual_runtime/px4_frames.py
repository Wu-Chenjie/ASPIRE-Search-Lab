"""Planner world ENU / body FLU to PX4 local NED / body FRD conversions."""
import math
import numpy as np
from scipy.spatial.transform import Rotation

ENU_NED = np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]])
FLU_FRD = np.diag([1., -1., -1.])


def vector_to_ned(value):
    value = np.asarray(value, float)
    if value.shape != (3,) or not np.isfinite(value).all():
        raise ValueError('Expected a finite 3D vector')
    return ENU_NED @ value


def yaw_to_ned(yaw):
    if not math.isfinite(yaw):
        raise ValueError('Expected finite yaw')
    angle = math.pi / 2 - yaw
    return math.atan2(math.sin(angle), math.cos(angle))


def attitude_to_enu(q_wxyz):
    q = np.asarray(q_wxyz, float)
    if q.shape != (4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q) - 1) > .05:
        raise ValueError('Invalid PX4 attitude quaternion')
    matrix = ENU_NED @ Rotation.from_quat(q[[1, 2, 3, 0]]).as_matrix() @ FLU_FRD
    return Rotation.from_matrix(matrix).as_quat()


def trajectory_to_ned(position, velocity, acceleration, yaw, yaw_rate, origin):
    if not math.isfinite(yaw_rate):
        raise ValueError('Expected finite yaw rate')
    return dict(position=vector_to_ned(np.asarray(position) - origin).tolist(),
                velocity=vector_to_ned(velocity).tolist(),
                acceleration=vector_to_ned(acceleration).tolist(),
                yaw=yaw_to_ned(yaw), yawspeed=-float(yaw_rate))
