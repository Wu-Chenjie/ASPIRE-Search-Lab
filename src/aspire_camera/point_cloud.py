"""Back-project measured pixels into the map frame."""
import numpy as np


def validate_transform(matrix):
    t = np.asarray(matrix, dtype=float)
    if t.shape != (4, 4) or not np.isfinite(t).all():
        raise ValueError('Missing or nonfinite camera-to-map transform')
    if not np.allclose(t[3], [0, 0, 0, 1]) or not np.allclose(t[:3,:3].T @ t[:3,:3], np.eye(3), atol=1e-6) or not np.isclose(np.linalg.det(t[:3,:3]), 1):
        raise ValueError('Invalid rigid camera-to-map transform')
    return t


def cloud(depth, k, camera_to_map):
    t = validate_transform(camera_to_map)
    k = np.asarray(k, dtype=float).reshape(3,3)
    if not np.isfinite(k).all() or min(k[0,0], k[1,1]) <= 0:
        raise ValueError('Invalid intrinsics')
    height,width = depth.shape
    v,u = np.ogrid[:height,:width]
    with np.errstate(invalid='ignore'):
        optical = np.stack([(u-k[0,2])*depth/k[0,0], (v-k[1,2])*depth/k[1,1], depth], axis=-1)
        return optical @ t[:3,:3].T + t[:3,3]
