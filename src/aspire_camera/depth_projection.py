"""Project calibrated X500 / OakD-Lite depth pixels into base_link coordinates.

Missing or invalid depth is unknown. Finite surfaces farther than the planning
range certify a free ray up to that range. No world geometry is read here.
"""
import numpy as np
from .mount import DEPTH_TRANSLATION

# Relative to base_link; includes the merged x500_base model-frame offset.
SENSOR_TRANSLATION = np.array(DEPTH_TRANSLATION)


def project_depth(image, intrinsics, stride=8, radius=4.5):
    return _project_depth(image, intrinsics, stride, radius, 1.)


def project_depth_message(message, intrinsics, stride=8, radius=4.5):
    image, scale = _message_pixels(message)
    return _project_depth(image, intrinsics, stride, radius, scale)


def _project_depth(image, intrinsics, stride, radius, scale):
    z = np.asarray(image)[::stride, ::stride].astype(float, copy=False)
    if scale != 1.:
        z *= scale
    v, u = np.mgrid[0:image.shape[0]:stride, 0:image.shape[1]:stride]
    k = np.asarray(intrinsics, float).reshape(3, 3)
    if not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0:
        raise ValueError('Invalid depth camera intrinsics')
    valid = (z >= .2) & (z < 19.1)
    z, u, v = z[valid], u[valid], v[valid]
    # Optical axes: right/down/forward; body axes: forward/left/up.
    points = np.column_stack([z, -(u-k[0, 2])*z/k[0, 0], -(v-k[1, 2])*z/k[1, 1]])
    ranges = np.linalg.norm(points, axis=1)
    hits = ranges < radius
    points *= (np.minimum(ranges, radius)/np.maximum(ranges, 1e-9))[:, None]
    return points, hits


def _depth_pixels(data, width, height, step, encoding, bigendian, *, require_padding=False):
    if encoding == '32FC1':
        kind, scale = 'f4', 1.
    elif encoding == '16UC1':
        kind, scale = 'u2', .001
    else:
        if require_padding:
            raise ValueError('expected 16UC1 millimetres or 32FC1 metres')
        raise ValueError('Expected 32FC1 metres or 16UC1 millimetres')
    dtype = np.dtype(('>' if bigendian else '<')+kind)
    if require_padding:
        if any(type(n) is not int or n <= 0 for n in (width, height, step)):
            raise ValueError('invalid image dimensions/stride')
        if step < width*dtype.itemsize or len(data) < height*step:
            raise ValueError('invalid image buffer/stride')
    elif (height <= 0 or (height > 1 and step < 0) or
            (height - 1)*step + width*dtype.itemsize > memoryview(data).nbytes):
        raise ValueError('invalid image buffer/stride')
    pixels = np.ndarray((height, width), dtype=dtype, buffer=data, strides=(step, dtype.itemsize))
    return pixels, scale


def _message_pixels(message):
    return _depth_pixels(message.data, message.width, message.height, message.step,
                         message.encoding, message.is_bigendian)


def decode_depth(message):
    """Return raw depth in metres, preserving invalids and optional trailing padding."""
    pixels, scale = _message_pixels(message)
    z = pixels.astype(float)
    z *= scale
    return z
