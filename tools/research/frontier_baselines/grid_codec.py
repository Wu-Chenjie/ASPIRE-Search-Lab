"""Lossless float64 transport; no occupancy or clearance quantization."""
import base64
import zlib
import numpy as np


def pack_grid(values):
    a=np.asarray(values,dtype='<f8')
    if a.ndim!=1 or not np.isfinite(a).all():raise ValueError('Invalid grid values')
    return dict(encoding='zlib_f64le',bytes=a.nbytes,data=base64.b64encode(zlib.compress(a.tobytes(),1)).decode('ascii'))
