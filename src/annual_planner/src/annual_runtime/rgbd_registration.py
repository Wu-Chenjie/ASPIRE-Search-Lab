"""Rectified RGB-D input geometry. No calibrated accuracy claim.

Only rectified optical cameras are supported. Time-dependent transforms must be
provided by a real pose/TF adapter. No mask dilation, hole filling, inferred body
pixels, or body-centre claim is made here.
"""
from dataclasses import dataclass
from functools import cached_property
import hashlib
import json
import numpy as np
from .depth_projection import _depth_pixels


@dataclass(frozen=True)
class RectifiedCamera:
    width: int
    height: int
    k: np.ndarray
    frame_id: str

    def __post_init__(self):
        k = np.asarray(self.k, dtype=float).reshape(3, 3).copy()
        if (not isinstance(self.width, int) or not isinstance(self.height, int)
                or min(self.width, self.height) <= 0 or not self.frame_id
                or not np.isfinite(k).all() or min(k[0, 0], k[1, 1]) <= 0
                or not np.allclose(k[2], [0, 0, 1])
                or not np.isclose(k[1, 0], 0) or not np.isclose(k[0, 1],0)):
            raise ValueError("invalid rectified camera")
        k.setflags(write=False)
        object.__setattr__(self, "k", k)

    @cached_property
    def inv_k(self):
        inverse = np.linalg.inv(self.k)
        inverse.setflags(write=False)
        return inverse


def rigid(matrix):
    t = np.asarray(matrix, dtype=float)
    if (t.shape != (4, 4) or not np.isfinite(t).all()
            or not np.allclose(t[3], [0, 0, 0, 1])
            or not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(t[:3, :3]), 1, atol=1e-6)):
        raise ValueError("invalid rigid transform")
    return t


@dataclass(frozen=True)
class CaptureTransform:
    matrix: np.ndarray
    source_frame: str
    target_frame: str
    source_stamp_ns: int
    target_stamp_ns: int

    def __post_init__(self):
        t = rigid(self.matrix).copy()
        if (not self.source_frame or not self.target_frame
                or any(type(v) is not int or v < 0 for v in
                       (self.source_stamp_ns, self.target_stamp_ns))):
            raise ValueError("invalid transform provenance")
        t.setflags(write=False)
        object.__setattr__(self, "matrix", t)


def decode_depth(data, width, height, step, encoding, bigendian=False):
    """Decode ROS-style depth bytes, including row padding. Invalids become NaN."""
    pixels, scale = _depth_pixels(data, width, height, step, encoding, bigendian,
                                  require_padding=True)
    z = pixels.astype(float)
    if scale != 1.:
        z *= scale
    z[~np.isfinite(z) | (z <= 0)] = np.nan
    return z


def register_depth(depth_m, depth_camera, rgb_camera, transform,
                   depth_stamp_ns, rgb_stamp_ns, *, max_skew_s=0.0,
                   static_scene=False):
    """Forward-project measured depth with a nearest-surface z-buffer.

    For unequal capture times, require an explicitly cross-time transform,
    an explicit skew bound, and a static-scene assumption. This function does
    not estimate camera motion or compensate moving people/disocclusions.
    Output z is RGB-optical-axis depth, never the original depth-camera z.
    """
    if (any(type(v) is not int or v < 0 for v in (depth_stamp_ns, rgb_stamp_ns))
            or not np.isfinite(max_skew_s) or max_skew_s < 0):
        raise ValueError("invalid capture timestamps/skew bound")
    skew = abs(rgb_stamp_ns - depth_stamp_ns) * 1e-9
    if skew > max_skew_s + 1e-12:
        raise ValueError("capture skew exceeds explicit bound")
    if skew and not static_scene:
        raise ValueError("cross-time registration requires static-scene assumption")
    if (transform.source_frame != depth_camera.frame_id
            or transform.target_frame != rgb_camera.frame_id
            or transform.source_stamp_ns != depth_stamp_ns
            or transform.target_stamp_ns != rgb_stamp_ns):
        raise ValueError("transform frames/timestamps do not match captures")
    z = np.asarray(depth_m, dtype=float)
    if z.shape != (depth_camera.height, depth_camera.width):
        raise ValueError("depth image/camera shape mismatch")
    v, u = np.nonzero((z > 0) & (z < np.inf))
    rays = np.column_stack((u, v, np.ones(len(u)))) @ depth_camera.inv_k.T
    p = rays * z[v, u, None]
    t = transform.matrix
    p = p @ t[:3, :3].T + t[:3, 3]
    p = p[np.isfinite(p).all(axis=1) & (p[:, 2] > 0)]
    projected = p @ rgb_camera.k.T
    uv = projected[:, :2] / projected[:, 2, None]
    inside = ((uv[:, 0] >= -0.5) & (uv[:, 0] < rgb_camera.width - 0.5)
              & (uv[:, 1] >= -0.5) & (uv[:, 1] < rgb_camera.height - 0.5))
    p, uv = p[inside], uv[inside]
    pixel = np.floor(uv + 0.5).astype(np.int64)
    index = pixel[:, 1] * rgb_camera.width + pixel[:, 0]
    output = np.full(rgb_camera.height * rgb_camera.width, np.inf)
    np.minimum.at(output, index, p[:, 2])
    hits = np.bincount(index, minlength=len(output))
    output[hits==0] = np.nan
    return output.reshape(rgb_camera.height, rgb_camera.width), {
        "rgb_stamp_ns": rgb_stamp_ns, "depth_stamp_ns": depth_stamp_ns,
        "skew_s": skew, "frame_id": rgb_camera.frame_id,
        "registered_pixels": int(np.count_nonzero(hits)),
        "collision_pixels": int((hits > 1).sum()),
        "filled_or_interpolated_pixels": 0,
        "scene_assumption": "static" if skew else "same_capture",
        "max_projection_quantization_m": float(.5*np.sqrt(1/rgb_camera.k[0,0]**2+1/rgb_camera.k[1,1]**2)*p[:,2].max()) if len(p) else 0.,
    }


def camera_info_model(info, image, mode='rectified'):
    """Bind intrinsics to one acquisition and mode; refuse unexpanded ROI/binning.

    rectified uses P in the advertised rectified optical frame. raw_pinhole uses
    K only for zero distortion. Nonzero stereo P offsets need an explicit driver
    adapter rather than silently interpreting a baseline as an optical origin.
    """
    from .rgbd_alignment import stamp_ns
    if (info.width,info.height,info.header.frame_id,stamp_ns(info.header))!=(image.width,image.height,image.header.frame_id,stamp_ns(image.header)):
        raise ValueError('camera_info_capture_mismatch')
    if info.binning_x not in (0,1) or info.binning_y not in (0,1) or any(getattr(info.roi,n) for n in ('x_offset','y_offset','width','height','do_rectify')):
        raise ValueError('ROI/binning must be expanded by the driver')
    if mode=='raw_pinhole':
        if np.any(np.asarray(info.d)!=0):raise ValueError('raw distortion requires rectification')
        k=np.asarray(info.k).reshape(3,3)
    elif mode=='rectified':
        if not np.allclose(np.asarray(info.r,float).reshape(3,3),np.eye(3),atol=1e-9,rtol=0):
            raise ValueError('nonidentity rectification requires explicit rectified optical-frame adapter')
        p=np.asarray(info.p,float).reshape(3,4)
        if not np.allclose(p[:,3],0):raise ValueError('stereo projection offset requires explicit optical-frame adapter')
        k=p[:,:3]
    else:raise ValueError('unsupported camera image mode')
    model=RectifiedCamera(image.width,image.height,k,image.header.frame_id)
    signature=dict(mode=mode,width=info.width,height=info.height,frame=info.header.frame_id,
                   k=list(info.k),p=list(info.p),r=list(info.r),d=list(info.d),distortion=info.distortion_model)
    identity=hashlib.sha256(json.dumps(signature,sort_keys=True,allow_nan=False).encode()).hexdigest()
    return model,identity

