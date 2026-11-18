"""Fail-closed contract for direct RGB-pixel / depth indexing.

Unregistered or asynchronous input must be synchronized and geometrically
reprojected by a separate implementation; this helper does not pretend to align it.
"""
import numpy as np


def stamp_ns(header):
    return int(header.stamp.sec)*1_000_000_000+int(header.stamp.nanosec)


def camera_uses_full_resolution(info):
    return (info.binning_x in (0,1) and info.binning_y in (0,1)
            and not any(getattr(info.roi,key) for key in ('x_offset','y_offset','width','height','do_rectify')))


def require_registered_pair(rgb, depth, rgb_info, depth_info, depth_to_rgb):
    """Validate actual Image/CameraInfo and TF messages, not generator promises."""
    sizes=[(m.width,m.height) for m in (rgb,depth,rgb_info,depth_info)]
    if len(set(sizes))!=1 or min(sizes[0])<=0:raise ValueError('RGB/depth dimensions differ')
    if rgb.encoding not in ('rgb8','bgr8') or depth.encoding!='32FC1':raise ValueError('Unsupported RGB/depth encoding')
    capture_ns=stamp_ns(rgb.header)
    if any(stamp_ns(m.header)!=capture_ns for m in (depth,rgb_info,depth_info)):
        raise ValueError('RGB/depth/calibration timestamps differ; synchronize before indexing')
    if stamp_ns(depth_to_rgb.header) not in (0,capture_ns):
        raise ValueError('registration_transform_time_mismatch')
    if rgb.header.frame_id!=rgb_info.header.frame_id or depth.header.frame_id!=depth_info.header.frame_id:
        raise ValueError('Image and calibration frames differ')
    if depth_to_rgb.header.frame_id!=rgb.header.frame_id or depth_to_rgb.child_frame_id!=depth.header.frame_id:
        raise ValueError('TF does not map depth optical to RGB optical')
    for key in ('k','r','p','d'):
        a,b=np.asarray(getattr(rgb_info,key)),np.asarray(getattr(depth_info,key))
        if a.shape!=b.shape or not np.isfinite(a).all() or not np.isfinite(b).all() or not np.allclose(a,b,rtol=0,atol=1e-9):
            raise ValueError(f'RGB/depth calibration {key} differs ({a.tolist()} vs {b.tolist()}); reproject before indexing')
    if rgb_info.k[0]<=0 or rgb_info.k[4]<=0 or rgb_info.distortion_model!=depth_info.distortion_model or np.any(np.asarray(rgb_info.d)!=0):
        raise ValueError('Invalid intrinsics or unhandled distortion')
    for info in (rgb_info,depth_info):
        if not camera_uses_full_resolution(info):
            raise ValueError('ROI/binning requires explicit registration')
    t=depth_to_rgb.transform.translation;q=depth_to_rgb.transform.rotation
    if not np.allclose([t.x,t.y,t.z],0,rtol=0,atol=1e-9) or not np.allclose([q.x,q.y,q.z,abs(q.w)],[0,0,0,1],rtol=0,atol=1e-9):
        raise ValueError('Nonidentity RGB/depth extrinsics; reproject before indexing')
    return True
