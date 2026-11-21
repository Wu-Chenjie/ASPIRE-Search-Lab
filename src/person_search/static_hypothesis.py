"""Registered-depth visibility components for static surface checks.

Depth Checks are intentionally narrow: only a wholly imaged hypothesis volume
with valid depth behind it can produce CLEAR. A surface within the volume remains UNKNOWN.
"""
from dataclasses import dataclass
from itertools import product
import math
import numpy as np
from .perception import validate_transform
from aspire_interfaces.visibility import VisibilityConfig


@dataclass(frozen=True)
class Visibility:
    state: str                 # clear / occluded / unknown / outside
    reason: str
    capture_s: float
    checked_pixels: int = 0


CORNER_SIGNS = np.array(list(product((-1, 1), repeat=3)))


def hypothesis_bounds(center, half_extent):
    center, extent = np.asarray(center, float), np.asarray(half_extent, float)
    if (center.shape != (3,) or extent.shape != (3,) or
            not np.isfinite(center).all() or not np.isfinite(extent).all() or np.any(extent <= 0)):
        raise ValueError('invalid hypothesis geometry')
    return center, extent


def visibility_capture(depth, k, camera_to_map, capture_s, now, config, sensor_valid=True):
    """Validate the capture once before projecting any hypothesis regions."""
    if not (sensor_valid and 0 <= capture_s <= now <= capture_s + config.max_capture_age_s
            and math.isfinite(capture_s) and math.isfinite(now)):
        return Visibility('unknown', 'sensor_or_capture_invalid', capture_s)
    try:
        matrix = np.asarray(k, float).reshape(3, 3)
        transform = validate_transform(camera_to_map)
        if (not np.isfinite(matrix).all() or min(matrix[0, 0], matrix[1, 1]) <= 0
                or not np.allclose(matrix[2], [0, 0, 1])):
            return Visibility('unknown', 'hypothesis_or_calibration_invalid', capture_s)
        image = np.asarray(depth)
        if image.ndim != 2 or not image.size:
            return Visibility('unknown', 'registered_depth_missing', capture_s)
        return image, matrix, transform
    except (TypeError, ValueError, IndexError, OverflowError):
        return Visibility('unknown', 'geometry_invalid', capture_s)


def depth_visibility(center,half_extent,depth,k,camera_to_map,capture_s,now,config,
                     *,sensor_valid=True):
    """Check the full projected region using registered depth and capture-time TF."""
    prepared = visibility_capture(depth,k,camera_to_map,capture_s,now,config,sensor_valid)
    if isinstance(prepared, Visibility):
        return prepared
    return projected_visibility(center,half_extent,prepared,capture_s,config,config.max_pixels)


def projected_visibility(center,half_extent,prepared,capture_s,config,max_pixels):
    def result(state,reason,pixels=0):
        return Visibility(state,reason,capture_s,pixels)
    image,matrix,transform=prepared
    try:
        center,extent=hypothesis_bounds(center,half_extent)
    except (TypeError,ValueError,OverflowError):
        return result('unknown','hypothesis_or_calibration_invalid')
    try:
        corners=center+extent*CORNER_SIGNS
        camera=(corners-transform[:3,3])@transform[:3,:3]
        near,far=float(camera[:,2].min()),float(camera[:,2].max())
        if near<=0:
            return result('outside','hypothesis_behind_camera')
        if near<config.min_range_m or np.linalg.norm(camera,axis=1).max()>config.max_range_m:
            return result('outside','imaging_range_invalid')
        uv=camera@matrix.T
        uv=uv[:,:2]/uv[:,2,None]
        lo=np.floor(uv.min(axis=0))
        hi=np.ceil(uv.max(axis=0))
        if not np.isfinite(uv).all():
            return result('unknown','geometry_invalid')
        if np.any(lo<0) or np.any(hi>=image.shape[::-1]):
            return result('outside','hypothesis_not_fully_in_frame')
        lo,hi=lo.astype(int),hi.astype(int)
        count=int(np.prod(hi-lo+1))
        if count<config.min_pixels:
            return result('unknown','insufficient_image_support')
        if count>max_pixels:
            return result('unknown','visibility_compute_budget')
        pixels=np.asarray(image[lo[1]:hi[1]+1,lo[0]:hi[0]+1],dtype=float)
        closest,furthest=float(pixels.min()),float(pixels.max())
        if not 0<closest<=furthest<=config.max_range_m:
            return result('unknown','unknown_depth_pixels',count)
        if closest<near-config.depth_margin_m:
            return result('occluded','foreground_depth',count)
        if closest<=far+config.depth_margin_m:
            return result('unknown','surface_intersects_hypothesis',count)
        return result('clear','observed_depth_behind_entire_hypothesis',count)
    except (TypeError,ValueError,IndexError,OverflowError):
        return result('unknown','geometry_invalid')
