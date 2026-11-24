"""Separate RGB person evidence from conservative mask-supported 3-D positions."""
from dataclasses import dataclass
from base64 import b64encode
import numpy as np
from scipy.ndimage import binary_erosion, binary_dilation, distance_transform_edt
from .perception import Candidate,cloud,person_depth_pixels
from .confirmation_quality import QUALITY_PROXY_VERSION


@dataclass
class PersonMask:
    box: tuple
    confidence: float
    mask: np.ndarray
    class_id: int = 0
    semantic_class: str = 'person'
    posture: str = 'unknown'
    visibility: str = 'unknown'


def person_mask_packet(ns, rgb, detections, associations):
    """Serialize each accepted instance in the original RGB pixel coordinates."""
    import cv2

    instances = []
    for index, track_id in associations:
        detection = detections[index]
        mask = np.asarray(detection.mask, dtype=bool)
        if mask.shape != (rgb.height, rgb.width):
            raise ValueError("Person mask dimensions differ from the RGB capture")
        ok, png = cv2.imencode(".png", mask.astype(np.uint8) * 255, [cv2.IMWRITE_PNG_COMPRESSION, 1])
        if not ok:
            raise ValueError("Failed to encode person mask as PNG")
        instances.append(dict(instance_id=index, track_id=track_id, confidence=float(detection.confidence),
                              mask_png_base64=b64encode(png).decode("ascii")))
    return dict(schema_version=1, stamp_ns=ns, frame_id=rgb.header.frame_id,
                width=rgb.width, height=rgb.height, instances=instances)


class LocalizedPerson(Candidate):
    """Visible-surface point with measured mask and depth quality proxies."""


def boundary_depth_quality(mask, depth):
    """Measured near foreground around the instance, including outside its box.

    This is a conservative veto, not complete-body visibility evidence. A
    nearby floor, another person, furniture or segmentation error can veto a
    clear view; same-depth/remote hidden occluders can remain undetected. Keep
    the RGB candidate and use bounded review. Never include ring pixels in xyz.
    """
    mask=np.asarray(mask,bool)
    valid_depth=person_depth_pixels(depth) if depth is not None and depth.shape==mask.shape else None
    interior=binary_erosion(mask)&valid_depth if valid_depth is not None else None
    return _boundary_depth_quality(mask,depth,valid_depth,interior)


def _boundary_depth_quality(mask,depth,valid_depth,interior):
    radius=max(1,int(np.ceil(2.*min(mask.shape[0]/240.,mask.shape[1]/320.))))
    ring=binary_dilation(mask,iterations=radius)&~mask
    ring_count=int(ring.sum())
    result=dict(boundary_ring_radius_px=radius,boundary_ring_pixels=ring_count,
                boundary_depth_margin_m=.25,boundary_valid_fraction=0.,boundary_near_fraction=None,
                boundary_valid_pixels=0,boundary_near_pixels=0,
                visibility_state='unknown',
                boundary_reference='nearest_valid_eroded_mask_depth')
    if interior is None or not ring_count or not interior.any():return result
    valid=ring&valid_depth
    valid_count=int(valid.sum())
    if not valid_count:return result
    # The reference follows the local visible surface rather than assuming a
    # standing body, one global body depth, or a known floor height.
    nearest=distance_transform_edt(~interior,return_distances=False,return_indices=True)
    near_count=int(np.count_nonzero(depth[valid]<depth[tuple(nearest[:,valid])]-.25))
    result.update(boundary_valid_fraction=float(valid_count/ring_count),
                  boundary_near_fraction=float(near_count/valid_count),
                  boundary_valid_pixels=valid_count,boundary_near_pixels=near_count)
    # This reports observed evidence, not an occluded/clear body classifier.
    # Any measured nearer pixel is positive evidence; policy separately checks
    # its fraction. With no positive evidence, partial ring coverage is unknown.
    result['visibility_state']=('near_foreground_evidence' if near_count else
        'no_near_foreground_evidence' if valid_count==ring_count else 'unknown')
    return result


def localize_masks(detections,depth,k,camera_to_map):
    """A semantic detection survives any depth rejection, with xyz=None.

    Only eroded instance-mask pixels contribute. Never use box centers or a
    connected surface found outside the instance mask. Output is visible surface.
    """
    semantic=[];localized=[];points=None;valid_depth=None;invalid_projection=False
    for detection in detections:
        mask=np.asarray(detection.mask,bool)
        x,y,w,h=map(float,detection.box)
        mask_pixels=int(mask.sum())
        quality=dict(quality_proxy_version=QUALITY_PROXY_VERSION,
                     mask_pixels=mask_pixels,mask_box_fraction=float(mask_pixels/max(w*h,1.)),
                     border_clipped=bool(mask.ndim!=2 or x<2 or y<2 or x+w>mask.shape[1]-2 or y+h>mask.shape[0]-2))
        row=dict(box=[x,y,w,h],confidence=float(detection.confidence),
                 state='unlocalized',semantic_state='person_candidate',localization_state='unlocalized',
                 visibility_state='unknown',
                 xyz=None,depth_reason=None,quality=quality)
        semantic.append(row)
        if depth is None or k is None or camera_to_map is None:
            row['depth_reason']='missing_registered_depth_or_capture_transform';continue
        if mask.ndim!=2 or mask.shape!=depth.shape:
            row['depth_reason']='mask_depth_shape_mismatch';continue
        if valid_depth is None:valid_depth=person_depth_pixels(depth)
        support=binary_erosion(mask)
        count=int(support.sum());valid=support&valid_depth
        valid_count=int(valid.sum());valid_fraction=valid_count/max(count,1)
        quality['valid_depth_fraction']=valid_fraction
        if valid_count<20 or valid_fraction<.25:
            row['depth_reason']='insufficient_valid_mask_depth';continue
        ds=depth[valid];lo,hi=np.quantile(ds,[.05,.95])
        quality['depth_span_m']=float(hi-lo)
        median_depth=float(np.median(ds))
        quality['median_depth_m']=median_depth
        quality.update(_boundary_depth_quality(mask,depth,valid_depth,valid))
        row['visibility_state']=quality['visibility_state']
        # A closer surface inside the RGB box but outside the instance support
        # is observable evidence of possible foreground occlusion. It cannot
        # detect every occlusion and must not be called body-completeness proof.
        box_slice=np.s_[max(0,int(y)):min(mask.shape[0],int(np.ceil(y+h))),
                        max(0,int(x)):min(mask.shape[1],int(np.ceil(x+w)))]
        box_depth=depth[box_slice]
        foreground=~mask[box_slice]&(box_depth>.25)&(box_depth<median_depth-.25)
        quality['foreground_occlusion_fraction']=float(foreground.sum()/max(box_depth.size,1))
        if hi-lo>.7:
            row['depth_reason']='ambiguous_depth_span';continue
        inliers=valid&(depth>=lo)&(depth<=hi)
        if points is None and not invalid_projection:
            try:points=cloud(depth,k,camera_to_map)
            except ValueError:invalid_projection=True
        if invalid_projection:
            row['depth_reason']='invalid_calibration_or_transform';continue
        visible=points[inliers]
        if len(visible)<20:
            row['depth_reason']='insufficient_interior_support';continue
        qlo,qhi=np.quantile(visible,[.05,.95],axis=0);extent=qhi-qlo
        # No floor-height assumption: a person can be on another observed floor.
        if extent[2]>2.2 or np.linalg.norm(extent[:2])>1.6:
            row['depth_reason']='noncompact_surface';continue
        if extent[2]<.12:
            row['depth_reason']='insufficient_vertical_support';continue
        xyz=np.median(visible,axis=0)
        quality['depth_pixels']=len(visible)
        row.update(state='localized',localization_state='localized_surface',xyz=xyz.tolist(),depth_reason=None,depth_pixels=len(visible))
        localized.append(LocalizedPerson(xyz,inliers,tuple(detection.box),True,float(detection.confidence),
                                   float(np.median(depth[inliers])),float(np.median(np.linalg.norm(visible-xyz,axis=1))),quality.copy()))
    return semantic,localized
