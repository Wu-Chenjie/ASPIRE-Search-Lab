"""Capture-aligned local SURFACE-region visibility, never whole-person absence.

Used only after registered RGB-D and capture-time TF validation by the adapter.
A historical visible surface is not a body centre or a full target volume.
"""
from itertools import islice
from math import gcd
import json
from .static_hypothesis import projected_visibility, visibility_capture, Visibility, VisibilityConfig


def visibility_ros_arguments(runtime):
    """Validate optional prepared-runtime calibration and preserve ROS string type."""
    config=runtime.get('static_visibility_config')
    if config is None:return []
    model=runtime.get('static_visibility_model_id')
    if runtime.get('confirmation_mode')!='registry' or not isinstance(model,str) or not 0<len(model)<=128:
        raise ValueError('Surface visibility requires registry and explicit calibration identity')
    if not isinstance(config,dict):raise ValueError('Visibility configuration must be an object')
    VisibilityConfig(**config)
    # Outer JSON quotes become a YAML string; ROS must not interpret a dict parameter.
    return ['-p','static_visibility_config:='+json.dumps(json.dumps(config,separators=(',',':'))),
            '-p','static_visibility_model_id:='+json.dumps(model)]


def visibility_batch(evidence,tracker,ns,depth,k,transform,now,config,model_id):
    if config is None or not model_id or ns<=evidence.last_visibility_ns:return None
    evidence.last_visibility_ns=ns
    capture=ns*1e-9
    detected={row['track_id'] for row in tracker.frames.get(ns,[])}
    remaining=config.max_pixels;rows=[]
    count = len(evidence.last_qualified)
    if not count:
        return dict(schema=1,kind='visibility',now_s=now,regions=rows)
    prepared=visibility_capture(depth,k,transform,capture,now,config)
    limit = min(64, count)
    start = evidence.visibility_cursor % count
    selected = list(islice(evidence.last_qualified.items(), start, start + limit))
    if len(selected) < limit:
        selected.extend(islice(evidence.last_qualified.items(), limit - len(selected)))

    # Stay near the 64-slot stride for large queues. Coprime rotation
    # gives every stable anchor first access to the full frame budget.
    stride = limit if limit < count else 1
    while gcd(stride, count) > 1:
        stride -= 1
    evidence.visibility_cursor = (start + stride) % count
    for ident,point in selected:
        if point['capture_time_s']>capture:continue
        radius=point['radius_m'];center=point['xyz'];extent=[radius]*3
        if remaining<config.min_pixels:
            v=Visibility('unknown','frame_visibility_compute_budget',capture)
        elif isinstance(prepared, Visibility):
            v=prepared
        else:
            v=projected_visibility(center,extent,prepared,capture,config,remaining)
            remaining-=v.checked_pixels
        rows.append(dict(robot=evidence.robot,session=evidence.session,counter=str(ident),
            capture_s=capture,anchor_capture_s=point['capture_time_s'],frame_version=evidence.frame_version,
            cx=center[0],cy=center[1],cz=center[2],ex=extent[0],ey=extent[1],ez=extent[2],
            state=v.state,reason=v.reason,scope='visible_surface_region',model_id=model_id,
            detected_source=ident in detected,checked_pixels=v.checked_pixels))
    return dict(schema=1,kind='visibility',now_s=now,regions=rows)
