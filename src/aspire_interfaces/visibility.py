"""Shared visibility parameters for ROS launch and perception."""
from dataclasses import dataclass
import json
import math


@dataclass(frozen=True)
class VisibilityConfig:
    # Must be frozen from the actual camera/imaging validation, not tuned on test.
    min_range_m: float
    max_range_m: float
    max_capture_age_s: float
    min_pixels: int
    depth_margin_m: float
    max_pixels: int = 65536

    def __post_init__(self):
        values=(self.min_range_m,self.max_range_m,self.max_capture_age_s,
                self.min_pixels,self.depth_margin_m,self.max_pixels)
        if any(not math.isfinite(x) or x<=0 for x in values):
            raise ValueError('positive finite visibility limits required')
        if (self.max_range_m<=self.min_range_m or self.max_pixels<self.min_pixels
                or int(self.min_pixels)!=self.min_pixels or int(self.max_pixels)!=self.max_pixels):
            raise ValueError('invalid visibility range or pixel limits')


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
