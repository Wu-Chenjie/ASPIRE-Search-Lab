"""Run-scoped motion envelope shared by planning and execution.

The launch process supplies a validated JSON environment value to each child,
including spawned planning workers. A trajectory cannot raise its own limits.
Missing configuration retains the historical envelope for old recordings.
"""
from dataclasses import asdict, dataclass
from functools import lru_cache
import json
import math
import os


ENVIRONMENT_KEY = 'ANNUAL_MOTION_LIMITS'


@dataclass(frozen=True)
class MotionLimits:
    speed: float = .6
    acceleration: float = .8
    jerk: float = 2.
    yaw_rate: float = .65

    def __post_init__(self):
        for name, value in asdict(self).items():
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError('Motion limit must be finite and positive: '+name)
            object.__setattr__(self, name, float(value))

    def accepts(self, curve, tolerance=.001):
        measured = curve.limits()
        if not all(math.isfinite(measured[k]) and measured[k] <= getattr(self, k)+tolerance
                   for k in ('speed', 'acceleration', 'jerk')):
            return False
        yaw_rate = curve.yaw_rate_limit()
        return math.isfinite(yaw_rate) and yaw_rate <= self.yaw_rate+tolerance

    def to_dict(self):
        return asdict(self)


# Pinned RACER single_drone_exploration.xml / single_drone_planner.xml.
# Experimental starting point, not an aircraft capability certification.
RACER_EXAMPLE = MotionLimits(1.5, 1., 4., 120.*3.1415926/180.)
# User-requested doubled translation limits; keep the same yaw-rate limit.
AGGRESSIVE = MotionLimits(3., 2., 8., RACER_EXAMPLE.yaw_rate)


@lru_cache(maxsize=16)
def _decode(raw):
    values = json.loads(raw)
    if not isinstance(values, dict):
        raise ValueError('motion_limits must be an object')
    return MotionLimits(**values)


def motion_limits():
    return _decode(os.environ.get(ENVIRONMENT_KEY, '{}'))


def motion_environment(config):
    limits = MotionLimits(**config.get('motion_limits', {}))
    return {ENVIRONMENT_KEY: json.dumps(limits.to_dict(), sort_keys=True)}
