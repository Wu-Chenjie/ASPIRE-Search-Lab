"""Visible support contracts for independent search traces."""
from dataclasses import dataclass
from sys import float_info
from typing import Tuple

def _finite_number(value: float, *, minimum: float = -float_info.max, maximum: float = float_info.max) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and minimum <= value <= maximum


def nonnegative_finite(value: float, name: str) -> None:
    if not _finite_number(value, minimum=0.):
        raise ValueError(f"{name} must be a finite nonnegative number")


def unit_interval(value: float, name: str) -> None:
    if _finite_number(value, minimum=0., maximum=1.):
        return
    nonnegative_finite(value, name)
    raise ValueError(f"{name} must be in [0, 1]")


def nonempty(value: str, name: str) -> None:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be a nonempty string")


@dataclass(frozen=True)
class SurfaceEstimate:
    xyz: Tuple[float, float, float]
    stamp: float
    frame_id: str
    quality: float
    valid_points: int

    def __post_init__(self) -> None:
        if not isinstance(self.xyz, (tuple, list)) or len(self.xyz) != 3 or any(
            not _finite_number(v) for v in self.xyz
        ):
            raise ValueError("xyz must contain three finite coordinates")
        object.__setattr__(self, "xyz", tuple(self.xyz))
        nonnegative_finite(self.stamp, "stamp")
        nonempty(self.frame_id, "frame_id")
        unit_interval(self.quality, "quality")
        if type(self.valid_points) is not int or self.valid_points < 1:
            raise ValueError("valid_points must be a positive integer")


