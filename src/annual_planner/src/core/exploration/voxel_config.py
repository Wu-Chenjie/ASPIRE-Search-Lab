"""Explicit per-run voxel resolution and locked-X500 collision profile.

Legacy defaults stay active. x500_swept_sphere is an experimental geometry
profile whose closed-loop flight acceptance has not passed.
"""
from dataclasses import dataclass, asdict
from functools import lru_cache
import json
import os

ENVIRONMENT_KEY='ANNUAL_VOXEL_MAP'


@dataclass(frozen=True)
class VoxelConfig:
    resolution: float=.3
    collision_profile: str='legacy'
    legacy_clearance_m: float=.6

    def __post_init__(self):
        if type(self.resolution) not in (int,float) or not .05<=self.resolution<=.5:
            raise ValueError('voxel resolution must be finite and in [0.05,0.5] m')
        if self.collision_profile not in ('legacy','x500_swept_sphere'):
            raise ValueError('Unknown collision profile')
        c = self.legacy_clearance_m
        if type(c) not in (float,int) or not .50<=c<=.60:
            raise ValueError('Legacy horizontal clearance must be in [0.50, 0.60] m')
        object.__setattr__(self,'resolution',float(self.resolution))

    @property
    def clearance(self):return .43 if self.collision_profile=='x500_swept_sphere' else self.legacy_clearance_m


@lru_cache(maxsize=16)
def _decode(raw):return VoxelConfig(**json.loads(raw))


def voxel_config():return _decode(os.environ.get(ENVIRONMENT_KEY,'{}'))


def voxel_environment(config):
    return {ENVIRONMENT_KEY:json.dumps(asdict(VoxelConfig(**config.get('voxel_map',{}))),sort_keys=True)}
