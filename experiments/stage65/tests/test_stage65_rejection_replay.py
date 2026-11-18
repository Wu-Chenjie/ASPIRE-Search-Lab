"""Saved spline plus explicitly UNKNOWN map fixtures, never flight evidence."""
import json
from pathlib import Path
from types import SimpleNamespace as N
import numpy as np
import pytest
pytest.importorskip('rclpy')
import stage65_capture as c
from stage65_rejection_replay import replay
from core.exploration.voxel_mapping import VoxelMap
from core.planning.execution_safety import NativeSafetyMap

def fixture():
    root=Path('/tmp/racer-stage64-p03-a');config=json.loads((root/'runtime.json').read_text())
    row=json.loads((root/'native-racer/trajectories.jsonl').read_text().splitlines()[0])
    m=VoxelMap([[-3,-4,0],[7,4,4]],flight_limits=(.7,3.));n=NativeSafetyMap()
    n.ingest([1,1,1,m.resolution,*m.origin,*m.shape,*np.zeros(m.state.size,np.uint8)])
    x=N(id=0,replica=N(map=m),native_safety=n,get_clock=lambda:N(now=lambda:N(nanoseconds=1000000000)),
        pose=np.array([0.,-.9,1.5]),safety_enabled=True,execution={'reference':[0.,-.9,1.5]})
    t,meta,arrays=c.snapshot(x,'gateway',extra=dict(rejected_proposal=row,rejection_reason='No known-free native prefix with C2 stop',execution_feedback=x.execution,last_sensor_stamp=1.))
    return config,meta,arrays

def test_original_authorizer_reproduces_unknown_map_rejection():
    config,meta,arrays=fixture();r=replay(meta,arrays,config)
    assert r['verified'] is True and r['status']=='matched_original_rejection'

def test_contradictory_saved_reason_is_not_silently_accepted():
    config,meta,arrays=fixture();meta['rejection_reason']='native_static_root_stale';r=replay(meta,arrays,config)
    assert r['verified'] is False and r['status']=='contradiction_requires_isolation'

def test_missing_native_state_is_not_reconstructed_as_free():
    config,meta,arrays=fixture();meta['native_state_available']=False;arrays.pop('native_state');r=replay(meta,arrays,config)
    assert r['verified'] is False and r['status']=='actual_context_incomplete'


def test_unmatched_cached_map_context_is_not_silently_rebuilt_as_equivalent():
    config,meta,arrays=fixture();meta['mapper_cache_matches_state']=False;r=replay(meta,arrays,config)
    assert r['verified'] is False and r['status']=='actual_context_incomplete'
