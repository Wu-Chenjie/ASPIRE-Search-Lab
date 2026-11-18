"""Actual native optimizer boundary regression; no synthetic actuator claim."""
from pathlib import Path
import json
import os
import subprocess
import pytest


def test_native_generated_moving_curve_preserves_future_position_and_yaw_jets(tmp_path,monkeypatch):
    pytest.importorskip('rclpy')
    from ament_index_python.packages import get_package_prefix
    import numpy as np
    import yaml
    from tools.racer_ros2.configuration import configuration
    from core.planning.bspline_trajectory import native_trajectory,safe_native_execution
    from core.planning.handoff import validate_handoff
    from core.exploration.voxel_mapping import VoxelMap
    from core.planning.motion_limits import motion_limits
    root=Path(__file__).resolve().parents[1]
    limits=dict(speed=1.,acceleration=1.2,jerk=4.,yaw_rate=.9)
    monkeypatch.setenv('ANNUAL_MOTION_LIMITS',json.dumps(limits))
    monkeypatch.setenv('ANNUAL_VOXEL_MAP',json.dumps(dict(resolution=.15,collision_profile='legacy')))
    hello=dict(starts=[[0,0,1.8]],bounds=[[-3,-4,0],[7,4,4]],intrinsics=[216,0,160,0,216,120,0,0,1],
        flight_limits=[.7,3.],motion_limits=limits,planning_limits=dict(speed=.3,acceleration=.4,jerk=4.,yaw_rate=.325),
        min_time=False,stop_and_plan=True,confirmation_mode='registry',execution_safety=True,
        static_cancel_enabled=True,static_moving_c2=True,static_c2_lead_s=1.)
    cfg=configuration(hello,tmp_path,root/'external/racer_ros2_ws/src/racer_core/config/single_drone_planner.xml')
    params=next(c['parameters'] for c in cfg if c['executable']=='exploration_node')
    params.update(use_sim_time=False,**{'task_probe.end_yaw':.4})
    path=tmp_path/'probe.yaml';path.write_text(yaml.safe_dump({'/**':{'ros__parameters':params}}))
    binary=Path(get_package_prefix('racer_core'))/'lib/racer_core/task_recovery_probe'
    result=subprocess.run([str(binary),'--case','moving-boundary','--output',str(tmp_path/'probe'),
        '--ros-args','--params-file',str(path)],capture_output=True,text=True,timeout=45)
    (tmp_path/'native.log').write_text(result.stdout+'\n'+result.stderr)
    assert result.returncode==0,result.stdout+'\n'+result.stderr
    meta=json.loads((tmp_path/'probe-moving.json').read_text());assert meta['succeeded']
    old=native_trajectory(json.loads((tmp_path/'probe-old-curve.json').read_text()))
    new=native_trajectory(json.loads((tmp_path/'probe-new-curve.json').read_text()))
    t=meta['boundary'];jet=old.sample(t)
    assert np.linalg.norm(jet[1])>1e-3 and np.linalg.norm(jet[2])>1e-3 and abs(jet[4])>1e-3
    errors=validate_handoff(old,new,t,max(0.,t-.2))
    assert max(errors.values())<1e-8
    observed=json.loads((tmp_path/'probe-new-map.json').read_text());origin=np.array(observed['origin'])
    runtime=VoxelMap(np.array([origin,origin+observed['size']]),resolution=observed['resolution'],clearance=.6,flight_limits=(.7,3.))
    runtime.state[:]=np.array(observed['state'],dtype=np.int8).reshape(runtime.shape);runtime.rebuild()
    stopped,_=safe_native_execution(new,runtime)
    assert motion_limits().accepts(stopped) and runtime.safe_path(stopped.path(.01))
    assert max(validate_handoff(old,stopped,t,max(0.,t-.2)).values())<1e-8
    (tmp_path/'boundary-result.json').write_text(json.dumps(dict(errors=errors,planner_wall_s=meta['planner_wall_s'],no_flight=True),indent=2))
