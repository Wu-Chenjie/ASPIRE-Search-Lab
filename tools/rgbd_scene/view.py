#!/usr/bin/env python3
"""Manually teleport the fixed camera test rig; never commands PX4."""
import argparse
import json
import math
from pathlib import Path
import subprocess


def switch(run_dir,name):
    config=json.loads((Path(run_dir)/'runtime.json').read_text())
    if not config.get('manual_camera_test'):raise ValueError('Refuse to teleport a flight setup')
    setup=json.loads((Path(run_dir)/'viewpoints.json').read_text())
    pose=setup['views'][name]['model_pose'];x,y,z,roll,pitch,yaw=pose
    if roll or pitch:raise ValueError('Expected level manual test views')
    request=f'name: "{setup["model"]}" position {{ x: {x} y: {y} z: {z} }} orientation {{ w: {math.cos(yaw/2)} x: 0 y: 0 z: {math.sin(yaw/2)} }}'
    cmd=['gz','service','-s',f'/world/{setup["world"]}/set_pose','--reqtype','gz.msgs.Pose','--reptype','gz.msgs.Boolean','--timeout','5000','--req',request]
    result=subprocess.run(cmd,capture_output=True,text=True,check=True)
    if 'data: true' not in result.stdout:raise RuntimeError('View switch rejected: '+result.stdout+result.stderr)
    return dict(view=name,pose=pose,response=result.stdout)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run_dir',type=Path);p.add_argument('view',choices=['visible','partial','hidden','revealed']);a=p.parse_args();print(json.dumps(switch(a.run_dir,a.view)))
