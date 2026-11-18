"""Actual node callbacks and generated messages, no DDS, simulator or flight."""
import json,copy
from pathlib import Path
import pytest
pytest.importorskip('rclpy');pytest.importorskip('px4_msgs')
from rclpy.serialization import serialize_message
from annual_runtime.control_timing import ControlTiming
from test_px4_bridge import bridge,reference
from test_execution_safety_ros import executor,activate

def read(path):return [json.loads(x) for x in path.read_text().splitlines()]

def test_actual_bridge_output_unchanged_and_received_to_ned_bound(bridge,tmp_path,monkeypatch):
 n,_=bridge;m=reference();raw=serialize_message(m);n.trajectory(m);n.tick();target=serialize_message(n.setpoint_pub.messages[-1]);heart=serialize_message(n.offboard_pub.messages[-1])
 monkeypatch.setenv('RACER_CONTROL_OBSERVATION','1');n.control_timing.close();n.control_timing=ControlTiming(tmp_path,1,'bridge')
 n.trajectory(m);n.tick();n.tick();bad=copy.deepcopy(m);bad.header.frame_id='map';n.trajectory(bad)
 assert serialize_message(m)==raw and serialize_message(n.setpoint_pub.messages[-1])==target
 assert serialize_message(n.offboard_pub.messages[-1])==heart
 n.control_timing.close();rows=read(tmp_path/'drone-1-bridge-observation.jsonl');accepted=next(r for r in rows if r['event']=='reference_accepted');sent=[r for r in rows if r['event']=='setpoint_sent'];assert len(sent)==2
 assert all(r['reference_key']==accepted['reference_key'] for r in sent)
 assert sent[0]['ned']['position']==list(n.setpoint_pub.messages[-1].position)
 assert next(r for r in rows if r['event']=='reference_rejected')['reason']=='state_frame_or_points_guard'
 assert not any(r['event']=='observation_error' for r in rows)

def test_actual_executor_output_and_clock_unchanged(executor,tmp_path,monkeypatch):
 h=executor;n=h.node;activate(h);n.tick();before=serialize_message(n.trajectory_pub.messages[-1]);progress=n.curve_time
 monkeypatch.setenv('RACER_CONTROL_OBSERVATION','1');n.control_timing.close();n.control_timing=ControlTiming(tmp_path,0,'executor');n.tick()
 assert serialize_message(n.trajectory_pub.messages[-1])==before and n.curve_time==progress
 n.control_timing.close();rows=read(tmp_path/'drone-0-executor-observation.jsonl');sent=next(r for r in rows if r['event']=='reference_sent')
 assert sent['executor']['token']=='native-test-1' and sent['executor']['position_stamp']==h.now()
 assert sent['executor']['curve_time']==progress and sent['applied_dt']==0.
 assert not any(r['event']=='observation_error' for r in rows)
