"""Exercise flight safety using the actual generated ROS2 and PX4 messages."""
import json
import time
from types import SimpleNamespace
import numpy as np
import pytest
pytest.importorskip('rclpy');pytest.importorskip('px4_msgs')
from rclpy.node import Node
from builtin_interfaces.msg import Time
from std_msgs.msg import String
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude, VehicleStatus, EstimatorStatusFlags
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist
from annual_runtime.px4_bridge_node import PX4Bridge, px4_topic


@pytest.fixture
def bridge(monkeypatch):
    params=dict(drone_id=1,px4_instance=2,spawn='[2,6,0.24]',sitl_auto_arm=True)
    monkeypatch.setattr(Node,'__init__',lambda *args,**kwargs:None)
    monkeypatch.setattr(Node,'declare_parameter',lambda self,key,default:SimpleNamespace(value=params.get(key,default)))
    monkeypatch.setattr(Node,'create_subscription',lambda *args,**kwargs:None)
    monkeypatch.setattr(Node,'create_timer',lambda *args,**kwargs:None)
    def publisher(*args):
        pub=SimpleNamespace(messages=[]);pub.publish=pub.messages.append;return pub
    monkeypatch.setattr(Node,'create_publisher',publisher)
    clock=[6_000_000_000]
    monkeypatch.setattr(Node,'get_clock',lambda self:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=clock[0],to_msg=lambda:Time(sec=clock[0]//1_000_000_000))))
    node=PX4Bridge();a=VehicleAttitude();a.timestamp_sample=6_000_000;a.q=[1.,0.,0.,0.];node.vehicle_attitude(a)
    flags=EstimatorStatusFlags();flags.timestamp=6_000_000;flags.cs_tilt_align=flags.cs_yaw_align=True
    node.estimator_status(flags)
    s=VehicleStatus();s.pre_flight_checks_pass=True;node.vehicle_status(s)
    m=VehicleLocalPosition();m.xy_valid=m.z_valid=m.v_xy_valid=m.v_z_valid=True
    m.heading_good_for_control=True
    for stamp in range(6_600_000,10_000_001,20_000):
        clock[0]=stamp*1000;a.timestamp_sample=stamp;node.vehicle_attitude(a)
        node.vehicle_status(s)
        flags.timestamp=stamp;node.estimator_status(flags)
        m.timestamp_sample=stamp;node.local_position(m)
    node.vehicle_status(s)
    return node,m


def next_attitude(node, stamp=10_000_002):
    a=VehicleAttitude();a.timestamp_sample=stamp;a.q=[1.,0.,0.,0.]
    node.vehicle_attitude(a)


def reference():
    m=MultiDOFJointTrajectory();m.header.frame_id='world';m.header.stamp=Time(sec=10)
    p=MultiDOFJointTrajectoryPoint();t=Transform();t.translation.x=3.;t.translation.y=6.;t.translation.z=1.5;t.rotation.w=1.
    v=Twist();v.linear.x=.5;v.angular.z=.4;a=Twist();a.linear.y=.2
    p.transforms=[t];p.velocities=[v];p.accelerations=[a];m.points=[p];return m


def test_offboard_target_frame_version_and_instance_routing(bridge):
    node,_=bridge;node.trajectory(reference());node.warmup=7.;node.tick()
    target=node.setpoint_pub.messages[-1]
    assert np.allclose(target.position,[0,1,-1.26])
    assert np.allclose(target.velocity,[0,.5,0]) and np.isclose(target.yawspeed,-.4)
    assert node.offboard_pub.messages[-1].position
    assert {m.target_system for m in node.command_pub.messages}=={3}
    assert px4_topic('/px4_2/fmu','out','vehicle_status',VehicleStatus).endswith('vehicle_status_v1')


def test_local_datum_does_not_depend_on_loiter_preflight_but_arming_does(bridge,monkeypatch):
    from annual_runtime.pose_history import GroundOriginWindow
    from px4_msgs.msg import VehicleCommand
    node,m=bridge;node.origin=None;node.origin_window=GroundOriginWindow()
    clock=[10_000_000_000]
    monkeypatch.setattr(node,'get_clock',lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=clock[0])))
    status=VehicleStatus(pre_flight_checks_pass=False,nav_state=VehicleStatus.NAVIGATION_STATE_AUTO_LOITER)
    for stamp in range(10_020_000,13_500_001,20_000):
        clock[0]=stamp*1000
        node.vehicle_status(status)
        node.estimator_status(EstimatorStatusFlags(timestamp=stamp,cs_tilt_align=True,cs_yaw_align=True))
        next_attitude(node,stamp);m.timestamp_sample=stamp;node.local_position(m)
    assert node.origin is not None and node.origin_window.reason=='calibrated'
    status.nav_state=VehicleStatus.NAVIGATION_STATE_OFFBOARD;node.vehicle_status(status)
    target=reference();target.header.stamp=Time(sec=13,nanosec=500_000_000)
    node.trajectory(target);node.warmup=10.;node.tick()
    assert node.offboard_pub.messages
    assert not any(x.command==VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM for x in node.command_pub.messages)


@pytest.mark.parametrize('fault',['reference','position','attitude','status'])
def test_stale_input_stops_offboard_heartbeat(bridge,fault):
    node,_=bridge;node.trajectory(reference());node.tick();before=len(node.offboard_pub.messages)
    setattr(node,fault+'_wall',time.monotonic()-31.)
    node.tick();assert len(node.offboard_pub.messages)==before and node.warmup is None


def test_ekf_reset_preserves_world_position_and_retires_old_reference(bridge):
    node,m=bridge;node.trajectory(reference());before=node.position.copy()
    m.timestamp_sample=10_000_001;m.x=1.;m.delta_xy=[1.,0.];m.xy_reset_counter=1
    next_attitude(node);node.local_position(m)
    assert np.allclose(node.position,before) and node.reference is None
    node.tick();assert not node.offboard_pub.messages


def test_reset_retires_setpoint_before_waiting_for_attitude(bridge):
    node,m=bridge;node.trajectory(reference())
    m.timestamp_sample=10_010_000;m.x=1.;m.delta_xy=[1.,0.];m.xy_reset_counter=1
    node.local_position(m)
    assert node.pending_position is not None and node.reference is None
    node.trajectory(reference())
    assert node.reference is None and node.position_reset_pending
    node.tick();assert not node.offboard_pub.messages


def test_invalid_estimator_and_wrong_frame_never_activate_offboard(bridge):
    node,m=bridge;packet=reference();packet.header.frame_id='map';node.trajectory(packet)
    assert node.reference is None
    node.trajectory(reference());m.timestamp_sample=10_000_001;m.xy_valid=False;node.local_position(m);node.tick()
    assert not node.offboard_pub.messages


def test_missing_ekf_reset_cannot_silently_change_world_origin(bridge):
    node,m=bridge;node.trajectory(reference())
    next_attitude(node)
    m.timestamp_sample=10_000_001;m.x=2.;m.delta_xy=[1.,0.];m.xy_reset_counter=2
    node.local_position(m);node.tick()
    assert node.position is None and not node.offboard_pub.messages
    assert node.alignment_fault=='lost_position_resets_require_new_map_session'


def test_corrupt_rotation_and_late_reference_are_rejected(bridge):
    node,_=bridge;packet=reference();packet.points[0].transforms[0].rotation.w=0.
    node.trajectory(packet);assert node.reference is None
    packet=reference();packet.header.stamp=Time(sec=8);node.trajectory(packet)
    assert node.reference is None


@pytest.mark.parametrize('clock_seconds', [9, 12])
def test_estimator_sample_from_another_clock_or_stale_queue_stops_flight(bridge, monkeypatch, clock_seconds):
    node,m=bridge;node.trajectory(reference())
    monkeypatch.setattr(Node,'get_clock',lambda self:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=clock_seconds*1_000_000_000)))
    m.timestamp_sample=10_000_001;node.local_position(m);node.tick()
    assert node.position is None and not node.offboard_pub.messages


def test_px4_failsafe_cannot_be_overridden_by_auto_offboard_requests(bridge):
    node,_=bridge;node.trajectory(reference());node.warmup=7.;node.status.failsafe=True;node.tick()
    assert not node.command_pub.messages and not node.offboard_pub.messages
    assert node.reason=='px4_failsafe'


def test_failsafe_landing_cannot_silently_trigger_another_auto_takeoff(bridge):
    node,_=bridge
    s=VehicleStatus();s.failsafe=True;s.arming_state=VehicleStatus.ARMING_STATE_ARMED
    node.vehicle_status(s)
    s=VehicleStatus();s.pre_flight_checks_pass=True
    node.vehicle_status(s);node.trajectory(reference());node.warmup=7.;node.tick()
    assert node.failsafe_latched and node.reference is None
    assert not node.command_pub.messages and not node.offboard_pub.messages


@pytest.mark.parametrize('sample', [1_000_000, 10_200_000, 0])
def test_stale_future_or_zero_attitude_cannot_create_odometry(bridge, sample):
    node,m=bridge;node.attitude=None;node.attitude_sample=0
    count=len(node.odom_pub.messages)
    a=VehicleAttitude();a.q=[1.,0.,0.,0.];a.timestamp_sample=sample
    node.vehicle_attitude(a);m.timestamp_sample=10_000_001
    node.local_position(m);node.trajectory(reference());node.tick()
    assert len(node.odom_pub.messages)==count
    assert node.position is None and not node.offboard_pub.messages


@pytest.mark.parametrize('sample', [10_000_000, 9_950_000, 1_000_000])
def test_duplicate_and_reordered_attitude_never_refresh_watchdog(bridge, sample):
    node,m=bridge;node.attitude_wall=time.monotonic()-2.1
    old_wall=node.attitude_wall;old_attitude=node.attitude.copy()
    a=VehicleAttitude();a.q=[0.,1.,0.,0.];a.timestamp_sample=sample
    node.vehicle_attitude(a)
    assert node.attitude_wall==old_wall and np.array_equal(node.attitude,old_attitude)
    m.timestamp_sample=10_000_001;node.local_position(m);node.trajectory(reference());node.tick()
    assert node.position is None and not node.offboard_pub.messages


@pytest.mark.parametrize('missing_left', [False, True])
def test_fresh_estimates_without_a_valid_bracket_never_publish(bridge, missing_left):
    from annual_runtime.pose_history import PoseHistory
    node,m=bridge
    if missing_left: node.attitude_history=PoseHistory()
    node.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=10_200_000_000))
    a=VehicleAttitude();a.q=[1.,0.,0.,0.];a.timestamp_sample=10_200_000
    node.vehicle_attitude(a);count=len(node.odom_pub.messages)
    m.timestamp_sample=10_100_000;node.local_position(m);node.trajectory(reference());node.tick()
    assert len(node.odom_pub.messages)==count and node.position is None
    assert not node.offboard_pub.messages


def test_sample_expiry_stops_heartbeat_even_with_recent_receipts(bridge, monkeypatch):
    node,_=bridge;node.trajectory(reference());node.tick();count=len(node.offboard_pub.messages)
    monkeypatch.setattr(Node,'get_clock',lambda self:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=11_600_000_000)))
    node.status_time=11.6;node.reference_wall=time.monotonic()
    node.tick()
    assert len(node.offboard_pub.messages)==count and node.warmup is None


def test_nearby_new_attitude_and_position_resume_after_invalid_pair(bridge):
    node,m=bridge;node.trajectory(reference());count=len(node.odom_pub.messages)
    m.timestamp_sample=10_200_000;node.local_position(m);node.tick()
    assert len(node.odom_pub.messages)==count
    next_attitude(node,10_040_000)
    a=VehicleAttitude();a.q=[1.,0.,0.,0.];a.timestamp_sample=10_060_000
    node.vehicle_attitude(a);m.timestamp_sample=10_050_000;node.local_position(m);node.tick()
    assert node.position is not None and len(node.odom_pub.messages)==count+1 and len(node.offboard_pub.messages)>=1


def test_unaligned_heading_cannot_publish_pose_or_start_offboard(bridge):
    node,m=bridge;count=len(node.odom_pub.messages);node.trajectory(reference())
    node.estimator_flags.cs_yaw_align=False
    m.timestamp_sample+=1;m.heading_good_for_control=False;node.local_position(m);node.tick()
    assert len(node.odom_pub.messages)==count and node.position is None
    assert not node.offboard_pub.messages and node.reason=='awaiting_heading_alignment'


def test_ground_initial_alignment_does_not_require_in_flight_final_alignment(bridge):
    node,m=bridge;next_attitude(node);m.timestamp_sample+=1;m.heading_good_for_control=False
    node.local_position(m);node.trajectory(reference());node.tick()
    assert node.position is not None and len(node.offboard_pub.messages)==1


def test_external_fusion_flags_alone_do_not_refresh_depth_measurement(bridge):
    node,_=bridge
    node.require_depth_odometry=True
    node.estimator_flags.cs_ev_pos=True
    node.estimator_flags.cs_ev_hgt=True
    node.estimator_flags.cs_ev_yaw=True
    assert not node.yaw_aligned(10.,time.monotonic())
    report=dict(source='gz_depth_rtabmap_icp',fault=None,accepted=7,time=10.,last_measurement_time=10.)
    node.depth_odometry_status(String(data=json.dumps(report)))
    assert node.yaw_aligned(10.,time.monotonic())
    report['last_measurement_time']=9.5
    node.depth_odometry_status(String(data=json.dumps(report)))
    assert not node.yaw_aligned(10.,time.monotonic())


def test_attitude_reset_after_mapping_clears_history_without_latching_session_fault(bridge):
    node,m=bridge;node.trajectory(reference());count=len(node.odom_pub.messages)
    a=VehicleAttitude();a.timestamp_sample=10_000_001;a.q=[1.,0.,0.,0.];a.quat_reset_counter=1
    node.vehicle_attitude(a);m.timestamp_sample+=1;node.local_position(m);node.tick()
    assert len(node.odom_pub.messages)==count and node.alignment_fault is None
    assert node.attitude_history.samples[-1][0] > 10.


def test_initial_flag_quaternion_race_never_creates_a_map_epoch(bridge):
    node,m=bridge;node.origin=None;node.alignment_since=10.;node.attitude_reset_time=10.
    count=len(node.odom_pub.messages);m.timestamp_sample+=1;node.local_position(m)
    assert len(node.odom_pub.messages)==count and node.origin is None
    a=VehicleAttitude();a.timestamp_sample=10_000_002;a.q=[1.,0.,0.,0.];a.quat_reset_counter=1
    node.vehicle_attitude(a)
    assert node.alignment_fault is None  # Startup reset before any map epoch is legal.
    m.timestamp_sample+=2;node.local_position(m)
    assert len(node.odom_pub.messages)==count and node.origin is None


@pytest.mark.parametrize('stamp',[10_010_000,10_025_000])
def test_position_waits_for_attitude_and_interpolates_at_its_timestamp(bridge, stamp):
    from scipy.spatial.transform import Rotation
    node,m=bridge;before=len(node.odom_pub.messages)
    node.trajectory(reference());node.tick();heartbeats=len(node.offboard_pub.messages)
    m.timestamp_sample=stamp
    node.local_position(m)
    assert len(node.odom_pub.messages)==before and node.pending_position is m
    node.tick()
    assert len(node.offboard_pub.messages)==heartbeats+1
    assert node.last_sample==10_000_000  # Waiting never refreshes pose freshness.
    a=VehicleAttitude();a.timestamp_sample=10_030_000
    q=Rotation.from_euler('z',.03).as_quat();a.q=[q[3],*q[:3]]
    node.vehicle_attitude(a)
    assert len(node.odom_pub.messages)==before+1 and node.pending_position is None
    from annual_runtime.px4_frames import attitude_to_enu
    q=Rotation.from_euler('z',(stamp-10_000_000)*1e-6).as_quat()
    expected=attitude_to_enu([q[3],*q[:3]])
    got=node.odom_pub.messages[-1].pose.pose.orientation
    assert np.allclose([got.x,got.y,got.z,got.w],expected)


@pytest.mark.parametrize('fault', ['position', 'attitude'])
def test_new_invalid_measurement_retires_waiting_position(bridge, fault):
    import copy
    node,m=bridge;before=len(node.odom_pub.messages)
    m.timestamp_sample=10_010_000;node.local_position(m)
    assert node.pending_position is not None
    if fault=='position':
        invalid=copy.deepcopy(m);invalid.timestamp_sample+=1;invalid.xy_valid=False
        node.local_position(invalid)
    else:
        a=VehicleAttitude();a.timestamp_sample=10_015_000;a.q=[0.,0.,0.,0.]
        node.vehicle_attitude(a)
    next_attitude(node,10_020_000)
    assert node.pending_position is None and len(node.odom_pub.messages)==before
    if fault=='position': assert node.position is None


def test_frontend_terminal_fault_retires_pose_and_reference(bridge):
    node,m=bridge;node.trajectory(reference());count=len(node.odom_pub.messages)
    node.depth_odometry_status(String(data=json.dumps(dict(source='gz_depth_rtabmap_icp',time=10.,
        fault='registration_discontinuity_requires_new_session',accepted=5,last_measurement_time=10.))))
    next_attitude(node);m.timestamp_sample+=1;node.local_position(m);node.trajectory(reference());node.tick()
    assert len(node.odom_pub.messages)==count and node.reference is None and node.alignment_fault
    node.depth_odometry_status(String(data=json.dumps(dict(source='gz_depth_rtabmap_icp',time=10.,
        fault=None,accepted=6,last_measurement_time=10.))))
    node.tick();assert not node.offboard_pub.messages


def test_failsafe_latch_also_stops_pose_publication_after_landing(bridge):
    node,m=bridge;count=len(node.odom_pub.messages)
    node.vehicle_status(VehicleStatus(arming_state=VehicleStatus.ARMING_STATE_ARMED,failsafe=True))
    node.vehicle_status(VehicleStatus(arming_state=VehicleStatus.ARMING_STATE_DISARMED,failsafe=False))
    next_attitude(node);m.timestamp_sample+=1;node.local_position(m)
    assert len(node.odom_pub.messages)==count and node.position is None and node.failsafe_latched


@pytest.mark.parametrize('payload',[None,[], 'invalid'])
def test_malformed_frontend_status_does_not_crash_bridge(bridge,payload):
    node,_=bridge
    node.depth_odometry_status(String(data=json.dumps(payload)))
    assert node.depth_odom_report is None and node.alignment_fault is None


def test_disarm_without_observed_failsafe_cannot_trigger_another_auto_takeoff(bridge):
    node,_=bridge;node.trajectory(reference())
    node.vehicle_status(VehicleStatus(timestamp=10_000_000,arming_state=VehicleStatus.ARMING_STATE_ARMED))
    # A paused bridge can miss the failsafe interval and resume after landing.
    node.vehicle_status(VehicleStatus(timestamp=10_000_002,arming_state=VehicleStatus.ARMING_STATE_DISARMED,
                                     nav_state=VehicleStatus.NAVIGATION_STATE_OFFBOARD,pre_flight_checks_pass=True))
    node.trajectory(reference());node.warmup=7.;node.tick()
    assert node.failsafe_latched and node.retirement_reason=='px4_disarmed_requires_new_session'
    assert node.reference is None and not node.command_pub.messages


def test_reordered_vehicle_status_cannot_fake_disarm_transition(bridge):
    node,_=bridge
    node.vehicle_status(VehicleStatus(timestamp=10_000_010,arming_state=VehicleStatus.ARMING_STATE_ARMED))
    node.vehicle_status(VehicleStatus(timestamp=10_000_009,arming_state=VehicleStatus.ARMING_STATE_DISARMED))
    assert not node.failsafe_latched and node.status.arming_state==VehicleStatus.ARMING_STATE_ARMED
