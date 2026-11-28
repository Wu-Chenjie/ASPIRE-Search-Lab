"""SITL-only PX4 boundary: EKF2 odometry and fresh authorized ENU trajectories.

The research executor remains responsible for leases, separation and observation
completion. PX4 runs the estimator, position/attitude loops and motor allocation.
A stale reference or estimator stops Offboard heartbeat, invoking PX4 failsafe.
"""
import json
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from builtin_interfaces.msg import Time
from trajectory_msgs.msg import MultiDOFJointTrajectory
from px4_msgs.msg import (VehicleLocalPosition, VehicleAttitude, VehicleStatus, EstimatorStatusFlags,
                          VehicleCommand, VehicleCommandAck, OffboardControlMode,
                          TrajectorySetpoint)
from annual_runtime.px4_frames import ENU_NED, attitude_to_enu, trajectory_to_ned
from annual_runtime.pose_history import PoseHistory, GroundOriginWindow
from annual_runtime.control_timing import ControlTiming
from annual_runtime.control_observation import record, reference_record, setpoint_record, local_record, odometry_record
from annual_runtime.heartbeat_diagnostics import HeartbeatDiagnostics
from annual_runtime.visual_odometry import depth_fusion_flags_ready

PX4_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)


def px4_topic(prefix, direction, name, message_type):
    version = getattr(message_type, 'MESSAGE_VERSION', 0)
    return f'{prefix}/{direction}/{name}' + (f'_v{version}' if version else '')


class PX4Bridge(Node):
    def __init__(self):
        super().__init__('px4_bridge')
        self.id = int(self.declare_parameter('drone_id', 0).value)
        self.control_timing = ControlTiming(self.declare_parameter('control_timing_dir', '').value, self.id, 'bridge')
        self.heartbeat_diagnostics = HeartbeatDiagnostics(self.control_timing)
        self.reference_stamp = -np.inf
        self.instance = int(self.declare_parameter('px4_instance', self.id + 1).value)
        self.spawn = np.array(json.loads(self.declare_parameter('spawn', '[0,0,0]').value), float)
        self.initial_position = np.array(json.loads(self.declare_parameter(
            'initial_position', json.dumps(self.spawn.tolist())).value), float)
        self.origin = None; self.resets = None; self.origin_ground_checks = {}
        self.position_reset_pending = False
        self.origin_window = GroundOriginWindow(float(self.declare_parameter('origin_calibration_s', 3.).value))
        self.require_depth_odometry = bool(self.declare_parameter('require_depth_odometry', False).value)
        self.depth_fuse_height=bool(self.declare_parameter('depth_odometry_fuse_height',True).value)
        self.depth_odom_report = None
        self.depth_odom_wall = -np.inf
        self.auto_arm = bool(self.declare_parameter('sitl_auto_arm', False).value)
        self.position = None; self.attitude = None; self.status = None
        self.position_wall = -np.inf; self.attitude_wall = -np.inf; self.status_wall = -np.inf; self.status_time = -np.inf
        self.reference = None; self.reference_wall = -np.inf; self.warmup = None
        self.last_command = -np.inf; self.last_ack = None; self.last_sample = 0
        self.attitude_sample = 0
        self.attitude_history = PoseHistory()
        self.pending_position = None
        self.attitude_reset = None; self.alignment_fault = None; self.heading_ready = False
        self.failsafe_latched = False
        self.retirement_reason = None
        self.estimator_flags = None; self.flags_wall = -np.inf; self.flags_time = -np.inf
        self.alignment_since = None; self.attitude_reset_time = np.inf
        self.reason = 'awaiting_estimator'; self.heartbeat_count = 0
        prefix = f'/px4_{self.instance}/fmu'
        self.offboard_pub = self.create_publisher(OffboardControlMode, px4_topic(prefix,'in','offboard_control_mode',OffboardControlMode), PX4_QOS)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, px4_topic(prefix,'in','trajectory_setpoint',TrajectorySetpoint), PX4_QOS)
        self.command_pub = self.create_publisher(VehicleCommand, px4_topic(prefix,'in','vehicle_command',VehicleCommand), PX4_QOS)
        self.odom_pub = self.create_publisher(Odometry, f'/drone_{self.id}/estimated_odometry', PX4_QOS)
        self.report_pub = self.create_publisher(String, f'/drone_{self.id}/flight_status', 1)
        self.create_subscription(VehicleLocalPosition, px4_topic(prefix,'out','vehicle_local_position',VehicleLocalPosition), self.local_position, PX4_QOS)
        self.create_subscription(VehicleAttitude, px4_topic(prefix,'out','vehicle_attitude',VehicleAttitude), self.vehicle_attitude, PX4_QOS)
        self.create_subscription(VehicleStatus, px4_topic(prefix,'out','vehicle_status',VehicleStatus), self.vehicle_status, PX4_QOS)
        self.create_subscription(EstimatorStatusFlags, px4_topic(prefix,'out','estimator_status_flags',EstimatorStatusFlags), self.estimator_status, PX4_QOS)
        if self.require_depth_odometry:
            self.create_subscription(String, f'/drone_{self.id}/depth_odometry/status', self.depth_odometry_status, 1)
        self.create_subscription(VehicleCommandAck, px4_topic(prefix,'out','vehicle_command_ack',VehicleCommandAck), self.command_ack, PX4_QOS)
        self.create_subscription(MultiDOFJointTrajectory, f'/drone_{self.id}/command/trajectory', self.trajectory, 1)
        # A wall clock catches stale ROS timers while the simulator is paused.
        self.create_timer(.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.create_timer(.2, self.report, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def observe_input(self,kind,stamp):
        self.heartbeat_diagnostics.input(kind,stamp,self.get_clock().now().nanoseconds*1e-9,time.monotonic())

    def attitude_fresh(self, now, wall):
        return (self.attitude is not None and self.attitude_sample > 0 and
                -.1 <= now-self.attitude_sample*1e-6 <= 1.5 and
                wall-self.attitude_wall < 2.)

    def vehicle_attitude(self, message):
        self.observe_input('attitude',message.timestamp_sample*1e-6)
        # Duplicate/reordered samples never refresh the receipt watchdog.
        if message.timestamp_sample <= self.attitude_sample:
            return
        now = self.get_clock().now().nanoseconds*1e-9
        if not -.1 <= now-message.timestamp_sample*1e-6 <= 1.5:
            self.pending_position = None; return
        try:
            attitude = attitude_to_enu(message.q)
        except ValueError:
            self.pending_position = None; return
        if (self.origin is not None and self.attitude_reset is not None and
                message.quat_reset_counter != self.attitude_reset):
            if getattr(self, '_logger', None) is not None:
                self.get_logger().warning('Attitude quaternion reset observed; resetting attitude history buffer', throttle_duration_sec=5.)
        if self.attitude_reset != message.quat_reset_counter:
            self.attitude_reset_time = message.timestamp_sample*1e-6
            self.attitude_history = PoseHistory()
            self.pending_position = None
        self.attitude_reset = message.quat_reset_counter
        self.attitude = attitude; self.attitude_sample = message.timestamp_sample
        self.attitude_wall = time.monotonic()
        self.attitude_history.add(message.timestamp_sample*1e-6, [0,0,0], attitude)
        if self.pending_position is not None:
            pending = self.pending_position
            self.pending_position = None
            self.local_position(pending)

    def vehicle_status(self, message):
        self.observe_input('status',message.timestamp*1e-6)
        if self.status and self.status.timestamp>0 and message.timestamp<=self.status.timestamp:
            return
        was_armed=bool(self.status and self.status.arming_state==VehicleStatus.ARMING_STATE_ARMED)
        disarmed=was_armed and message.arming_state!=VehicleStatus.ARMING_STATE_ARMED
        if (message.failsafe and message.arming_state == VehicleStatus.ARMING_STATE_ARMED) or disarmed:
            # Auto-arm is for the initial SITL takeoff, never an automatic retry
            # after PX4 has taken over and landed due to a flight failure.
            self.failsafe_latched = True
            if self.retirement_reason is None:
                self.retirement_reason=('px4_disarmed_requires_new_session' if disarmed else
                                        'px4_failsafe_requires_new_session')
            self.reference = None; self.warmup = None
        self.status = message; self.status_wall = time.monotonic()
        self.status_time = self.get_clock().now().nanoseconds*1e-9

    def estimator_status(self, message):
        self.observe_input('flags',message.timestamp*1e-6)
        # heading_good_for_control additionally requires an IN-FLIGHT mag
        # alignment in this PX4 revision. Initial yaw alignment is the boot gate.
        if message.timestamp <= self.flags_time*1e6:
            return
        self.estimator_flags = message
        self.flags_time = message.timestamp*1e-6; self.flags_wall = time.monotonic()
        if not (message.cs_tilt_align and message.cs_yaw_align):
            self.alignment_since = None
        elif self.alignment_since is None:
            self.alignment_since = self.get_clock().now().nanoseconds*1e-9

    def depth_odometry_status(self, message):
        try:
            report=json.loads(message.data)
            if not isinstance(report,dict):
                raise ValueError('Invalid depth odometry report')
            now=self.get_clock().now().nanoseconds*1e-9
            if (report.get('source')=='gz_depth_rtabmap_icp' and
                    isinstance(report.get('fault'),str) and report['fault'] and
                    -.02<=now-report['time']<=.3):
                self.alignment_fault='depth_odometry_'+report['fault']
                self.position=None; self.reference=None; self.warmup=None
            if (report['source']!='gz_depth_rtabmap_icp' or report['fault'] is not None or
                    report['accepted']<=0 or not np.isfinite([report['time'],report['last_measurement_time']]).all()):
                raise ValueError('Invalid measured depth odometry')
            self.depth_odom_report=report
            self.depth_odom_wall=time.monotonic()
        except (ValueError,TypeError,KeyError):
            self.depth_odom_report=None

    def depth_odometry_fresh(self, now, wall):
        report=self.depth_odom_report
        return bool(report and -.02<=now-report['last_measurement_time']<=.3 and
                    -.02<=now-report['time']<=.3 and wall-self.depth_odom_wall<.5)

    def yaw_aligned(self, now, wall):
        return bool(self.estimator_flags and self.estimator_flags.cs_tilt_align and
                    self.estimator_flags.cs_yaw_align and
                    (not self.require_depth_odometry or
                     (depth_fusion_flags_ready(self.estimator_flags,self.depth_fuse_height) and
                      self.depth_odometry_fresh(now,wall))) and
                    -.1 <= now-self.flags_time <= 2. and wall-self.flags_wall < 3.)

    def command_ack(self, message):
        self.last_ack = dict(command=int(message.command), result=int(message.result))

    def local_position(self, message):
        record(self, 'local_position_received', lambda: local_record(self, message))
        self.observe_input('raw_position',message.timestamp_sample*1e-6)
        wall = time.monotonic()
        now = self.get_clock().now().nanoseconds*1e-9
        if message.timestamp_sample <= self.last_sample:
            return
        # A newer invalid measurement must also retire an older waiting sample.
        self.pending_position = None
        self.heading_ready = bool(message.heading_good_for_control)
        if (self.alignment_fault or self.failsafe_latched or not self.yaw_aligned(now, wall) or
                not all((message.xy_valid, message.z_valid, message.v_xy_valid, message.v_z_valid)) or
                not self.attitude_fresh(now, wall)):
            self.position = None; return
        local = np.array([message.x, message.y, message.z], float)
        velocity = np.array([message.vx, message.vy, message.vz], float)
        sample=message.timestamp_sample*1e-6
        if not np.isfinite([*local, *velocity]).all() or not -.1 <= now-sample <= 1.5:
            return
        counters = (message.xy_reset_counter, message.z_reset_counter)
        if self.origin is not None and self.resets != counters:
            # A position reset changes the setpoint frame immediately, even if
            # this new position is still waiting for its attitude bracket.
            self.reference = None; self.warmup = None
            self.position_reset_pending = True
        matched = self.attitude_history.at(sample)
        if matched is None:
            # The DDS position topic may arrive before its attitude bracket.
            # Hold the sample briefly; do not stamp a previous rotation as current.
            self.pending_position = message if not self.attitude_history.passed(sample) else None
            # Waiting for the right bracket is not a failed estimator. Keep the
            # LAST published synchronized pose under its original watchdog;
            # otherwise a timer between the two DDS callbacks starves Offboard.
            # No pose or receipt timestamp is advanced while waiting.
            if self.pending_position is None: self.position = None
            return
        sample_attitude = matched[1].as_quat()
        self.pending_position = None
        if self.origin is None:
            # Flags and quaternion travel on independent DDS topics. The yaw
            # aligned flag can arrive before the corresponding reset quaternion.
            # Start one map epoch only after both have settled, on the ground.
            if (self.alignment_since is None or now-self.alignment_since < .5 or
                    sample-self.attitude_reset_time < .5):
                self.position = None; return
            # Freeze one datum only after a stationary, disarmed sample window.
            # A single noisy first sample otherwise biases the entire map.
            # PX4's preflight bit is for the CURRENT navigation mode. Before
            # Offboard it may reject global-position loiter despite a valid
            # local estimator. Datum collection is not an arming request:
            # keep local pose/velocity/heading, stationarity and reset checks;
            # tick() still requires Offboard preflight before sending ARM.
            grounded = bool(self.status and not self.status.failsafe and
                            -.1 <= now-self.status_time < 2. and wall-self.status_wall < 3. and
                            self.status.arming_state != VehicleStatus.ARMING_STATE_ARMED and
                            not self.estimator_flags.cs_in_air)
            self.origin_ground_checks=dict(status_present=bool(self.status),
                status_not_failsafe=bool(self.status and not self.status.failsafe),
                status_sim_age_valid=bool(-.1<=now-self.status_time<2.),
                status_wall_age_valid=bool(wall-self.status_wall<3.),
                status_disarmed=bool(self.status and self.status.arming_state!=VehicleStatus.ARMING_STATE_ARMED),
                estimator_not_in_air=not self.estimator_flags.cs_in_air,
                status_wall_age_s=wall-self.status_wall,status_sim_age_s=now-self.status_time)
            center = self.origin_window.observe(sample, local, velocity,
                (*counters, self.attitude_reset), grounded)
            if center is None:
                self.position = None; return
            self.origin = self.initial_position - ENU_NED @ center
        elif self.resets != counters:
            if any((new-old) % 256 > 1 for new,old in zip(counters,self.resets)):
                self.alignment_fault='lost_position_resets_require_new_map_session'
                self.position = None; self.reference = None; self.warmup = None
                return  # A skipped reset cannot reconstruct the world origin.
            delta = np.array([*message.delta_xy, message.delta_z], float)
            if counters[0] == self.resets[0]: delta[:2] = 0.
            if counters[1] == self.resets[1]: delta[2] = 0.
            if not np.isfinite(delta).all():
                self.position = None; return
            self.origin -= ENU_NED @ delta
            # Old setpoints use the previous EKF origin. A new executor reference
            # is required before Offboard heartbeat may resume.
            self.reference = None; self.warmup = None
        self.resets = counters; self.position_reset_pending = False; self.last_sample = message.timestamp_sample
        self.position = self.origin + ENU_NED @ local; self.position_wall = wall
        odom = Odometry(); odom.header.stamp = Time(sec=message.timestamp_sample//1_000_000,
                                                  nanosec=(message.timestamp_sample%1_000_000)*1000)
        odom.header.frame_id = 'world'; odom.child_frame_id = f'drone_{self.id}/base_link'
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = map(float, self.position)
        q = odom.pose.pose.orientation
        q.x, q.y, q.z, q.w = map(float, sample_attitude)
        # ROS Odometry twist is expressed in child_frame_id (body FLU).
        body_velocity = matched[1].inv().apply(ENU_NED @ velocity)
        odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = map(float, body_velocity)
        self.odom_pub.publish(odom)
        record(self, 'odometry_sent', lambda: dict(local_record(self, message), odometry=odometry_record(self, odom), attitude_quaternion=sample_attitude))

    def trajectory(self, message):
        record(self, 'reference_received', lambda: reference_record(self, message))
        self.control_timing.event('reference_receive', stamp_ns=message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec)
        if self.alignment_fault or self.failsafe_latched or self.position_reset_pending or self.origin is None or message.header.frame_id != 'world' or len(message.points) != 1:
            record(self, 'reference_rejected', lambda: dict(reason='state_frame_or_points_guard', frame=message.header.frame_id, point_count=len(message.points)))
            return
        point = message.points[0]
        if len(point.transforms) != 1 or len(point.velocities) != 1 or len(point.accelerations) != 1:
            record(self, 'reference_rejected', lambda: dict(reason='pva_shape_guard'))
            return
        p = point.transforms[0].translation; q = point.transforms[0].rotation
        v = point.velocities[0]; a = point.accelerations[0]
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        stamp = message.header.stamp.sec + message.header.stamp.nanosec*1e-9
        now = self.get_clock().now().nanoseconds*1e-9
        if (not -.1 <= now-stamp <= 1.0 or not np.isfinite([q.x,q.y,q.z,q.w]).all() or
                abs(np.linalg.norm([q.x,q.y,q.z,q.w])-1.) > .05):
            record(self, 'reference_rejected', lambda: dict(reason='stamp_or_quaternion_guard', reference_sim_age=now-stamp))
            return
        try:
            self.reference = trajectory_to_ned([p.x,p.y,p.z], [v.linear.x,v.linear.y,v.linear.z],
                [a.linear.x,a.linear.y,a.linear.z], yaw, v.angular.z, self.origin)
        except ValueError:
            record(self, 'reference_rejected', lambda: dict(reason='pva_conversion_guard'))
            return
        self.reference_wall = time.monotonic(); self.reference_stamp = stamp
        self.observe_input('reference',stamp)
        self.control_timing.event('reference_accept', stamp_ns=message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec)
        record(self, 'reference_accepted', lambda: dict(reference_record(self, message, accepted=True), ned=self.reference))

    def command(self, command, param1=0., param2=0.):
        message = VehicleCommand(); message.timestamp = self.get_clock().now().nanoseconds//1000
        message.command = command; message.param1 = float(param1); message.param2 = float(param2)
        message.target_system = self.instance + 1; message.target_component = 1
        message.source_system = 255; message.source_component = 1; message.from_external = True
        self.command_pub.publish(message)

    def tick(self):
        self.control_timing.event('tick')
        wall = time.monotonic(); now = self.get_clock().now().nanoseconds*1e-9
        jump=self.heartbeat_diagnostics.clock(now,wall)
        if jump and self.origin is not None:
            self.alignment_fault='ros_clock_discontinuity_requires_new_session'
            self.position=None;self.reference=None;self.warmup=None
            self.control_timing.event('clock_discontinuity',**jump)
        flags=self.estimator_flags
        checks=dict(alignment_fault_clear=not self.alignment_fault,failsafe_not_latched=not self.failsafe_latched,
            flags_present=flags is not None,tilt_aligned=bool(flags and flags.cs_tilt_align),
            yaw_aligned=bool(flags and flags.cs_yaw_align),flags_sim_age=-.1<=now-self.flags_time<=2.,
            flags_wall_age=wall-self.flags_wall<3.,
            depth_fusion=not self.require_depth_odometry or depth_fusion_flags_ready(flags,self.depth_fuse_height),
            depth_fresh=not self.require_depth_odometry or self.depth_odometry_fresh(now,wall),
            position_present=self.position is not None,reference_present=self.reference is not None,
            status_present=self.status is not None,px4_failsafe_clear=bool(self.status and not self.status.failsafe),
            position_wall_age=wall-self.position_wall<2.,position_sim_age=-.1<=now-self.last_sample*1e-6<=1.5,
            attitude_present=self.attitude is not None and self.attitude_sample>0,
            attitude_sim_age=-.1<=now-self.attitude_sample*1e-6<=1.5,attitude_wall_age=wall-self.attitude_wall<2.,
            reference_wall_age=wall-self.reference_wall<2.,status_sim_age=-.1<=now-self.status_time<2.,
            status_wall_age=wall-self.status_wall<30.)
        ages=dict(flags_sim_s=now-self.flags_time,flags_wall_s=wall-self.flags_wall,
            position_sim_s=now-self.last_sample*1e-6,position_wall_s=wall-self.position_wall,
            attitude_sim_s=now-self.attitude_sample*1e-6,attitude_wall_s=wall-self.attitude_wall,
            reference_sim_s=now-self.reference_stamp,reference_wall_s=wall-self.reference_wall,
            status_receipt_sim_s=now-self.status_time,status_wall_s=wall-self.status_wall)
        fresh=self.heartbeat_diagnostics.gate(checks,ages,now,wall)
        if not fresh:
            self.warmup = None
            self.reason = (self.alignment_fault or ('awaiting_heading_alignment' if not self.yaw_aligned(now, wall) else
                'px4_failsafe' if self.failsafe_latched or (self.status and self.status.failsafe) else 'estimator_or_reference_timeout'))
            return
        if self.warmup is None: self.warmup = now
        heartbeat = OffboardControlMode(); heartbeat.timestamp = int(now*1e6); heartbeat.position = True
        target = TrajectorySetpoint(); target.timestamp = heartbeat.timestamp
        for key, value in self.reference.items(): setattr(target, key, value)
        self.control_timing.publish(self.setpoint_pub, target, 'setpoint_publish', target.timestamp*1000)
        record(self, 'setpoint_sent', lambda: setpoint_record(self, target))
        self.control_timing.publish(self.offboard_pub, heartbeat, 'heartbeat_publish', heartbeat.timestamp*1000)
        self.heartbeat_count += 1
        self.heartbeat_diagnostics.published(now,wall)
        self.reason = 'offboard' if self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD else 'warming_up'
        if self.auto_arm and now-self.warmup >= 2. and wall-self.last_command >= 1.:
            if self.status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
            elif self.status.arming_state != VehicleStatus.ARMING_STATE_ARMED and self.status.pre_flight_checks_pass:
                self.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.)
            self.last_command = wall

    def report(self):
        self.report_pub.publish(String(data=json.dumps(dict(drone=self.id, px4_instance=self.instance,
            time=self.get_clock().now().nanoseconds*1e-9, reason=self.reason,
            armed=bool(self.status and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED),
            offboard=bool(self.status and self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD),
            preflight=bool(self.status and self.status.pre_flight_checks_pass),
            failsafe=bool(self.status and self.status.failsafe),
            heartbeat_count=self.heartbeat_count, last_ack=self.last_ack,
            heartbeat_diagnostics=self.heartbeat_diagnostics.last_snapshot,
            heading_ready=self.heading_ready, alignment_fault=self.alignment_fault,
            failsafe_latched=self.failsafe_latched,
            attitude_reset_counter=self.attitude_reset,
            awaiting_attitude_bracket=self.pending_position is not None,
            flight_session_fault=self.alignment_fault or self.retirement_reason,
            position_reset_pending=self.position_reset_pending,
            origin_calibration=self.origin_window.reason,
            origin_samples=len(self.origin_window.samples),origin_window=self.origin_window.diagnostics(),
            origin_ground_checks=self.origin_ground_checks,
            depth_odometry_required=self.require_depth_odometry,
            depth_measurement_fresh=self.depth_odometry_fresh(self.get_clock().now().nanoseconds*1e-9,time.monotonic()),
            external_vision_fused=depth_fusion_flags_ready(self.estimator_flags,self.depth_fuse_height),
            external_vision_height_required=self.depth_fuse_height,
            pose_age_s=float(self.get_clock().now().nanoseconds*1e-9-self.last_sample*1e-6),
            flags_age_sim_s=float(self.get_clock().now().nanoseconds*1e-9-self.flags_time),
            flags_age_wall_s=float(time.monotonic()-self.flags_wall),
            origin=self.origin.tolist() if self.origin is not None else None,
            reference_age_wall_s=float(time.monotonic()-self.reference_wall)))))


def main():
    rclpy.init(); node = PX4Bridge()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.control_timing.close()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
