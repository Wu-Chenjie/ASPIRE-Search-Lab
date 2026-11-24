#!/usr/bin/env python3
"""One aircraft's position/yaw executor with peer and odometry watchdogs."""
import json
import os
import queue
import threading
import time
from pathlib import Path
import math
import numpy as np
from aspire_interfaces.topics import view_execution
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Transform, Twist
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float64MultiArray
from core.exploration.decentralized import fused_execution_lease, MapReplica
from core.planning.execution_safety import (
    EXECUTION_DELAY_BUDGET, MAX_MAP_AGE, REACTION_HORIZON, RECHECK_PERIOD,
    TRACKING_MARGIN, TRACKING_BRAKE_TRIGGER, ExecutionCorridor, NativeSafetyMap,
    certified_brake, certified_stopped_reference, certified_terminal_hold,
    reference_step_distance, relaxed_sitl_execution, frontier_sitl_execution, static_reference_jump_limit,
    stopping_reserve, tracking_brake_trigger,
)
from core.planning.continuous_trajectory import ContinuousTrajectory
from core.planning.motion_limits import motion_limits
from core.planning.handoff import validate_handoff
from core.exploration.observation_completion import ObservationCompletion
from .flight_session import FlightSessionFault
from .control_timing import ControlTiming
from .map_evidence import bind_capture,close_capture,capture_safety,evidence_corridor
from .control_observation import record, reference_record, executor_state, executor_odometry
from .static_cancellation import StaticCancellationExecutor
from .guarded_c2 import GuardedC2Executor


def wrap(x):
    return math.atan2(math.sin(x), math.cos(x))


class ViewExecutor(GuardedC2Executor, StaticCancellationExecutor, Node):
    def __init__(self):
        super().__init__('view_executor')
        self.id = int(self.declare_parameter('drone_id', 0).value)
        bind_capture(self,'executor')
        self.motion_limits = motion_limits()
        self.safety_enabled=os.environ.get('RACER_EXECUTION_SAFETY')=='1'
        self.current_static_handoff = True
        session = os.environ.get('RACER_STATIC_CANCEL_SESSION', '')
        if not (0 < len(session) <= 16 and session.isdecimal() and 0 < int(session) < 2**53):
            session = ''
        self._static_cancel_setup(session)
        self._c2_setup(self.static_cancellations is not None and os.environ.get('RACER_STATIC_MOVING_C2') == '1')
        self.collision_stop_enabled=(os.environ.get('ASPIRE_NATIVE_COLLISION_STOP')=='1' and
                                     self.safety_enabled)
        self.collision_stop_request=None; self.collision_stop_receipt=None
        self.collision_stop_refusal=None
        self.actual_speed=np.inf; self.actual_yaw_rate=np.inf
        self.safety_output=self.declare_parameter('execution_safety_dir','').value
        self.safety_certificate_count = 0
        self.safety_certificate_queue = None
        self.safety_certificate_writer = None
        self._last_certificate_replacement = None
        if self.safety_output:
            self.safety_certificate_queue = queue.Queue(maxsize=8)
            self.safety_certificate_stop = threading.Event()
            self.safety_certificate_writer = threading.Thread(
                target=self._write_safety_certificates, name='execution-certificates', daemon=True)
            self.safety_certificate_writer.start()
        self.last_safety_check=-np.inf; self.stopping_reserve_status=None
        self.terminal_hold_since=None; self.terminal_hold_status=None; self.observation_failure=None
        self.observation_failure_count=0
        self.safety_stop_deadline=None; self.safety_hold_valid=True; self.activation_rejected=None
        self.native_safety=NativeSafetyMap(); self.safety_revision=None; self.safety_fault=None
        self.safety_dirty=False; self.safety_stamp=-np.inf; self.safety_stopping=False
        if self.safety_enabled:
            bounds=json.loads(self.declare_parameter('bounds','[]').value)
            limits=json.loads(self.declare_parameter('safety_flight_limits','null').value)
            self.safety_replica=MapReplica(bounds,self.id,volumetric=True,flight_limits=limits)
        self.control_timing = ControlTiming(self.declare_parameter('control_timing_dir', '').value, self.id, 'executor')
        self.flight_fault = FlightSessionFault(self.id)
        self.seeds = json.loads(self.declare_parameter('fleet_starts', '[]').value); self.network_isolated = set()
        self.ref = np.array(json.loads(self.declare_parameter('start', '[]').value), float)
        self.bootstrap_target = self.ref.copy(); self.bootstrap_phase = 0
        # A forward camera mounted above the body cannot see the nearby lower
        # collision envelope from one height. Observe it at a lower hover first,
        # then scan at the requested start height. Both maps use real depth.
        levels=json.loads(self.declare_parameter('bootstrap_altitudes','[]').value)
        self.bootstrap_heights=levels or [max(.7,self.ref[2]-.45),float(self.ref[2])]
        if (not 1<=len(self.bootstrap_heights)<=12 or not np.isfinite(self.bootstrap_heights).all()
                or min(self.bootstrap_heights)<.7 or abs(self.bootstrap_heights[-1]-self.bootstrap_target[2])>1e-6):
            raise ValueError('Invalid declared bootstrap scan heights')
        self.ref[2] = self.bootstrap_heights[0]
        self.heading = 0.; self.actual_yaw = 0.; self.positions = {}; self.stamps = {}; self.peers = {}
        self.curve = None; self.curve_time = 0.; self.velocity = np.zeros(3); self.acceleration = np.zeros(3)
        self.path = None; self.token = None; self.native_plan_id = None; self.index = 0; self.epoch = 0; self.goal_yaw = 0.
        self.ready = False; self.arrived = True; self.scanned = 0.; self.hold_since = None
        self.done = False; self.paused = False; self.last = None; self.waits = 0; self.reason = 'takeoff'
        self.pending = None; self.handoff_count = 0; self.handoff_from_token = None; self.handoff_continuity = None
        self.yaw_rate = 0.
        self.handoff_time = None
        self.observation_completion = ObservationCompletion(self.id)
        self.observation_proof = None; self.observation_grid = None
        q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(PoseStamped, f'/drone_{self.id}/target', 1)
        self.trajectory_pub = self.create_publisher(MultiDOFJointTrajectory, f'/drone_{self.id}/command/trajectory', 1)
        self.report_pub = self.create_publisher(String, view_execution(self.id), q)
        self.create_subscription(String, f'/drone_{self.id}/view_command', self.command, q)
        self.create_subscription(String, '/experiment/control', self.control, q)
        self.create_subscription(String, f'/drone_{self.id}/observation', self.observation, QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        if self.safety_enabled:
            self.create_subscription(Float64MultiArray,f'/baseline/safety_map_{self.id}',self.native_snapshot,10)
        self.create_subscription(String, f'/drone_{self.id}/flight_status', self.flight_status, 1)
        for i in range(len(self.seeds)):
            self.create_subscription(Odometry, f'/drone_{i}/estimated_odometry', lambda m, i=i: self.odom(i, m), qos_profile_sensor_data)
            self.create_subscription(String, f'/drone_{i}/peer_state', lambda m, i=i: self.peer(i, m), q)
        self.create_timer(.02, self.tick); self.create_timer(.2, self.report)

    def destroy_node(self):
        if self.safety_certificate_writer is not None:
            self.safety_certificate_stop.set()
            self.safety_certificate_writer.join(timeout=2.)
        close_capture(self)
        return super().destroy_node()

    def save_safety_certificate(self,kind,original,progress,replacement=None,token=None):
        if self.safety_certificate_queue is None:
            self._last_certificate_replacement = None
            return None
        self.safety_certificate_count += 1
        path = Path(self.safety_output)/f'{self.safety_certificate_count:04}-{kind}.npz'
        m=self.safety_replica.map; n=self.native_safety
        replacement_dict = replacement.to_dict() if replacement else None
        self._last_certificate_replacement = (replacement, replacement_dict) if replacement_dict is not None else None
        # Exact online maps at the decision, not reconstructed sensor replay.
        payload = dict(token=token or self.token or "",decision_time=self.get_clock().now().nanoseconds*1e-9,mapper_state=m.state.copy(),native_state=n.state.copy(),
            mapper_origin=m.origin,mapper_resolution=m.resolution,flight_limits=m.flight_limits,
            clearance=m.clearance,native_origin=n.origin,native_resolution=n.resolution,
            native_stamp=n.stamp,native_sequence=n.sequence,mapper_stamp=self.safety_stamp,
            mapper_version=m.version,tracking_margin=TRACKING_MARGIN,
            actual=self.positions[self.id].copy(),reference=self.ref.copy(),progress=progress,
            terminal_hold=json.dumps(self.terminal_hold_status),observation_failure=json.dumps(self.observation_failure),
            original=json.dumps(original.to_dict()),replacement=json.dumps(replacement_dict) if replacement_dict else '')
        if kind in ('brake', 'terminal_hold', 'observation_abort', 'fault'):
            payload['static_handoff'] = self.current_static_handoff
        try:
            self.safety_certificate_queue.put_nowait((path, payload))
        except queue.Full:
            self.get_logger().warning('Execution certificate queue full', throttle_duration_sec=5.)
            return None
        return str(path)

    def _write_safety_certificates(self):
        pending = self.safety_certificate_queue
        while not self.safety_certificate_stop.is_set() or not pending.empty():
            try:
                path, payload = pending.get(timeout=.1)
            except queue.Empty:
                continue
            try:
                path.parent.mkdir(parents=True, exist_ok=True)
                temporary = path.with_suffix('.tmp')
                with temporary.open('xb') as stream:
                    try:
                        np.savez_compressed(stream, **payload)
                        stream.flush()
                        os.link(temporary, path)
                    finally:
                        temporary.unlink()
            except (OSError, ValueError, TypeError) as error:
                self.get_logger().warning(f'Execution certificate write failed: {error}', throttle_duration_sec=5.)

    def native_snapshot(self,m):
        try:
            received_ns=time.monotonic_ns() if os.environ.get('ASPIRE_SAFETY_MAP_TIMING')=='1' else None
            if self.native_safety.ingest(m.data):
                self.safety_dirty=True
                if received_ns is not None:
                    record(self,'native_map_received',lambda:dict(received_ns=received_ns,
                        native_sequence=self.native_safety.sequence,
                        capture_ns=round(self.native_safety.stamp*1e9)))
        except (TypeError, ValueError, OverflowError) as error:
            record(self, 'native_map_rejected', lambda: dict(reason=str(error)))

    def fail_execution_safety(self,reason):
        if self.safety_fault: return
        self.safety_fault=reason; self.reason=reason; self.safety_hold_valid=False
        self.save_safety_certificate('fault',self.curve,self.curve_time)
        self.pending=None; self.velocity[:]=0.; self.acceleration[:]=0.; self.yaw_rate=0.
        record(self, 'safety_fault', lambda: executor_state(self))

    def _tracking_brake_trigger(self):
        return tracking_brake_trigger() if self.current_static_handoff else TRACKING_BRAKE_TRIGGER

    @capture_safety
    def check_execution_safety(self,t):
        if not self.safety_enabled or self.curve is None or self.safety_fault: return
        if (self.safety_stopping and self.safety_stop_deadline is not None and
                t>self.safety_stop_deadline and self.curve_time<self.curve.duration):
            # The deadline bounds the finite reference stop. `arrived` also
            # requires a later integrated sensor receipt; that proof must stay
            # separate and cannot extend the stop or create a false timeout.
            self.fail_execution_safety('certified_stop_deadline_exceeded'); return
        if not self._odometry_fresh(self.id, t):
            self._defer_execution_check()
            self.reason = 'odometry_timeout'
            return
        started=time.monotonic()
        fresh=self.native_safety.fresh(t) and -.05 <= t-self.safety_stamp <= MAX_MAP_AGE
        tracking_error=float(np.linalg.norm(self.positions[self.id]-self.ref))
        at_terminal = self.curve_time >= self.curve.duration
        if tracking_error>TRACKING_MARGIN:
            if self.arrived or at_terminal:
                self.safety_hold_valid = False
                self.reason = 'tracking_tube_hold'
                return
            if self.safety_stopping:
                self.safety_hold_valid = False
                self.reason = 'stopping_tracking_tube_exceeded'
                return
        relaxed = self.current_static_handoff and relaxed_sitl_execution()
        brake_trigger = tracking_brake_trigger() if relaxed else TRACKING_BRAKE_TRIGGER
        tracking_pressure=(not self.arrived and not self.safety_stopping and
            tracking_error>=brake_trigger)
        cancel_waiting=self._static_cancel_waiting()
        collision_waiting=self.collision_stop_request is not None and self.collision_stop_receipt is None
        collision_brake=collision_waiting and not self.safety_stopping and self.curve_time<self.curve.duration
        review_stop=collision_waiting and self.collision_stop_request.get('purpose')=='review'
        terminal_pending = (not self.arrived or cancel_waiting) and at_terminal
        if terminal_pending and self.terminal_hold_since is None:self.terminal_hold_since=t
        # Elective parking has its own measured-settle window after the
        # finite brake. It cannot extend motion or bypass fixed-root checks.
        settle_s=2. if review_stop else self.observation_completion.max_age_s
        expired=bool(terminal_pending and not self.observation_failure and
                     (not collision_waiting or review_stop) and t>self.terminal_hold_since+settle_s)
        if terminal_pending and t < (self.observation_failure or {}).get('closed_at', self.terminal_hold_since):
            self.fail_execution_safety('terminal_clock_discontinuity');return
        if terminal_pending and not fresh:
            self.safety_hold_valid = False
            return
        if (not collision_brake and not tracking_pressure and not self.safety_dirty and not expired and
                not (terminal_pending and (self.terminal_hold_status is None or not self.safety_hold_valid)) and
                t-self.last_safety_check<RECHECK_PERIOD): return
        self.last_safety_check=t
        if self.safety_dirty:
            self.safety_replica.map.rebuild(); self.safety_dirty=False
        corridor=evidence_corridor(self,ExecutionCorridor(self.safety_replica.map,self.native_safety,
            relaxed_sitl=relaxed and not at_terminal,
            frontier_sitl=relaxed and frontier_sitl_execution()))
        def elapsed(): return time.monotonic()-started
        if terminal_pending:
            # Keep the stationary reference until the next safe authorization.
            # A failed observation closes this token without observation credit.
            try:
                if collision_waiting or cancel_waiting or self.observation_failure or expired:
                    geometry=certified_stopped_reference(self.curve,self.curve_time,self.ref,
                        self.positions[self.id],corridor)
                else:
                    geometry=certified_terminal_hold(self.curve,self.curve_time,self.ref,
                        self.positions[self.id],corridor,t,self.terminal_hold_since,
                        self.observation_completion.max_age_s)
                just_failed = expired and not collision_waiting and not cancel_waiting
                failure = self.observation_failure
                if just_failed:
                    failure = dict(token=self.token, epoch=self.epoch,
                        outcome='unobserved', reason='terminal_observation_wait_deadline_exceeded',
                        closed_at=t, started_at=self.terminal_hold_since,
                        observation_deadline=self.terminal_hold_since+self.observation_completion.max_age_s,
                        mission_failure_count=self.observation_failure_count+1)
                if cancel_waiting:
                    state=dict(geometry,task_state='cancelling')
                elif collision_waiting:
                    state=dict(geometry,task_state='collision_stop')
                elif failure:
                    state=dict(geometry,started_at=self.terminal_hold_since,
                        absolute_deadline=failure['observation_deadline'],
                        wait_budget_s=self.observation_completion.max_age_s,task_state='failed')
                else:
                    state=dict(geometry,task_state='pending')
                state.update(tracking_pressure=bool(tracking_pressure),tracking_error_m=tracking_error,checked_at=t)
                delay=elapsed()
                native_age = t - self.native_safety.stamp
                mapper_age = t - self.safety_stamp
                if native_age > MAX_MAP_AGE or mapper_age > MAX_MAP_AGE:
                    self.safety_hold_valid = False
                    return
                state.update(check_wall_s=delay, native_input_age_s=native_age, mapper_input_age_s=mapper_age)
                first=self.terminal_hold_status is None
                self.terminal_hold_status=state;self.safety_hold_valid=True
                if just_failed:
                    self.observation_failure_count += 1
                    self.observation_failure = failure
                    self.hold_since = None; self.observation_proof = None; self.pending = None
                if first or just_failed:self.save_safety_certificate('observation_abort' if just_failed else 'terminal_hold',self.curve,self.curve_time)
                if cancel_waiting:self._static_cancel_retire(t,geometry)
                if (collision_waiting and t-self.terminal_hold_since>=.15 and
                        t-self.stamps[self.id]>=0. and tracking_error<=.15 and
                        self.actual_speed<=.05 and self.actual_yaw_rate<=.20 and
                        abs(wrap(self.goal_yaw-self.actual_yaw))<=.20):
                    self.collision_stop_receipt=dict(self.collision_stop_request,**geometry,checked_at=t,
                        odometry_stamp=self.stamps[self.id],actual_speed_m_s=self.actual_speed,
                        actual_yaw_rate_rad_s=self.actual_yaw_rate,tracking_error_m=tracking_error)
                    self.arrived=True
                    record(self,'native_collision_stop_rest',lambda:self.collision_stop_receipt)
                elif review_stop and expired:
                    self.decline_review_stop(self.collision_stop_request, 'review_stop_settle_window_expired')
                if just_failed:record(self,'observation_aborted',lambda:dict(executor_state(self),failure=self.observation_failure))
            except ValueError as error:
                if str(error) in ('terminal_root_no_longer_certified', 'tracking_tube_exceeded'):
                    self.safety_hold_valid = False
                    self.reason = str(error)
                else:
                    self.fail_execution_safety(str(error))
            return
        if self.arrived:
            root_safe = fresh and corridor.box_safe(self.ref, self.ref)
            self.safety_hold_valid = root_safe
            if fresh and not root_safe:
                self.reason = 'arrived_root_needs_replan'
            elif not self.safety_hold_valid:
                self.reason = 'hold_waiting_fresh_safety_map'
            return
        remaining_safe=corridor.safe_curve(self.curve,self.curve_time)
        if self.safety_stopping and remaining_safe:
            # Keep the ORIGINAL finite stop on stale or unchanged maps. A new
            # snapshot cannot restart its clock or indefinitely extend a brake.
            self.safety_hold_valid = True
            return
        reason='remaining_corridor_changed' if fresh else 'measured_map_stale'
        elective_stop=None
        if collision_brake and self.collision_stop_request.get('purpose')=='review':
            strict=ExecutionCorridor(self.safety_replica.map,self.native_safety)
            bearing=self.collision_stop_request.get('view_yaw')
            refusal = None
            if not fresh:
                refusal = 'elective_stop_requires_fresh_maps'
            elif not remaining_safe or tracking_pressure:
                refusal = 'elective_stop_preempted_by_safety_brake'
            else:
                try:
                    elective_stop=certified_brake(self.curve,self.curve_time,corridor,
                        accept_endpoint=lambda p,duration:duration<=1.5 and strict.box_safe(p,p),
                        measured_bearing=bearing)
                except ValueError as error:
                    refusal = str(error)
                if elapsed() > EXECUTION_DELAY_BUDGET:
                    self._decline_delayed_review_stop(t, corridor, started)
                    return
                if t-self.native_safety.stamp > MAX_MAP_AGE or t-self.safety_stamp > MAX_MAP_AGE:
                    refusal = 'elective_stop_requires_fresh_maps'
                    fresh = False
                    reason = 'measured_map_stale'
            if refusal is not None:
                self.decline_review_stop(self.collision_stop_request, refusal)
                elective_stop = None
                collision_brake = False
        if elective_stop is not None:
            reason='fresh_review_requested_stop'
        elif collision_brake:
            reason='native_collision_alert'
        elif fresh and remaining_safe and tracking_pressure:
            reason='tracking_step_headroom'
        if fresh and remaining_safe and not tracking_pressure and not collision_brake:
            delay=elapsed()
            native_age = t - self.native_safety.stamp
            mapper_age = t - self.safety_stamp
            if native_age <= MAX_MAP_AGE and mapper_age <= MAX_MAP_AGE:
                self.safety_hold_valid = True
                self.stopping_reserve_status=dict(checked_at=t,reference_progress=self.curve_time,
                    reaction_horizon_s=REACTION_HORIZON,brake_boundary=min(self.curve.duration, self.curve_time+REACTION_HORIZON),
                    stopped_at_progress=self.curve.duration,native_input_age_s=native_age,
                    mapper_input_age_s=mapper_age,check_sim_s=None,
                    check_wall_s=delay,check_delay_bound_s=delay,
                    check_clock='monotonic_wall_upper_bound_at_rtf_le_1',
                    tracking_error_m=tracking_error,tracking_margin_m=TRACKING_MARGIN)
                return
            reason='stopping_reserve_unavailable: Snapshot expired during safety computation'
        try:
            if self.curve_time >= self.curve.duration: raise ValueError('Unsafe terminal hold')
            stopped=elective_stop if elective_stop is not None else certified_brake(self.curve,self.curve_time,corridor)
            deadline=t+stopped.duration-self.curve_time+EXECUTION_DELAY_BUDGET
            if self.safety_stop_deadline is not None and deadline>self.safety_stop_deadline+1e-9:
                raise ValueError('New stop would extend original absolute deadline')
        except ValueError as error:
            self.fail_execution_safety(reason+': '+str(error)); return
        certificate=self.save_safety_certificate('brake',self.curve,self.curve_time,stopped)
        cached = getattr(self, '_last_certificate_replacement', None)
        self._last_certificate_replacement = None
        if elective_stop is not None and elapsed() > EXECUTION_DELAY_BUDGET:
            self._decline_delayed_review_stop(t, corridor, started)
            return
        stopped_path = stopped.path()
        stopped_yaw = stopped.sample(stopped.duration)[3]
        stopped_trajectory = cached[1] if cached is not None and cached[0] is stopped else stopped.to_dict()
        self.curve=stopped; self.path=stopped_path; self.goal_yaw=stopped_yaw
        self.safety_hold_valid = True
        self.terminal_hold_since=None; self.terminal_hold_status=None; self.observation_failure=None
        self.pending=None; self.hold_since=None; self.observation_proof=None; self.safety_stopping=True
        self.safety_stop_deadline=deadline
        self.safety_revision=dict(number=(self.safety_revision or {}).get('number',0)+1,
            time=t,reason=reason,certificate=certificate,boundary=self.curve_time,duration=stopped.duration,
            absolute_stop_deadline=deadline,check_wall_s=elapsed(),
            native_stamp=self.native_safety.stamp,native_sequence=self.native_safety.sequence,
            mapper_stamp=self.safety_stamp,mapper_version=self.safety_replica.map.version,
            path=self.path.tolist(),trajectory=stopped_trajectory)
        if elective_stop is not None and bearing is not None:
            self.safety_revision['review_bearing_yaw_rad']=bearing
        record(self, 'brake_applied', lambda: dict(executor_state(self), boundary=self.safety_revision['boundary']))

    def _hold_after_delayed_check(self, now, corridor, started):
        # Recheck the actual fixed reference within both measured input ages.
        if not corridor.box_safe(self.ref, self.ref):
            return False
        delay = time.monotonic() - started
        if (now - self.native_safety.stamp + delay > MAX_MAP_AGE
                or now - self.safety_stamp + delay > MAX_MAP_AGE
                or not 0 <= now - self.stamps.get(self.id, -np.inf) + delay <= .5):
            return False
        self._defer_execution_check()
        self.reason = 'safety_check_delay_retry'
        return True

    def _defer_execution_check(self):
        self.safety_hold_valid = False
        self.safety_dirty = True
        self.stopping_reserve_status = None

    def reject_activation(self,packet,reason):
        self.reason=reason
        self.activation_rejected=dict(token=packet['token'],reason=reason,
            time=self.get_clock().now().nanoseconds*1e-9)
        record(self, 'activation_rejected', lambda: dict(executor_state(self), rejected=self.activation_rejected))
        self.report()

    def _decline_delayed_review_stop(self, now, corridor, started):
        self.decline_review_stop(self.collision_stop_request, 'elective_stop_delay_budget_exceeded')
        if not self._hold_after_delayed_check(now, corridor, started):
            self.fail_execution_safety('safety_check_delay_budget_exceeded')

    def decline_review_stop(self,request,reason):
        self.collision_stop_request=None
        self.collision_stop_refusal=dict(request,reason=reason)
        record(self,'review_stop_declined',lambda:self.collision_stop_refusal)

    def peer(self, i, m):
        if i != self.id and (self.id in self.network_isolated or i in self.network_isolated):
            return
        try:
            packet = json.loads(m.data)
        except (TypeError, ValueError):
            return
        if not isinstance(packet, dict) or type(packet.get('time')) not in (int, float):
            return
        self.peers[i] = packet

    def _odometry_fresh(self, drone, now):
        return -.1 <= now - self.stamps.get(drone, -np.inf) <= .5

    def odom(self, i, m):
        if self.flight_fault.reason or not 0 <= i < len(self.seeds):
            return
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        now = self.get_clock().now().nanoseconds * 1e-9
        previous_stamp = self.stamps.get(i, -np.inf)
        if stamp < previous_stamp or not -.1 <= now - stamp <= .5:
            return
        p = m.pose.pose.position
        position = np.array([p.x, p.y, p.z])
        if not np.isfinite(position).all():
            return
        self.positions[i] = position
        self.stamps[i] = stamp
        if i == self.id:
            q = m.pose.pose.orientation
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            if self.collision_stop_enabled:
                twist=m.twist.twist
                self.actual_speed=math.hypot(twist.linear.x,twist.linear.y,twist.linear.z)
                # PX4 odometry publishes angular pose and linear velocity.
                dt=self.stamps[i]-previous_stamp
                self.actual_yaw_rate=abs(wrap(yaw-self.actual_yaw))/dt if 0.<dt<=.5 else np.inf
            self.actual_yaw=yaw
            record(self, 'odometry_used', lambda: executor_odometry(self, m))

    def stop(self, reason='explicit_cancel'):
        self.pending = None; self.token = None; self.native_plan_id = None
        self.reason = reason
        self.path = None; self.curve = None; self.velocity[:] = 0.; self.acceleration[:] = 0.; self.yaw_rate = 0.; self.arrived = True
        self.hold_since = None; self.observation_proof = None
        self.collision_stop_request=None; self.collision_stop_receipt=None
        self.collision_stop_refusal=None
        self.terminal_hold_since=None; self.terminal_hold_status=None; self.observation_failure=None
        self.safety_hold_valid = not self.safety_fault and not self.flight_fault.reason
        if self.id in self.positions:
            self.ref = self.positions[self.id].copy()

    def flight_status(self, m):
        try: packet=json.loads(m.data)
        except (ValueError,TypeError): return
        if self.flight_fault.observe(packet,self.get_clock().now().nanoseconds*1e-9):
            self.stop(self.flight_fault.reason)
            self.ready=False; self.arrived=False
            self.report()

    def command(self, m):
        if self.flight_fault.reason or self.safety_fault: return
        command_started=time.monotonic()
        try:
            p = json.loads(m.data)
        except (TypeError, ValueError):
            return
        if not isinstance(p, dict) or not self._static_command_session_valid(p):
            return
        if self._static_cancel_command(p):return
        if 'native_collision_stop' in p:
            request=p['native_collision_stop']
            if (self.collision_stop_enabled and self.collision_stop_request is None and
                    self.curve is not None and self.static_task is None and not self.arrived and
                    not self.observation_failure and isinstance(request,dict) and
                    request.get('token')==self.token and request.get('epoch')==self.epoch and
                    request.get('native_id')==self.native_plan_id and
                    ('view_yaw' not in request or (request.get('purpose')=='review' and
                        type(request['view_yaw']) in (int,float) and abs(request['view_yaw'])<=math.pi)) and
                    type(request.get('requested_at')) in (int,float) and
                    0.<=request['requested_at']<=self.get_clock().now().nanoseconds*1e-9):
                if request.get('purpose')=='review' and (self.safety_stopping or self.curve_time>=self.curve.duration):
                    self.decline_review_stop(request,'stop_already_in_progress')
                elif not (self.collision_stop_refusal and self.collision_stop_refusal['requested_at']==request['requested_at']):
                    self.collision_stop_request=request
                    self.pending=None; self.hold_since=None; self.observation_proof=None
                    record(self,'native_collision_stop_received',lambda:request)
            return
        record(self, 'command_received', lambda: dict(executor_state(self), command_token=p.get('token'), command_native_id=p.get('native_id'), command_epoch=p.get('epoch'), command_yaw=p.get('yaw'), command_root=p.get('path',[None])[0] if p.get('path') else None))
        if p.get('cancel_pending'):
            if self.pending and self.pending['token'] == p['cancel_pending']: self.pending = None
            return
        if type(p.get('epoch')) is not int or p['epoch'] <= self.epoch:
            return
        if not p.get('cancel') and (not isinstance(p.get('token'), str) or not p['token']):
            return
        if not self._static_task_admit(p):
            self.reject_activation(p,'static_task_cancelled_or_invalid');return
        if p.get('cancel'):
            initial_takeoff = not self.ready and self.epoch == 0
            takeoff_reference = self.ref.copy()
            self.epoch = p['epoch']; self.stop(p.get('reason','explicit_cancel'))
            if initial_takeoff:self.ref = takeoff_reference
            if p.get('scan'):
                if self.ready:self.bootstrap_phase = len(self.bootstrap_heights)-1
                self.ready = False; self.scanned = 0.
            return
        if self.collision_stop_request is not None and not self.arrived:
            self.reject_activation(p,'native_collision_stop_in_progress');return
        try:
            path = np.asarray(p['path'], float)
            yaw = float(p['yaw'])
        except (KeyError, TypeError, ValueError, OverflowError):
            return
        if path.ndim != 2 or path.shape[1] != 3 or not len(path) or not np.isfinite(path).all() or not np.isfinite(yaw):
            return
        p['yaw'] = yaw
        if self.id not in self.positions:
            return
        try:
            curve = ContinuousTrajectory.from_dict(p['trajectory']) if p.get('trajectory') else None
        except (ValueError, KeyError, TypeError):
            return
        if self.safety_enabled and p.get('handoff'):
            if self.moving_c2_enabled:
                self._c2_admit(p,curve,command_started);return
            self.reject_activation(p,'execution_safety_requires_static_handoff'); return
        if p.get('handoff'):
            boundary = p['handoff']
            if self.curve is None or curve is None or self.arrived or self.paused or self.done:
                return
            if boundary.get('from_epoch') != self.epoch or boundary.get('from_token') != self.token:
                return
            try: errors = validate_handoff(self.curve, curve, boundary['trajectory_time'], self.curve_time)
            except (ValueError, KeyError): return
            self.pending = dict(p, curve=curve, errors=errors)
            return
        root_error = np.linalg.norm(path[0]-self.positions[self.id])
        if root_error > .6:
            return
        self.pending = None
        if self.safety_enabled:
            t=self.get_clock().now().nanoseconds*1e-9
            if not -.1 <= t-self.stamps.get(self.id, -np.inf) <= .5:
                self.reject_activation(p, 'activation_odometry_stale'); return
            if root_error>TRACKING_MARGIN or np.linalg.norm(path[0]-self.ref)>static_reference_jump_limit():
                self.reject_activation(p,'activation_root_outside_tracking_tube'); return
            if self.safety_dirty:
                self.safety_replica.map.rebuild(); self.safety_dirty=False
            corridor=ExecutionCorridor(self.safety_replica.map,self.native_safety,
                relaxed_sitl=relaxed_sitl_execution(), frontier_sitl=frontier_sitl_execution())
            detail = None
            if curve is None:
                detail = 'missing_curve'
            elif not self.native_safety.fresh(t):
                detail = 'native_map_age'
            elif not 0 <= t-self.safety_stamp <= MAX_MAP_AGE:
                detail = 'mapper_age'
            elif not corridor.safe_curve(curve):
                detail = 'continuous_geometry'
            if detail:
                reason = 'latest_execution_corridor_rejected'
                if os.environ.get('ASPIRE_FRONTIER_STEPS') == '1':
                    reason += ':' + detail
                self.reject_activation(p, reason)
                return
            self.save_safety_certificate('activation',curve,0.,token=p['token'])
        self.terminal_hold_since=None; self.terminal_hold_status=None; self.observation_failure=None
        self.collision_stop_request=None; self.collision_stop_receipt=None
        self.collision_stop_refusal=None
        self.safety_revision=None; self.safety_stopping=False; self.safety_stop_deadline=None
        self.safety_hold_valid=True; self.activation_rejected=None; self.last_safety_check=-np.inf
        self.current_static_handoff = True
        self.curve = curve; self.curve_time = 0.
        self.epoch = p['epoch']; self.path = path; self.index = 0; self.arrived = False
        self.goal_yaw = p['yaw']; self.token = p['token']; self.native_plan_id = p.get('native_id'); self.hold_since = None
        self.observation_grid = p.get('observation_grid'); self.observation_proof = None
        self._static_task_activated(p)
        record(self, 'activation_applied', lambda: dict(executor_state(self), new_curve_root=list(self.curve.sample(0.)) if self.curve else None))

    def observation(self, m):
        if self.flight_fault.reason: return
        try:
            packet = json.loads(m.data)
        except (ValueError, TypeError):
            return
        if (not isinstance(packet, dict) or packet.get('source') != self.id
                or packet.get('sensor') != 'gz_depth' or packet.get('integration_completed') is not True):
            return
        if self.safety_enabled:
            try:
                self.safety_dirty = self.safety_replica.merge(packet) or self.safety_dirty
                self.safety_stamp = self.safety_replica.sensor_stamps.get(self.id, -np.inf)
            except (KeyError, TypeError, ValueError, OverflowError) as error:
                record(self, 'mapper_packet_rejected', lambda: dict(reason=str(error)))
                return
        self.observation_completion.ingest(packet, self.get_clock().now().nanoseconds*1e-9, self.observation_grid)

    def control(self, m):
        try:
            packet = json.loads(m.data)
            if not isinstance(packet, dict):
                return
            isolated = set(packet.get('isolated', []))
            paused = self.id in packet.get('paused', [])
        except (TypeError, ValueError):
            return
        self.network_isolated = isolated
        self.done = packet.get('done') is True
        self.paused = paused
        if self.done or self.paused:
            self.stop('experiment_complete' if self.done else 'experiment_pause')

    def report(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        fresh_odometry = self._odometry_fresh(self.id, now)
        tracking_error=float(np.linalg.norm(self.positions.get(self.id,self.ref)-self.ref))
        valid = fresh_odometry and not self.flight_fault.reason and not self.safety_fault and self.safety_hold_valid
        self.report_pub.publish(String(data=json.dumps(dict(drone=self.id,time=now,
            ready=self.ready and valid, arrived=self.arrived and valid,
            safety_fault=self.safety_fault,safety_revision=self.safety_revision,stopping_reserve=self.stopping_reserve_status,
            static_cancel_ack=self.static_cancel_ack,
            native_collision_stop_receipt=self.collision_stop_receipt,
            native_collision_stop_refusal=self.collision_stop_refusal,
            tracking_brake_trigger_m=self._tracking_brake_trigger(),terminal_hold=self.terminal_hold_status,observation_failure=self.observation_failure,
            safety_stop_deadline=self.safety_stop_deadline,activation_rejected=self.activation_rejected,
            flight_session_fault=self.flight_fault.reason,
            epoch=self.epoch, reason=self.reason, traffic_wait_samples=self.waits, yaw_error=wrap(self.goal_yaw-self.actual_yaw),
            token=self.token, native_plan_id=self.native_plan_id,
            actual_position=self.positions.get(self.id,self.ref).tolist(),position_stamp=self.stamps.get(self.id), trajectory_duration=self.curve.duration if self.curve else 0., reference_yaw=self.heading,
            reference_yaw_rate=self.yaw_rate, pending_token=self.pending['token'] if self.pending else None,
            handoff_count=self.handoff_count, handoff_from_token=self.handoff_from_token, handoff_continuity=self.handoff_continuity,
            handoff_time=self.handoff_time,
            reference=self.ref.tolist(), reference_velocity=self.velocity.tolist(), reference_acceleration=self.acceleration.tolist(),
            tracking_error=tracking_error,
            observation_dwell=bool(self.hold_since is not None and not self.arrived),
            observation_proof=self.observation_proof,
            observation_wait_reason=self.observation_completion.wait_reason if self.hold_since is not None and not self.arrived else None,
            observation_rejected_reason=self.observation_completion.rejected_reason,
            trajectory_method=self.curve.method if self.curve else None, trajectory_time=self.curve_time))))

    def tick(self):
        self.control_timing.event('tick')
        if self.flight_fault.reason: return
        now = self.get_clock().now(); t = now.nanoseconds*1e-9
        self.velocity[:] = 0.; self.acceleration[:] = 0.; self.yaw_rate = 0.
        record(self, 'tick_boundary', lambda: executor_state(self))
        dt = 0 if self.last is None else min(max(t-self.last, 0), .04); self.last = t
        if self.id not in self.positions:
            return
        safety_cycle_started=time.monotonic()
        self.check_execution_safety(t)
        error = np.linalg.norm(self.positions[self.id]-self.ref)
        stale = not all(self._odometry_fresh(i, t) for i in range(len(self.seeds)))
        if not self.ready and error < .15 and not stale and not self.done and not self.paused:
            # Physical yaw scan, actual finite-FOV observations populate the map.
            if abs(wrap(self.heading-self.actual_yaw)) < .2:
                delta = min(.65*dt, 2*math.pi-self.scanned)
                self.heading = wrap(self.heading+delta); self.scanned += delta
            self.reason = 'initial_yaw_scan'
            if self.scanned >= 2*math.pi-1e-6 and abs(wrap(self.heading-self.actual_yaw)) < .08:
                if self.bootstrap_phase+1 < len(self.bootstrap_heights):
                    self.bootstrap_phase += 1; self.scanned = 0.
                    self.ref[2] = self.bootstrap_heights[self.bootstrap_phase]
                    self.reason = 'initial_scan_ascent'
                else:
                    self.ready = True
        elif self.ready and not self.done and not self.paused and not self.safety_fault and self.safety_hold_valid:
            single = len(self.seeds) == 1
            peer_stale = False if single else (len(self.peers) != len(self.seeds) or any(t-p['time'] > 3. for p in self.peers.values()))
            self.reason = ('odometry_timeout' if stale else 'peer_timeout' if peer_stale else
                           'view_observed' if self.arrived and self.observation_proof else 'idle')
            lease_valid = True if single else fused_execution_lease(self.peers, self.id, self.token, t, self.seeds)
            if self.path is not None and not self.arrived and not lease_valid:
                self.reason = 'lease_confirmation_hold'
            if self.path is not None and not self.arrived and not self.observation_failure and not stale and lease_valid:
                r = self.ref.copy(); index = self.index
                advance = dt if error < .25 else 0.
                if self.curve:
                    next_time = min(self.curve.duration, self.curve_time+advance)
                    r, velocity, acceleration, heading, rate = self.curve.sample(next_time)
                    index = len(self.path) if next_time >= self.curve.duration else 0
                else:
                    next_time = self.curve_time
                    heading = wrap(self.heading+np.clip(wrap(self.goal_yaw-self.heading), -self.motion_limits.yaw_rate*dt, self.motion_limits.yaw_rate*dt))
                    velocity = np.zeros(3); acceleration = np.zeros(3)
                    rate = 0.
                    remaining = self.motion_limits.speed*advance
                    while index < len(self.path) and remaining > 0:
                        d = self.path[index]-r; norm = np.linalg.norm(d)
                        if norm <= remaining:
                            r = self.path[index].copy(); index += 1; remaining -= norm
                        else:
                            r += d/norm*remaining; remaining = 0
                conflict = any(np.linalg.norm(r[:2]-p[:2]) < 1.05 for i, p in self.positions.items() if i != self.id)
                if conflict:
                    self.waits += 1; self.reason = 'separation_hold'
                else:
                    switch_ready=(self.pending and self.curve and next_time >= self.pending['handoff']['trajectory_time'] and
                            self.curve_time <= self.pending['handoff']['trajectory_time'] and
                            error < .1 and fused_execution_lease(self.peers, self.id, self.pending['token'], t, self.seeds))
                    if switch_ready and self.safety_enabled:
                        switch_ready=self._c2_transition(self.pending,next_time,t,safety_cycle_started)
                        if not self.safety_hold_valid:
                            # This tick no longer has a timely proof to advance
                            # either curve. Keep the last reference and lease.
                            self._publish_reference(now, 0.)
                            return
                    if switch_ready:
                        new_time = next_time-self.pending['handoff']['trajectory_time']
                        self.handoff_from_token = self.token; self.handoff_continuity = self.pending['errors']; self.handoff_count += 1
                        self.handoff_time=t
                        self.curve = self.pending['curve']; self.path = np.asarray(self.pending['path'], float)
                        self.token = self.pending['token']; self.native_plan_id = self.pending.get('native_id'); self.epoch = self.pending['epoch']; self.goal_yaw = self.pending['yaw']
                        if self.safety_enabled:self._c2_applied(self.pending,t)
                        self.observation_grid = self.pending.get('observation_grid'); self.observation_proof = None
                        r, velocity, acceleration, heading, rate = self.curve.sample(new_time)
                        next_time = new_time; index = 0; self.hold_since = None; self.pending = None
                    elif (self.safety_enabled and self.pending
                          and next_time > self.pending['handoff']['trajectory_time']):
                        expired = self.pending
                        self.pending = None
                        self._c2_reject_candidate(expired, 'handoff_boundary_already_consumed')
                    if self.safety_enabled and np.linalg.norm(r-self.positions[self.id])>TRACKING_MARGIN:
                        self.reason = 'tracking_error_hold'
                        self._publish_reference(now, 0.)
                        return
                    self.ref = r; self.index = index; self.heading = heading; self.curve_time = next_time
                    if advance > 0:
                        self.velocity = velocity; self.acceleration = acceleration
                        self.yaw_rate = rate
                    self.reason = 'tracking_view' if advance>0 else 'tracking_error_hold'
                    if index>=len(self.path): self.reason='observation_dwell'
                if (self.collision_stop_request is None and not self._static_cancel_waiting() and
                        not conflict and index >= len(self.path)):
                    if error < .15 and abs(wrap(self.goal_yaw-self.actual_yaw)) < .20:
                        if self.hold_since is None:
                            self.hold_since = t
                        proof = self.observation_completion.proof(t, self.hold_since, self.path[-1], self.goal_yaw, self.observation_grid)
                        if proof is not None:
                            self.observation_proof = dict(proof, token=self.token, native_plan_id=self.native_plan_id, epoch=self.epoch, completed_at=t)
                            self.arrived = True; self.reason = 'view_observed'
                    elif error > TRACKING_MARGIN:
                        self.hold_since = None
                else:
                    self.hold_since = None
            elif not self.arrived:
                self.hold_since = None
                if self.observation_failure:self.reason='observation_failed_safe_hold'
        self._publish_reference(now, dt if self.safety_hold_valid else 0.)

    def _publish_reference(self, now, dt):
        m = PoseStamped(); m.header.frame_id = 'world'; m.header.stamp = now.to_msg()
        m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, self.ref)
        m.pose.orientation.w = math.cos(self.heading/2); m.pose.orientation.z = math.sin(self.heading/2)
        self.control_timing.publish(self.pub, m, 'target_publish', now.nanoseconds)
        msg = MultiDOFJointTrajectory(); msg.header = m.header; msg.joint_names = ['base_link']
        point = MultiDOFJointTrajectoryPoint(); transform = Transform()
        transform.translation.x, transform.translation.y, transform.translation.z = map(float, self.ref)
        transform.rotation = m.pose.orientation
        velocity = Twist(); acceleration = Twist()
        velocity.linear.x, velocity.linear.y, velocity.linear.z = map(float, self.velocity)
        acceleration.linear.x, acceleration.linear.y, acceleration.linear.z = map(float, self.acceleration)
        velocity.angular.z = float(self.yaw_rate)
        point.transforms = [transform]; point.velocities = [velocity]; point.accelerations = [acceleration]
        msg.points = [point]
        self.control_timing.publish(self.trajectory_pub, msg, 'reference_publish', now.nanoseconds)
        record(self, 'reference_sent', lambda: dict(reference_record(self, msg), executor=executor_state(self), applied_dt=dt))


def main():
    rclpy.init(); node = ViewExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.control_timing.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
