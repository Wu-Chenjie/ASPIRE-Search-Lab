#!/usr/bin/env python3
"""Measured depth/odometry and unanimous leases for native RACER proposals.

The native planner owns exploration decisions and swarm allocation. This adapter
validates its exact curves and runs the existing authorized player. The DDS
backend runs the migrated rclcpp core; TCP retains the historical ROS1 baseline.
"""
import os
import argparse
from collections import deque
import json
import math
from pathlib import Path
import queue
import signal
import time
import uuid
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image,CameraInfo
from std_msgs.msg import String, Float64MultiArray
from aspire_interfaces.topics import camera_topic, view_execution
from aspire_camera.depth_projection import project_depth_message,SENSOR_TRANSLATION
from aspire_camera.poses import PoseHistory
from annual_runtime.flight_session import FlightSessionFault
from core.exploration.decentralized import MapReplica,PeerLedger,remainder
from core.exploration.mrdtg import observation_grid
from core.exploration.observation_completion import completed_execution_proof, failed_observation_proof
from core.exploration.regions import visible_cells
from core.exploration.team_evidence import intent_records
from core.planning.bspline_trajectory import native_trajectory,safe_native_execution
from core.planning.handoff import validate_handoff, curve_fingerprint, validate_safe_handoff, check_handoff_budget
from core.planning.execution_safety import (
    NativeSafetyMap, ExecutionCorridor, stopping_reserve, MAX_MAP_AGE, REACTION_HORIZON,
    TRACKING_MARGIN, relaxed_sitl_execution, frontier_sitl_execution, static_reference_jump_limit,
    stopped_planning_reference,
)
from core.planning.motion_limits import motion_environment, motion_limits
from core.exploration.voxel_config import voxel_environment


from annual_runtime.static_cancel_gateway import StaticCancelGateway, task_wire


def fresh_static_source_frame(frame, now):
    return isinstance(frame,dict) and 0.<=now-frame.get('capture_s',-math.inf)<=.75


class NativeLeaseAgent(StaticCancelGateway, Node):
    def __init__(self,drone,config,transport,output):
        super().__init__('racer_authorization_'+str(drone))
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
        self.id=drone;self.config=config;self.transport=transport;self.session=uuid.uuid4().hex
        self.seeds=config['fleet_starts']
        self.replica=MapReplica(config['bounds'],drone,volumetric=True,
                                flight_limits=config.get('racer_flight_limits'))
        self.safety_enabled=os.environ.get("RACER_EXECUTION_SAFETY")=="1"
        if self.safety_enabled and len(self.seeds) != 1:
            raise ValueError('Execution safety requires a single aircraft')
        session = os.environ.get('RACER_STATIC_CANCEL_SESSION', '')
        self.authorization_session = session if (
            0 < len(session) <= 16 and session.isdecimal() and 0 < int(session) < 2**53) else ''
        from tools.racer_ros2.configuration import static_feature_options
        self.config = config = static_feature_options(dict(
            config, execution_safety=self.safety_enabled, static_cancel_session=self.authorization_session,
            static_moving_c2=config.get('static_moving_c2') is True and os.environ.get('RACER_STATIC_MOVING_C2') == '1'))
        self.use_stopped_reference = (os.environ.get('ASPIRE_FRONTIER_STEPS') == '1'
            and self.safety_enabled and config.get('stop_and_plan')
            and not (config.get('active_review') or config.get('static_event_replan')))
        self._static_gateway_setup(config['static_cancel_session'])
        self.static_moving_c2 = config['static_moving_c2']
        self.native_safety = NativeSafetyMap()
        self.collision_stop_enabled=(os.environ.get('ASPIRE_NATIVE_COLLISION_STOP')=='1' and
                                     self.safety_enabled)
        self.fresh_static_signing=bool(self.collision_stop_enabled and os.environ.get('ASPIRE_STATIC_FRESH_SIGNING')=='1')
        self.fast_stop_feedback=bool(self.collision_stop_enabled and os.environ.get('ASPIRE_FAST_STOP_HANDOFF')=='1')
        self.collision_stop_released=None
        self.stop_release_wait=None
        self.ledger=PeerLedger(drone,tuple(range(len(self.seeds))));self.ledger.seeds=self.seeds
        self.pose=None;self.yaw=0.;self.execution={};self.poses=PoseHistory();self.calibration=None
        self.images=deque(maxlen=8);self.last_depth=-10.;self.last_odom=-10.;self.hello=False
        self.intent=None;self.pending=None;self.curves={};self.completed=set();self.failed_observations={};self.sequence=0;self.epoch=0
        self.last_rejection=-1;self.last_sensor=-10.;self.last_grid=-10.;self.measured=np.zeros(self.replica.map.state.size,bool)
        self.flight_fault=FlightSessionFault(drone);self.acks=[];self.dirty=False
        self.proposals=queue.Queue()
        self.folder=output/str(drone);self.diagnostics=0;self.reported_safety_fault=False;self.last_completed_id=-1
        self.log = None
        try:
            self.folder.mkdir(parents=True, exist_ok=True)
            self.log = (self.folder/'events.jsonl').open('x', buffering=1)
        except OSError as error:
            self.get_logger().warning(f'Authorization evidence unavailable: {error}')
        q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub=self.create_publisher(String,f'/drone_{drone}/peer_state',q)
        self.command_pub=self.create_publisher(String,f'/drone_{drone}/view_command',q)
        if self.static_cancel_session:
            self.static_cancel_pub=self.create_publisher(Float64MultiArray,f'/baseline/static_cancel_ack_{drone}',10)
            self.create_subscription(Float64MultiArray,f'/baseline/static_cancel_{drone}',self.static_cancel_request,10)
        if self.collision_stop_enabled:
            self.create_subscription(Float64MultiArray,f'/baseline/collision_stop_{drone}',self.collision_stop_request,10)
        self.bootstrap=self.create_publisher(String,f'/drone_{drone}/map_bootstrap',q)
        self.bootstrap.publish(String(data='{}'))
        self.create_subscription(Odometry,f'/drone_{drone}/estimated_odometry',self.odom,qos_profile_sensor_data)
        self.create_subscription(CameraInfo,camera_topic(drone, 'depth', 'camera_info'),self.camera,qos_profile_sensor_data)
        if config.get('native_frame_source') not in ('mapper_observation','mapper_dds'):
            self.create_subscription(Image,camera_topic(drone, 'depth', 'image'),self.image,qos_profile_sensor_data)
        self.create_subscription(String,f'/drone_{drone}/observation',self.observation,QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String,view_execution(drone),self.executed,q)
        self.create_subscription(String,f'/drone_{drone}/flight_status',self.flight,qos_profile_sensor_data)
        for i in range(len(self.seeds)):
            if i!=drone:self.create_subscription(String,f'/drone_{i}/peer_state',self.peer,q)
        if self.safety_enabled:
            # Each native safety message is a complete map. Depth observation
            # deltas above retain their reliable history.
            self.create_subscription(Float64MultiArray,f'/baseline/safety_map_{drone}',self.native_snapshot,
                                     1 if self.fast_stop_feedback else 10)
        self.create_timer(.05,self.tick)

    def _static_send_ack(self,task,status):
        self.static_cancel_pub.publish(Float64MultiArray(data=list(map(float,task_wire(task)+[status]))))

    def static_cancel_request(self,m):
        try:self._static_cancel_request(m.data)
        except ValueError as error:self.event('static_cancel_rejected',reason=str(error))

    def collision_stop_request(self,m):
        intent=self.intent
        if (len(m.data) not in (3,4,5) or not intent or intent.get('static_task') is not None or
                intent.get('retiring') or self.flight_fault.reason or self.execution.get('safety_fault') or
                self.execution.get('token')!=intent['token']):return
        native_id,requested_at,sent_at=m.data[:3]
        if (native_id!=intent['native_id'] or not 0.<=requested_at<=sent_at or
                not 0.<=self.now()-sent_at<=.5):return
        request=dict(native_id=intent['native_id'],epoch=intent['epoch'],token=intent['token'],requested_at=requested_at)
        if len(m.data)>=4:
            if not self.fresh_static_signing or m.data[3]!=1. or intent.get('review_stop_declined')==requested_at:return
            request['purpose']='review'
        if len(m.data)==5:
            if not abs(m.data[4])<=math.pi:return
            request['view_yaw']=m.data[4]
        previous=intent.get('collision_stop_request')
        if previous is None:
            intent['collision_stop_request']=request
            if self.pending:
                self.reject(dict(traj_id=self.pending['native_id']),'native_collision_stop_in_progress')
                self.pending=None
            self.event('native_collision_stop_requested',**request)
        elif previous!=request:return
        self.publish(self.command_pub,dict(native_collision_stop=request))

    def review_stop_refusal(self,refusal):
        intent=self.intent
        request=intent.get('collision_stop_request') if intent else None
        if (request and request.get('purpose')=='review' and isinstance(refusal,dict) and
                self.execution.get('token')==intent['token'] and
                all(refusal.get(key)==value for key,value in request.items())):
            intent['review_stop_declined']=request['requested_at']
            intent.pop('collision_stop_request')
            self.event('native_review_stop_declined',receipt=refusal)

    def release_collision_stop(self,receipt,*,trigger='executor_report'):
        intent=self.intent
        request=intent.get('collision_stop_request') if intent else None
        if (not request or not isinstance(receipt,dict) or self.execution.get('token')!=intent['token'] or
                any(receipt.get(key)!=value for key,value in request.items()) or
                receipt.get('reference_stopped') is not True or receipt.get('observation_complete') is not False):return
        fast=getattr(self,'fast_stop_feedback',False)
        if fast and (intent.get('retiring') or self.flight_fault.reason or self.execution.get('safety_fault')):return
        t=self.now()
        ready=self.execution.get('ready') and self.execution.get('arrived')
        report_age=t-self.execution['time']
        mapper_age=t-self.last_sensor
        report_fresh=0.<=report_age<=.5
        mapper_fresh=0.<=mapper_age<=MAX_MAP_AGE
        native_fresh=self.native_safety.fresh(t)
        if not (ready and report_fresh and mapper_fresh and native_fresh):
            if fast:self.trace_stop_release(request,receipt,trigger,ready,report_fresh,mapper_fresh,native_fresh,None)
            return
        if self.dirty:self.replica.map.rebuild();self.dirty=False
        root=np.asarray(receipt['fixed_endpoint'])
        root_safe=np.array_equal(root,np.asarray(self.execution['reference'])) and self.corridor().box_safe(root,root)
        if fast:self.trace_stop_release(request,receipt,trigger,ready,report_fresh,mapper_fresh,native_fresh,root_safe)
        if not root_safe:return
        self.intent=None
        self.collision_stop_released=(request['native_id'],request['requested_at'])
        self.event('native_collision_stop_released',receipt=receipt,completed_observation_awarded=False)
        if fast:
            self.stop_release_wait=None
            feedback_stamp=self.now()
            self.send_execution_feedback(feedback_stamp)
            self.event('native_collision_stop_feedback_enqueued',native_id=request['native_id'],
                       requested_at=request['requested_at'],feedback_stamp_s=feedback_stamp)

    def trace_stop_release(self,request,receipt,trigger,ready,report_fresh,mapper_fresh,native_fresh,root_safe):
        state=(request['native_id'],request['requested_at'],bool(ready),report_fresh,mapper_fresh,native_fresh,root_safe)
        if state==self.stop_release_wait:return
        self.stop_release_wait=state
        self.event('native_collision_stop_release_check',native_id=request['native_id'],requested_at=request['requested_at'],
                   trigger=trigger,rest_checked_at=receipt.get('checked_at'),execution_report_s=self.execution['time'],
                   mapper_stamp_s=self.last_sensor,
                   native_stamp_s=self.native_safety.stamp if self.native_safety.state is not None else None,
                   ready=bool(ready),report_fresh=report_fresh,mapper_fresh=mapper_fresh,
                   native_fresh=native_fresh,root_safe=root_safe)

    def native_snapshot(self,m):
        try: updated=self.native_safety.ingest(m.data)
        except (TypeError, ValueError, OverflowError) as error: self.event('native_safety_snapshot_invalid',reason=str(error))
        else:
            if updated and self.fast_stop_feedback:
                self.release_collision_stop(self.execution.get('native_collision_stop_receipt'),trigger='native_map')

    def corridor(self, *, at_rest=False):
        return ExecutionCorridor(self.replica.map,self.native_safety,
            relaxed_sitl=at_rest and relaxed_sitl_execution(),
            frontier_sitl=at_rest and frontier_sitl_execution())

    def now(self):return self.get_clock().now().nanoseconds*1e-9
    def event(self, kind, **fields):
        if self.log is None:
            return
        try:
            self.log.write(json.dumps(dict(type=kind,time=self.now(),**fields))+'\n')
        except OSError as error:
            self.get_logger().warning(f'Authorization evidence write failed: {error}', throttle_duration_sec=5.)
    def publish(self,pub,row):
        if pub is self.command_pub:
            if self.authorization_session:row=dict(row,authorization_session=self.authorization_session)
            self._static_command_sent(row)
        pub.publish(String(data=json.dumps(row,separators=(',',':'))))
    def camera(self,m):
        self.calibration=m
        if self.id==0 and not self.hello:
            self.transport.send(dict(type='hello',bounds=self.config['bounds'],starts=self.seeds,intrinsics=list(m.k),
                                     flight_limits=list(self.replica.map.flight_limits), motion_limits=motion_limits().to_dict(),
                                     voxel_resolution=self.replica.map.resolution,body_clearance=self.replica.map.clearance,
                                     planning_scale=self.config.get('racer_planning_scale',.85),planning_limits=self.config.get('racer_planning_limits'),min_time=self.config.get('racer_min_time',True),feasible_time=self.config.get('racer_feasible_time',False),
                                     no_path_recovery=self.config.get('no_path_recovery',False),stop_and_plan=self.config.get('stop_and_plan',False),active_review=self.config.get('active_review',False),recover_frontiers=self.config.get('recover_frontiers',self.config.get('active_review',False)),
                                     confirmation_mode=self.config.get('confirmation_mode',''),bearing_bias=self.config.get('bearing_bias',False),
                                     static_event_replan=self.config.get('static_event_replan',False),static_event_audit=self.config.get('static_event_audit',False),
                                     static_cancel_session=self.static_cancel_session,
                                     static_cancel_enabled=self.config.get('static_cancel_enabled',False),
                                     static_moving_c2=self.static_moving_c2,static_c2_lead_s=self.config.get('static_c2_lead_s',-1.),
                                     static_weak_enabled=self.config.get('static_weak_enabled',False),static_weak_trajectory_s=self.config.get('static_weak_trajectory_s',-1.),
                                     static_weak_extra_cost=self.config.get('static_weak_extra_cost',-1.),static_weak_extra_path_m=self.config.get('static_weak_extra_path_m',-1.),
                                     static_event_min_interval_s=self.config.get('static_event_min_interval_s',-1.),static_guidance_budget=self.config.get('static_guidance_budget'),execution_safety=self.safety_enabled,review_continuation=self.config.get('review_continuation',False),review_max_segments=self.config.get('review_max_segments',3),
                                     review_max_duration_s=self.config.get('review_max_duration_s',60.),review_max_path_m=self.config.get('review_max_path_m',6.),
                                     review_observation_hold_s=self.config.get('review_observation_hold_s',5.),
                                     vertical_samples=self.config.get('vertical_samples',False),frame_source=self.config.get('native_frame_source','independent_depth_callback')))
            self.hello=True

    def odom(self,m):
        if self.flight_fault.reason:return
        p=m.pose.pose.position;q=m.pose.pose.orientation;stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        position = np.array([p.x, p.y, p.z]); quat = [q.x, q.y, q.z, q.w]
        v = m.twist.twist.linear
        if not np.isfinite([v.x, v.y, v.z]).all() or not self.poses.add(stamp, position, quat):
            return
        self.pose, rotation = position, self.poses.samples[-1][2]
        self.yaw = float(rotation.as_euler('xyz')[2])
        if stamp-self.last_odom>=.09:
            self.last_odom=stamp
            self.transport.send(dict(type='odom',drone=self.id,time=stamp,position=self.pose.tolist(),quaternion=quat,
                                     velocity=rotation.apply([v.x,v.y,v.z]).tolist()))
        self.drain()

    def image(self,m):
        if self.flight_fault.reason:return
        stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        if stamp-self.last_depth<.35:return
        self.images.append(m);self.drain()

    def drain(self):
        while self.images:
            m=self.images[0];stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
            if not self.poses.passed(stamp):return
            self.images.popleft();pose=self.poses.at(stamp)
            if pose is None or self.calibration is None or stamp-self.last_depth<.35:continue
            p,r,_=pose
            try:
                points, hits = project_depth_message(m, self.calibration.k)
            except (TypeError, ValueError, OverflowError) as error:
                self.event('native_depth_frame_rejected', reason=str(error))
                continue
            points[~hits]*=4.51/4.5;origin=p+r.apply(SENSOR_TRANSLATION)
            self.transport.send(dict(type='frame',drone=self.id,time=stamp,origin=origin.tolist(),points=(r.apply(points)+origin).tolist()))
            self.last_depth=stamp

    def observation(self,m):
        if self.flight_fault.reason:return
        try:
            packet = json.loads(m.data)
            if (not isinstance(packet, dict) or packet.get('source') != self.id
                    or packet.get('sensor') != 'gz_depth' or packet.get('integration_completed') is not True):
                return
            frame = None
            if self.config.get('native_frame_source') == 'mapper_observation':
                frame = packet['native_frame']
                origin = np.asarray(frame['origin'], float)
                points = np.asarray(frame['points'], float)
                if origin.shape != (3,) or points.ndim != 2 or points.shape[1] != 3 or (
                        not np.isfinite(origin).all() or not np.isfinite(points).all()):
                    raise ValueError('Invalid native depth frame')
            before = (self.replica.sensor_sessions.get(self.id), self.replica.sequences.get(self.id))
            self.dirty = self.replica.merge(packet) or self.dirty
        except (KeyError, TypeError, ValueError, OverflowError) as error:
            self.event('mapper_packet_rejected', reason=str(error))
            return
        after = (self.replica.sensor_sessions.get(self.id), self.replica.sequences.get(self.id))
        if after == before:
            return
        self.last_sensor = self.replica.sensor_stamps[self.id]
        if frame is not None:
            self.transport.send(dict(type='frame', drone=self.id, time=packet['time'],
                                     origin=origin.tolist(), points=points.tolist()))
        cells = np.asarray(packet.get('measured_indices', []), int).reshape(-1, 3)
        if len(cells):
            self.measured[np.ravel_multi_index(cells.T, self.replica.map.shape)] = True
        if self.fast_stop_feedback:
            self.release_collision_stop(self.execution.get('native_collision_stop_receipt'),trigger='mapper')

    def peer(self,m):
        try:
            packet = json.loads(m.data)
            if not isinstance(packet, dict) or type(packet.get('time')) not in (int, float):
                return
            self.ledger.receive(packet)
        except (KeyError, TypeError, ValueError, OverflowError):
            return

    def flight(self,m):
        try:
            packet = json.loads(m.data)
        except (TypeError, ValueError):
            return
        if self.flight_fault.observe(packet, self.now()):
            self.cancel('flight_session_fault')

    def executed(self,m):
        try:
            execution = json.loads(m.data)
        except (TypeError, ValueError):
            return
        if not isinstance(execution, dict):
            return
        self.execution = execution
        self._static_execution_ack(self.execution)
        rejected=self.execution.get('activation_rejected')
        if rejected and self.pending and rejected['token']==self.pending['token']:
            self.reject(dict(traj_id=self.pending['native_id']),rejected['reason']); self.pending=None
        if self.execution.get('safety_fault'):
            if not self.reported_safety_fault:
                self.reported_safety_fault=True; self.event('execution_safety_fault',reason=self.execution['safety_fault'])
            return
        revision=self.execution.get('safety_revision')
        if revision and self.intent and self.execution.get('token')==self.intent['token'] and revision.get('number',0)>self.intent.get('safety_revision',0):
            self.intent['safety_revision']=revision['number']
            self.intent['native_duration']=min(self.intent['native_duration'],revision['boundary'])
            self.intent['path']=revision['path']
            self.pending=None
            self.event('execution_safety_brake',token=self.intent['token'],decision_time=revision['time'],
                       **{k:v for k,v in revision.items() if k!='time'})
        if self.pending and self.execution.get('token')==self.pending['token']:
            old=self.intent;self.intent=self.pending;self.pending=None
            self.collision_stop_released=None
            if old:self.event('handoff_consumed',old_token=old['token'],token=self.intent['token'],continuity=self.execution.get('handoff_continuity'))
            self.event('native_trajectory_activated',traj_id=self.intent['native_id'],token=self.intent['token'])
        if self.collision_stop_enabled:
            self.release_collision_stop(self.execution.get('native_collision_stop_receipt'))
            self.review_stop_refusal(self.execution.get('native_collision_stop_refusal'))
        if (self.intent and self.intent['token'] == self.execution.get('token')
                and _failed_observation_root_safe(self, self.now())):
            token, native_id = self.intent['token'], self.intent['native_id']
            self.failed_observations[token] = native_id
            self.intent = None
            self.event('terminal_observation_failed', token=token, native_id=native_id,
                       outcome=self.execution['observation_failure'], reference_retained=True,
                       completed_observation_awarded=False, native_feedback_active=-1)
        if completed_execution_proof(self.execution):
            token=self.execution['token']
            if token not in self.completed:
                self.completed.add(token);self.event('terminal_observation_completed',token=token,proof=self.execution['observation_proof'])
            if self.intent and self.intent['token']==token:self.intent=None

    def state(self):
        def public(intent):
            if intent is None:return None
            row={k:v for k,v in intent.items() if k not in ('trajectory','target_cells')}
            if row.get('committed') and row['token']==self.execution.get('token'):row['path']=remainder(self.pose,row['path']).tolist()
            return row
        return dict(drone=self.id,session=self.session,time=self.now(),sequence=self.sequence,epoch=self.epoch,
            position=self.pose.tolist(),yaw=self.yaw,ready=self.execution.get('ready',False),available=not self.flight_fault.reason,
            stopped=bool(np.linalg.norm(self.execution.get('reference_velocity',[0,0,0]))<.05),rejoin_ready=True,
            bids={},intent=public(self.intent),pending_intent=public(self.pending),acks=self.acks,execution=self.execution,
            planner='RACER_authorized_native',flight_session_fault=self.flight_fault.reason)

    def reject(self,row,reason):
        self.last_rejection=row['traj_id'];self.event('native_proposal_rejected',traj_id=row['traj_id'],reason=reason)

    def propose(self,row):
        proposal_started=time.monotonic()
        trace=[] if os.environ.get('ASPIRE_RECORD_PREFIX_FAILURES')=='1' else None
        self.event('native_planning_attempt',native_id=row['traj_id'],compute_wall_s=row.get('planning_wall_s'))
        if row['traj_id'] in getattr(self, 'failed_observations', {}).values():
            return
        if not self._static_before_propose(row):return self.reject(row,'static_task_cancelled_or_invalid')
        if (row.get('static_task') and self.fresh_static_signing and
                not fresh_static_source_frame(row.get('static_source_frame'),self.now())):
            return self.reject(row,'review_source_frame_expired_at_gateway')
        if self.pose is None or self.flight_fault.reason:return self.reject(row,'not_ready')
        if self.intent and self.intent.get('collision_stop_request'):
            return self.reject(row,'native_collision_stop_in_progress')
        if self.pending:return self.reject(row,'pending_authorization')
        if not {'from_id','boundary_progress'}<=row.keys():return self.reject(row,'missing_native_boundary')
        try:
            curve=native_trajectory(row);proposed_duration=curve.duration
            if self.dirty:self.replica.map.rebuild();self.dirty=False
            if self.safety_enabled and not self.native_safety.fresh(self.now()):
                raise ValueError('native_safety_map_stale')
            runtime=self.corridor(at_rest=row['boundary_progress']<0) if self.safety_enabled else self.replica.map
            frontier=self.safety_enabled and runtime.frontier_sitl
            stepped=frontier or (self.safety_enabled and os.environ.get('ASPIRE_FRONTIER_STEPS')=='1')
            moving=row['boundary_progress']>=0
            checks=dict(diagnostics=trace) if trace is not None else {}
            if frontier:
                checks.update(validate_stop=lambda c:stopping_reserve(c,0.,runtime),reserve_horizon=REACTION_HORIZON)
            curve,native_duration=safe_native_execution(curve,runtime,**checks)
            path=curve.path(.08)
            # Continuous stepped, frontier, and moving C2 modes certify corridors already.
            if self.safety_enabled and not stepped and not moving:
                if not runtime.safe_curve(curve): raise ValueError('continuous_native_corridor_invalid')
            # Frontier mode checks the prefix and brake continuously. Certify
            # the composed sampled path here; other modes already did so.
            if frontier and not self.replica.map.safe_path(path):raise ValueError('native_curve_not_known_free')
            if self.now()-self.last_sensor>1.:raise ValueError('actual_depth_stale')
            handoff=None
            if moving:
                if not self.intent or self.intent['native_id']!=row['from_id'] or self.execution.get('token')!=self.intent['token']:
                    raise ValueError('native_execution_root_changed')
                old=self.curves[self.intent['token']]
                handoff=dict(from_epoch=self.intent['epoch'],from_token=self.intent['token'],trajectory_time=row['boundary_progress'])
                if self.safety_enabled:
                    if not self.static_moving_c2 or row.get('static_task') is None:
                        raise ValueError('moving_C2_disabled_or_unbound')
                    identity=curve_fingerprint(old)
                    validate_safe_handoff(old,curve,row['boundary_progress'],self.execution.get('trajectory_time',0.),
                        expected_old=identity,old_safety_revision=1 if self.execution.get('safety_revision') else 0,
                        now=self.now(),native_stamp=self.native_safety.stamp,mapper_stamp=self.last_sensor,
                        corridor=runtime,started=proposal_started)
                    handoff['old_curve_sha256']=identity
                else:
                    validate_handoff(old,curve,row['boundary_progress'],self.execution.get('trajectory_time',0.))
            elif self.intent or np.linalg.norm(path[0]-self.pose)>(TRACKING_MARGIN if self.safety_enabled else .25):
                raise ValueError('native_static_root_stale')
            if self.safety_enabled and not handoff and np.linalg.norm(path[0]-np.asarray(self.execution.get('reference',self.pose)))>static_reference_jump_limit():
                raise ValueError('native_static_reference_discontinuity')
            terminal_yaw=float(curve.yaw_sample(curve.duration))
            if not self.ledger.can_propose(path,-10000-self.id,self.now()):raise ValueError('reservation_conflict')
            if self.safety_enabled and handoff:
                check_handoff_budget(self.now(),self.native_safety.stamp,self.last_sensor,proposal_started)
        except (ValueError,KeyError,TypeError) as exc:
            if trace:
                grid=self.replica.map
                np.savez_compressed(self.folder/f"prefix-rejected-{row['traj_id']}.npz",
                    state=grid.state,native_state=self.native_safety.state,origin=grid.origin,
                    resolution=grid.resolution,flight_limits=grid.flight_limits,clearance=grid.clearance,
                    native_stamp=self.native_safety.stamp,mapper_stamp=self.last_sensor,
                    native_sequence=self.native_safety.sequence,mapper_version=grid.version,
                    odometry=self.pose,reference=self.execution.get('reference',self.pose),
                    reason=str(exc),trial_steps=json.dumps(trace))
            if self.diagnostics<3 and 'curve' in locals():
                runtime=self.replica.map;self.diagnostics+=1
                try:
                    np.savez_compressed(self.folder/f'rejected-{self.diagnostics}.npz',state=runtime.state,origin=runtime.origin,resolution=runtime.resolution,path=curve.path(.05),flight_limits=runtime.flight_limits,clearance=runtime.clearance)
                except OSError as error:
                    self.event('rejection_evidence_unavailable', reason=str(error))
                self.event('rejection_geometry',root=curve.sample(0)[0].tolist(),root_safe=runtime.safe_path([curve.sample(0)[0]]),current_safe=runtime.safe_path([self.pose]),reason=str(exc))
            return self.reject(row,str(exc))
        self.epoch+=1
        intent=dict(token=f'racer:{self.id}:{self.session[:8]}:{self.epoch}',created=self.now(),epoch=self.epoch,
            region=-10000-self.id,native_id=row['traj_id'],committed=False,path=path.tolist(),yaw=terminal_yaw,duration=curve.duration,
            trajectory=curve.to_dict(),native_duration=native_duration,observation_grid=observation_grid(self.replica.map),voters=[i for i in range(len(self.seeds)) if i!=self.id],
            target_cells=sorted(visible_cells(self.replica.map,path[-1],terminal_yaw)),native_sequence=self.native_safety.sequence)
        if handoff:intent['handoff']=handoff
        self.curves[intent['token']]=curve;self.pending=intent
        self._static_attach_pending(row)
        if row.get('static_source_frame') is not None:intent['static_source_frame']=row['static_source_frame']
        self.event('planning_result',compute_wall_s=row.get('planning_wall_s'),native_id=row['traj_id'],trajectory_duration=curve.duration)
        self.event('path_proposed',token=intent['token'],trajectory_method=curve.method,trajectory_limits=intent['trajectory']['limits'],
                   trajectory_yaw_peak=intent['trajectory']['yaw_rate_limit'],native_id=row['traj_id'],
                   proposed_native_duration=proposed_duration,
                   native_duration=native_duration,contingency_brake_s=curve.duration-native_duration)

    def cancel(self,reason):
        self.epoch+=1;self.publish(self.command_pub,dict(epoch=self.epoch,cancel=True,reason=reason))
        if self.pending:self.last_rejection=self.pending['native_id']
        if self.intent:self.intent['retiring']=True
        self.pending=None;self.event('execution_cancelled',reason=reason)

    def tick(self):
        if self.pose is None:return
        # Let queued real sensor callbacks catch up before evaluating a new
        # proposal. Moving C2 boundaries still expire normally; stale proposals
        # never gain authorization merely by waiting here.
        if not self.proposals.empty() and self.now()-self.last_sensor<.5:
            self.propose(self.proposals.get_nowait())
        t=self.now();self.sequence+=1
        if self.intent and self.intent.get('retiring') and self.execution.get('token') is None:self.intent=None
        self.acks=self.ledger.acknowledge(self.state(),t)
        pending=self.pending
        if pending and pending['committed']:
            boundary=pending.get('handoff')
            if ((boundary and self.execution.get('trajectory_time',0.)>boundary['trajectory_time']+.25)
                    or (not boundary and t-pending['created']>6.)):
                self.publish(self.command_pub,dict(cancel_pending=pending['token'])) if boundary else self.cancel('native_activation_timeout')
                self.reject(dict(traj_id=pending['native_id']),'authorized_curve_not_activated')
                self.pending=None;pending=None
        if pending and not pending['committed']:
            boundary=pending.get('handoff');progress=self.execution.get('trajectory_time',0.)
            reason=None
            if boundary and progress>=boundary['trajectory_time']-.15:
                reason='missed_C2_boundary_or_sensor_gate'
            elif (pending.get('static_task') and self.fresh_static_signing and
                    not fresh_static_source_frame(pending.get('static_source_frame'),t)):
                reason='review_source_frame_expired_before_commit'
            elif t-pending['created']>6. or self.ledger.loses(pending):
                reason='authorization_timeout_or_conflict'
            if reason:
                self.reject(dict(traj_id=pending['native_id']),reason);self.pending=None
            elif self.ledger.quorum(pending['token'],t,pending['voters']):
                cells=(self.intent or {}).get('target_cells',[])
                sensor_ready=not boundary or (len(cells)==0 or int(self.measured[cells].sum())>=.6*len(cells))
                if sensor_ready:
                    maps_changed = self.dirty or (self.safety_enabled and self.native_safety.sequence != pending.get('native_sequence'))
                    if self.dirty:
                        self.replica.map.rebuild(); self.dirty=False
                    geometry_ok = (not maps_changed or (self.replica.map.safe_path(pending['path']) and
                        (not self.safety_enabled or self.corridor(at_rest=boundary is None).safe_curve(self.curves[pending['token']]))))
                    if (not geometry_ok or (self.safety_enabled and not self.native_safety.fresh(t))):
                        self.reject(dict(traj_id=pending['native_id']),'latest_geometry_invalid');self.pending=None
                    else:
                        pending['committed']=True;pending['committed_at']=t
                        self.publish(self.state_pub,self.state())
                        self.publish(self.command_pub,{k:v for k,v in pending.items() if k not in ('target_cells','native_sequence')})
                        self.event('path_committed',token=pending['token'],native_id=pending['native_id'],
                                   **({'static_source_frame':pending['static_source_frame']} if 'static_source_frame' in pending else {}))
                        if boundary:self.event('handoff_authorized',token=pending['token'],old_token=boundary['from_token'],
                            trajectory_time=boundary['trajectory_time'],observed_cells=int(self.measured[cells].sum()),target_cells=len(cells),required_fraction=.6)
        self.publish(self.state_pub,self.state())
        self.send_execution_feedback(self.now() if self.fast_stop_feedback else t)

    def send_execution_feedback(self,t):
        token=self.execution.get('token');active=self.intent if self.intent and self.intent['token']==token else None
        # A completed curve is retained for native FINISH/next-root semantics.
        if active is None and token in self.completed and token not in self.static_retired_tokens:
            active_id=self.last_completed_id
        else:active_id=active['native_id'] if active else -1
        if active:self.last_completed_id=active_id
        candidate=self.pending['native_id'] if self.pending else self.last_rejection
        status=1. if self.pending else -1.
        feedback=[float(active_id),float(self.execution.get('trajectory_time',0.)),float(token in self.completed and token not in self.static_retired_tokens and completed_execution_proof(self.execution)),float(candidate),status,t,
                  float(active['native_duration']) if active else 0.]
        healthy = not (self.flight_fault.reason or self.execution.get('safety_fault')
                       or self.execution.get('flight_session_fault'))
        ready = bool(self.execution.get('ready') and healthy)
        stopped_id = self.failed_observations.get(token)
        if self.collision_stop_released and active_id < 0:
            feedback.extend(self.collision_stop_released)
        elif (healthy and active_id < 0 and feedback[2] == 0.
              and token not in self.static_retired_tokens and stopped_id is not None):
            # Repeat the verified stop receipt until the executor token changes.
            # A stale sensor pauses new admission independently of this receipt.
            feedback.append(float(stopped_id))
            ready = ready and _failed_observation_root_safe(self, t)
        row = dict(type='feedback', drone=self.id, time=t, ready=ready, data=feedback)
        if getattr(self, 'use_stopped_reference', False) and ready and self.pending is None and len(feedback) == 7:
            reference = stopped_planning_reference(self.execution, active_id, bool(feedback[2]), t)
            if reference is not None:
                row['stopped_reference'] = [float(active_id), *reference]
        self.transport.send(row)


def _failed_observation_root_safe(agent, now):
    if (not failed_observation_proof(agent.execution, now) or not agent.safety_enabled
            or not -.05 <= now-agent.last_sensor <= 1. or not agent.native_safety.fresh(now)):
        return False
    if agent.dirty:
        agent.replica.map.rebuild()
        agent.dirty = False
    root = np.asarray(agent.execution['reference'], float)
    return agent.corridor().box_safe(root, root)


def main():
    p=argparse.ArgumentParser();p.add_argument('--runtime',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--port',type=int,default=23901)
    p.add_argument('--transport',choices=('dds',),default='dds')
    a=p.parse_args();config=json.loads(a.runtime.read_text())
    os.environ.update(motion_environment(config))
    os.environ.update(voxel_environment(config))
    rclpy.init()
    import sys
    sys.path.insert(0,str(Path(__file__).resolve().parents[3]))
    from tools.racer_ros2.transport import NativeDDSTransport
    transport=NativeDDSTransport(a.output.parent/'native-racer',fleet_size=len(config['vehicles']))
    nodes=[NativeLeaseAgent(i,config,transport,a.output) for i in range(len(config['vehicles']))]
    # Each node's default callback group remains mutually exclusive: its map,
    # ledger and trajectory state cannot be mutated during grid generation.
    # Independent aircraft get their own worker instead of starving one another.
    # The transport has its own mutually exclusive callback group. Drain its
    # queue during expensive gateway map and planning callbacks.
    transport_worker=int(any(node.fast_stop_feedback for node in nodes))
    executor=MultiThreadedExecutor(num_threads=len(nodes)+transport_worker)
    for node in nodes:executor.add_node(node)
    executor.add_node(transport)
    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=.01)
            if not transport.incoming.empty():
                row=transport.incoming.get_nowait();nodes[row['drone']].proposals.put(row)
            if transport.error:
                raise RuntimeError(transport.error)
    except KeyboardInterrupt:pass
    finally:
        # Launch and the capture process may both deliver SIGINT. A second
        # interrupt must not skip child cleanup while executor workers join.
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        try:transport.close()
        finally:executor.shutdown()
        for node in nodes:
            if transport.error:node.cancel('native_gateway_lost')
            node.event('gateway_closed',dropped_outbound=transport.drops)
            if node.log is not None:
                try:
                    node.log.close()
                except OSError as error:
                    node.get_logger().warning(f'Authorization evidence close failed: {error}')
            node.destroy_node()
        transport.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
