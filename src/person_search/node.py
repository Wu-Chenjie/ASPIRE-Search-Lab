"""Live ROS2 perception adapter; refuses missing capture-time map transforms.

Run with the ROS environment sourced and prepend src plus src/annual_planner/src to PYTHONPATH.
Publish detections only; this node cannot arm, teleport, or fly a vehicle.
"""
import json
import os
import time
import threading
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from .perception import PeopleDetector,candidates_from_cloud,cloud
from .tracking import Tracker
from .active_confirmation import review_execution_ready, review_pose
from .policy import ObservedVolume,choose_view
from .segmentation import localize_masks
from annual_runtime.rgbd_alignment import camera_uses_full_resolution,require_registered_pair,stamp_ns


def matrix(transform):
    t=transform.translation;q=transform.rotation
    xyzw=np.array([q.x,q.y,q.z,q.w]);xyz=np.array([t.x,t.y,t.z])
    if not np.isclose(np.linalg.norm(xyzw),1,atol=1e-5) or not np.isfinite(xyz).all():
        raise ValueError('Invalid map TF')
    result=np.eye(4);result[:3,:3]=Rotation.from_quat(xyzw).as_matrix();result[:3,3]=xyz
    return result


def rgb_pixels(rgb):
    if rgb.encoding not in ('rgb8','bgr8'):
        raise ValueError('Expected rgb8 or bgr8')
    data=bytes(rgb.data) if isinstance(rgb.data,list) else rgb.data
    pixels = np.ndarray((rgb.height,rgb.width,3), np.uint8, data, strides=(rgb.step,3,1))
    if rgb.encoding == 'bgr8':
        pixels = pixels[:, :, ::-1]
    return pixels.copy()


def capture_age_valid(stamp_ns,now_ns,max_age):
    return max_age<=0 or -50_000_000<=now_ns-stamp_ns<=max_age*1e9


def newest_bracketed_capture(stamps,history,now_ns,max_age):
    fresh = (ns for ns in stamps if capture_age_valid(ns, now_ns, max_age))
    if history is None:
        return max(fresh, default=None)
    return next((ns for ns in sorted(fresh, reverse=True) if history.brackets(ns*1e-9)), None)


def review_request_snapshot(read_execution, read_clock, *, validate_execution=True):
    """Take a new execution snapshot and timestamp AFTER viewpoint selection.

    The DDS callback may replace execution while CPU selection is running.
    Read the clock after the snapshot; never compare a newer report to the
    pre-selection time or publish a request for a vehicle that already left.
    """
    execution=dict(read_execution());stamp=read_clock()
    if validate_execution and not review_execution_ready(execution, stamp.nanoseconds*1e-9):
        return None
    return stamp,execution


def confirmation_options(mode, old_active_confirmation, active_review):
    """An explicit policy never silently enables movement for baseline runs."""
    if mode not in ('','legacy','strict','hybrid','decoupled','registry'):raise ValueError('Unknown confirmation_mode')
    if mode in ('decoupled','registry'):
        return mode,False,False
    if mode:return mode,bool(active_review),mode!='legacy'
    return ('strict' if old_active_confirmation else 'legacy',
            bool(active_review or old_active_confirmation),False)


def target_markers(tracks, stamp):
    """Same ROS marker mapping for live tracks and explicitly labelled log replay."""
    from visualization_msgs.msg import Marker,MarkerArray
    arr=MarkerArray()
    for t in tracks:
        m=Marker();m.header.frame_id='map';m.header.stamp=stamp;m.ns='person_candidates';m.id=t.id;m.type=Marker.SPHERE;m.action=Marker.ADD;m.pose.orientation.w=1.;m.pose.position.x,m.pose.position.y,m.pose.position.z=map(float,t.xyz);m.scale.x=m.scale.y=m.scale.z=.25;m.color.a=.9;m.color.g=1. if t.state=='confirmed' else .5;m.color.r=0. if t.state=='confirmed' else 1.;arr.markers.append(m)
    return arr


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image,CameraInfo
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped
    from annual_runtime.pose_history import PoseHistory
    from std_msgs.msg import String,Float64MultiArray
    from visualization_msgs.msg import MarkerArray
    from tf2_ros import Buffer,TransformListener,TransformException
    from cv2 import error as OpenCVError
    from .presence_adapter import capture_world, process_presence, snapshot_inputs
    class Search(Node):
        def __init__(self):
            super().__init__('person_search')
            if not self.has_parameter('use_sim_time'):self.declare_parameter('use_sim_time',True)
            self.queues={k:{} for k in ('rgb','depth','ri','di')};self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
            self.received_wall={};self.rgb_received_total=0;self.rgb_consumed_total=0;self.process_calls=0;self.latest_rgb_received_ns=-1;self.last_rgb_consumed_ns=-1;self.callback_started_wall=None;self.callback_started_ns=None;self.previous_callback_wall_gap=None;self.previous_callback_sim_gap=None;self.last_callback_wall_s=None;self.input_lock=threading.RLock()
            self.require_pose=self.declare_parameter('require_estimated_pose',False).value;self.pose_history=PoseHistory()
            if self.require_pose:
                from rclpy.qos import QoSProfile,ReliabilityPolicy
                # Keep odometry arriving during CPU inference. Bracket tolerance
                # stays 40 ms; no extrapolation or relaxed alignment is added.
                self.create_subscription(Odometry,'/drone_0/estimated_odometry',self.odom,QoSProfile(depth=100,reliability=ReliabilityPolicy.BEST_EFFORT))
            old_confirmation=self.declare_parameter('active_confirmation',False).value
            mode=self.declare_parameter('confirmation_mode','').value
            review=self.declare_parameter('active_review',False).value
            self.confirmation_mode,self.active_review,self.require_request_status=confirmation_options(mode,old_confirmation,review)
            self.registry_mode=self.confirmation_mode=='registry'
            self.review_depth_continuity=self.registry_mode and os.environ.get('ASPIRE_REVIEW_SOURCE_RETENTION')=='1'
            self.decoupled=self.confirmation_mode in ('decoupled','registry')
            if self.decoupled and review:
                self.get_logger().warning('Legacy XYZ review is disabled for '+self.confirmation_mode)
            self.active_confirmation=self.confirmation_mode not in ('legacy','decoupled','registry')
            self.volume=ObservedVolume();self.last_volume=-1e9;self.last_review=-1e9;self.previous=[];self.execution={};self.review_status_queue=[]
            self.review_pub=self.create_publisher(PoseStamped,'/search/review_goal',2)
            self.review_cancel_pub=self.create_publisher(String,'/search/review_cancel',10)
            if self.active_review or self.active_confirmation or self.decoupled:
                from rclpy.qos import QoSProfile,DurabilityPolicy
                self.create_subscription(String,'/drone_0/view_execution',lambda m:setattr(self,'execution',json.loads(m.data)),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
            if self.require_request_status:
                self.create_subscription(String,'/search/review_status',self.receive_review_status,10)
            self.backend=self.declare_parameter('detector_backend','hog').value
            if self.decoupled and self.backend=='hog':
                raise ValueError('decoupled requires explicit RGB segmentation backend')
            weights=self.declare_parameter('segmentation_weights','').value
            manifest=self.declare_parameter('detector_manifest','').value
            if self.backend=='yolo11n-seg':
                from .yolo import YoloPersonDetector
                detector_options={'preserve_mask_candidates':True} if self.decoupled else {}
                if manifest:
                    from .detector_spec import DetectorSpec
                    spec=DetectorSpec.load(manifest)
                    if spec.purpose!='person':raise ValueError('Human pipeline requires a person manifest')
                    detector_options['spec']=spec
                self.detector=YoloPersonDetector(weights,**detector_options)
            elif self.backend=='hog':self.detector=PeopleDetector()
            else:raise ValueError('Unknown detector_backend')
            from .bearing import BearingMemory
            self.bearings=BearingMemory()
            self.bearing_pub=self.create_publisher(Float64MultiArray,'/search/bearing_evidence',30)
            self.bearing_output=self.create_publisher(String,'/search/bearings',30)
            self.semantic_cache={};self.last_semantic_ns=-1;self.inference_rgb_ns=None
            self.max_age=self.declare_parameter('max_frame_age_s',.75).value
            self.rgbd_registration_mode=self.declare_parameter('rgbd_registration_mode','strict').value
            self.rgbd_image_mode=self.declare_parameter('rgbd_image_mode','rectified').value
            self.rgbd_max_skew_s=self.declare_parameter('rgbd_max_skew_s',0.).value
            self.rgbd_static_scene=self.declare_parameter('rgbd_static_scene',False).value
            if self.rgbd_registration_mode not in ('strict','reproject'):
                raise ValueError('Invalid RGB-D registration mode')
            if self.rgbd_registration_mode == 'reproject':
                if self.rgbd_image_mode not in ('rectified','raw_pinhole'):
                    raise ValueError('Invalid RGB-D image mode for reprojection')
                if not np.isfinite(self.rgbd_max_skew_s) or not 0 <= self.rgbd_max_skew_s <= .05:
                    self.get_logger().warning('Invalid RGB-D skew bound, using same-time registration')
                    self.rgbd_max_skew_s = 0.
                elif self.rgbd_max_skew_s and not self.rgbd_static_scene:
                    self.get_logger().warning('Cross-time registration requires a static scene, using same-time registration')
                    self.rgbd_max_skew_s = 0.
            evidence = self.declare_parameter('evidence_dir', '').value
            self.evidence_dir = None
            self.evidence_log = None
            if evidence:
                try:
                    directory = Path(evidence).resolve()
                    source_root = Path(__file__).resolve().parents[2]
                    if directory.is_relative_to(source_root):
                        raise ValueError('Perception evidence must be outside source')
                    directory.mkdir(parents=True, exist_ok=False)
                    self.evidence_log = (directory/'observations.jsonl').open('x', buffering=1)
                    self.evidence_dir = directory
                except (OSError, ValueError) as error:
                    self.get_logger().warning(f'Perception evidence unavailable: {error}')
            if self.evidence_dir and self.backend == 'yolo11n-seg':
                self.record_evidence(dict(kind='detector_runtime', **self.detector.runtime_diagnostics()))
            self.diagnostic_output=self.create_publisher(String,'/search/perception_timing',10)
            self.trace_detector=None;self.trace_pipeline=None
            trace_manifest=self.declare_parameter('trace_detector_manifest','').value
            trace_weights=self.declare_parameter('trace_weights','').value
            if bool(trace_manifest) != bool(trace_weights) or (trace_manifest and not self.registry_mode):
                self.get_logger().warning('Trace detection disabled because it requires manifest, weights and registry mode')
                trace_manifest = ''
            if self.decoupled:
                from .presence import PresenceTracker,PresenceConfig,LocalizationConfig,PresenceBearings
                from dataclasses import asdict
                visual=PresenceConfig(**{name:self.declare_parameter('presence_'+name,value).value for name,value in asdict(PresenceConfig()).items()})
                localization=LocalizationConfig(**{name:self.declare_parameter('localization_'+name,value).value for name,value in asdict(LocalizationConfig()).items()})
                self.tracker=PresenceTracker(visual,localization);self.bearings=PresenceBearings(self.tracker)
                if self.registry_mode:
                    from .registry_evidence import RegistryEvidence
                    robot=self.declare_parameter('robot','uav0').value
                    frame_version=self.declare_parameter('coordinate_version','').value
                    aligned=self.declare_parameter('shared_coordinates_reliable',False).value
                    clock_domain=self.declare_parameter('capture_clock_domain','mission').value
                    self.registry_evidence=RegistryEvidence(robot=robot,frame_version=frame_version or None,shared_aligned=aligned,clock_domain=clock_domain)
                    if trace_manifest:
                        try:
                            from .detector_spec import DetectorSpec
                            from .yolo import YoloPersonDetector
                            from .trace_pipeline import TracePipeline
                            spec=DetectorSpec.load(trace_manifest)
                            if spec.purpose!='trace':raise ValueError('Trace pipeline requires trace manifest')
                            self.trace_detector=YoloPersonDetector(trace_weights,spec=spec)
                            self.trace_pipeline=TracePipeline(self.registry_evidence.session,self.registry_evidence.frame_version,
                                clock_domain=self.registry_evidence.clock_domain)
                            self.trace_pub=self.create_publisher(String,'/'+robot+'/search/trace_hints',10)
                        except (ImportError, OSError, TypeError, ValueError, RuntimeError) as error:
                            self.trace_detector = None
                            self.trace_pipeline = None
                            self.get_logger().warning(f'Trace detection disabled: {error}')
                    from .static_hypothesis import VisibilityConfig
                    visibility_json=self.declare_parameter('static_visibility_config','').value
                    self.static_visibility_model=self.declare_parameter('static_visibility_model_id','').value
                    self.static_visibility_config = None
                    if visibility_json:
                        try:
                            if not self.static_visibility_model:
                                raise ValueError('Visibility requires an explicit calibration/model identity')
                            self.static_visibility_config = VisibilityConfig(**json.loads(visibility_json))
                        except (TypeError, ValueError) as error:
                            self.get_logger().warning(f'Static visibility disabled: {error}')
                    self.pending_visibility=None
                    self.registry_pub=self.create_publisher(String,'/'+robot+'/search/target_observations',20)
            elif self.active_confirmation:
                from .active_confirmation import ActiveConfirmationTracker
                quality = None
                if self.confirmation_mode == 'hybrid':
                    from .confirmation_quality import ConfirmationQuality
                    from dataclasses import asdict
                    quality = ConfirmationQuality(**{
                        name: self.declare_parameter('confirmation_' + name, value).value
                        for name, value in asdict(ConfirmationQuality()).items()
                    })
                self.tracker=ActiveConfirmationTracker(mode=self.confirmation_mode,quality=quality,
                    require_request_status=self.require_request_status,
                    execution_timeout=self.declare_parameter('review_execution_timeout_s',60. if mode else 25.).value,
                    observation_timeout=self.declare_parameter('review_observation_timeout_s',8.).value)
            else:self.tracker=Tracker()
            self.review_events=self.create_publisher(String,'/search/active_confirmation',10)
            self.presence_events=self.create_publisher(String,'/search/presence_events',30)
            self.last_ns=-1;self.processed=0;self.rejected=0
            self.output=self.create_publisher(String,'/search/tracks',10);self.markers=self.create_publisher(MarkerArray,'/search/targets',10)
            self.semantic_output=self.create_publisher(String,'/search/semantic_candidates',10)
            self.annotated_output=self.create_publisher(Image,'/search/detection_image',2)
            for key,kind,topic in [('rgb',Image,'rgb/image'),('depth',Image,'depth/image'),('ri',CameraInfo,'rgb/camera_info'),('di',CameraInfo,'depth/camera_info')]:
                self.create_subscription(kind,'/drone_0/'+topic,lambda m,k=key:self.receive(k,m),qos_profile_sensor_data)
            from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
            self.inference_group=MutuallyExclusiveCallbackGroup()
            self.input_event=threading.Event();self.worker_stop=threading.Event()
            default_period = .2 if self.decoupled else 0.
            period = self.declare_parameter('min_rgb_period_s', default_period).value
            self.input_queue_limit = self.declare_parameter('input_queue_limit', 40).value
            if not np.isfinite(period*1e9) or period < 0:
                self.get_logger().warning('Invalid RGB sampling period, using the default')
                period = default_period
            if self.input_queue_limit < 2:
                self.get_logger().warning('Input queue needs at least two samples, using the default')
                self.input_queue_limit = 40
            self.worker=None;self.expiry_requested=False;self.min_rgb_period_ns=round(period*1e9)
            if self.decoupled:self.create_timer(.2,self.wake_process)
            else:self.create_timer(.2,self.process,callback_group=self.inference_group)
        def receive_review_status(self,m):
            try:row=json.loads(m.data)
            except json.JSONDecodeError:return
            with self.input_lock:self.review_status_queue.append(row)
        def odom(self,m):
            p=m.pose.pose.position;q=m.pose.pose.orientation
            with self.input_lock:self.pose_history.add(stamp_ns(m.header)*1e-9,[p.x,p.y,p.z],[q.x,q.y,q.z,q.w])
            if self.decoupled:self.input_event.set()
        def receive(self,k,m):
            with self.input_lock:
                ns = stamp_ns(m.header)
                q = self.queues[k]
                if k == 'rgb':
                    self.rgb_received_total += 1
                    if ns in q or ns in self.semantic_cache or ns==self.inference_rgb_ns:
                        return
                    self.received_wall[ns] = time.monotonic()
                    self.latest_rgb_received_ns = max(self.latest_rgb_received_ns, ns)
                q[ns] = m
                if len(q) > self.input_queue_limit:
                    oldest = min(q)
                    del q[oldest]
                    if k == 'rgb':
                        del self.received_wall[oldest]
            if self.decoupled:self.input_event.set()
        def fresh_capture(self,ns):
            if not capture_age_valid(ns,self.get_clock().now().nanoseconds,self.max_age):
                return False
            if self.max_age<=0:
                return True
            with self.input_lock:
                received=self.received_wall.get(ns)
            return received is not None and time.monotonic()-received<=self.max_age
        def publish_semantics(self,ns,frame,rows):
            if self.active_confirmation:self.tracker.semantic_memory.update(rows,ns*1e-9)
            report=dict(stamp_ns=ns,rgb_frame=frame,position_frame='map',backend=self.backend,candidates=rows,
                        inference_wall_s=self.semantic_cache.get(ns, (None, None))[1],published_sim_ns=self.get_clock().now().nanoseconds)
            if self.active_confirmation:report['semantic_memory']=self.tracker.semantic_memory.candidates
            msg=String();msg.data=json.dumps(report)
            self.semantic_output.publish(msg)
            if self.evidence_dir:self.record_evidence(dict(kind='semantic',**report))
            with self.input_lock:source=self.queues['rgb'].get(ns)
            if source is not None and ns in self.semantic_cache:
                import cv2
                pixels=rgb_pixels(source)
                for detection,row in zip(self.semantic_cache[ns][0],rows):
                    color=np.array([255,210,30] if row['state']=='unlocalized' else [30,210,255])
                    pixels[detection.mask]=(pixels[detection.mask]*.65+color*.35).astype(np.uint8)
                    x,y,w,h=map(int,detection.box);cv2.rectangle(pixels,(x,y),(x+w,y+h),color.tolist(),1)
                    cv2.putText(pixels,f"person {detection.confidence:.2f} {row['state']}",(max(0,x-5),max(12,y-4)),cv2.FONT_HERSHEY_SIMPLEX,.28,color.tolist(),1)
                m=Image();m.header=source.header;m.height=source.height;m.width=source.width;m.encoding='rgb8';m.step=source.width*3;m.data=pixels.tobytes();self.annotated_output.publish(m)
        def record_evidence(self, row):
            try:
                self.evidence_log.write(json.dumps(row)+'\n')
            except (OSError, ValueError, TypeError) as error:
                self.get_logger().warning(f'Perception evidence write failed: {error}', throttle_duration_sec=5.)
        def save_evidence(self, name, **arrays):
            try:
                np.savez_compressed(self.evidence_dir/name, **arrays)
            except (OSError, ValueError, TypeError) as error:
                self.get_logger().warning(f'Perception snapshot write failed: {error}', throttle_duration_sec=5.)
        def timing(self,ns,status,reason=None):
            now=self.get_clock().now().nanoseconds
            row=dict(stamp_ns=ns,now_ns=now,age_s=(now-ns)*1e-9,received_age_wall_s=time.monotonic()-self.received_wall.get(ns,time.monotonic()),inference_wall_s=self.semantic_cache.get(ns, (None, None))[1],status=status,reason=reason)
            row.update(processing_wall_monotonic_s=time.monotonic(),confirmation_mode=self.confirmation_mode,
                rgb_received_total=self.rgb_received_total,rgb_consumed_total=self.rgb_consumed_total,
                process_calls=self.process_calls,latest_rgb_received_stamp_ns=self.latest_rgb_received_ns,
                last_rgb_consumed_stamp_ns=self.last_rgb_consumed_ns,rgb_queue_length=len(self.queues['rgb']),
                callback_started_sim_ns=self.callback_started_ns,callback_started_wall=self.callback_started_wall,
                callback_elapsed_wall_s=time.monotonic()-self.callback_started_wall if self.callback_started_wall is not None else None,
                previous_callback_sim_gap_s=self.previous_callback_sim_gap,previous_callback_wall_gap_s=self.previous_callback_wall_gap,
                last_callback_wall_s=self.last_callback_wall_s,queue_latest_stamps={k:max(q,default=None) for k,q in self.queues.items()})
            with self.input_lock:
                row.update(pose_samples=len(self.pose_history.samples),
                    pose_latest_s=self.pose_history.samples[-1][0] if self.pose_history.samples else None,
                    selected_capture_pose_bracket_available=self.pose_history.brackets(ns*1e-9),
                    last_bearing_capture_ns=self.bearings.last_ns)
            msg=String();msg.data=json.dumps(row);self.diagnostic_output.publish(msg)
            if self.evidence_dir:self.record_evidence(dict(kind='timing',**row))
        def detect_semantics(self,ns,rgb):
            if ns in self.semantic_cache:
                return self.semantic_cache[ns][0]
            with self.input_lock:
                if self.queues['rgb'].get(ns,rgb) is not rgb:
                    raise ValueError('RGB capture replaced before inference')
                self.inference_rgb_ns=ns
            try:
                pixels=rgb_pixels(rgb)
                self.timing(ns,'inference_start')
                started=time.perf_counter()
                detections=self.detector.detect(pixels)
                elapsed=time.perf_counter()-started
                with self.input_lock:
                    self.semantic_cache[ns]=detections,elapsed
                    if len(self.semantic_cache)>self.input_queue_limit:
                        del self.semantic_cache[min(self.semantic_cache)]
            finally:
                with self.input_lock:self.inference_rgb_ns=None
            self.timing(ns,'inference_complete')
            if self.evidence_dir:
                self.save_evidence(str(ns)+'-rgb-masks.npz',stamp_ns=ns,rgb=pixels,
                    masks=np.array([d.mask for d in detections],bool),boxes=np.array([d.box for d in detections]),scores=np.array([d.confidence for d in detections]))
            return detections
        def flush_review_events(self):
            if not self.active_confirmation:return
            while self.tracker.cancellations:
                self.review_cancel_pub.publish(String(data=json.dumps(self.tracker.cancellations.pop(0))))
            while self.tracker.events:
                row=self.tracker.events.pop(0);self.review_events.publish(String(data=json.dumps(row)))
                self.get_logger().info('Active confirmation '+json.dumps(row))
                if self.evidence_dir:self.record_evidence(dict(kind='active_confirmation',**row))
        def publish_bearings(self,now):
            from .bearing import bearing_packet
            for report in self.bearings.active(now):
                self.bearing_output.publish(String(data=json.dumps(report)))
                if report['persistent'] and not self.registry_mode:
                    self.bearing_pub.publish(Float64MultiArray(data=bearing_packet(report)))
                if self.evidence_dir:self.record_evidence(dict(kind='bearing',**report))
        def bearing_capture(self,ns,queues,history):
            if ns is None or ns<=self.bearings.last_ns or ns not in self.semantic_cache:return
            rgb=queues['rgb'].get(ns);info=queues['ri'].get(ns)
            try:
                if rgb is None or info is None:raise ValueError('missing_rgb_intrinsics_at_capture')
                if self.rgbd_registration_mode=='reproject':
                    from annual_runtime.rgbd_registration import camera_info_model
                    model,_=camera_info_model(info,rgb,self.rgbd_image_mode);intrinsics=model.k
                else:
                    if info.header.frame_id!=rgb.header.frame_id or (info.width,info.height)!=(rgb.width,rgb.height):raise ValueError('rgb_calibration_frame_or_size')
                    if np.any(np.asarray(info.d)!=0) or not camera_uses_full_resolution(info):raise ValueError('rgb_requires_rectification')
                    intrinsics=info.k
                transform=capture_world(self,rgb.header.frame_id,ns,history,Time)
                self.bearings.update(self.semantic_cache[ns][0],intrinsics,transform,rgb.width,rgb.height,ns,
                    self.get_clock().now().nanoseconds*1e-9,self.execution)
                self.publish_bearings(self.get_clock().now().nanoseconds*1e-9)
                self.timing(ns,'bearing_accepted')
            except (ValueError,TransformException) as error:
                self.timing(ns,'bearing_rejected',str(error))
                if self.decoupled:self.tracker.reject_bearing(ns,self.get_clock().now().nanoseconds*1e-9,str(error))
                if self.evidence_dir:self.record_evidence(dict(kind='bearing_rejected',capture_time_s=ns*1e-9,processing_time_s=self.get_clock().now().nanoseconds*1e-9,reason=str(error),semantic_candidate_retained=True))
        def wake_process(self):
            self.expiry_requested=True;self.input_event.set()
        def pending_capture(self):
            from .presence_adapter import select_presence_capture
            now = self.get_clock().now().nanoseconds
            with self.input_lock:
                selected, _, _ = select_presence_capture(self, self.queues, self.pose_history, now)
                return selected is not None
        def start_worker(self):
            def work():
                while not self.worker_stop.is_set():
                    self.input_event.wait(timeout=.1);self.input_event.clear()
                    if self.worker_stop.is_set():break
                    force=self.expiry_requested;self.expiry_requested=False
                    try:
                        if force or self.pending_capture():self.process()
                    except Exception as error:
                        if not self.worker_stop.is_set() and rclpy.ok():
                            self.get_logger().error(f'Perception work item failed: {error!r}', throttle_duration_sec=5.)
                        self.worker_stop.wait(.1)
            self.worker=threading.Thread(target=work,name='perception-inference',daemon=True)
            self.worker.start()
        def stop_worker(self):
            if self.worker is None:
                return True
            self.worker_stop.set();self.input_event.set();self.worker.join(timeout=3.)
            if self.worker.is_alive():
                self.get_logger().warning('Perception worker still busy at shutdown')
                return False
            return True
        def process(self):
            self.process_calls+=1
            now=time.monotonic();sim=self.get_clock().now().nanoseconds
            self.previous_callback_wall_gap=(now-self.callback_started_wall if self.callback_started_wall is not None else None)
            self.previous_callback_sim_gap=((sim-self.callback_started_ns)*1e-9 if self.callback_started_ns is not None else None)
            self.callback_started_wall=now;self.callback_started_ns=sim
            self.timing(self.latest_rgb_received_ns,'process_start')
            try:
                return self._process()
            except Exception as error:
                if not self.worker_stop.is_set() and rclpy.ok():
                    self.get_logger().error(f'Perception processing failed: {error!r}', throttle_duration_sec=5.)
            finally:
                self.last_callback_wall_s=time.monotonic()-now
                if not self.worker_stop.is_set():
                    self.timing(self.last_rgb_consumed_ns if self.last_rgb_consumed_ns>=0 else self.latest_rgb_received_ns,'callback_complete')
        def _process(self):
            if self.active_confirmation:
                now=self.get_clock().now().nanoseconds*1e-9
                with self.input_lock:statuses=self.review_status_queue;self.review_status_queue=[]
                for status in statuses:self.tracker.review_status(status,now)
                self.tracker.advance(now,self.execution)
                self.flush_review_events()
            queues, history = snapshot_inputs(self)
            if self.decoupled:
                process_presence(self,queues,history);return
            common=[ns for ns in queues['rgb'].keys() & queues['ri'].keys() & queues['depth'].keys() & queues['di'].keys()
                    if ns>self.last_ns]
            ns=(newest_bracketed_capture(common,history,self.get_clock().now().nanoseconds,self.max_age)
                if self.require_pose else max(common,default=None))
            # RGB evidence must survive missing depth, calibration or map TF.
            if self.backend=='yolo11n-seg':
                fresh=[ns for ns in queues['rgb'] if ns>self.last_semantic_ns]
                rgb_common=[v for v in set(queues['rgb'])&set(queues['ri']) if v>self.bearings.last_ns]
                rgb_ns=newest_bracketed_capture(rgb_common,history,self.get_clock().now().nanoseconds,self.max_age) if self.require_pose else max(rgb_common,default=None)
                semantic_ns=ns if ns is not None else rgb_ns if rgb_ns is not None else max(fresh,default=None)
                if semantic_ns is None:
                    self.timing(max(queues['rgb'],default=-1),'rgb_waiting','no_new_rgb_or_bracketed_retry')
                elif rgb_ns is None and self.require_pose:
                    self.timing(semantic_ns,'rgb_waiting','no_fresh_rgb_pose_bracket')
                if semantic_ns is not None and semantic_ns not in self.semantic_cache:
                    rgb=queues['rgb'][semantic_ns]
                    try:
                        detections=self.detect_semantics(semantic_ns,rgb)
                        rows,_=localize_masks(detections,None,None,None)
                        self.publish_semantics(semantic_ns,rgb.header.frame_id,rows);self.last_semantic_ns=max(self.last_semantic_ns,semantic_ns)
                    except (ValueError,RuntimeError,OSError,OpenCVError) as e:
                        self.get_logger().warning('RGB inference rejected: '+str(e),throttle_duration_sec=5)
                self.bearing_capture(semantic_ns,queues,history)
            if not common:
                if self.active_confirmation and self.last_semantic_ns>0:
                    self.tracker.reject_capture(self.last_semantic_ns*1e-9,'missing_registered_capture_pair')
                    self.flush_review_events()
                return
            if ns is None:
                self.timing(max(common),'waiting','no_fresh_bracketed_capture')
                if self.active_confirmation:
                    self.tracker.reject_capture(max(common)*1e-9,'no_fresh_bracketed_capture')
                    self.flush_review_events()
                return
            rgb,depth,ri,di=[queues[k][ns] for k in ('rgb','depth','ri','di')]
            try:
                transform=capture_world(self,depth.header.frame_id,ns,history,Time,pose_bracketed=True)
                registration=self.buffer.lookup_transform(rgb.header.frame_id,depth.header.frame_id,Time(nanoseconds=ns))
                require_registered_pair(rgb,depth,ri,di,registration)
                data=np.ndarray((depth.height,depth.width),np.dtype('>f4' if depth.is_bigendian else '<f4'),bytes(depth.data),strides=(depth.step,4)).copy()
                points = None
                if self.backend=='yolo11n-seg':
                    rows,observations=localize_masks(self.detect_semantics(ns,rgb),data,di.k,transform)
                    if not self.fresh_capture(ns):raise ValueError('stale_after_inference')
                    self.publish_semantics(ns,rgb.header.frame_id,rows)
                    self.bearings.mark_localization(ns,rows)
                    self.publish_bearings(self.get_clock().now().nanoseconds*1e-9)
                else:
                    points = cloud(data, di.k, transform)
                    observations = candidates_from_cloud(rgb_pixels(rgb), data, points, self.detector)
                context=dict(semantic_candidate_count=len(rows),localization_rejections=[r.get('depth_reason') for r in rows if r['localization_state']=='unlocalized'],capture_time_s=ns*1e-9,processing_time_s=self.get_clock().now().nanoseconds*1e-9) if self.backend=='yolo11n-seg' else None
                tracks=self.tracker.update(observations,transform[:3,3],ns*1e-9,capture_context=context) if self.active_confirmation else self.tracker.update(observations,transform[:3,3],ns*1e-9)
            except (ValueError,RuntimeError,OSError,OpenCVError,TransformException) as e:
                self.rejected+=1;self.timing(ns,'rejected',str(e))
                if self.active_confirmation:self.tracker.reject_capture(ns*1e-9,str(e))
                if self.backend=='yolo11n-seg' and ns in self.semantic_cache:
                    rows,_=localize_masks(self.semantic_cache[ns][0],None,None,None)
                    for row in rows:row['depth_reason']=str(e)
                    self.publish_semantics(ns,rgb.header.frame_id,rows)
                self.get_logger().warning('Frame rejected: '+str(e),throttle_duration_sec=5);return
            self.last_ns=ns;self.processed+=1
            self.timing(ns,'accepted')
            if self.evidence_dir:
                self.save_evidence(str(ns)+'-geometry.npz',stamp_ns=ns,depth=data,k=di.k,camera_to_map=transform)
            if self.active_review and ns*1e-9-self.last_volume>1.:
                self.volume.add(data, di.k, transform, points=points)
                self.last_volume=ns*1e-9
            now=self.get_clock().now().nanoseconds*1e-9
            execution=self.execution
            if (self.active_review and now-self.last_review>3. and self.review_pub.get_subscription_count()
                    and (self.active_confirmation or review_execution_ready(execution,now))):
                for track in tracks:
                    if self.active_confirmation and not self.tracker.eligible(track,now,execution):continue
                    goal=choose_view(track,transform[:3,3],self.volume,self.previous,
                                     min_horizontal=.8 if self.active_confirmation else 0.)
                    if goal is None:continue
                    base, yaw = review_pose(track.xyz, goal)
                    # Active submit checks this refreshed snapshot together with track eligibility.
                    context=review_request_snapshot(lambda:self.execution,self.get_clock().now,
                                                    validate_execution=not self.active_confirmation)
                    if context is None:break
                    request_stamp,request_execution=context
                    request_now=request_stamp.nanoseconds*1e-9
                    m=PoseStamped();m.header.frame_id='map';m.header.stamp=request_stamp.to_msg()
                    m.pose.position.x,m.pose.position.y,m.pose.position.z=map(float,base)
                    m.pose.orientation.z=float(np.sin(yaw/2));m.pose.orientation.w=float(np.cos(yaw/2))
                    accepted=(self.tracker.review(track,transform[:3,3],goal,request_now,request_execution,request_id=str(request_stamp.nanoseconds))
                              if self.active_confirmation else self.tracker.review(track))
                    if accepted:
                        self.review_pub.publish(m);self.previous.append(goal);self.last_review=request_now
                        self.get_logger().info('Review request '+json.dumps(dict(track=track.id,attempt=track.attempts,stamp_ns=stamp_ns(m.header),goal=base.tolist())))
                    break
            self.flush_review_events()
            msg=String();msg.data=json.dumps({'mode':'active_confirmation' if self.active_confirmation else 'legacy',
                'confirmation_policy':self.confirmation_mode,'active_review':self.active_review,'stamp_ns':ns,'frame':'map',
                'position_representation':'visible_surface_not_body_centre',
                'tracks':[{'id':t.id,'xyz':t.xyz.tolist(),'state':t.state,'attempts':t.attempts,'semantic_views':len(t.semantic_views),
                    **({'reason':t.reason,'qualified_capture_time':t.qualified_capture_time,'completed_token':t.completed_token,
                        'post_review_frames':len(t.post_hits),'request_id':t.request_id,'direct_frames':len(t.direct_hits),
                        'position_time':t.confirmed_time if t.state=='confirmed' else t.last_seen,
                        'position_status':'frozen_confirmed_surface' if t.state=='confirmed' else 'candidate_estimate',
                        'review_closed_time':t.review_closed_time,'review_closed_reason':t.review_closed_reason,
                        'quality_evidence':t.quality_evidence,'rejection_reasons':t.rejection_reasons} if self.active_confirmation else {})} for t in tracks]});self.output.publish(msg)
            self.markers.publish(target_markers(tracks,depth.header.stamp))
    rclpy.init();node=Search()
    executor=(rclpy.executors.SingleThreadedExecutor() if node.decoupled else rclpy.executors.MultiThreadedExecutor(num_threads=2))
    executor.add_node(node)
    if node.decoupled:node.start_worker()
    try:executor.spin()
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        worker_stopped = node.stop_worker()
        executor.shutdown()
        if worker_stopped and node.evidence_log is not None:
            try:
                node.evidence_log.close()
            except OSError as error:
                node.get_logger().warning(f'Perception evidence close failed: {error}')
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
