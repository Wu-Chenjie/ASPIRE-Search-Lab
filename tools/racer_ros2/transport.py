"""Actual observations and authorized RACER proposals over native ROS2 DDS.

No TCP, ROS1 master, ROS1 client library, or synthetic clock. The planner shares
Gazebo's /clock with the existing executor and PX4 ROS2 bridge.
"""
from contextlib import suppress
import json
import math
import os
from pathlib import Path
import queue
import signal
import subprocess
from threading import Lock

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float64MultiArray
from racer_interfaces.msg import Bspline
from .configuration import configuration, native_node_command


def stamp(t):
    if t<0:raise ValueError('Negative observation stamp')
    ns=round(t*1e9)
    return Time(sec=ns//1000000000,nanosec=ns%1000000000)


_INPUT_ERRORS = (KeyError, TypeError, ValueError, OverflowError, AssertionError)
_STREAM_FILES = {
    'raw': 'trajectories.jsonl',
    'input_stream': 'measured-input.jsonl.gz',
    'stop_feedback_log': 'stop-feedback.jsonl',
}


def _native_id(value, minimum=-1):
    return minimum <= value <= 2**31-1 and int(value)==value


class NativeDDSTransport(Node):
    def __init__(self,output,fleet_size=2):
        super().__init__('racer_native_transport')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
        if fleet_size not in (1,2):raise ValueError('Supported fleet sizes: one or two')
        self.fleet_size=fleet_size
        self.output=Path(output);self.output.mkdir(parents=True,exist_ok=False)
        self.incoming=queue.Queue();self.outgoing=queue.Queue(512)
        self.drops=0;self.error=None
        self.recording_failures={}
        self.children=[];self.logs=[];self.metadata={};self.waiting={};self.ready=[False]*fleet_size
        self.triggered=False;self.started=False;self.closing=False;self.publishers_by_kind={}
        self.counts=dict(odom=0,frame=0,feedback=0,execution_grid=0)
        self.fast_stop_feedback=os.environ.get('ASPIRE_FAST_STOP_HANDOFF')=='1'
        if self.fast_stop_feedback:
            self.feedback_pending={}
            self.feedback_lock=Lock()
            self.feedback_guard=self.create_guard_condition(self.flush_feedback)
            self.stop_feedback_log=self._open_stream('stop_feedback_log')
            self.last_stop_feedback={}
        self.raw=self._open_stream('raw')
        self.map_timing=None
        if os.environ.get('ASPIRE_SAFETY_MAP_TIMING')=='1':
            from annual_runtime.control_observation import ObservationLog
            self.map_timing=self._evidence('native-map-transport', ObservationLog, self.output,0,'native-map-transport')
        self.input_stream=None
        if os.environ.get('RACER_RECORD_INPUT')=='1':
            import gzip
            self.input_stream=self._evidence(
                _STREAM_FILES['input_stream'], gzip.open,
                self.output/_STREAM_FILES['input_stream'], 'xt', compresslevel=1)
        for i in range(fleet_size):
            for kind,msg_type in [('odom',Odometry),('cloud',PointCloud2),('sensor',PoseStamped),('feedback',Float64MultiArray)]:
                self.publishers_by_kind[kind,i]=self.create_publisher(msg_type,f'/baseline/{kind}_{i}',20)
            self.create_subscription(Float64MultiArray,f'/baseline/boundary_{i}',lambda m,i=i:self.boundary(i,m),20)
            self.create_subscription(Bspline,f'/baseline/bspline_{i}',lambda m,i=i:self.proposal(i,m),20)
        self.goal=self.create_publisher(PoseStamped,'/move_base_simple/goal',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(.02,self.poll)

    def send(self,row):
        if not isinstance(row, dict):
            self.drops+=1
            return
        if self.fast_stop_feedback and row.get('type')=='feedback':
            # Feedback is a complete state sample. Keep the latest one per
            # aircraft outside the odometry/grid queue. Its ROS stamp stays
            # the gateway's stamp and therefore still expires at the receiver.
            try:
                i=self._drone_index(row)
                data=self._feedback_data(row)
                stamp(row['time'])
            except _INPUT_ERRORS:
                self.drops+=1
                return
            with self.feedback_lock:
                pending=self.feedback_pending.get(i)
                if pending is None or pending['data'][5]<=data[5]:
                    self.feedback_pending[i]=dict(row,data=data)
            self.feedback_guard.trigger()
            return
        timing=self.map_timing if row.get('type')=='frame' else None
        if timing:
            with suppress(*_INPUT_ERRORS):
                self._evidence('native-map-transport', timing.event, 'cloud_enqueue',
                               drone=row['drone'], capture_ns=round(row['time']*1e9))
        try:self.outgoing.put_nowait(row)
        except queue.Full:
            self.drops+=1
            if timing:
                with suppress(*_INPUT_ERRORS):
                    self._evidence('native-map-transport', timing.event, 'cloud_enqueue_dropped',
                                   drone=row['drone'], capture_ns=round(row['time']*1e9))

    def flush_feedback(self):
        with self.feedback_lock:
            batch=self.feedback_pending
            self.feedback_pending={}
        if self.started and not self.closing:
            try:
                for row in batch.values():self._publish_input(row)
            except Exception as exc:
                self.error=str(exc)

    def start(self,hello):
        share=Path(get_package_share_directory('racer_core'))
        prefix=Path(get_package_prefix('racer_core'))
        configs=configuration(hello,self.output,share/'config/single_drone_planner.xml')
        # Solvers start first; each has an independent process and private files.
        configs.sort(key=lambda c:c['executable']=='exploration_node')
        for cfg in configs:
            args=native_node_command(cfg,self.output,prefix/'lib/racer_core')
            log_name=cfg['name']+'.log'
            log=self._evidence(log_name, (self.output/log_name).open, 'x')
            if log is not None:self.logs.append((log_name,log))
            # Stay in the launch process group so a group shutdown also reaches
            # native children if the Python adapter is interrupted during cleanup.
            self.children.append(subprocess.Popen(
                args,stdout=log if log is not None else subprocess.DEVNULL,stderr=subprocess.STDOUT))
        self.started=True
        self._record_json('protocol.json', {'inputs':hello,'transport':'native_ros2_dds',
            'variant':'native-core','clock':'shared Gazebo /clock','actuation':'common authorized ROS2 executor + locked PX4',
            'nodes':configs})

    def boundary(self,i,msg):
        try:
            data=list(map(float,msg.data))
        except (AttributeError, *_INPUT_ERRORS):
            return
        if (len(data) not in (4,7,10) or not all(map(math.isfinite,data)) or
                not _native_id(data[0],0) or not _native_id(data[1]) or
                (data[2]<0. and data[2]!=-1.) or data[3]<0.):return
        key=(i,int(data[0]))
        metadata=dict(from_id=int(data[1]),boundary_progress=data[2],planning_wall_s=data[3])
        if len(data)>=7:
            from annual_runtime.static_cancel_gateway import task_from_wire
            try:metadata['static_task']=task_from_wire([1,data[4],data[5],data[0],data[6],1])
            except ValueError:return
        if len(data)==10:
            counter,sequence,capture=data[7:]
            if not all(0<v<2**53 and int(v)==v for v in (counter,sequence)) or capture<0.:return
            metadata['static_source_frame']=dict(counter=int(counter),sequence=int(sequence),capture_s=capture)
        self.metadata[key]=metadata
        if key in self.waiting:self.deliver(key,self.waiting.pop(key))

    def proposal(self,i,m):
        row=dict(type='proposal',drone=i,time=self.get_clock().now().nanoseconds*1e-9,
            start_t=m.start_time.sec+m.start_time.nanosec*1e-9,order=m.order,traj_id=m.traj_id,
            knots=list(m.knots),points=[[p.x,p.y,p.z] for p in m.pos_pts],yaw_points=list(m.yaw_pts),yaw_dt=m.yaw_dt)
        key=(i,m.traj_id)
        if key in self.metadata:self.deliver(key,row)
        else:self.waiting[key]=row

    def deliver(self,key,row):
        row.update(self.metadata.pop(key))
        self.incoming.put(row)
        self._record('raw', row)
        # Bound unmatched data; a missing pair must never yield an executable curve.
        if len(self.waiting)>100:self.waiting.pop(next(iter(self.waiting)))

    def poll(self):
        if self.closing:return
        try:
            if any(p.poll() is not None for p in self.children):raise RuntimeError('A native ROS2 RACER process exited')
            for _ in range(100):
                try:row=self.outgoing.get_nowait()
                except queue.Empty:break
                if isinstance(row,dict) and row.get('type')=='hello':
                    if not self.started:self.start(row)
                    continue
                if not self.started:continue
                self._publish_input(row)
            if self.started and all(self.ready) and not self.triggered and self.goal.get_subscription_count()>=self.fleet_size:
                m=PoseStamped();m.header.stamp=self.get_clock().now().to_msg();m.pose.position.z=1.5;m.pose.orientation.w=1.
                self.goal.publish(m);self.triggered=True
        except Exception as exc:
            self.error=str(exc)

    def _publish_input(self, row):
        # A malformed sample must not interrupt the queue or the live node.
        try:
            self.publish_row(row)
        except _INPUT_ERRORS:
            self.drops+=1

    def _evidence(self, filename, operation, *args, **kwargs):
        try:
            return operation(*args, **kwargs)
        except (OSError,TypeError,ValueError,OverflowError) as exc:
            if filename not in self.recording_failures:
                self.recording_failures[filename]=str(exc)
                with suppress(OSError, RuntimeError):
                    self.get_logger().warning(f'Native evidence incomplete for {filename} ({exc})')

    def _open_stream(self, name):
        filename=_STREAM_FILES[name]
        return self._evidence(filename, (self.output/filename).open, 'x', buffering=1)

    def _close_stream(self, name):
        stream=getattr(self,name)
        if stream is not None:
            setattr(self,name,None)
            self._evidence(_STREAM_FILES[name], stream.close)

    def _record_json(self, filename, row):
        self._evidence(filename, lambda: (self.output/filename).write_text(json.dumps(row,indent=2)+'\n'))

    def _record(self, name, row):
        stream=getattr(self,name)
        if stream is None:return
        filename=_STREAM_FILES[name]
        line=self._evidence(filename, json.dumps, row, separators=(',',':'), allow_nan=False)
        if line is not None and self._evidence(filename, stream.write, line+'\n') is None:
            self._close_stream(name)

    def _feedback_data(self,row):
        data=list(map(float,row['data']))
        if (len(data) not in (7,8,9) or not all(map(math.isfinite,data)) or
                not _native_id(data[0]) or not _native_id(data[3]) or
                data[1]<0. or data[2] not in (0.,1.) or data[4] not in (-1.,0.,1.) or
                data[5]<0. or data[6]<0. or not isinstance(row['ready'],bool)):
            raise ValueError('Malformed execution feedback')
        if len(data)>7 and (data[0]!=-1. or data[2]!=0. or not _native_id(data[7],0)):
            raise ValueError('Malformed stopped execution receipt')
        if len(data)==9 and data[8]<0.:
            raise ValueError('Malformed collision stop request timestamp')
        return data

    def _drone_index(self,row):
        i=row['drone']
        if not isinstance(i,int) or not 0<=i<self.fleet_size:raise ValueError('Invalid native drone index')
        return i

    def publish_row(self,row):
        kind=row['type'];i=self._drone_index(row);t=stamp(row['time'])
        if kind=='execution_grid':return # Native core owns its map; no executor-grid algorithm patch.
        if kind=='odom':
            position,quaternion,velocity=(tuple(map(float,row[k])) for k in ('position','quaternion','velocity'))
            if not all(map(math.isfinite,position+quaternion+velocity)):
                raise ValueError('Nonfinite odometry')
            m=Odometry();m.header.stamp=t;m.header.frame_id='world';m.child_frame_id='world_velocity'
            m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z=position
            m.pose.pose.orientation.x,m.pose.pose.orientation.y,m.pose.pose.orientation.z,m.pose.pose.orientation.w=quaternion
            m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.linear.z=velocity
            self.publishers_by_kind['odom',i].publish(m)
        elif kind=='frame':
            origin=tuple(map(float,row['origin']))
            if not all(map(math.isfinite,origin)):raise ValueError('Nonfinite measured origin')
            m=PoseStamped();m.header.stamp=t;m.header.frame_id='world';m.pose.orientation.w=1.
            m.pose.position.x,m.pose.position.y,m.pose.position.z=origin
            points=np.asarray(row['points'],dtype='<f4').reshape(-1,3)
            if not np.isfinite(points).all():raise ValueError('Nonfinite measured cloud')
            cloud=PointCloud2();cloud.header=m.header;cloud.height=1;cloud.width=len(points)
            cloud.point_step=12;cloud.row_step=12*len(points);cloud.is_dense=True
            cloud.fields=[PointField(name=k,offset=j*4,datatype=PointField.FLOAT32,count=1) for j,k in enumerate(('x','y','z'))]
            cloud.data=points.tobytes()
            self.publishers_by_kind['sensor',i].publish(m)
            if self.map_timing:
                self._evidence('native-map-transport', self.map_timing.event, 'cloud_publish',
                               drone=i, capture_ns=t.sec*10**9+t.nanosec)
            self.publishers_by_kind['cloud',i].publish(cloud)
        elif kind=='feedback':
            data=self._feedback_data(row)
            self.publishers_by_kind['feedback',i].publish(Float64MultiArray(data=data))
            self.ready[i]=row['ready']
            if self.fast_stop_feedback and len(data)==9:
                key=tuple(data[7:9])
                if self.last_stop_feedback.get(i)!=key:
                    self.last_stop_feedback[i]=key
                    record=dict(drone=i,native_id=key[0],requested_at=key[1],feedback_stamp_s=row['time'],
                                published_s=self.get_clock().now().nanoseconds*1e-9)
                    self._record('stop_feedback_log', record)
        else:raise ValueError('Unsupported native input '+kind)
        self.counts[kind]+=1
        self._record('input_stream', row)

    def close(self):
        if self.closing:return
        self.closing=True
        if self.map_timing:self._evidence('native-map-transport', self.map_timing.close)
        for child in self.children:
            if child.poll() is None:child.send_signal(signal.SIGINT)
        for child in self.children:
            try:child.wait(timeout=5)
            except subprocess.TimeoutExpired:child.kill();child.wait()
        for filename,log in self.logs:self._evidence(filename, log.close)
        self._close_stream('raw')
        if self.fast_stop_feedback:self._close_stream('stop_feedback_log')
        self._close_stream('input_stream')
        counts=dict(self.counts)
        if self.recording_failures:counts['recording_failures']=dict(self.recording_failures)
        self._record_json('gateway-counts.json', counts)
