#!/usr/bin/env python3
"""Integrate actual Gazebo calibrated depth images into a private 3D occupancy map."""
import json
import os
import uuid
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from annual_runtime.pose_history import PoseHistory
from annual_runtime.flight_session import FlightSessionFault
from annual_runtime.depth_projection import project_depth_message, SENSOR_TRANSLATION
from std_msgs.msg import String
from core.exploration.voxel_mapping import VoxelMap
from .map_evidence import bind_capture,close_capture,mapper_received,mapper_before,mapper_after


# X500 base_link_collision_0 (0.35355x0.35355x0.05m, centered at z=.007).
_BODY_LATERAL = np.linspace(-.1767766953, .1767766953, 5)
_BODY_SAMPLES = np.array(np.meshgrid(_BODY_LATERAL, _BODY_LATERAL, [-.018,.007,.032])).reshape(3,-1).T
from .control_observation import ObservationLog


class PointCloudMapper(Node):
    def __init__(self):
        super().__init__('pointcloud_mapper')
        self.publish_native_frames=bool(self.declare_parameter('publish_native_frames',False).value)
        self.publish_native_dds=self.declare_parameter('publish_native_dds',False).value
        self.session = uuid.uuid4().hex
        self.id = int(self.declare_parameter('drone_id', 0).value)
        timing_dir=self.declare_parameter('control_timing_dir','').value
        self.map_timing=(ObservationLog(timing_dir,self.id,'depth-map') if timing_dir and
                         os.environ.get('ASPIRE_SAFETY_MAP_TIMING')=='1' else None)
        bind_capture(self,'mapper')
        self.flight_fault = FlightSessionFault(self.id)
        bounds = json.loads(self.declare_parameter('bounds', '[]').value)
        self.map = VoxelMap(bounds); self.belief = np.zeros(self.map.shape, np.float32); self.ego_cells = set(); self.poses = PoseHistory(); self.pending = deque(maxlen=8); self.sequence = 0; self.full_requested = False; self.last = -10.
        q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(String, f'/drone_{self.id}/observation', q)
        if self.publish_native_dds:
            self.native_pose_pub=self.create_publisher(PoseStamped,f'/baseline/sensor_{self.id}',20)
            self.native_cloud_pub=self.create_publisher(PointCloud2,f'/baseline/cloud_{self.id}',20)
        self.create_subscription(String, f'/drone_{self.id}/flight_status', self.flight_status, 1)
        self.create_subscription(String, f'/drone_{self.id}/map_bootstrap', lambda m: setattr(self, 'full_requested', True), q)
        self.create_subscription(Odometry, f'/drone_{self.id}/estimated_odometry', self.odom, qos_profile_sensor_data)
        self.calibration = None
        self.create_subscription(CameraInfo, f'/drone_{self.id}/depth/camera_info', self.camera_info, qos_profile_sensor_data)
        self.create_subscription(Image, f'/drone_{self.id}/depth/image', self.cloud, qos_profile_sensor_data)

    def destroy_node(self):
        close_capture(self)
        if self.map_timing:self.map_timing.close()
        return super().destroy_node()

    def camera_info(self, m):
        self.calibration = m

    def flight_status(self, m):
        try:
            packet=json.loads(m.data)
        except (ValueError,TypeError):
            return
        if self.flight_fault.observe(packet,self.get_clock().now().nanoseconds*1e-9):
            # Preserve accumulated real evidence for inspection, but do not mix
            # new rays/queued poses into a retired coordinate session.
            self.poses=PoseHistory(); self.pending.clear(); self.ego_cells.clear()

    def odom(self, m):
        if self.flight_fault.reason: return
        p = m.pose.pose.position; q = m.pose.pose.orientation
        position = np.array([p.x, p.y, p.z])
        if not self.poses.add(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                              position, [q.x,q.y,q.z,q.w]):
            return
        rotation = self.poses.samples[-1][2].as_matrix()
        # Track body-traversed cells separately from measured depth evidence.
        cells = self.map.indices(_BODY_SAMPLES@rotation.T+position)
        cells = cells[np.all((cells >= 0) & (cells < self.map.shape), axis=1)]
        self.ego_cells.update(map(tuple, cells.tolist()))
        self.drain()

    def cloud(self, m):
        if self.flight_fault.reason: return
        stamp = (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        if stamp-self.last < .35:
            return
        if self.map_timing:self.map_timing.event('depth_received',capture_ns=round(stamp*1e9),
            sim_ns=self.get_clock().now().nanoseconds)
        mapper_received(self,m)
        self.pending.append(m)
        self.drain()

    def drain(self):
        while self.pending:
            m = self.pending[0]
            stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            if not self.poses.passed(stamp):
                return  # Wait for the right bracket; never use the last pose as extrapolation.
            self.pending.popleft()
            pose = self.poses.at(stamp)
            if pose is not None and stamp-self.last >= .35:
                self.integrate(m, stamp, pose)

    def integrate(self, m, stamp, pose):
        timing=self.map_timing
        if timing:timing.event('mapper_start',capture_ns=round(stamp*1e9))
        position, attitude, bracket = pose
        rotation = attitude.as_matrix()
        if self.calibration is None or (self.calibration.width, self.calibration.height) != (m.width, m.height):
            return
        try:
            points, hits = project_depth_message(m, self.calibration.k)
        except ValueError as error:
            self.get_logger().error(str(error)); return
        n = len(points)
        if not n:
            return
        evidence=mapper_before(self,m,stamp,pose)
        origin = position+rotation@SENSOR_TRANSLATION
        world = points@rotation.T+origin
        cells, values = self.map.integrate(origin, world, hits)
        ray_cell_count = len(cells)
        key, occupied = tuple(cells.T), values == 1
        belief = np.clip(self.belief[key]+np.where(occupied, 1.2, -.45), -3., 3.).astype(self.belief.dtype)
        np.maximum(belief, 1.2, out=belief, where=occupied)
        self.belief[key] = belief
        values = (belief > .5).astype(np.int8)
        self.map.update(cells, values)
        measured_cells, measured_values = cells.tolist(), values.tolist()
        indices, observations = measured_cells, measured_values
        full_snapshot = self.full_requested
        ego_cleared = []
        if self.ego_cells:
            ego = np.array(sorted(self.ego_cells))
            # Body samples do not authorize erasing externally measured obstacles.
            ego = ego[self.map.state[tuple(ego.T)] != 1]
            ego_cleared = ego.tolist()
            self.belief[tuple(ego.T)] = -3.; self.map.update(ego, np.zeros(len(ego), np.int8))
            if len(ego) and not full_snapshot:
                ray_ids = np.ravel_multi_index(key, self.map.shape)
                ego_ids = np.ravel_multi_index(tuple(ego.T), self.map.shape)
                extra = ego[~np.isin(ego_ids, ray_ids)].tolist()
                if extra:
                    indices, observations = measured_cells+extra, measured_values+[0]*len(extra)
        if full_snapshot:
            cells = np.argwhere(self.map.state != -1)
            indices, observations = cells.tolist(), self.map.state[tuple(cells.T)].tolist()
            self.full_requested = False
        self.ego_cells.clear()
        self.sequence += 1; self.last = stamp
        packet = dict(schema='annual.observation/2', dimensions=3, resolution=self.map.resolution,
                      source=self.id, sensor_session=self.session, sequence=self.sequence, time=stamp, indices=indices, values=observations,
                      measured_indices=measured_cells, measured_values=measured_values, full_snapshot=full_snapshot,
                      ego_cleared_indices=ego_cleared,
                      origin=self.map.origin.tolist(), shape=list(map(int, self.map.shape)),
                      sensor='gz_depth', pose_source='px4_ekf2', point_count=n,
                      integration_completed=True, ray_cell_count=ray_cell_count, map_version=int(self.map.version),
                      sensor_position=position.tolist(), sensor_yaw=float(np.arctan2(rotation[1,0],rotation[0,0])), pose_time=stamp, pose_bracket=list(bracket), pose_interpolation='linear_slerp')
        if self.publish_native_dds or self.publish_native_frames:
            native_points=points*np.where(hits,1.,4.51/4.5)[:,None]
            world_points=native_points@rotation.T+origin
            if self.publish_native_dds:
                pose_message=PoseStamped()
                pose_message.header.stamp=m.header.stamp
                pose_message.header.frame_id='world'
                pose_message.pose.orientation.w=1.
                pose_message.pose.position.x,pose_message.pose.position.y,pose_message.pose.position.z=map(float,origin)
                cloud=PointCloud2(header=pose_message.header,height=1,width=len(world_points),
                    point_step=12,row_step=12*len(world_points),is_dense=True,
                    fields=[PointField(name=axis,offset=4*i,datatype=PointField.FLOAT32,count=1)
                            for i,axis in enumerate('xyz')],data=np.asarray(world_points,dtype='<f4').tobytes())
                self.native_pose_pub.publish(pose_message)
                if timing:timing.event('cloud_publish',capture_ns=round(stamp*1e9))
                self.native_cloud_pub.publish(cloud)
            else:
                packet['native_frame']=dict(origin=origin.tolist(),points=world_points.tolist())
        message=String(data=json.dumps(packet, separators=(',', ':')))
        if timing:timing.event('mapper_publish',capture_ns=round(stamp*1e9),
            sim_ns=self.get_clock().now().nanoseconds,sequence=self.sequence)
        self.pub.publish(message)
        mapper_after(self,evidence,packet,points,hits,origin,world)


def main():
    rclpy.init(); node = PointCloudMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__ == '__main__':
    main()
