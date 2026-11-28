"""RViz views of measured surfaces and native occupied cells."""

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32, read_points_numpy
from std_msgs.msg import Float64MultiArray, Header

from core.planning.execution_safety import NativeSafetyMap


class MapDisplay(Node):
    def __init__(self):
        super().__init__('aspire_live_map_display')
        self.map = NativeSafetyMap()
        self.latest = None
        self.surfaces = {}
        self.surface_header = None
        snapshot_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(
            PointCloud2, '/search/mapped_cloud', snapshot_qos,
        )
        self.surface_publisher = self.create_publisher(PointCloud2, '/search/surface_cloud', snapshot_qos)
        self.create_subscription(PointCloud2, '/drone_0/surface_hits', self.receive_hits, 10)
        self.create_subscription(Float64MultiArray, '/baseline/safety_map_0', self.receive, 1)
        self.create_timer(0.5, self.publish_latest)
        self.create_timer(0.5, self.publish_surfaces)

    def receive_hits(self, message):
        if not message.width:
            return
        xyz = read_points_numpy(message, field_names=['x', 'y', 'z'])
        # Keep one measured coordinate per 5 cm display cell.
        cells = np.floor(xyz.astype(np.float64) / 0.05).astype(np.int64)
        previous_size = len(self.surfaces)
        for cell, point in zip(cells.tolist(), xyz.tolist()):
            self.surfaces.setdefault(tuple(cell), point)
        if len(self.surfaces) != previous_size:
            self.surface_header = message.header

    def publish_surfaces(self):
        if self.surface_header is not None:
            points = np.asarray(list(self.surfaces.values()), dtype=np.float32)
            self.surface_publisher.publish(create_cloud_xyz32(self.surface_header, points))
            self.surface_header = None

    def receive(self, message):
        self.latest = message

    def publish_latest(self):
        message, self.latest = self.latest, None
        if message is None:
            return
        try:
            if not self.map.ingest(message.data):
                return
        except ValueError as exc:
            self.get_logger().warning(f'Skipping invalid map snapshot: {exc}', throttle_duration_sec=5.0)
            return

        # Native state 2 is occupied. The inflation flag (4) is independent.
        cells = np.argwhere((self.map.state & 3) == 2)
        centers = self.map.origin + (cells + 0.5) * self.map.resolution
        header = Header(frame_id='map')
        stamp_ns = round(self.map.stamp * 1_000_000_000)
        header.stamp.sec, header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
        self.publisher.publish(create_cloud_xyz32(header, centers.astype(np.float32)))


def main():
    rclpy.init()
    node = MapDisplay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
