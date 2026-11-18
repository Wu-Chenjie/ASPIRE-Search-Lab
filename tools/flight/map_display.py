"""Read-only RViz view of the occupied cells in the fused native map."""

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Float64MultiArray, Header

from core.planning.execution_safety import NativeSafetyMap


class MapDisplay(Node):
    def __init__(self):
        super().__init__('aspire_live_map_display')
        self.map = NativeSafetyMap()
        self.latest = None
        self.publisher = self.create_publisher(
            PointCloud2, '/search/mapped_cloud',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.create_subscription(Float64MultiArray, '/baseline/safety_map_0', self.receive, 1)
        self.create_timer(0.5, self.publish_latest)

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
