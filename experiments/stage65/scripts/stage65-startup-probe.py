"""A real diagnostic topic trigger. No flight/authorization/target messages."""
import json,time,sys
from pathlib import Path
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import String
rclpy.init();node=rclpy.create_node('stage65_startup_capture_probe',parameter_overrides=[Parameter('use_sim_time',value=True)])
pub=node.create_publisher(String,'/drone_0/diagnostics/map_evidence_trigger',10)
end=time.monotonic()+8
while pub.get_subscription_count()<3 or node.get_clock().now().nanoseconds<=0:
    rclpy.spin_once(node,timeout_sec=.05)
    if time.monotonic()>end:raise RuntimeError('capture_three_subscriptions_or_actual_clock_missing')
trigger=dict(schema=1,kind='map_evidence_trigger',id='stage65-startup-probe',drone=0,
    session=sys.argv[2],sim_s=node.get_clock().now().nanoseconds*1e-9,reason='bounded_startup_capture_probe')
for _ in range(3):pub.publish(String(data=json.dumps(trigger)));rclpy.spin_once(node,timeout_sec=.1)
Path(sys.argv[1]).write_text(json.dumps(dict(trigger=trigger,subscriptions=pub.get_subscription_count(),diagnostic_only=True))+'\n')
node.destroy_node();rclpy.shutdown()
