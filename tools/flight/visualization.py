"""Read-only RViz overlays from actual estimate, authorized path and detections."""
import json,math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from nav_msgs.msg import Odometry,Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker,MarkerArray

class Display(Node):
 def __init__(self):
  super().__init__('search_flight_display');self.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
  self.path=Path();self.path.header.frame_id='map';self.tracks=[]
  self.pathpub=self.create_publisher(Path,'/search/estimated_path',10);self.planpub=self.create_publisher(Path,'/search/authorized_path',10);self.markerpub=self.create_publisher(MarkerArray,'/search/display',10)
  self.create_subscription(Odometry,'/drone_0/estimated_odometry',self.odom,qos_profile_sensor_data)
  self.create_subscription(String,'/search/tracks',lambda m:setattr(self,'tracks',json.loads(m.data)['tracks']),10)
  self.create_subscription(String,'/drone_0/view_command',self.command,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
 def command(self,m):
  row=json.loads(m.data)
  if 'path' not in row:return
  p=Path();p.header.frame_id='map';p.header.stamp=self.get_clock().now().to_msg()
  for xyz in row['path']:
   v=PoseStamped();v.header=p.header;v.pose.position.x,v.pose.position.y,v.pose.position.z=map(float,xyz);v.pose.orientation.w=1.;p.poses.append(v)
  self.planpub.publish(p)
 def odom(self,m):
  p=PoseStamped();p.header=m.header;p.header.frame_id='map';p.pose=m.pose.pose
  if not self.path.poses or math.dist([p.pose.position.x,p.pose.position.y,p.pose.position.z],[self.path.poses[-1].pose.position.x,self.path.poses[-1].pose.position.y,self.path.poses[-1].pose.position.z])>.025:self.path.poses.append(p)
  self.path.header.stamp=m.header.stamp;self.pathpub.publish(self.path)
  marks=[];v=Marker();v.header=p.header;v.ns='estimated_aircraft';v.id=0;v.type=Marker.ARROW;v.pose=p.pose;v.scale.x=.6;v.scale.y=.15;v.scale.z=.15;v.color.b=1.;v.color.g=.5;v.color.a=1.;marks.append(v)
  for t in self.tracks:
   # A visual track can precede a measured 3D position.
   if t['xyz'] is None:continue
   v=Marker();v.header=p.header;v.ns='target_labels';v.id=t['id'];v.type=Marker.TEXT_VIEW_FACING;v.pose.position.x,v.pose.position.y,v.pose.position.z=map(float,t['xyz']);v.pose.position.z+=.35;v.pose.orientation.w=1.;v.scale.z=.18;v.color.a=1.;v.color.r=v.color.g=v.color.b=1.;v.text=f"#{t['id']} {t['state']}\n({t['xyz'][0]:.2f}, {t['xyz'][1]:.2f}, {t['xyz'][2]:.2f})";marks.append(v)
  self.markerpub.publish(MarkerArray(markers=marks))

def main():
 rclpy.init();n=Display()
 try:rclpy.spin(n)
 except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
 finally:
  n.destroy_node()
  if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
