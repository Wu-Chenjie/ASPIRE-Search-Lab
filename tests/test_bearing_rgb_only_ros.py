"""RGB-only interface fixture, not detector accuracy or autonomous flight."""
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import numpy as np
import pytest
rclpy=pytest.importorskip('rclpy')
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String,Float64MultiArray

def test_rgb_only_without_any_depth_produces_persistent_world_bearing(tmp_path):
    root=Path(__file__).resolve().parents[1]
    script='''import sys,types,numpy as np
from person_search.segmentation import PersonMask
class Stub:
 def __init__(self,*args):pass
 def detect(self,rgb):
  mask=np.zeros((240,320),bool);mask[80:160,150:170]=True
  return [PersonMask((140,70,40,100),.9,mask)]
sys.modules['person_search.yolo']=types.SimpleNamespace(YoloPersonDetector=Stub)
from person_search.node import main
main()
'''
    env={**os.environ,'PYTHONDONTWRITEBYTECODE':'1','ROS_DOMAIN_ID':'183','PYTHONPATH':str(root/'src')+os.pathsep+str(root/'src/annual_planner/src')+os.pathsep+os.environ.get('PYTHONPATH','')}
    log=(tmp_path/'node.log').open('w')
    proc=subprocess.Popen([sys.executable,'-c',script,'--ros-args','-p','use_sim_time:=true','-p','detector_backend:=yolo11n-seg','-p','confirmation_mode:=hybrid','-p','require_estimated_pose:=true'],env=env,stdout=log,stderr=subprocess.STDOUT)
    rclpy.init(domain_id=183);node=rclpy.create_node('bearing_rgb_only_fixture')
    rgbpub=node.create_publisher(Image,'/drone_0/rgb/image',qos_profile_sensor_data);infopub=node.create_publisher(CameraInfo,'/drone_0/rgb/camera_info',qos_profile_sensor_data)
    odompub=node.create_publisher(Odometry,'/drone_0/estimated_odometry',qos_profile_sensor_data);clockpub=node.create_publisher(Clock,'/clock',10)
    tfpub=node.create_publisher(TFMessage,'/tf_static',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    packets=[];semantics=[];timings=[]
    node.create_subscription(String,'/search/perception_timing',lambda m:timings.append(json.loads(m.data)),100)
    node.create_subscription(Float64MultiArray,'/search/bearing_evidence',lambda m:packets.append(list(m.data)),30)
    node.create_subscription(String,'/search/semantic_candidates',lambda m:semantics.append(json.loads(m.data)),30)
    try:
        start=time.monotonic()
        while not (rgbpub.get_subscription_count() and infopub.get_subscription_count() and odompub.get_subscription_count() and clockpub.get_subscription_count() and tfpub.get_subscription_count()):
            rclpy.spin_once(node,timeout_sec=.02);assert proc.poll() is None;assert time.monotonic()-start<10
        for index in range(70):
            sim=10.+index*.2;clock=Clock();clock.clock.sec=int(sim);clock.clock.nanosec=int(round((sim-int(sim))*1e9));clockpub.publish(clock)
            tf=TransformStamped();tf.header.frame_id='map';tf.child_frame_id='rgb_optical';tf.transform.translation.z=1.5
            tf.transform.rotation.x=-.5;tf.transform.rotation.y=.5;tf.transform.rotation.z=-.5;tf.transform.rotation.w=.5;tfpub.publish(TFMessage(transforms=[tf]))
            for stamp in [sim-.01,sim+.01]:
                m=Odometry();m.header.stamp.sec=int(stamp);m.header.stamp.nanosec=int(round((stamp-int(stamp))*1e9));m.pose.pose.orientation.w=1.;m.pose.pose.position.z=1.5;odompub.publish(m)
            info=CameraInfo();info.header.stamp=clock.clock;info.header.frame_id='rgb_optical';info.width=320;info.height=240;info.k=[160.,0,160.,0,160.,120.,0,0,1.];info.d=[0.]*5;infopub.publish(info)
            rgb=Image();rgb.header=info.header;rgb.width=320;rgb.height=240;rgb.encoding='rgb8';rgb.step=960;rgb.data=np.ones((240,320,3),np.uint8).tobytes();rgbpub.publish(rgb)
            until=time.monotonic()+.10
            while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.01)
            assert proc.poll() is None,(tmp_path/'node.log').read_text()
            if packets:break
        assert packets,(tmp_path/'node.log').read_text()
        assert all(len(p)==17 and p[14]==0 for p in packets)
        assert any(r['candidates'] and r['candidates'][0]['xyz'] is None for r in semantics)
        assert packets[-1][12]>=3
    finally:
        (tmp_path/'interface-diagnostics.json').write_text(json.dumps(dict(packets=packets,semantics=semantics,timings=timings),indent=2))
        proc.terminate()
        try:proc.wait(timeout=5)
        except subprocess.TimeoutExpired:proc.kill();proc.wait(timeout=5)
        log.close();node.destroy_node();rclpy.shutdown()
