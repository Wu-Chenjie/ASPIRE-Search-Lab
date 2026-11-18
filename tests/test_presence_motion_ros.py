"""Real ROS RGB/TF path for camera-turn neighbour identity; no depth/GT."""
import json,os,subprocess,sys,time
from pathlib import Path
import numpy as np
import pytest
rclpy=pytest.importorskip('rclpy')
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from scipy.spatial.transform import Rotation
from person_search.presence import association_view,rotated_box

def test_actual_ros_capture_rotation_preserves_second_person_source(tmp_path):
    root=Path(__file__).resolve().parents[1]
    script="""import sys,types,numpy as np
from person_search.segmentation import PersonMask
from person_search.presence import association_view,rotated_box
from scipy.spatial.transform import Rotation
k=[[200,0,160],[0,200,120],[0,0,1]]
a=np.eye(4);b=np.eye(4);b[:3,:3]=Rotation.from_euler('y',.21).as_matrix()
box=rotated_box((125,80,35,90),association_view(k,a,320,240),association_view(k,b,320,240))
class Stub:
 def __init__(self,weights,*,preserve_mask_candidates):
  assert preserve_mask_candidates
 def detect(self,rgb):
  boxes=[(80,80,35,90),(125,80,35,90)] if rgb[0,0,0]==1 else [box]
  return [PersonMask(tuple(x),.9,np.zeros((240,320),bool)) for x in boxes]
sys.modules['person_search.yolo']=types.SimpleNamespace(YoloPersonDetector=Stub)
from person_search.node import main
main()
"""
    env={**os.environ,'PYTHONDONTWRITEBYTECODE':'1','PYTHONPATH':str(root/'src')+os.pathsep+str(root/'src/annual_planner/src')+os.pathsep+os.environ.get('PYTHONPATH','')}
    log=(tmp_path/'node.log').open('w')
    proc=subprocess.Popen([sys.executable,'-c',script,'--ros-args','-p','use_sim_time:=true','-p','detector_backend:=yolo11n-seg','-p','confirmation_mode:=registry'],env=env,stdout=log,stderr=subprocess.STDOUT)
    rclpy.init();node=rclpy.create_node('motion_presence_fixture')
    pub=node.create_publisher(Image,'/drone_0/rgb/image',qos_profile_sensor_data);info_pub=node.create_publisher(CameraInfo,'/drone_0/rgb/camera_info',qos_profile_sensor_data)
    clock_pub=node.create_publisher(Clock,'/clock',10);tf_pub=node.create_publisher(TFMessage,'/tf_static',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    reports=[];events=[];timings=[]
    node.create_subscription(String,'/search/tracks',lambda m:reports.append(json.loads(m.data)),100)
    node.create_subscription(String,'/search/presence_events',lambda m:events.append(json.loads(m.data)),100)
    node.create_subscription(String,'/search/perception_timing',lambda m:timings.append(json.loads(m.data)),100)
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
        assert proc.poll() is None,(tmp_path/'node.log').read_text()
    try:
        start=time.monotonic()
        while not(pub.get_subscription_count() and info_pub.get_subscription_count() and clock_pub.get_subscription_count()):
            spin(.03);assert time.monotonic()-start<12
        for phase in (0,1):
            tf=TransformStamped();tf.header.frame_id='map';tf.child_frame_id='optical'
            q=Rotation.from_euler('y',phase*.21).as_quat();tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w=map(float,q)
            tf_pub.publish(TFMessage(transforms=[tf]));spin(.3)
            for i in range(12):
                clock=Clock();ns=round((10+(phase*12+i)*.2)*1e9);clock.clock.sec=ns//10**9;clock.clock.nanosec=ns%10**9;clock_pub.publish(clock)
                info=CameraInfo();info.header.stamp=clock.clock;info.header.frame_id='optical';info.width=320;info.height=240;info.k=[200.,0,160.,0,200.,120.,0,0,1.];info_pub.publish(info)
                # Distinct best-effort topics have no cross-topic ordering.
                # This rotation fixture requires calibration before its first RGB.
                spin(.05)
                rgb=Image();rgb.header=info.header;rgb.width=320;rgb.height=240;rgb.encoding='rgb8';rgb.step=960;rgb.data=np.full((240,320,3),phase+1,np.uint8).tobytes();pub.publish(rgb);spin(.14)
        (tmp_path/'messages.json').write_text(json.dumps(dict(reports=reports,events=events,timings=timings),indent=2))
        assert all(t['bearing_reason']!='missing_rgb_intrinsics_at_capture' for r in reports for t in r['tracks'])
        tracks=reports[-1]['tracks'];assert len(tracks)==2
        assert all(t['visual_state']=='visually_confirmed' and t['position'] is None for t in tracks)
        assert tracks[1]['capture_count']>tracks[0]['capture_count']
        assert tracks[1]['capture_time_s']>tracks[0]['capture_time_s']
        assert sum(e['event']=='visual_confirmed' for e in events)==2
        assert any(e.get('association_reason')=='unique_rotation_compensated_overlap' for e in events)
    finally:
        proc.terminate()
        try:proc.wait(timeout=5)
        except subprocess.TimeoutExpired:proc.kill();proc.wait(timeout=5)
        log.close();node.destroy_node();rclpy.shutdown()
