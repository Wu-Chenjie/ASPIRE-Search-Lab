"""Actual DDS/TF interface with explicit detector and execution test doubles.

This proves wiring and evidence ordering, not autonomous flight or YOLO accuracy.
"""
import json
import os
import subprocess
import sys
import time
from pathlib import Path
import numpy as np
import pytest
rclpy=pytest.importorskip('rclpy')
from rclpy.qos import QoSProfile,DurabilityPolicy
from sensor_msgs.msg import Image,CameraInfo
from geometry_msgs.msg import TransformStamped,PoseStamped
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String


def test_active_confirmation_ros_parameter_goals_and_completion_events(tmp_path):
    root=Path(__file__).resolve().parents[1]
    script='''import sys,types,numpy as np
from person_search.segmentation import PersonMask
class Stub:
 def __init__(self,*args):pass
 def runtime_diagnostics(self):return {'detector_fixture':True}
 def detect(self,rgb):
  if not rgb.any():return []
  m=np.zeros(rgb.shape[:2],bool);m[90:150,145:175]=True
  return [PersonMask((145,90,30,60),.4,m)]
sys.modules['person_search.yolo']=types.SimpleNamespace(YoloPersonDetector=Stub)
from person_search import node
import time
original_choice=node.choose_view
def slow_choice(*args,**kwargs):
 result=original_choice(*args,**kwargs)
 time.sleep(.25) # executor reports advance while view selection is running
 return result
node.choose_view=slow_choice
node.main()
'''
    env={**os.environ,'PYTHONDONTWRITEBYTECODE':'1',
         'PYTHONPATH':str(root/'src')+os.pathsep+str(root/'src/annual_planner/src')+os.pathsep+os.environ.get('PYTHONPATH','')}
    cmd=[sys.executable,'-c',script,'--ros-args','-p','use_sim_time:=true','-p','active_confirmation:=true',
         '-p','detector_backend:=yolo11n-seg','-p','max_frame_age_s:=0.0','-p','evidence_dir:='+str(tmp_path/'perception')]
    log=(tmp_path/'node.log').open('w');child=subprocess.Popen(cmd,env=env,stdout=log,stderr=subprocess.STDOUT)
    rclpy.init();node=rclpy.create_node('active_confirmation_contract_test')
    latch=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    tfpub=node.create_publisher(TFMessage,'/tf_static',latch)
    execpub=node.create_publisher(String,'/drone_0/view_execution',latch)
    clockpub=node.create_publisher(Clock,'/clock',10)
    pubs={k:node.create_publisher(cls,'/drone_0/'+topic,10) for k,cls,topic in [
        ('rgb',Image,'rgb/image'),('depth',Image,'depth/image'),('ri',CameraInfo,'rgb/camera_info'),('di',CameraInfo,'depth/camera_info')]}
    tracks=[];events=[];goals=[];semantics=[]
    node.create_subscription(String,'/search/tracks',lambda m:tracks.append(json.loads(m.data)),10)
    node.create_subscription(String,'/search/active_confirmation',lambda m:events.append(json.loads(m.data)),10)
    node.create_subscription(String,'/search/semantic_candidates',lambda m:semantics.append(json.loads(m.data)),10)
    node.create_subscription(PoseStamped,'/search/review_goal',goals.append,10)
    frames={}
    for key in ('rgb','depth'):
        msg=Image();msg.header.frame_id=key+'_optical';msg.height=240;msg.width=320
        msg.encoding='rgb8' if key=='rgb' else '32FC1';msg.step=320*(3 if key=='rgb' else 4)
        frames[key]=msg
    for key,base in [('ri','rgb'),('di','depth')]:
        msg=CameraInfo();msg.header.frame_id=base+'_optical';msg.height=240;msg.width=320
        msg.k=[160.,0.,160.,0.,160.,120.,0.,0.,1.]
        msg.r=[1.,0.,0.,0.,1.,0.,0.,0.,1.]
        msg.p=[160.,0.,160.,0.,0.,160.,120.,0.,0.,0.,1.,0.]
        msg.distortion_model='plumb_bob';msg.d=[0.]*5;frames[key]=msg
    from scipy.spatial.transform import Rotation
    lens=np.array([0.,0.,1.2]);yaw=0.;depth=4.;visible=True;ready=False
    token='baseline';arrived=True;reference=[0.,0.,1.17922];sim=10.
    def tf():
        reg=TransformStamped();reg.header.frame_id='rgb_optical';reg.child_frame_id='depth_optical';reg.transform.rotation.w=1.
        pose=TransformStamped();pose.header.frame_id='map';pose.child_frame_id='rgb_optical'
        pose.transform.translation.x,pose.transform.translation.y,pose.transform.translation.z=map(float,lens)
        q=Rotation.from_matrix(Rotation.from_euler('z',yaw).as_matrix()@np.array([[0,0,1],[-1,0,0],[0,-1,0]])).as_quat()
        pose.transform.rotation.x,pose.transform.rotation.y,pose.transform.rotation.z,pose.transform.rotation.w=map(float,q)
        tfpub.publish(TFMessage(transforms=[reg,pose]))
    def pump(count=1,until=None):
        nonlocal sim
        started=time.monotonic()
        for _ in range(count):
            sim+=.25;clock=Clock();clock.clock.sec=int(sim);clock.clock.nanosec=int(round((sim-int(sim))*1e9));clockpub.publish(clock)
            tf()
            execpub.publish(String(data=json.dumps(dict(time=sim,ready=ready,arrived=arrived,token=token,
                reference=reference,observation_proof={'fixture':True} if arrived else None))))
            for msg in frames.values():msg.header.stamp=clock.clock
            frames['rgb'].data=np.full((240,320,3),1 if visible else 0,np.uint8).tobytes()
            frames['depth'].data=np.full((240,320),depth,dtype='<f4').tobytes()
            for key,pub in pubs.items():pub.publish(frames[key])
            end=time.monotonic()+.12
            while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
            assert child.poll() is None,(tmp_path/'node.log').read_text()
            if until is not None and until():return
            assert time.monotonic()-started<25.,'Bounded interface wait expired'
        if until is not None:assert until(),dict(events=events,tracks=tracks[-2:],goals=len(goals))
    try:
        began=time.monotonic()
        while not (all(p.get_subscription_count() for p in pubs.values()) and execpub.get_subscription_count() and tfpub.get_subscription_count() and clockpub.get_subscription_count()):
            rclpy.spin_once(node,timeout_sec=.03)
            assert child.poll() is None
            assert time.monotonic()-began<10.,'DDS discovery timeout'
        pump(70,until=lambda:tracks and tracks[-1]['tracks'])
        assert tracks[-1]['mode']=='active_confirmation'
        assert not goals and all(t['state']!='confirmed' for x in tracks for t in x['tracks'])
        ready=True
        pump(55,until=lambda:bool(goals) and any(e['reason']=='visual_candidate_requested_safe_view' for e in events))
        request=goals[0];p=request.pose.position;q=request.pose.orientation
        requested=next(e for e in events if e['reason']=='visual_candidate_requested_safe_view')
        request_time=request.header.stamp.sec+request.header.stamp.nanosec*1e-9
        assert abs(requested['time']-request_time)<1e-8
        base=np.array([p.x,p.y,p.z]);new_yaw=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_euler('xyz')[2]
        # A stale baseline token cannot create completed evidence.
        pump(3);assert not any(e['reason']=='active_multiview_confirmed' for e in events)
        token='explicit_execution_fixture';arrived=False;reference=base.tolist()
        pump(25,until=lambda:any(e['reason']=='new_execution_after_request_observed' and e.get('token')==token for e in events))
        arrived=True;lens=base+Rotation.from_euler('z',new_yaw).apply([.13233,0.,.02078]);yaw=float(new_yaw)
        depth=float(np.linalg.norm((np.array([4.,0.,1.2])-lens)[:2]))
        pump(45,until=lambda:any(e['reason']=='active_multiview_confirmed' for e in events))
        confirmed=[e for e in events if e['reason']=='active_multiview_confirmed'][0]
        completed=[e for e in events if e['reason']=='execution_completed_waiting_for_fresh_visual_evidence'][0]
        assert confirmed['token']=='explicit_execution_fixture'
        assert min(confirmed['post_capture_times'])>completed['time']
        assert confirmed['post_capture_times'][-1]-confirmed['post_capture_times'][0]>=.4
        assert np.linalg.norm(np.array(confirmed['capture_origin'])-lens)<.001
        # Actual empty RGB fixture stays empty; no new confirmed identities.
        visible=False
        first_empty_ns=int(round((sim+.25)*1e9))
        # DDS and the deliberately slow view selector can retain older visible
        # packets. Wait for the empty input's capture stamp, never a wall-time
        # guess or merely any empty result; fresh nonempty output still fails.
        pump(45,until=lambda:any(x['stamp_ns']>=first_empty_ns for x in semantics))
        assert all(x['candidates']==[] for x in semantics if x['stamp_ns']>=first_empty_ns)
        assert len({t['id'] for packet in tracks for t in packet['tracks'] if t['state']=='confirmed'})==1
    finally:
        (tmp_path/'interface-evidence.json').write_text(json.dumps(dict(events=events,tracks=tracks,semantics=semantics,review_goals=len(goals)),indent=2))
        child.terminate()
        try:child.wait(timeout=5)
        except subprocess.TimeoutExpired:child.kill();child.wait()
        node.destroy_node();rclpy.shutdown();log.close()
