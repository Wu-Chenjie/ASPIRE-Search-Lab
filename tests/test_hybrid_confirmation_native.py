"""Real DDS/TF contract with explicit detector fixtures, not flight evidence."""
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import numpy as np
import pytest
rclpy=pytest.importorskip('rclpy')
from rclpy.qos import QoSProfile,DurabilityPolicy
from sensor_msgs.msg import Image,CameraInfo
from geometry_msgs.msg import TransformStamped,PoseStamped
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String


def test_hybrid_no_review_direct_confirmation_and_missing_depth_semantics(tmp_path):
    root=Path(__file__).resolve().parents[1]
    script='''import sys,types,numpy as np
from person_search.segmentation import PersonMask
class Stub:
 def __init__(self,*args):pass
 def runtime_diagnostics(self):return {'detector_fixture':True}
 def detect(self,rgb):
  if not rgb.any():return []
  m=np.zeros(rgb.shape[:2],bool);m[90:150,145:175]=True
  return [PersonMask((145,90,30,60),.9 if rgb[0,0,0]==1 else .4,m)]
sys.modules['person_search.yolo']=types.SimpleNamespace(YoloPersonDetector=Stub)
from person_search.node import main
main()
'''
    env={**os.environ,'PYTHONDONTWRITEBYTECODE':'1',
         'PYTHONPATH':str(root/'src')+os.pathsep+str(root/'src/annual_planner/src')+os.pathsep+os.environ.get('PYTHONPATH','')}
    cmd=[sys.executable,'-c',script,'--ros-args','-p','use_sim_time:=false',
         '-p','confirmation_mode:=hybrid','-p','active_review:=false',
         '-p','detector_backend:=yolo11n-seg','-p','evidence_dir:='+str(tmp_path/'perception')]
    log=(tmp_path/'node.log').open('w');child=subprocess.Popen(cmd,env=env,stdout=log,stderr=subprocess.STDOUT)
    rclpy.init();node=rclpy.create_node('hybrid_confirmation_contract_test')
    latch=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    tfpub=node.create_publisher(TFMessage,'/tf_static',latch)
    execpub=node.create_publisher(String,'/drone_0/view_execution',latch)
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
        msg.encoding='rgb8' if key=='rgb' else '32FC1';msg.step=320*(3 if key=='rgb' else 4);frames[key]=msg
    for key,base in [('ri','rgb'),('di','depth')]:
        msg=CameraInfo();msg.header.frame_id=base+'_optical';msg.height=240;msg.width=320
        msg.k=[160.,0.,160.,0.,160.,120.,0.,0.,1.];msg.r=[1.,0.,0.,0.,1.,0.,0.,0.,1.]
        msg.p=[160.,0.,160.,0.,0.,160.,120.,0.,0.,0.,1.,0.]
        msg.distortion_model='plumb_bob';msg.d=[0.]*5;frames[key]=msg
    from scipy.spatial.transform import Rotation
    reg=TransformStamped();reg.header.frame_id='rgb_optical';reg.child_frame_id='depth_optical';reg.transform.rotation.w=1.
    pose=TransformStamped();pose.header.frame_id='map';pose.child_frame_id='rgb_optical';pose.transform.translation.z=1.2
    q=Rotation.from_matrix(np.array([[0,0,1],[-1,0,0],[0,-1,0]])).as_quat()
    pose.transform.rotation.x,pose.transform.rotation.y,pose.transform.rotation.z,pose.transform.rotation.w=map(float,q)
    depth=np.nan;visible=True;last_stamp=0;signal=1;foreground=False
    def pump(count, until=None):
        nonlocal last_stamp
        for _ in range(count):
            stamp=node.get_clock().now();last_stamp=stamp.nanoseconds
            tfpub.publish(TFMessage(transforms=[reg,pose]))
            # Startup scan is not yet ready for motion requests, but passive
            # image evidence can still qualify. This fixture does not fly.
            execpub.publish(String(data=json.dumps(dict(time=last_stamp*1e-9,ready=False,arrived=True,token='fixture'))))
            for msg in frames.values():msg.header.stamp=stamp.to_msg()
            frames['rgb'].data=np.full((240,320,3),signal if visible else 0,np.uint8).tobytes()
            depth_pixels=np.full((240,320),depth,dtype='<f4')
            if foreground:depth_pixels[:,:145]=1.
            frames['depth'].data=depth_pixels.tobytes()
            for key,pub in pubs.items():pub.publish(frames[key])
            end=time.monotonic()+.35
            while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
            assert child.poll() is None,(tmp_path/'node.log').read_text()
            if until and until():return
        if until:assert until(),dict(events=events[-5:],tracks=tracks[-2:])
    try:
        began=time.monotonic()
        while not (all(p.get_subscription_count() for p in pubs.values()) and execpub.get_subscription_count() and tfpub.get_subscription_count()):
            rclpy.spin_once(node,timeout_sec=.02);assert child.poll() is None
            assert time.monotonic()-began<10.,'DDS discovery timeout'
        pump(10,lambda:bool(semantics) and bool(tracks))
        assert semantics[-1]['candidates'][0]['semantic_state']=='person_candidate'
        assert semantics[-1]['candidates'][0]['localization_state']=='unlocalized'
        assert tracks[-1]['tracks']==[] and not goals
        depth=4.;foreground=True
        pump(10,lambda:any(e['reason']=='direct_confirmation_rejected' and 'boundary_near_fraction_gate' in e.get('rejections',[]) for e in events))
        assert not any(e['reason']=='reliable_direct_confirmed' for e in events) and not goals
        foreground=False
        pump(20,lambda:any(e['reason']=='reliable_direct_confirmed' for e in events))
        confirmed=next(e for e in events if e['reason']=='reliable_direct_confirmed')
        assert len(set(confirmed['capture_times']))>=3 and confirmed['quality_thresholds']['min_confidence']==.7
        assert tracks[-1]['confirmation_policy']=='hybrid' and tracks[-1]['active_review'] is False
        assert not goals and confirmed['attempt']==0 and confirmed['is_before_ready']
        # A later low-score position is still reported as semantic evidence,
        # while the confirmed coordinate retains its own capture provenance.
        depth=4.6;signal=2
        pump(10,lambda:any(e['reason']=='confirmed_position_frozen' for e in events))
        frozen=next(t for t in tracks[-1]['tracks'] if t['state']=='confirmed')
        np.testing.assert_allclose(frozen['xyz'],confirmed['confirmed_position'])
        assert frozen['position_time']==confirmed['confirmed_time']
        assert frozen['position_status']=='frozen_confirmed_surface'
        visible=False;empty_after=node.get_clock().now().nanoseconds
        pump(10,lambda:any(s['stamp_ns']>=empty_after and not s['candidates'] for s in semantics))
        assert len({t['id'] for p in tracks for t in p['tracks'] if t['state']=='confirmed'})==1
    finally:
        (tmp_path/'interface-evidence.json').write_text(json.dumps(dict(events=events,tracks=tracks,semantics=semantics,review_goals=len(goals)),indent=2))
        child.terminate()
        try:child.wait(timeout=5)
        except subprocess.TimeoutExpired:child.kill();child.wait()
        node.destroy_node();rclpy.shutdown();log.close()
