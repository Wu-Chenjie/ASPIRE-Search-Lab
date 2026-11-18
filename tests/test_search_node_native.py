"""Real ROS2 RGB-D messages: missing map TF fails closed, then actual pose works."""
import os,sys,json,time,subprocess
from pathlib import Path
import numpy as np
import pytest
rclpy=pytest.importorskip('rclpy')
from sensor_msgs.msg import Image,CameraInfo
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
from rclpy.qos import QoSProfile,DurabilityPolicy


@pytest.mark.parametrize('backend',['hog','stub','yolo','yolo_ambiguous'])
def test_node_requires_capture_time_map_tf(tmp_path,backend):
    segmentation_contract=backend=='stub'
    if backend.startswith('yolo') and not os.environ.get('SEARCH_YOLO_WEIGHTS'):pytest.skip('Optional actual locked model integration')
    capture=os.environ.get('SEARCH_CAPTURE')
    if backend=='yolo':capture=os.environ.get('SEARCH_YOLO_CAPTURE')
    if not capture:pytest.skip('Set SEARCH_CAPTURE to native static RGB-D run')
    root=Path(__file__).resolve().parents[1];folder=Path(capture)/'visible';record=json.loads((folder/'frame.json').read_text())
    env={**os.environ,'PYTHONPATH':str(root/'src')+os.pathsep+str(root/'src/annual_planner/src')+os.pathsep+os.environ.get('PYTHONPATH',''),'PYTHONDONTWRITEBYTECODE':'1'}
    cmd=[sys.executable,'-m','person_search.node','--ros-args','-p','max_frame_age_s:=0.0']
    if backend.startswith('yolo'):
        env['PYTHONPATH']=os.environ['SEARCH_YOLO_PYTHON']+os.pathsep+env['PYTHONPATH']
        cmd.extend(['-p','detector_backend:=yolo11n-seg','-p','segmentation_weights:='+os.environ['SEARCH_YOLO_WEIGHTS']])
    if segmentation_contract:
        # Explicit deterministic detector stub: tests real DDS/TF and the optional
        # interface, NOT YOLO weights, accuracy, or a detector rollout.
        script='''import sys,types,numpy as np
from person_search.segmentation import PersonMask
class Stub:
 def __init__(self,*args):pass
 def detect(self,rgb):
  m=np.zeros(rgb.shape[:2],bool);m[85:160,140:180]=True
  return [PersonMask((140,85,40,75),.9,m)]
sys.modules['person_search.yolo']=types.SimpleNamespace(YoloPersonDetector=Stub)
from person_search.node import main
main()
'''
        cmd=[sys.executable,'-c',script,'--ros-args','-p','detector_backend:=yolo11n-seg','-p','max_frame_age_s:=0.0']
    log=(tmp_path/'node.log').open('w');child=subprocess.Popen(cmd,env=env,stdout=log,stderr=subprocess.STDOUT)
    rclpy.init();node=rclpy.create_node('search_node_native_test');out=[]
    sub=node.create_subscription(String,'/search/tracks',out.append,10)
    semantic=[];semantic_sub=node.create_subscription(String,'/search/semantic_candidates',semantic.append,10)
    pubs={k:node.create_publisher(cls,'/drone_0/'+topic,10) for k,cls,topic in [('rgb',Image,'rgb/image'),('depth',Image,'depth/image'),('ri',CameraInfo,'rgb/camera_info'),('di',CameraInfo,'depth/camera_info')]}
    tfpub=node.create_publisher(TFMessage,'/tf_static',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    frames={}
    for k,base in [('rgb','rgb'),('depth','depth')]:
        m=Image();m.header.frame_id=record[base]['frame'];m.header.stamp.sec=10;m.height=240;m.width=320;m.encoding=record[k+'_encoding'];m.step=record[k+'_step'];m.data=(folder/(k+'.bin')).read_bytes();frames[k]=m
    for k,base in [('ri','rgb'),('di','depth')]:
        m=CameraInfo();m.header=frames[base].header;m.height=240;m.width=320
        for key in ('k','r','p','d'):setattr(m,key,record[base][key])
        m.distortion_model=record[base]['distortion_model'];frames[k]=m
    reg=TransformStamped();reg.header.frame_id=frames['rgb'].header.frame_id;reg.child_frame_id=frames['depth'].header.frame_id;reg.transform.rotation.w=1.
    waits=[]
    def pump(seconds,only_rgb=False,until=None):
        began=time.monotonic();end=began+seconds
        while time.monotonic()<end:
            for k,p in pubs.items():
                if not only_rgb or k=='rgb':p.publish(frames[k])
            rclpy.spin_once(node,timeout_sec=.03)
            assert child.poll() is None
            if until is not None and until():break
        waits.append(dict(timeout_s=seconds,elapsed_s=time.monotonic()-began,condition_met=bool(until and until())))
        if until is not None:assert until(),'No matching response within bounded wait'
    try:
        # Establish the actual DDS graph before the fixture sends a one-shot stamp.
        # The production node intentionally does not reprocess an accepted stamp.
        began=time.monotonic()
        while not (all(p.get_subscription_count()>0 for p in pubs.values()) and sub.get_publisher_count()>0 and semantic_sub.get_publisher_count()>0 and tfpub.get_subscription_count()>0):
            rclpy.spin_once(node,timeout_sec=.05)
            assert child.poll() is None
            assert time.monotonic()-began<10.,'DDS discovery did not become ready'
        waits.append(dict(stage='dds_ready',elapsed_s=time.monotonic()-began))
        tfpub.publish(TFMessage(transforms=[reg]));pump(2.5,only_rgb=segmentation_contract)
        assert not out,'No map TF must produce no target'
        if segmentation_contract:
            assert semantic,'RGB alone must retain semantic evidence without any depth'
            row=json.loads(semantic[-1].data)['candidates'][0]
            assert row['state']=='unlocalized' and row['xyz'] is None
        # This is test-fixture camera pose, not an estimated production transform.
        from scipy.spatial.transform import Rotation
        v=json.loads((Path(capture)/'viewpoints.json').read_text())['views']['visible']
        t=TransformStamped();t.header.frame_id='map';t.child_frame_id=frames['rgb'].header.frame_id
        t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=v['lens_xyz']
        q=Rotation.from_matrix(Rotation.from_euler('z',v['lens_yaw']).as_matrix()@np.array([[0,0,1],[-1,0,0],[0,-1,0]])).as_quat()
        t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w=map(float,q)
        tfpub.publish(TFMessage(transforms=[reg,t]))
        if segmentation_contract:
            frames['depth'].data=np.full((240,320),np.nan,dtype='<f4').tobytes();pump(8.,until=lambda:bool(out) and json.loads(out[-1].data)['stamp_ns']==10_000_000_000)
            assert out and json.loads(out[-1].data)['tracks']==[]
            row=json.loads(semantic[-1].data)['candidates'][0]
            assert row['state']=='unlocalized' and row['xyz'] is None
            for frame in frames.values():frame.header.stamp.sec=11
            frames['depth'].data=np.full((240,320),4.,dtype='<f4').tobytes()
        expected_stamp=11_000_000_000 if segmentation_contract else 10_000_000_000
        pump(8.,until=lambda:bool(out) and json.loads(out[-1].data)['stamp_ns']==expected_stamp)
        assert out;tracks=json.loads(out[-1].data)['tracks']
        if backend=='yolo_ambiguous':
            # Real old MakeHuman/HOG fixture: YOLO sees the person but its
            # mask depth spans foreground/background. Do not fabricate xyz.
            assert not tracks and semantic
            row=json.loads(semantic[-1].data)['candidates'][0]
            assert row['xyz'] is None and row['depth_reason']=='ambiguous_depth_span'
            return
        assert len(tracks)==1
        assert tracks[0]['state']=='tentative';assert 3.5<tracks[0]['xyz'][0]<4.3
    finally:
        (tmp_path/'waits.json').write_text(json.dumps(waits,indent=2))
        child.terminate()
        try:child.wait(timeout=5)
        except subprocess.TimeoutExpired:child.kill();child.wait()
        node.destroy_node();rclpy.shutdown();log.close()
