"""Real ROS message/TF/marker fault fixture; not a flight or detector benchmark."""
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
from std_msgs.msg import String,Float64MultiArray
from visualization_msgs.msg import Marker,MarkerArray

@pytest.mark.parametrize('pace',['original_fast','semantic'])
@pytest.mark.parametrize('mode',['decoupled','registry'])
def test_visual_rgb_only_then_depth_recovery_loss_and_missing_tf(tmp_path,mode,pace):
    root=Path(__file__).resolve().parents[1]
    script="""import sys,types,numpy as np,cv2
from person_search.segmentation import PersonMask
class Stub:
 def __init__(self,weights,*,preserve_mask_candidates):
  assert preserve_mask_candidates
 def detect(self,rgb):
  mask=np.zeros((240,320),bool);mask[80:160,150:170]=True
  return [PersonMask((140,70,40,100),.9,mask),PersonMask((141,70,40,100),.8,mask)]
sys.modules['person_search.yolo']=types.SimpleNamespace(YoloPersonDetector=Stub)
from person_search.node import main
main()
"""
    env={**os.environ,'PYTHONDONTWRITEBYTECODE':'1','ROS_DOMAIN_ID':'184','PYTHONPATH':str(root/'src')+os.pathsep+str(root/'src/annual_planner/src')+os.pathsep+os.environ.get('PYTHONPATH','')}
    log=(tmp_path/'node.log').open('w');proc=subprocess.Popen([sys.executable,'-c',script,'--ros-args','-p','use_sim_time:=true','-p','detector_backend:=yolo11n-seg','-p','confirmation_mode:='+mode],env=env,stdout=log,stderr=subprocess.STDOUT)
    rclpy.init(domain_id=184);node=rclpy.create_node('presence_fixture')
    pubs={k:node.create_publisher(c,t,qos_profile_sensor_data) for k,c,t in [('rgb',Image,'/drone_0/rgb/image'),('depth',Image,'/drone_0/depth/image'),('ri',CameraInfo,'/drone_0/rgb/camera_info'),('di',CameraInfo,'/drone_0/depth/camera_info')]}
    clockpub=node.create_publisher(Clock,'/clock',10);tfpub=node.create_publisher(TFMessage,'/tf_static',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    reports=[];packets=[];markers=[];events=[];registry=[];timings=[];semantics=[]
    node.create_subscription(String,'/search/semantic_candidates',lambda m:semantics.append(json.loads(m.data)),100)
    node.create_subscription(String,'/search/perception_timing',lambda m:timings.append(json.loads(m.data)),100)
    node.create_subscription(String,'/search/tracks',lambda m:reports.append(json.loads(m.data)),30)
    node.create_subscription(String,'/search/presence_events',lambda m:events.append(json.loads(m.data)),30)
    node.create_subscription(Float64MultiArray,'/search/bearing_evidence',lambda m:packets.append(list(m.data)),30)
    node.create_subscription(MarkerArray,'/search/targets',lambda m:markers.append(m),30)
    node.create_subscription(String,'/uav0/search/target_observations',lambda m:registry.append(json.loads(m.data)),30)
    # Slow clock models the existing <=1 RTF contract; sensor stamp is 50ms old.
    # Evidence thresholds and 40-image limit are unchanged; not a throughput test.
    clock_start=time.monotonic();last_clock=[-1.]
    def fixture_clock(offset=0.):
        sim=10.+.5*(time.monotonic()-clock_start)+offset;m=Clock();m.clock.sec=int(sim);m.clock.nanosec=round((sim-int(sim))*1e9);return m
    def spin(duration):
        end=time.monotonic()+duration
        while time.monotonic()<end:
            if pace=='semantic' and time.monotonic()-last_clock[0]>=.05:
                clockpub.publish(fixture_clock());last_clock[0]=time.monotonic()
            rclpy.spin_once(node,timeout_sec=.01)
        assert proc.poll() is None,(tmp_path/'node.log').read_text()
    def capture(index,depth=None,frame='optical'):
        if pace=='original_fast':
            clock=Clock();sim=10.+index*.2;clock.clock.sec=int(sim);clock.clock.nanosec=round((sim-int(sim))*1e9);clockpub.publish(clock)
        else:clock=fixture_clock(-.05);clockpub.publish(fixture_clock())
        info=CameraInfo();info.header.stamp=clock.clock;info.header.frame_id=frame;info.width=320;info.height=240;info.k=[160.,0,160.,0,160.,120.,0,0,1.];info.d=[0.]*5
        pubs['ri'].publish(info)
        rgb=Image();rgb.header=info.header;rgb.width=320;rgb.height=240;rgb.encoding='rgb8';rgb.step=960;rgb.data=np.ones((240,320,3),np.uint8).tobytes();pubs['rgb'].publish(rgb)
        if depth is not None:
            pubs['di'].publish(info);m=Image();m.header=info.header;m.width=320;m.height=240;m.encoding='32FC1';m.step=1280;m.data=np.full((240,320),depth,np.float32).tobytes();pubs['depth'].publish(m)
        spin(.14 if pace=='original_fast' else .4)
    def latest():return max(reports[-1]['tracks'],key=lambda t:t['capture_time_s'])
    try:
        start=time.monotonic()
        while not all(p.get_subscription_count() for p in pubs.values()) or not clockpub.get_subscription_count():
            spin(.03);assert time.monotonic()-start<12
        tf=TransformStamped();tf.header.frame_id='map';tf.child_frame_id='optical';tf.transform.translation.z=1.5;tf.transform.rotation.x=-.5;tf.transform.rotation.y=.5;tf.transform.rotation.z=-.5;tf.transform.rotation.w=.5
        tfpub.publish(TFMessage(transforms=[tf]));spin(.2)
        for i in range(12):capture(i)
        (tmp_path/'rgb-phase.json').write_text(json.dumps(dict(reports=reports,timings=timings),indent=2))
        assert latest()['visual_state']=='visually_confirmed' and latest()['xyz'] is None
        if mode=='decoupled':assert packets and packets[-1][14]==0
        else:assert not packets and registry and not registry[-1]['observations'][0]['has_position']
        if mode=='decoupled':assert not any(m.type==Marker.SPHERE for m in markers[-1].markers)
        recovery_start=len(reports);marker_start=len(markers);wire_start=len(registry)
        for i in range(12,24):capture(i,3.)
        recovery=[t for report in reports[recovery_start:] for t in report['tracks'] if t['localization_state']=='reliable']
        (tmp_path/'recovery-reports.json').write_text(json.dumps(reports[recovery_start:],indent=2))
        (tmp_path/'timings.json').write_text(json.dumps(timings,indent=2))
        assert recovery,(tmp_path/'recovery-reports.json').read_text()
        assert all(t['position']['frame']=='map' for t in recovery)
        if mode=='decoupled':assert any(m.type==Marker.SPHERE for array in markers[marker_start:] for m in array.markers)
        else:assert any(row['has_position'] for batch in registry[wire_start:] for row in batch['observations'])
        for i in range(24,32):capture(i,float('nan'))
        assert latest()['xyz'] is None and latest()['visual_state']=='visually_confirmed'
        assert latest()['last_reliable_location_history']['history_only'] is True
        if mode=='decoupled':assert not any(m.type==Marker.SPHERE for m in markers[-1].markers)
        if mode=='decoupled':assert any(m.type==Marker.LINE_LIST for m in markers[-1].markers)
        else:assert not registry[-1]['observations'][0]['has_position']
        prior_id=latest()['id'];prior_track_count=len(reports[-1]['tracks'])
        for i in range(32,40):capture(i,3.,frame='unavailable_optical')
        assert latest()['xyz'] is None and latest()['bearing_state']=='unavailable'
        assert latest()['visual_state']=='visually_confirmed'
        # Missing RGB geometry breaks certified local continuity, while the new
        # RGB-only source must independently confirm and old history survives.
        assert reports[-1]['counts']['local_visual_track_records']==prior_track_count+1
        assert latest()['id']!=prior_id
        assert next(t for t in reports[-1]['tracks'] if t['id']==prior_id)['visual_state']=='visually_confirmed'
        assert reports[-1]['counts']['independent_true_people'] is None
        assert latest()['capture_count']>=5
        assert reports[-1]['counts']['same_frame_duplicates_suppressed_total']==sum(t['capture_count'] for t in reports[-1]['tracks'])
        (tmp_path/'all-reports.json').write_text(json.dumps(dict(mode=mode,pace=pace,published_rgb_frames=40,reports=reports,registry=registry,events=events),indent=2))
        assert sum(e['event']=='visual_confirmed' for e in events)==2
        if mode=='decoupled':assert not any(m.id==latest()['id'] and m.type in (Marker.SPHERE,Marker.LINE_LIST) for m in markers[-1].markers)
        else:assert not registry[-1]['observations'][0]['has_position'] and not registry[-1]['observations'][0]['has_bearing']
        # Advance the timer without publishing an RGB image to exercise a repeat.
        if pace == 'original_fast':
            tick = Clock()
            tick.clock.sec = 18  # Last image is at 17.8 s.
            tick.clock.nanosec = 200_000_000
            clockpub.publish(tick)
        deadline = time.monotonic() + 3
        while len({msg['stamp_ns'] for msg in semantics}) == len(semantics) and time.monotonic() < deadline:
            spin(.05)
        first_by_stamp = {}
        for msg in semantics:
            timing = (msg['received_wall_s'], msg['first_semantic_latency_wall_s'])
            assert timing[0] > 0 and timing[1] >= 0
            assert first_by_stamp.setdefault(msg['stamp_ns'], timing) == timing
        assert 0 < len(first_by_stamp) < len(semantics)
    finally:
        proc.terminate()
        try:proc.wait(timeout=5)
        except subprocess.TimeoutExpired:proc.kill();proc.wait(timeout=5)
        log.close();node.destroy_node();rclpy.shutdown()
