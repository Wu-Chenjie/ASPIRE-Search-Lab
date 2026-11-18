#!/usr/bin/env python3
"""Display a real recorded confirmed output. No detector, planner or flight runs."""
import argparse,json,os,sys,time,hashlib,subprocess
from pathlib import Path
from types import SimpleNamespace
import numpy as np,yaml
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'src'),str(ROOT/'src/annual_planner/src')]
from person_search.node import target_markers
from person_search.perception import cloud
from tools.racer_ros2.light.processes import terminate

def main():
 p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--recorded-run',default='matrix-visible-left');p.add_argument('--output',type=Path,required=True);p.add_argument('--seconds',type=float,default=120);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
 archive_source=None
 if a.run.is_file():
  from tools.flight.recording import restore
  restored=a.output/'recording'
  archive_source=restore(a.run,a.recorded_run,restored)
  a.run=restored
 rows=[json.loads(s) for s in (a.run/'telemetry.jsonl').read_text().splitlines()];source=next(r for r in rows if r['kind']=='tracks' and any(t['state']=='confirmed' for t in r['data']['tracks']))
 pairs=[(f,int(np.load(f)['stamp_ns'])) for f in a.run.glob('rgbd-*.npz')];frame,frame_ns=min(pairs,key=lambda f:abs(f[1]-source['data']['stamp_ns']));data=np.load(frame)
 evidence=dict(archive_source=archive_source,mode='RECORDED OUTPUT REPLAY, not new flight or a new detection',source_run=str(a.run),source_telemetry_sha256=hashlib.sha256((a.run/'telemetry.jsonl').read_bytes()).hexdigest(),source_row=source,context_frame=frame.name,context_frame_sha256=hashlib.sha256(frame.read_bytes()).hexdigest(),context_frame_offset_s=(frame_ns-source['data']['stamp_ns'])*1e-9,marker_mapping_sha256=hashlib.sha256((ROOT/'src/person_search/node.py').read_bytes()).hexdigest())
 (a.output/'source-record.json').write_text(json.dumps(evidence,indent=2)+'\n')
 os.environ.update(ROS_DOMAIN_ID='198',ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=str(a.output/'roslog'),DISPLAY=':121',LIBGL_ALWAYS_SOFTWARE='1',PYTHONDONTWRITEBYTECODE='1')
 import rclpy
 from visualization_msgs.msg import Marker,MarkerArray
 from sensor_msgs.msg import Image
 from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
 from sensor_msgs.msg import PointCloud2
 from std_msgs.msg import String,Header
 from nav_msgs.msg import Odometry
 from rosgraph_msgs.msg import Clock
 from rclpy.time import Time
 from rosidl_runtime_py.convert import message_to_ordereddict
 from scipy.spatial.transform import Rotation
 from tools.flight.visualization import Display
 rclpy.init();node=rclpy.create_node('recorded_confirmed_replay');display=Display();stamp=Time(nanoseconds=source['data']['stamp_ns']).to_msg()
 marks=target_markers([SimpleNamespace(**t) for t in source['data']['tracks']],stamp)
 received=[]
 def check(m):
  if received:return
  expected={t['id']:t for t in source['data']['tracks'] if t['state']=='confirmed'}
  for marker in m.markers:
   if marker.id in expected:
    t=expected[marker.id];assert marker.header.frame_id=='map' and marker.type==Marker.SPHERE
    assert marker.color.r==0. and marker.color.g==1. and marker.color.b==0. and marker.color.a>0.
    assert [marker.pose.position.x,marker.pose.position.y,marker.pose.position.z]==t['xyz']
    assert marker.header.stamp==stamp
    received.append(marker.id)
  if received:(a.output/'received-marker.json').write_text(json.dumps(message_to_ordereddict(m),indent=2)+'\n')
 node.create_subscription(MarkerArray,'/search/targets',check,10)
 pubs={k:node.create_publisher(cls,topic,10) for k,cls,topic in [('marks',MarkerArray,'/search/targets'),('tracks',String,'/search/tracks'),('image',Image,'/drone_0/rgb/image'),('cloud',PointCloud2,'/baseline/cloud_0'),('banner',MarkerArray,'/search/replay_notice'),('odom',Odometry,'/drone_0/estimated_odometry'),('clock',Clock,'/clock')]}
 rgb=Image();rgb.header=Header(stamp=stamp,frame_id='map');rgb.height,rgb.width=data['rgb'].shape[:2];rgb.encoding='rgb8';rgb.step=rgb.width*3;rgb.data=data['rgb'].tobytes()
 rot=Rotation.from_quat(data['base_quaternion']);t=np.eye(4);t[:3,:3]=rot.as_matrix()@Rotation.from_quat([-.5,.5,-.5,.5]).as_matrix();t[:3,3]=data['base_position']+rot.apply([.13233,0,.02078]);points=cloud(data['depth'],data['k'],t);valid=np.isfinite(points).all(axis=2)&(data['depth']>.25)&(data['depth']<8);pc=create_cloud_xyz32(Header(stamp=stamp,frame_id='map'),points[valid][::3].astype(np.float32))
 odom=Odometry();odom.header=Header(stamp=stamp,frame_id='map');odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z=map(float,data['base_position']);q=data['base_quaternion'];odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w=map(float,q)
 banner=Marker();banner.header=odom.header;banner.ns='replay_notice';banner.id=0;banner.type=Marker.TEXT_VIEW_FACING;banner.pose.orientation.w=1.;banner.pose.position.x=3.8;banner.pose.position.z=2.5;banner.scale.z=.19;banner.color.r=1.;banner.color.g=.85;banner.color.a=1.;banner.text='RECORDED OUTPUT REPLAY\n'+(archive_source['run'] if archive_source else a.run.name)+'\nNo new flight / no new detection'
 cfg=yaml.safe_load((ROOT/'config/search.rviz').read_text());vm=cfg['Visualization Manager'];vm['Displays']=[d for d in vm['Displays'] if d['Name'] not in ('Estimated flight path','Authorized RACER path')];vm['Displays'].append(dict(Class='rviz_default_plugins/MarkerArray',Name='Recorded replay notice',Enabled=True,Topic=dict(Value='/search/replay_notice')));vm['Views']['Current'].update({'Distance':6,'Focal Point':dict(X=3.4,Y=0,Z=1.35),'Pitch':.3,'Yaw':-1.4});(a.output/'replay.rviz').write_text(yaml.safe_dump(cfg))
 children=[];logs=[]
 def start(name,args):
  f=(a.output/(name+'.log')).open('x');logs.append(f);children.append(subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT,start_new_session=True))
 try:
  start('xvfb',['Xvfb',':121','-screen','0','1280x900x24','-nolisten','tcp']);time.sleep(.7)
  start('rviz',['rviz2','-d',str(a.output/'replay.rviz'),'--ros-args','-p','use_sim_time:=true']);start('x11vnc',['x11vnc','-display',':121','-localhost','-rfbport','5902','-nopw','-forever','-shared']);start('websockify',['websockify','--web=/usr/share/novnc','127.0.0.1:18682','127.0.0.1:5902'])
  until=time.monotonic()+a.seconds
  while time.monotonic()<until:
   pubs['clock'].publish(Clock(clock=stamp));pubs['tracks'].publish(String(data=json.dumps(source['data'])));pubs['marks'].publish(marks);pubs['image'].publish(rgb);pubs['cloud'].publish(pc);pubs['odom'].publish(odom);pubs['banner'].publish(MarkerArray(markers=[banner]));rclpy.spin_once(node,timeout_sec=.02)
   for _ in range(3):rclpy.spin_once(display,timeout_sec=.02)
   time.sleep(.1)
   if any(c.poll() is not None for c in children):raise RuntimeError('Replay display process exited')
  if not received:raise RuntimeError('No verified confirmed Marker received through DDS')
 finally:
  cleanup=terminate(children,timeout=8);(a.output/'result.json').write_text(json.dumps(dict(mode='recorded output replay',verified_confirmed_marker_ids=received,cleanup=cleanup),indent=2)+'\n')
  for f in logs:f.close()
  display.destroy_node();node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
