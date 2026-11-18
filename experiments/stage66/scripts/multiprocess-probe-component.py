"""No-flight comparative transport fixtures. Real ViewExecutor is a separate process."""
from pathlib import Path
import json,os,subprocess,sys,time,signal,yaml,gzip
import rclpy
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
case=sys.argv[1];assert case in ['normal-new','normal-new-v2','silent-old','silent-new','delayed-old','delayed-new','loaded-old','loaded-new','loaded-new-final','delayed-new-final'];mode='normal-new' if case.startswith('normal-new') else case.removesuffix('-final');root=Path('/tmp/stage66-components')/case;root.mkdir(parents=True,exist_ok=False)
source=Path('/tmp/racer-stage65-p01-a');config=source/'runtime.json';params=root/'nodes.yaml';doc=yaml.safe_load((source/'nodes.yaml').read_text());settings=doc['/**']['ros__parameters'];settings['control_timing_dir']=str(root/'timing');settings['execution_safety_dir']=str(root/'execution-safety');params.write_text(yaml.safe_dump(doc))
env=dict(os.environ,ROS_DOMAIN_ID='216',ROS_LOCALHOST_ONLY='1',PYTHONDONTWRITEBYTECODE='1',RACER_MAP_EVIDENCE_DIR=str(root/'map-evidence'),RACER_OBSERVATION_SESSION='component-'+case,
 STAGE66_COMPONENT_CONFIG=str(config),STAGE66_COMPONENT_OUTPUT=str(root),STAGE66_SILENT_EXECUTOR='1' if case.startswith('silent') else '0',STAGE66_DELAY_EXECUTOR='.6' if case.startswith('delayed') else '0',RACER_EXECUTION_SAFETY='1',RACER_CONTROL_OBSERVATION='1',RACER_CONTROL_OBSERVATION_GZIP='1',ROS_LOG_DIR=str(root/'roslog'))
rclpy.init(domain_id=216);sender=rclpy.create_node('stage66_fixture_clock');pub=sender.create_publisher(Clock,'/clock',10);odompub=sender.create_publisher(Odometry,'/drone_0/estimated_odometry',10);children=[];logs=[];probe=None
replay=[];used=0;load_start=time.monotonic()
if mode.startswith('loaded'):
 with gzip.open(source/'timing/drone-0-executor-observation.jsonl.gz','rt') as f:
  unique={}
  for line in f:
   x=json.loads(line)
   if x['event']=='odometry_used':unique[x['stamp_ns']]=x
  replay=[unique[k] for k in sorted(unique)]
def clock_step():
 global used
 ns=1000000000 if not replay else replay[0]['stamp_ns']+round((time.monotonic()-load_start)*.5/0.004)*4000000
 msg=Clock();msg.clock.sec=ns//10**9;msg.clock.nanosec=ns%10**9;pub.publish(msg)
 while used<len(replay) and replay[used]['stamp_ns']<=ns:
  x=replay[used];used+=1;m=Odometry();m.header.frame_id=x['frame'];m.child_frame_id=x['child_frame'];m.header.stamp.sec=x['stamp_ns']//10**9;m.header.stamp.nanosec=x['stamp_ns']%10**9
  m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z=map(float,x['position'])
  m.pose.pose.orientation.x,m.pose.pose.orientation.y,m.pose.pose.orientation.z,m.pose.pose.orientation.w=map(float,x['quaternion'])
  m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.linear.z=map(float,x['body_velocity']);odompub.publish(m)
 rclpy.spin_once(sender,timeout_sec=.001 if replay else .05)
try:
 for role in ['pointcloud_mapping_node','view_executor_node','gateway']:
  f=(root/(role+'.log')).open('x');logs.append(f);args=[sys.executable,'/tmp/stage66-measurement/component-node.py',role]
  if role!='gateway':args+=['--ros-args','--params-file',str(params)]
  children.append(subprocess.Popen(args,env=env,stdout=f,stderr=subprocess.STDOUT))
 end=time.monotonic()+12
 while not all((root/'map-evidence'/('drone-0-'+r)/'health.json').exists() for r in ['mapper','executor','gateway']):
  clock_step();assert time.monotonic()<end
 script='/tmp/stage65-measurement/stage65-startup-probe.py' if mode.endswith('old') else '/tmp/stage66-measurement/stage66-startup-probe.py'
 f=(root/'producer.log').open('x');logs.append(f);probe=subprocess.Popen([sys.executable,script,str(root/'startup-capture-probe.json'),'component-'+case],env=env,stdout=f,stderr=subprocess.STDOUT)
 end=time.monotonic()+12
 while probe.poll() is None:
  clock_step();assert time.monotonic()<end
 # Allow already-delivered callbacks to finish; no policy time or physics advances.
 end=time.monotonic()+.5
 while time.monotonic()<end:
  clock_step()
finally:
 if probe is not None and probe.poll() is None:probe.send_signal(signal.SIGINT);probe.wait(timeout=3)
 for child in children:
  if child.poll() is None:child.send_signal(signal.SIGINT)
 for child in children:child.wait(timeout=8)
 for f in logs:f.close()
 sender.destroy_node();rclpy.shutdown()
response={r:len(list((root/'map-evidence'/('drone-0-'+r)).glob('*-probe.npz'))) for r in ['mapper','executor','gateway']}
report=dict(case=case,no_flight=True,no_Gazebo_or_PX4=True,real_ViewExecutor_separate_process=True,saved_estimated_odometry_fixture=bool(replay),actual_replayed_pose_headers=used,fixture_clock_not_physics=True,controlled_silent_executor=case.startswith('silent'),controlled_first_point6s_diagnostic_drop=case.startswith('delayed'),producer_returncode=probe.returncode,responses=response,producer_report_present=(root/'startup-capture-probe.json').exists(),children_returncodes=[x.returncode for x in children])
if mode in ['normal-new','delayed-new','loaded-new']:assert probe.returncode==0 and all(x==1 for x in response.values()),report
elif mode=='loaded-old':assert probe.returncode==0,report
elif mode in ['silent-old','delayed-old']:assert probe.returncode==0 and response==dict(mapper=1,executor=0,gateway=1),report
else:assert probe.returncode!=0 and not report['producer_report_present'] and response==dict(mapper=1,executor=0,gateway=1),report
(root/'RESULTS.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
