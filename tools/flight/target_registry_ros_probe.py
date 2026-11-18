"""Actual ROS2 two-source interface fixture; no aircraft or detector is simulated."""
import json,time,subprocess,signal,sys
from pathlib import Path
import rclpy
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray
out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=False)
rclpy.init();node=rclpy.create_node('stage32_registry_probe')
states={};summaries=[];markers={};events=[];children=[];logs=[]
def receive(robot,m):
 states[robot]=json.loads(m.data)
 with (out/'states.jsonl').open('a') as f:f.write(json.dumps(dict(robot=robot,wall=time.time(),state=states[robot]))+'\n')
for robot in ('uav0','uav1'):
 node.create_subscription(String,f'/{robot}/search/person_records',lambda m,r=robot:receive(r,m),100)
 node.create_subscription(MarkerArray,f'/{robot}/search/targets',lambda m,r=robot:markers.update({r:m}),10)
 node.create_subscription(String,f'/{robot}/search/target_registry_events',lambda m:events.append(json.loads(m.data)),100)
node.create_subscription(String,'/search/target_summary',lambda m:summaries.append(m.data),100)
pubs={r:node.create_publisher(String,f'/{r}/search/target_observations',20) for r in ('uav0','uav1')}
peer=node.create_publisher(String,'/search/target_summary',20)
def spin(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
def wait(predicate,label,seconds=12):
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  spin(.05)
  if predicate():checks.append(label);return
 raise AssertionError(label+': '+json.dumps(states))
def start(robot):
 f=(out/(robot+'-'+str(len(children))+'.log')).open('w');logs.append(f)
 p=subprocess.Popen([(sys.argv[2] if len(sys.argv)>2 else '/tmp/stage32-ros-build/registry_ros'),'--ros-args','-p','robot:='+robot],stdout=f,stderr=subprocess.STDOUT);children.append(p);return p
def batch(robot,items):
 now=node.get_clock().now().nanoseconds*1e-9
 rows=[]
 for ident,seq,first,pos,aligned in items:
  row=dict(robot=robot,session='actual-ros-fixture',counter=str(ident),sequence=str(seq),revision='0',
   first_s=first,capture_s=now,frame='map',frame_version='fixture-aligned-v1',shared_aligned=aligned,
   visual_state='visually_confirmed',localization_state='reliable' if pos else 'unknown',
   has_position=bool(pos),has_bearing=not bool(pos),confidence=.95)
  if pos:row.update(px=pos[0],py=pos[1],pz=pos[2],radius_m=.15)
  else:row.update(ox=0,oy=0,oz=1,dx=1,dy=0,dz=0,half_angle_rad=.1,camera_half_fov_rad=.637,support_captures=5,support_first_s=now-.8)
  rows.append(row)
 body=json.dumps(dict(schema=1,kind='observations',observations=rows,raw_tracks_total=len(items)))
 pubs[robot].publish(String(data=body));return now
def groups(robot):return states[robot]['counts']['persistent_hypotheses']
checks=[];result={}
try:
 a=start('uav0');b=start('uav1');spin(1)
 first=node.get_clock().now().nanoseconds*1e-9
 batch('uav0',[(1,1,first,[4,0,1],True)])
 batch('uav1',[(1,1,first,[4.1,0,1],True)])
 wait(lambda:all(r in states and groups(r)==1 and states[r]['counts']['source_records']==2 for r in pubs),'aligned_two_sources_one_reversible_hypothesis')
 batch('uav1',[(1,2,first,[4.1,0,1],False)])
 wait(lambda:all(groups(r)==2 for r in pubs),'unaligned_coordinates_revoke_correspondence')
 batch('uav1',[(1,3,first,[4.1,0,1],True)])
 wait(lambda:all(groups(r)==1 for r in pubs),'realignment_can_reassociate')
 f2=node.get_clock().now().nanoseconds*1e-9
 batch('uav0',[(2,1,f2,[8,0,1],True),(3,1,f2,[8.2,0,1],True)])
 wait(lambda:all(states[r]['counts']['source_records']==4 and groups(r)==3 for r in pubs),'same_camera_two_people_remain_separate_across_individual_summaries')
 old=next(s for s in reversed(summaries) if json.loads(s).get('robot')=='uav0' and json.loads(s).get('kind')=='target_summary')
 ignored=states['uav1']['communication']['ignored_messages']
 peer.publish(String(data=old))
 wait(lambda:states['uav1']['communication']['ignored_messages']>ignored,'duplicate_revision_ignored')
 f3=node.get_clock().now().nanoseconds*1e-9
 batch('uav0',[(4,1,f3,None,True)])
 wait(lambda:states['uav0']['counts']['source_records']==5,'RGB_presence_without_position_retained')
 wait(lambda:'uav0' in markers and any(m.ns=='persistent_bearing_no_range' for m in markers['uav0'].markers),'RViz_bearing_without_fabricated_range')
 spin(2.2)
 assert all(g['position'] is None for g in states['uav0']['hypotheses'])
 checks.append('current_geometry_expires_without_erasing_presence')
 for p in (a,b):p.send_signal(signal.SIGINT);p.wait(timeout=8)
 states.clear();summaries.clear();a=start('uav0');spin(.6)
 first=node.get_clock().now().nanoseconds*1e-9
 batch('uav0',[(i,1,first,[4+2*i,0,1],True) for i in range(10,20)])
 wait(lambda:states.get('uav0',{}).get('counts',{}).get('source_records')==10,'reconnect_fixture_source_ready')
 spin(2.4);b=start('uav1')
 wait(lambda:states.get('uav1',{}).get('counts',{}).get('source_records')==10,'bounded_two_page_reconnect_recovers_ten_records',18)
 assert all(s['interface_errors']==0 for s in states.values())
 assert all(s['counts']['independent_true_people'] is None for s in states.values())
 checks.append('no_interface_errors_or_true_person_count_claim')
 result=dict(status='passed',checks=checks,states=states,summary_message_count=len(summaries),
  summary_utf8_bytes=sum(len(s.encode('utf-8')) for s in summaries),network_overhead_included=False,
  scope='actual C++ ROS nodes and messages, explicit synthetic observations; not two-UAV flight')
except Exception as e:
 result=dict(status='failed',checks=checks,error=repr(e),states=states);raise
finally:
 for p in children:
  if p.poll() is None:
   p.send_signal(signal.SIGINT)
   try:p.wait(timeout=8)
   except subprocess.TimeoutExpired:p.kill();p.wait()
 for f in logs:f.close()
 (out/'RESULTS.json').write_text(json.dumps(result,indent=2))
 (out/'summaries.jsonl').write_text('\n'.join(summaries)+'\n')
 (out/'events.json').write_text(json.dumps(events,indent=2))
 node.destroy_node();rclpy.shutdown()
 print(json.dumps(result))
