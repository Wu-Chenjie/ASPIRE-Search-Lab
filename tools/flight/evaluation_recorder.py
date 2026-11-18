"""Independent raw Gazebo recorder; no ROS publisher or decision input."""
import gc,json,signal,sys,threading,time
from pathlib import Path
from gz.transport13 import Node
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.contacts_pb2 import Contacts

def main():
 out=Path(sys.argv[1]);node=Node();stop=threading.Event();condition=threading.Condition()
 active=0;closing=False;counts=dict(poses=0,contacts=0);errors=[]
 files=[(out/'truth-poses.jsonl').open('x',buffering=1),(out/'contacts.jsonl').open('x',buffering=1)]
 status=out/'evaluation-recorder.status.json'
 def save(state,**extra):status.write_text(json.dumps(dict(state=state,counts=counts,errors=errors,**extra),indent=2)+'\n')
 def enter():
  nonlocal active
  with condition:
   if closing:return False
   active+=1;return True
 def leave():
  nonlocal active
  with condition:active-=1;condition.notify_all()
 def poses(m):
  if not enter():return
  try:
   stamp=m.header.stamp.sec+m.header.stamp.nsec*1e-9
   for p in m.pose:
    if p.name!='annual_x500_0':continue
    q=p.orientation;v=p.position
    files[0].write(json.dumps(dict(stamp=stamp,parent='world',child=p.name,xyz=[v.x,v.y,v.z],quat=[q.x,q.y,q.z,q.w],definition='model origin; base_link is rotated [0,0,0.24] offset'))+'\n')
    counts['poses']+=1
  except Exception as e:errors.append(str(e));stop.set()
  finally:leave()
 def contacts(m):
  if not enter():return
  try:
   files[1].write(json.dumps(dict(stamp=m.header.stamp.sec+m.header.stamp.nsec*1e-9,contacts=[dict(a=c.collision1.name,b=c.collision2.name,depths=list(c.depth)) for c in m.contact]))+'\n')
   counts['contacts']+=1
  except Exception as e:errors.append(str(e));stop.set()
  finally:leave()
 topics=['/world/annual/dynamic_pose/info','/world/annual/model/annual_x500_0/link/base_link/sensor/evaluation_contact/contact']
 node.subscribe(Pose_V,topics[0],poses);node.subscribe(Contacts,topics[1],contacts)
 for sig in (signal.SIGTERM,signal.SIGINT):signal.signal(sig,lambda *_:stop.set())
 save('recording');stop.wait()
 # Release native subscriptions while Python callbacks/interpreter are alive.
 with condition:closing=True
 unsubscribed={topic:bool(node.unsubscribe(topic)) for topic in topics}
 deadline=time.monotonic()+3.
 with condition:
  while active and time.monotonic()<deadline:condition.wait(timeout=.05)
  drained=active==0
 if not drained:errors.append('callback drain timeout')
 if drained:
  for f in files:f.flush();f.close()
 save('closed' if drained and not errors else 'failed',callbacks_drained=drained,unsubscribed=unsubscribed)
 del node;gc.collect()
 if errors:raise SystemExit(1)
if __name__=='__main__':main()

