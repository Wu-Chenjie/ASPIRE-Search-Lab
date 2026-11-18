"""Record simulator-only collision probe without publishing any decision input."""
import json
from pathlib import Path
import signal
import sys
import threading
from gz.transport13 import Node
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.contacts_pb2 import Contacts

out=Path(sys.argv[1]);node=Node();stop=threading.Event()
f=(out/'collision-probe.jsonl').open('x',buffering=1)
def poses(m):
    for p in m.pose:
        if p.name=='hm3d_collision_probe':
            f.write(json.dumps(dict(kind='pose',stamp=m.header.stamp.sec+m.header.stamp.nsec*1e-9,
                                   xyz=[p.position.x,p.position.y,p.position.z]))+'\n')
def contacts(m):
    f.write(json.dumps(dict(kind='contact',stamp=m.header.stamp.sec+m.header.stamp.nsec*1e-9,
        contacts=[dict(a=c.collision1.name,b=c.collision2.name,depths=list(c.depth)) for c in m.contact]))+'\n')
node.subscribe(Pose_V,'/world/annual/dynamic_pose/info',poses)
node.subscribe(Contacts,'/world/annual/model/hm3d_collision_probe/link/body/sensor/contact/contact',contacts)
for sig in (signal.SIGTERM,signal.SIGINT):signal.signal(sig,lambda *_:stop.set())
stop.wait()
