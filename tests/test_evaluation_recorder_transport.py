"""Actual Gazebo messages and clean recorder shutdown; no simulator or flight."""
import os,signal,subprocess,sys,time,json
from pathlib import Path
import pytest
gz=pytest.importorskip('gz.transport13')
from gz.msgs10.pose_v_pb2 import Pose_V

@pytest.mark.parametrize('termination',[signal.SIGINT,signal.SIGTERM])
def test_recorder_drains_native_callbacks_before_interpreter_shutdown(tmp_path,termination):
 root=Path(__file__).resolve().parents[1]
 env={**os.environ,'GZ_PARTITION':'stage32-recorder-'+str(os.getpid())+'-'+str(int(termination))}
 # Publisher and recorder must share only this explicit synthetic truth topic.
 old=os.environ.get('GZ_PARTITION');os.environ['GZ_PARTITION']=env['GZ_PARTITION']
 node=gz.Node();pub=node.advertise('/world/annual/dynamic_pose/info',Pose_V)
 try:
  with (tmp_path/'recorder.log').open('w') as log:
   proc=subprocess.Popen([sys.executable,'-m','tools.flight.evaluation_recorder',str(tmp_path)],
                         cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT)
   try:
    deadline=time.monotonic()+8
    while not (tmp_path/'evaluation-recorder.status.json').exists():
     assert proc.poll() is None
     assert time.monotonic()<deadline
     time.sleep(.02)
    for i in range(120):
     m=Pose_V();m.header.stamp.sec=10;m.header.stamp.nsec=i*1000000;p=m.pose.add();p.name='annual_x500_0';p.orientation.w=1;p.position.z=.1
     pub.publish(m);time.sleep(.01)
    proc.send_signal(termination);assert proc.wait(timeout=6)==0,(tmp_path/'recorder.log').read_text()
    status=json.loads((tmp_path/'evaluation-recorder.status.json').read_text())
    assert status['state']=='closed' and status['callbacks_drained'] and not status['errors']
    rows=[json.loads(l) for l in (tmp_path/'truth-poses.jsonl').read_text().splitlines()]
    assert len(rows)>0 and len(rows)==status['counts']['poses']
    assert not (root/'core').exists()
   finally:
    if proc.poll() is None:proc.kill();proc.wait(timeout=3)
 finally:
  del pub;del node
  if old is None:os.environ.pop('GZ_PARTITION',None)
  else:os.environ['GZ_PARTITION']=old

