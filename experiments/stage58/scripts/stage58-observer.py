import base64,gzip,json,os,queue,signal,sys,threading,time
from pathlib import Path
from gz.transport13 import Node
from gz.msgs10.imu_pb2 import IMU
from gz.msgs10.clock_pb2 import Clock
from gz.msgs10.image_pb2 import Image

def ns(t): return int(t.sec)*1000000000+int(t.nsec)
class Recorder:
 def __init__(self,out,maxqueue=20000):
  self.out=Path(out);self.q=queue.Queue(maxqueue);self.lock=threading.Lock();self.closing=False;self.seq=0;self.counts={'imu':0,'clock':0,'dropped':0,'written':0,'ignored_after_close':0};self.errors=[];self.last_clock_ns=0;self.done=False
  self.writer=threading.Thread(target=self.write,daemon=True);self.writer.start()
 def capture(self,channel,msg):
  wall=time.monotonic_ns()
  with self.lock:
   if self.closing:self.counts['ignored_after_close']+=1;return
   self.seq+=1;self.counts[channel]+=1
   row=dict(channel=channel,callback_seq=self.seq,wall_monotonic_ns=wall,protobuf_base64=base64.b64encode(msg.SerializeToString()).decode())
   if channel=='imu':
    row.update(header_ns=ns(msg.header.stamp),entity_name=msg.entity_name,accel_flu=[msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z],gyro_flu=[msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z])
    if self.counts['imu']==1:
     temp=self.out/'first-imu.tmp';temp.write_text(json.dumps(row)+'\n');temp.replace(self.out/'first-imu.json')
   else:
    self.last_clock_ns=ns(msg.sim);row.update(sim_ns=self.last_clock_ns,real_ns=ns(msg.real),system_ns=ns(msg.system))
   try:self.q.put_nowait(row)
   except queue.Full:self.counts['dropped']+=1
 def status(self):
  with self.lock:return dict(counts=dict(self.counts),errors=list(self.errors),last_clock_ns=self.last_clock_ns,queue_size=self.q.qsize(),closing=self.closing,writer_done=self.done,pid=os.getpid())
 def save(self):
  p=self.out/'observer-status.json';tmp=self.out/'observer-status.tmp';tmp.write_text(json.dumps(self.status(),indent=2)+'\n');tmp.replace(p)
 def write(self):
  try:
   with gzip.open(self.out/'gazebo-raw.jsonl.gz','wt',compresslevel=1) as f:
    while True:
     try:row=self.q.get(timeout=.1)
     except queue.Empty:
      if self.closing:break
      continue
     f.write(json.dumps(row,separators=(',',':'),allow_nan=False)+'\n')
     with self.lock:self.counts['written']+=1
     if f.tell()>2000000000:raise RuntimeError('Observer uncompressed byte cap exceeded')
  except Exception as e:
   with self.lock:self.errors.append(repr(e))
  finally:self.done=True
 def close(self):
  with self.lock:self.closing=True
  self.writer.join(1)
  if self.writer.is_alive():self.errors.append('writer did not drain within 1 second')
  self.save()

def selftest():
 import tempfile
 out=Path(tempfile.mkdtemp(prefix='stage48-observer-test-'));r=Recorder(out)
 imu=IMU();imu.header.stamp.sec=123;imu.header.stamp.nsec=456789123;imu.entity_name='imu';imu.linear_acceleration.z=9.80665
 clock=Clock();clock.sim.sec=123;clock.sim.nsec=456789123
 for _ in range(30):r.capture('imu',imu);r.capture('clock',clock)
 r.close();rows=[json.loads(x) for x in gzip.open(out/'gazebo-raw.jsonl.gz','rt')];assert len(rows)==60 and r.counts['dropped']==0 and not r.errors
 for row in rows:
  raw=base64.b64decode(row['protobuf_base64']);m=IMU() if row['channel']=='imu' else Clock();m.ParseFromString(raw);assert m.SerializeToString()==raw
 assert rows[0]['header_ns']==123456789123
 r2=Recorder(out/'unused') if False else None
 # Exercise visible overflow separately without a writer racing queue occupancy.
 saved=r.q;r.q=queue.Queue(1);r.closing=False;r.capture('imu',imu);r.capture('imu',imu);assert r.counts['dropped']==1;r.closing=True;r.q=saved
 print(json.dumps(dict(selftest='passed',rows=len(rows),integer_ns=rows[0]['header_ns'],overflow_visible=True,artifact=str(out))))
if __name__=='__main__':
 if '--selftest' in sys.argv:selftest();sys.exit()
 out=Path(sys.argv[1]);r=Recorder(out);stop=threading.Event()
 for sig in [signal.SIGINT,signal.SIGTERM]:signal.signal(sig,lambda *_:stop.set())
 node=Node();imu_topic='/world/annual/model/annual_x500_0/link/base_link/sensor/imu_sensor/imu';clock_topic='/world/annual/clock'
 node.subscribe(IMU,imu_topic,lambda msg:r.capture('imu',msg));node.subscribe(Clock,clock_topic,lambda msg:r.capture('clock',msg))
 initial={};initial_lock=threading.Lock();initial_unsubscribed=False
 camera_topics={'rgb':'/drone_0/rgb/image','depth':'/drone_0/depth/image'}
 def camera_initial(name,msg):
  with initial_lock:
   if name in initial:return
   row=dict(topic=camera_topics[name],stamp_ns=ns(msg.header.stamp),width=int(msg.width),height=int(msg.height),step=int(msg.step),data_bytes=len(msg.data),wall_monotonic_ns=time.monotonic_ns())
   if row['stamp_ns']!=0 or row['width']!=320 or row['height']!=240 or row['step']<=0 or row['data_bytes']<=0:
    r.errors.append('Invalid initial paused actual camera '+name);return
   initial[name]=row
   if len(initial)==2:
    temp=out/'camera-ready.tmp';temp.write_text(json.dumps(dict(actual_initial_camera=initial,zero_sim_no_physics=True),indent=2)+'\n');temp.replace(out/'camera-ready.json')
 for name,topic in camera_topics.items():node.subscribe(Image,topic,lambda msg,k=name:camera_initial(k,msg))
 (out/'observer-ready.json').write_text(json.dumps(dict(imu_topic=imu_topic,clock_topic=clock_topic,pid=os.getpid())))
 try:
  while not stop.wait(.2):
   if not initial_unsubscribed and (out/'camera-ready.json').exists():
    for topic in camera_topics.values():node.unsubscribe(topic)
    initial_unsubscribed=True
   r.save()
   if r.errors or r.counts['dropped']:break
 finally:
  # Closing is atomic with callbacks; unsubscribe stops new reception, not a fake data drain.
  with r.lock:r.closing=True
  node.unsubscribe(imu_topic);node.unsubscribe(clock_topic)
  for topic in camera_topics.values():node.unsubscribe(topic)
  r.close()
 if r.errors or r.counts['dropped']:sys.exit(2)
