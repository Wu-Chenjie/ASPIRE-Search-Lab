from stage68_publication import publish_ready
from pathlib import Path
import concurrent.futures as cf
import gzip,hashlib,json,math,os,queue,re,select,signal,subprocess,sys,threading,time

def parse_snapshot(topic, text):
 def integer(k):
  m=re.search(r"\b"+re.escape(k)+r":\s*(\d+)",text)
  if not m:raise ValueError('Missing '+k+' in '+topic)
  return int(m.group(1))
 def vector(k):
  m=re.search(r"\b"+re.escape(k)+r":\s*\[([^]]+)\]",text)
  if not m:raise ValueError('Missing vector '+k+' in '+topic)
  v=[float(x.strip()) for x in m.group(1).split(',')]
  if len(v)!=3 or not all(math.isfinite(x) for x in v):raise ValueError('Invalid vector '+k)
  return v
 v={'timestamp':integer('timestamp'),'timestamp_sample':integer('timestamp_sample')}
 if topic=='vehicle_imu':
  for k in ['delta_angle','delta_velocity']:v[k]=vector(k)
  for k in ['delta_angle_dt','delta_velocity_dt','delta_angle_clipping','delta_velocity_clipping']:v[k]=integer(k)
 else:
  for k in ['x','y','z']:
   m=re.search(r"\b"+k+r":\s*([^\s]+)",text)
   if not m:raise ValueError('Missing raw '+k)
   v[k]=float(m.group(1))
   if not math.isfinite(v[k]):raise ValueError('Invalid raw '+k)
 return v

def validate_ground_guard(text):
 if not re.search(r"\bDisarmed\b",text) or 'offboard' in text.lower() or not re.search(r'in failsafe:\s*no',text):
  raise RuntimeError('First-source initialization requires actual Disarmed, noOffboard and nofailsafe commander status')
 return dict(disarmed=True,no_offboard=True,no_failsafe=True)

from stage62_public_rpc import read_public

def main():
 from gz.transport13 import Node
 from gz.msgs10.clock_pb2 import Clock
 out=Path(sys.argv[1]);target=int(float(sys.argv[2])*1e6);wallcap=float(sys.argv[3]);listener='/tmp/stage54-precise-public-px4-v2/bin/px4-listener'
 cv=threading.Condition();stop=threading.Event();writerstop=threading.Event();q=queue.Queue(20000);errors=[];clock=[None];ack=None
 counts=dict(clock=0,requests=0,snapshot_attempts=0,snapshots=0,ack=0,queue_drop=0,written=0);latencies=[]
 def emit(x):
  x['wall_monotonic_ns']=time.monotonic_ns()
  try:q.put_nowait(x)
  except queue.Full:counts['queue_drop']+=1;stop.set()
 def writer():
  try:
   with gzip.open(out/'pacer-events.jsonl.gz','wt',compresslevel=1) as f:
    while not writerstop.is_set() or not q.empty():
     try:x=q.get(timeout=.05)
     except queue.Empty:continue
     f.write(json.dumps(x,separators=(',',':'),allow_nan=False)+'\n');counts['written']+=1
  except Exception as e:errors.append(repr(e));stop.set()
 w=threading.Thread(target=writer,daemon=True);w.start()
 def cb(m):
  t=int(m.sim.sec)*1000000+int(m.sim.nsec)//1000
  with cv:
   if clock[0] is not None and t<clock[0]:errors.append('Clock regressed');stop.set()
   clock[0]=t;counts['clock']+=1;emit(dict(kind='clock',timestamp=t));cv.notify_all()
 validated_topics=set()
 def snapshot(topic,wanted,deadline):
  attempts=0
  while True:
   remaining=deadline-time.monotonic()
   if remaining<=0 or stop.is_set():raise RuntimeError('Actual public feedback deadline '+topic+' '+str(wanted))
   begin=time.monotonic_ns()
   p=read_public(topic,timeout=min(.25,remaining))
   attempts+=1
   row=dict(kind='native_snapshot',topic=topic,wanted_us=wanted,returncode=p.returncode,stdout=p.stdout,stderr=p.stderr,read_begin_wall_ns=begin,raw_native_reply_base64=p.raw_reply_base64,native_protocol_elapsed_s=p.elapsed_s,transport='existing unmodified PX4 CLI Unix socket; topic whitelist')
   try:values=parse_snapshot(topic,p.stdout);row['values']=values
   except ValueError as e:values=None;row['parse_error']=str(e)
   emit(row)
   if p.returncode:raise RuntimeError('Original public listener failed '+topic)
   if values is not None:
    t=values['timestamp_sample']
    if t>wanted:raise RuntimeError('Public snapshot from future '+topic)
    if t==wanted:
     if topic not in validated_topics:
      cli=subprocess.run([listener,'--instance','1',topic],capture_output=True,text=True,timeout=.25)
      emit(dict(kind='public_protocol_equivalence',topic=topic,timestamp_sample=t,existing_CLI_stdout=cli.stdout,existing_CLI_stderr=cli.stderr,existing_CLI_returncode=cli.returncode,direct_original_protocol_stdout=p.stdout,direct_original_protocol_returncode=p.returncode,byte_equal=cli.stdout==p.stdout))
      if cli.returncode!=p.returncode or cli.stdout!=p.stdout:raise RuntimeError('Original public CLI and same-protocol reply differ')
      validated_topics.add(topic)
     if topic=='vehicle_imu' and (values['delta_angle_dt']>20000 or values['delta_velocity_dt']>20000):raise RuntimeError('Actual integral exceeds20ms')
     return values,attempts
   if time.monotonic()>=deadline:raise RuntimeError('No actual fresh public snapshot '+topic)
   stop.wait(.001)
 for sig in [signal.SIGTERM,signal.SIGINT]:signal.signal(sig,lambda *_:stop.set())
 node=Node();node.subscribe(Clock,'/world/annual/clock',cb)
 rpcerr=(out/'native-request.log').open('w');rpc=subprocess.Popen(['/tmp/stage49-native-request'],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=rpcerr,text=True)
 def request(cmd,ms):
  rpc.stdin.write(cmd+'\n');rpc.stdin.flush()
  ready,_,_=select.select([rpc.stdout],[],[],ms/1000+.25)
  if not ready:raise RuntimeError('Native control request timed out; no retry')
  line=rpc.stdout.readline()
  if not line:raise RuntimeError('Native control child exited')
  answer=json.loads(line);emit(dict(kind='native_rpc',command=cmd,**answer))
  return answer['executed'] and answer['handler_result'] and answer['data']
 bootstrap_guards=[];initial_source_wait=None;first_public_feedback=None
 def ground_guard():
  p=subprocess.run(['/tmp/stage54-precise-public-px4-v2/bin/px4-commander','--instance','1','status'],capture_output=True,text=True,timeout=.25)
  row=dict(kind='bootstrap_guard',stdout=p.stdout,stderr=p.stderr,returncode=p.returncode)
  emit(row)
  if p.returncode:raise RuntimeError('Actual ground state query failed')
  row.update(validate_ground_guard(p.stdout));bootstrap_guards.append(row)
 result=dict(candidate='metadata readiness plus lossless readonly public snapshots; no trace, no estimator modification',target_us=target,wallcap_s=wallcap,no_synthetic_measurement=True)
 begin=time.monotonic();pool=cf.ThreadPoolExecutor(max_workers=3);nextprogress=0
 try:
  deadline=time.monotonic()+5
  while '/world/annual/control' not in node.service_list():
   if time.monotonic()>deadline or stop.is_set():raise RuntimeError('Public control undiscovered')
   time.sleep(.02)
  if not request('pause',2000):raise RuntimeError('Nonstepping pause handshake failed')
  with cv:
   if clock[0] not in [None,0]:raise RuntimeError('World not paused at zero')
  ground_guard()
  for wanted in range(4000,target+1,4000):
   if time.monotonic()-begin>=wallcap or stop.is_set():raise RuntimeError('Wall/integrity/stop boundary')
   stepbegin=time.monotonic();success=request('step',500);counts['requests']+=1;emit(dict(kind='request',wanted_us=wanted,success=bool(success)))
   if not success:raise RuntimeError('Public single-step failed; no retry')
   deadline=time.monotonic()+.5
   with cv:
    while clock[0] is None or clock[0]<wanted:
     if time.monotonic()>deadline or stop.is_set():raise RuntimeError('Clock deadline')
     cv.wait(.002)
    if clock[0]!=wanted:raise RuntimeError('Extra physical step')
   if wanted==4000:
    # Only the grounded first physical initialization may wait for source generation.
    # This is a new explicitly declared5s initialization cap; it is not a pass of old500ms first-source gate.
    source_begin=time.monotonic();source_deadline=source_begin+5
    while not (out/'first-imu.json').exists():
     if time.monotonic()>source_deadline or stop.is_set():raise RuntimeError('First actual source generation exceeded grounded5s initialization cap')
     stop.wait(.002)
    first=json.loads((out/'first-imu.json').read_text())
    if first['header_ns']!=4000000:raise RuntimeError('First actual source not4ms; no synthesis or skip')
    initial_source_wait=time.monotonic()-source_begin
   public_begin=time.monotonic()
   deadline=public_begin+.5
   topics=['sensor_accel','sensor_gyro']+(['vehicle_imu'] if wanted>=16000 else [])
   pending={topic:pool.submit(snapshot,topic,wanted,deadline) for topic in topics}
   for topic,future in pending.items():
    values,attempts=future.result(timeout=max(.001,deadline-time.monotonic())+.01)
    counts['snapshot_attempts']+=attempts;counts['snapshots']+=1
    if topic=='vehicle_imu':ack=values['timestamp_sample'];counts['ack']+=1
   if wanted==4000:
    first_public_feedback=time.monotonic()-public_begin;ground_guard()
    publish_ready(out/'initial-bootstrap-ready.json',json.dumps(dict(physical_sim_us=4000,actual_raw_topics=['sensor_accel','sensor_gyro'],commander_guards=bootstrap_guards,first_source_wait_wall_s=initial_source_wait,public_feedback_wall_s=first_public_feedback,no_actuation_graph_required_during_initialization=True),indent=2)+'\n')
    if os.environ.get('STAGE58_REQUIRE_PARENT_RELEASE')=='1':
     release_deadline=time.monotonic()+6
     while not (out/'allow-pacing.json').exists():
      if time.monotonic()>release_deadline or stop.is_set():raise RuntimeError('Parent did not release after starting original bridge')
      stop.wait(.002)
   latencies.append(time.monotonic()-stepbegin)
   if time.monotonic()-begin>=nextprogress:
    print(json.dumps(dict(sim_us=clock[0],ack_us=ack,wall_s=time.monotonic()-begin)),flush=True);nextprogress+=10
  result['completed']=True
 except Exception as e:errors.append(repr(e));result['completed']=False
 finally:
  elapsed=time.monotonic()-begin
  if elapsed>wallcap and result.get('completed'):errors.append('Completed beyond wallcap');result['completed']=False
  stop.set();pool.shutdown(wait=True,cancel_futures=True)
  rpc.stdin.close()
  try:rpc.wait(timeout=1)
  except subprocess.TimeoutExpired:errors.append('Owned request child EOF timeout');rpc.terminate();rpc.wait(timeout=1)
  rpcerr.close();node.unsubscribe('/world/annual/clock');writerstop.set();w.join(1)
  if w.is_alive():errors.append('Writer drain deadline')
  ordered=sorted(latencies)
  result.update(public_protocol_validated_topics=sorted(validated_topics),initial_source_cap_s=5,initial_source_wait_wall_s=initial_source_wait,first_public_feedback_wall_s=first_public_feedback,initial_commander_guards=bootstrap_guards,last_clock_us=clock[0],last_ack_us=ack,elapsed_wall_s=elapsed,errors=errors,counts=counts,native_request_exit=rpc.returncode,native_request_binary_sha256=hashlib.sha256(Path('/tmp/stage49-native-request').read_bytes()).hexdigest(),step_wall_s=dict(max=max(ordered,default=0),p50=ordered[len(ordered)//2] if ordered else None,p95=ordered[min(len(ordered)-1,int(len(ordered)*.95))] if ordered else None))
  (out/'pacer-result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)
 return 0 if result['completed'] and not errors and not counts['queue_drop'] else 2
if __name__=='__main__':sys.exit(main())
