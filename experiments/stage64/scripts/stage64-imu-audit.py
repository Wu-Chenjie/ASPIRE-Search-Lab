from pathlib import Path
import base64,gzip,hashlib,json,struct,subprocess,sys
import numpy as np
from pyulog import ULog
from gz.msgs10.imu_pb2 import IMU
from gz.msgs10.clock_pb2 import Clock
from stage54_pacer_import import parse_snapshot
from stage64_ulog_subset import logged_subset
R=Path(sys.argv[1]);output=Path(sys.argv[2]) if len(sys.argv)>2 else R;output.mkdir(parents=True,exist_ok=True);A=output/'audit';A.mkdir(exist_ok=False)
result=json.loads((R/'result.json').read_text());result['pacer']=json.loads((R/'pacer-result.json').read_text());result['observer_final']=json.loads((R/'observer-status.json').read_text());result['actual_camera_ready']=json.loads((R/'camera-ready.json').read_text());result['target_sim_s']=result['budget_seconds'];result['wallcap_s']=result['harness_wall_watchdog_seconds'];fail=[]
def check(ok,name):
 if not ok and name not in fail:fail.append(name)
def bits(v):return b''.join(struct.pack('<f',float(x)) for x in v)
ready=result.get('actual_camera_ready',{}).get('actual_initial_camera',{});check(set(ready)=={'rgb','depth'} and all(x['stamp_ns']==0 and x['width']==320 and x['height']==240 and x['data_bytes']>0 for x in ready.values()),'two actual initial zero-sim camera frames')
check(not result.get('error') and not result.get('cleanup_error'),'controller integrity');p=result.get('pacer',{})
check(p.get('completed') and not p.get('errors') and p.get('counts',{}).get('queue_drop',1)==0,'pacer complete')
check(p.get('initial_source_cap_s')==5 and p.get('initial_source_wait_wall_s',1e10)<=5 and p.get('first_public_feedback_wall_s',1e10)<=.5 and len(p.get('initial_commander_guards',[]))==2 and all(x.get('disarmed') and x.get('no_offboard') and x.get('no_failsafe') and x.get('returncode')==0 for x in p.get('initial_commander_guards',[])),'grounded initialization and original actual-public500ms feedback')
expected=list(range(4000,int(result['target_sim_s']*1e6)+1,4000));integrals=expected[3:]
check(p.get('last_clock_us')==expected[-1] and p.get('last_ack_us')==expected[-1],'exact final feedback')
check(p.get('elapsed_wall_s',1e10)<=result['wallcap_s'],'wall budget')
c=result.get('cleanup',{});check(not c.get('forced_kill_pids') and not c.get('remaining_pids') and bool(c.get('gazebo_stop_requests')) and all(x.get('exit_code')==0 for x in c.get('gazebo_stop_requests',[])),'original normal cleanup')
o=result['observer_final'];check(not o['errors'] and o['counts']['dropped']==0 and o['counts']['written']==o['counts']['imu']+o['counts']['clock'],'observer complete')
source=[json.loads(l) for l in gzip.open(R/'gazebo-raw.jsonl.gz','rt')];raw=[x for x in source if x['channel']=='imu'];lookup={x['header_ns']//1000:x for x in raw}
check([x['header_ns']//1000 for x in raw]==expected,'all actual source headers')
for x in source:
 b=base64.b64decode(x['protobuf_base64']);m=IMU() if x['channel']=='imu' else Clock();m.ParseFromString(b);check(m.SerializeToString()==b,'protobuf exact roundtrip')
events=[json.loads(l) for l in gzip.open(R/'pacer-events.jsonl.gz','rt')];pub={k:{} for k in ['sensor_accel','sensor_gyro','vehicle_imu']};attempts=0
for row in events:
 if row['kind']=='public_protocol_equivalence':
  check(row['byte_equal'] and row['existing_CLI_returncode']==row['direct_original_protocol_returncode']==0 and row['existing_CLI_stdout']==row['direct_original_protocol_stdout'],'original CLI same-protocol complete stdout bytes equal')
 if row['kind']!='native_snapshot':continue
 attempts+=1;check(row['returncode']==0,'native public listener success');topic=row['topic']
 reply=base64.b64decode(row['raw_native_reply_base64']);check(reply[-2:]==bytes([0,row['returncode']]) and reply[:-2].decode()==row['stdout'] and row['native_protocol_elapsed_s']<=.25,'actual unmodified CLI reply/status/full-query250ms')
 try:v=parse_snapshot(topic,row['stdout'])
 except ValueError:continue
 check(v==row.get('values'),'raw stdout parsed values match saved')
 t=v['timestamp_sample'];check(t<=row['wanted_us'],'no future public feedback')
 if t!=row['wanted_us']:continue
 check(t not in pub[topic],'no duplicate accepted snapshot');pub[topic][t]=v
check(attempts==p['counts']['snapshot_attempts'],'attempt accounting complete')
check(p.get('public_protocol_validated_topics')==['sensor_accel','sensor_gyro','vehicle_imu'] and len([x for x in events if x['kind']=='public_protocol_equivalence'])==3,'all3 topics original CLI byte-equivalence proven')
for topic in ['sensor_accel','sensor_gyro']:check(list(pub[topic])==expected,topic+' complete actual public sequence')
check(list(pub['vehicle_imu'])==integrals,'every actual public integral')
public_raw={}
for topic,field in [('sensor_accel','accel_flu'),('sensor_gyro','gyro_flu')]:
 bad=[]
 for t,v in pub[topic].items():
  s=lookup.get(t)
  if s is None or bits([v[k] for k in ['x','y','z']])!=bits([s[field][0],-s[field][1],-s[field][2]]):bad.append(t)
 check(not bad,topic+' public raw Float32 source bit match');public_raw[topic]={'records':len(pub[topic]),'bit_mismatch_times':bad}
vi=pub['vehicle_imu'];csv=A/'full-actual-public-imu.csv'
with csv.open('w') as f:
 for t,v in vi.items():
  check(v['delta_angle_dt']==v['delta_velocity_dt']==(16000 if t==16000 else 4000),'actual integration period first16 then4ms')
  check(v['delta_angle_clipping']==v['delta_velocity_clipping']==0,'actual integral clipping flags')
  f.write(','.join(map(str,[t,v['delta_angle_dt']/1e6,v['delta_velocity_dt']/1e6,*v['delta_angle'],*v['delta_velocity']]))+'\n')
native=subprocess.check_output(['/tmp/stage46-native-period-matrix/probe',str(csv),'10000'],text=True);(A/'native-full-downsample.csv').write_text(native)
nr=[list(map(float,l.split(','))) for l in native.splitlines()];clips=[x for x in nr if abs(x[1]-x[2])>1e-7];check(not clips and len(nr)>0,'complete actual public integral native DownSampler zero clips')
u=ULog(str(next(R.rglob('*.ulg'))));check(not u.dropouts,'ULog dropout zero');check(int(u.initial_parameters['EKF2_PREDICT_US'])==10000,'original predict parameter');ul={}
for topic in ['sensor_accel','sensor_gyro','vehicle_imu']:
 d=u.get_dataset(topic).data;ts=list(map(int,d['timestamp_sample']));sequence=integrals if topic=='vehicle_imu' else expected
 meta=logged_subset(sequence,ts);check(meta['valid'],topic+' ULog ordered unique actual public subset');bad=[]
 for i,t in enumerate(ts):
  v=pub[topic].get(t)
  if v is None:bad.append(t);continue
  if topic=='vehicle_imu':
   if any(int(d[k][i])!=v[k] for k in ['delta_angle_dt','delta_velocity_dt','delta_angle_clipping','delta_velocity_clipping']):bad.append(t)
   if any(bits([d[k+'['+str(j)+']'][i] for j in range(3)])!=bits(v[k]) for k in ['delta_angle','delta_velocity']):bad.append(t)
  elif bits([d[k][i] for k in ['x','y','z']])!=bits([v[k] for k in ['x','y','z']]):bad.append(t)
 check(not bad,topic+' all ULog/public overlap Float32 bit match');ul[topic]={'records':len(ts),'missing_actual_public_times':meta['missing_times'],'missing_interior_times':meta['missing_interior_times'],'missing_tail_times':meta['missing_tail_times'],'metadata':meta,'bit_mismatch_times':bad}
armed=bool(np.any(u.get_dataset('actuator_armed').data['armed']));offboard=bool(np.any(u.get_dataset('vehicle_control_mode').data['flag_control_offboard_enabled']));commands=[int(c) for ds in u.data_list if ds.name=='vehicle_command' for c in ds.data['command']];check(armed and offboard and 400 in commands,'actual flight arm and offboard')
load=None
if False:
 a=[json.loads(x) for x in (R/'load.log').read_text().splitlines() if x.startswith('{')];b=[json.loads(x) for x in (R/'person_search.log').read_text().splitlines() if x.startswith('{')];load={'workers':len(a[0]['processes']),'threads':b[0]['actual_threads'],'completed_inferences':b[-1]['completed_inferences']};check(load['workers']==5 and load['threads']==2 and load['completed_inferences']>0,'actual fixed load')
check(hashlib.sha256(Path('/tmp/racer-stage13-px4/source/build/px4_sitl_default/bin/px4').read_bytes()).hexdigest()=='c359b7f3fde4678476d984a75b60d0d258146a90b0d1a3ad66e6a5b142c7968c','original production unchanged')
check(result['px4_binary_sha256']=='16f3efeceda2ce9793da5b1896e852f9f5930fbc1cc8b69b2bb58bdc70fa5ef2','new precise public binary exact')
report=dict(measurement_audit_version='Stage64-secondary-ULog-ordered-subset; all primary actual source/public requirements unchanged',ULog_not_claimed_complete=True,component_passed=not fail,failures=fail,scope='Complete actual lossless public snapshot component; no direct internal generation trace; no formal flight acceptance',target_sim_s=result['target_sim_s'],wall_s=p.get('elapsed_wall_s'),source_imu_count=len(raw),source_protobuf_roundtrips=len(source),actual_public_raw=public_raw,actual_public_integrals=len(vi),first_sample_us=min(vi,default=None),max_delta_dt_us=max([v['delta_velocity_dt'] for v in vi.values()],default=None),native_downsampled=len(nr),native_clips=clips,actual_ULog=ul,actual_load=load,actual_state_flags={'armed_any':armed,'offboard_any':offboard,'commands':commands},source_input_sha256={str(x.relative_to(R)):hashlib.sha256(x.read_bytes()).hexdigest() for x in [R/'result.json',R/'gazebo-raw.jsonl.gz',R/'pacer-events.jsonl.gz',*R.rglob('*.ulg')]})
(output/'audit.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
