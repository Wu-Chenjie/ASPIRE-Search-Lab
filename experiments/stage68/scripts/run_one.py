from pathlib import Path
import json,os,subprocess,time,signal,datetime,hashlib,sys
B=Path('/Users/wuchenjie/Documents/Codex/2026-10-13');S=B/'task-3-stage45';E=Path('/Volumes/WuChenjie/Mac-runtime/runtime-stage52-20261018T1315Z/evidence/stage68');D=['docker','--context','colima-racer-gvp'];C='annual-px4-research';run='racer-stage68-'+sys.argv[1].lower();condition=sys.argv[2];arm=sys.argv[3];assert condition in ['C0','C1','C2'] and arm in ['A','B'];remote='/tmp/'+run

def ex(args):return subprocess.run(D+['exec',C]+args,capture_output=True,text=True,check=True)
def status(**kwargs):
 p=E/'STATUS.json';x=json.loads(p.read_text());x.update(updated_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),**kwargs);p.write_text(json.dumps(x,indent=2)+'\n')
def progress(s):
 print(s,flush=True)
 with (E/'PROGRESS.md').open('a') as f:f.write('\n'+datetime.datetime.now(datetime.timezone.utc).isoformat()+'\n\n'+s+'\n')
pre=json.loads((E/'PREFLIGHT.json').read_text());assert pre['stable_native_shutdown_validation_passed']
batch=json.loads((E/'BATCH.json').read_text());allowed={p['tag'].lower():(p['condition'],p['arm']) for p in batch['new_queue']};assert allowed.get(sys.argv[1].lower())==(condition,arm);assert len(list((E/'attempts').glob('racer-stage68-*.json')))<4
assert pre['jazzy_regression']==[519,7] and pre['host_regression']==[367,36] and pre['complete_imu_engineering_passed'] and pre['new_entry_tests_passed']
manifest=json.loads((E/'BASE-SOURCE-MANIFEST.json').read_text());assert all(hashlib.sha256((S/n).read_bytes()).hexdigest()==h for n,h in manifest.items())
layer=json.loads((E/'ENTRY-MANIFEST.json').read_text());assert hashlib.sha256((Path('/Volumes/WuChenjie/Mac-runtime/runtime-stage52-20261018T1315Z/evidence/stage68/scripts/stage68-flight-entry.py')).read_bytes()).hexdigest()==layer['actual_entry_sha256']
remote_layer=pre['verified_current_hashes'];probe='from pathlib import Path;import hashlib;expected='+repr(remote_layer)+';assert all(hashlib.sha256(Path(n).read_bytes()).hexdigest()==h for n,h in expected.items())';ex(['python3','-c',probe])
free=int(ex(['python3','-c','import os;v=os.statvfs("/tmp");print(v.f_bavail*v.f_frsize)']).stdout);assert free>=2284793818
assert ex(['test','!','-e',remote]).returncode==0
assert ex(['test','-f','/tmp/stage29-logger-topics.txt']).returncode==0
assert ex(['bash','-c','test -z "$RACER_FLIGHT_FIXTURE" || test "$RACER_FLIGHT_FIXTURE" = none']).returncode==0
scenes=json.loads((E/'scene-manifest.json').read_text());scene_reference=Path(pre['scene_reference_root'])
assert all(hashlib.sha256((scene_reference/n).read_bytes()).hexdigest()==h for n,h in scenes.items())
assert not [p for p in (E/'attempts').glob('*.json') if p.stem==run] if (E/'attempts').exists() else True

cmd='source /tmp/stage45-setup.sh\nexport RACER_GZ_SHUTDOWN_GUARD=/tmp/racer-stage44-gazebo-guard/libgz_sensor_stop_guard.so\nexport RACER_RECORD_INPUT=1 RACER_EXECUTION_SAFETY=1 RACER_CPU_PROFILE=render3-perception2\nexport RACER_CONTROL_OBSERVATION=1 RACER_CONTROL_OBSERVATION_GZIP=1 RACER_OBSERVATION_SESSION='+run+'\nexport RACER_MAP_EVIDENCE_DIR='+remote+'/map-evidence\nexport RACER_PX4_LOGGER_TOPICS=/tmp/stage48-ground-chain/logger-topics.txt\necho $$ > /tmp/'+run+'.runner.pid\nexec /usr/bin/python3 /tmp/stage68-bootstrap/stage68-flight-entry.py --px4-build /tmp/stage54-precise-public-px4-v2 --detector yolo11n-seg --segmentation-weights /tmp/racer-stage10-assets/yolo11n-seg.pt --detector-python-path /tmp/racer-stage10-python --performance consistent --stop-and-plan --recover-no-path --confirmation-mode registry --budget-clock sim --prepared /tmp/racer-stage42-scenes/'+condition+' --seconds 180 --wall-watchdog-seconds 1260 '+('--bearing-bias ' if arm=='B' else '')+('--negative ' if condition=='C0' else '')+'--output '+remote
(E/'attempts').mkdir(exist_ok=True)
with (E/'attempts'/(run+'.json')).open('x') as f:json.dump(dict(run=run,command=cmd,source=str(S),free=free,max_runs=4,no_relaunch=True,condition=condition,arm=arm,start_utc=datetime.datetime.now(datetime.timezone.utc).isoformat()),f,indent=2)
status(phase='single_dynamic_run',active_simulation=True,actual_runs=len(list((E/'attempts').glob('racer-stage68-*.json'))),new_dynamic_search_runs=len(list((E/'attempts').glob('racer-stage68-*.json'))),current_run=run)
progress('START '+run+'; one attempt per declared ID, 180sim/1260wall, frozen original safety limits.')
start=time.monotonic();abort=None;probe=None;probe_started=None;probe_done=False;checkpoint_checked=False;failed_transport=False

def stop(reason):
 global abort
 if abort:return
 abort=reason
 script='from pathlib import Path;import os,signal;p=int(Path("/tmp/'+run+'.runner.pid").read_text());raw=(Path("/proc")/str(p)/"cmdline").read_bytes();assert b"stage68-flight-entry.py" in raw and '+repr(remote.encode())+' in raw;os.kill(p,signal.SIGINT)'
 ex(['python3','-c',script]);progress('SUPERVISOR_STOP '+reason)
with (E/(run+'-runner.log')).open('x') as log,(E/(run+'-monitor.jsonl')).open('x',buffering=1) as observations:
 proc=subprocess.Popen(D+['exec',C,'bash','-c',cmd],stdout=log,stderr=subprocess.STDOUT)
 while proc.poll() is None:
  time.sleep(1)
  try:
   m=json.loads(ex(['python3','/tmp/stage68-bootstrap/stage68-monitor.py',remote,run]).stdout);observations.write(json.dumps(dict(wall_elapsed_s=time.monotonic()-start,host_loadavg=os.getloadavg(),**m))+'\n')
   if m['errors'] and not m['closed_status'] and not m['safety_fault']:stop(';'.join(m['errors']))
   h=m['health']
   if not abort and not m['safety_fault'] and probe is None and all(role in h for role in ['mapper','executor','gateway']):
    ps='source /tmp/stage45-setup.sh; export ROS_DOMAIN_ID=197 ROS_LOCALHOST_ONLY=1; timeout 12s python3 /tmp/stage66-measurement/stage66-startup-probe.py '+remote+'/startup-capture-probe.json '+run
    probe_log=(E/(run+'-startup-probe.log')).open('x');probe=subprocess.Popen(D+['exec',C,'bash','-c',ps],stdout=probe_log,stderr=subprocess.STDOUT);probe_started=time.monotonic();progress('STARTUP_CAPTURE_PROBE '+str(m['last_sim_s'])+' '+run)
   if probe is not None and probe.poll() is not None and not probe_done:
    probe_done=True
    if probe.returncode:stop('startup_capture_probe_process_failed')
    else:progress('STARTUP_CAPTURE_PROBE_PUBLISHED')
   if probe_done and probe.returncode==0 and not checkpoint_checked and time.monotonic()-probe_started>10 and not m['closed_status'] and not m['safety_fault']:
    script="from pathlib import Path\nimport json,numpy as np\np=Path(REMOTE)/'map-evidence';out={}\nfor role in ['mapper','executor','gateway']:\n rows=[]\n for f in (p/('drone-0-'+role)).glob('*-probe.npz'):\n  with np.load(f,allow_pickle=False) as z:\n   x=json.loads(bytes(z['metadata_json']).decode());a=x['record']\n   if a.get('kind')=='probe_current_state' and a.get('probe_id')=='stage65-startup-probe' and a.get('policy_query_performed') is False:rows.append(a)\n out[role]=len(rows)\nassert all(v==1 for v in out.values()),out\nprint(json.dumps(out))\n".replace('REMOTE',repr(remote))
    try:record=ex(['python3','-c',script]);checkpoint_checked=True;(E/(run+'-startup-checkpoint.json')).write_text(record.stdout);progress('STARTUP_CAPTURE_CHECKPOINT_VERIFIED '+record.stdout.strip())
    except Exception:stop('startup_probe_current_state_missing_or_corrupt')
   if time.monotonic()-start>1320 and not abort:stop('host_transport_wait_after_runner_watchdog')
   if time.monotonic()-start>1360:failed_transport=True;break
  except Exception as e:
   progress('MONITOR_EXCEPTION '+repr(e))
   with (E/(run+'-monitor-failures.log')).open('a') as detail:
    detail.write(repr(e)+'\nSTDOUT\n'+str(getattr(e,'stdout',None))+'\nSTDERR\n'+str(getattr(e,'stderr',None))+'\n')
   if proc.poll() is None and not abort:
    try:stop('supervision_transport_or_decode_failure')
    except Exception:failed_transport=True;break
 if failed_transport:
  status(phase='transport_recovery_required',active_simulation=True,supervisor_abort=abort);raise RuntimeError('Inspect actual runner; never relaunch')
 code=proc.wait()
 if probe is not None:probe.wait(timeout=15)
progress('RUNNER_FINISHED '+str(code)+'; audit before any next declared run.')
status(phase='preserving_single_run',active_simulation=False,supervisor_abort=abort,runner_exit_code=code)
# Independently audit real public IMU before the archival snapshot; a failed audit never suppresses preservation.
imu_log=E/'diagnostics'/(run+'-imu-audit.log')
with imu_log.open('x') as f:
 imu_proc=subprocess.run(D+['exec','-e','PYTHONDONTWRITEBYTECODE=1',C,'bash','-c','source /tmp/stage45-setup.sh\nPYTHONPATH=/tmp:$PYTHONPATH python3 /tmp/stage64-imu-audit.py '+remote],stdout=f,stderr=subprocess.STDOUT)
if imu_proc.returncode:
 payload=json.dumps(dict(component_passed=False,failures=['Independent full-IMU audit process failed; see external log'],audit_process_exit=imu_proc.returncode,missing_inputs_not_fabricated=True))
 fallback='from pathlib import Path; p=Path('+repr(remote+'/audit.json')+');assert not p.exists();p.write_text('+repr(payload)+')'
 ex(['python3','-c',fallback])
# Keep both new copies. No extra deletion authorization.
subprocess.run([sys.executable,str(E.parents[1]/'scripts/archive-container-directory.py'),remote,run],check=True)
view_record=json.loads(subprocess.check_output([sys.executable,str(E.parents[1]/'scripts/create-analysis-view.py'),str(E.parents[1]/'evidence'/(run+'.pax.tar')),str(E.parents[1]/'audit'/(run+'-source-manifest.json')),'/Volumes/ColimaRuntime/runtime/research-stage68/runs',run],text=True))
saved=Path(view_record['analysis_view'])
host={str(p.relative_to(saved)):hashlib.sha256(p.read_bytes()).hexdigest() for p in saved.rglob('*') if p.is_file() and not p.is_symlink()}
script='from pathlib import Path;import hashlib,json;p=Path('+repr(remote)+');print(json.dumps({str(f.relative_to(p)):hashlib.sha256(f.read_bytes()).hexdigest() for f in p.rglob("*") if f.is_file() and not f.is_symlink()}))'
actual=json.loads(ex(['python3','-c',script]).stdout);assert actual==host
(E/(run+'-raw-manifest.json')).write_text(json.dumps(host,indent=2)+'\n')
r=json.loads((saved/'result.json').read_text()) if (saved/'result.json').exists() else dict(error='missing_result')
stopinfo=dict(run=run,runner_exit_code=code,supervisor_abort=abort,sim_s=r.get('budget_end_sim'),wall_s=r.get('wall_s'),error=r.get('error'),raw_files=len(host),raw_bytes=sum((saved/n).stat().st_size for n in host),raw_hashes_equal=True,remote_copy_retained=True,external_PAX_verified=True,analysis_view=str(saved),symlink_metadata_in_PAX=True,no_relaunch=True,batch_closed=False,actual_runs=len(list((E/'attempts').glob('racer-stage68-*.json'))),condition=condition,arm=arm,probe_launched=probe is not None,startup_checkpoint_verified=False,startup_probe_states_verified=checkpoint_checked,capture_contract_version='Stage66 bounded actual acknowledgment; immutable Stage65 typed state/query capture; old-count sensitivity separate')
(E/(run+'-stop.json')).write_text(json.dumps(stopinfo,indent=2)+'\n');status(phase='offline_audit',active_simulation=False,batch_closed=False,stop=stopinfo);progress('ATTEMPT_CLOSED '+json.dumps(stopinfo))
