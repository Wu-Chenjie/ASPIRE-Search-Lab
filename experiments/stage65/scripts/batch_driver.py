from pathlib import Path
import subprocess,sys,json,datetime,hashlib
E=Path(__file__).resolve().parents[1];D=['docker','--context','colima-racer-gvp'];C='annual-px4-research'
def save(**kwargs):
 p=E/'STATUS.json';s=json.loads(p.read_text());s.update(updated_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),**kwargs);p.write_text(json.dumps(s,indent=2)+'\n')
def log(s):print(datetime.datetime.now(datetime.timezone.utc).isoformat(),s,flush=True)
pre=json.loads((E/'PREFLIGHT.json').read_text());assert pre['complete_imu_engineering_passed'] and pre['new_entry_tests_passed'] and pre['prospective_capture_contract_frozen'] and pre['no_flight_components_passed']
g=json.loads((E/'GATE-REUSE.json').read_text());f=Path(g['actual_gate']);assert hashlib.sha256(f.read_bytes()).hexdigest()==g['actual_gate_sha256'];assert all(json.loads(f.read_text())['checks'].values());assert g['not_a_new_full_Stage65_stack_gate']
b=json.loads((E/'BATCH.json').read_text());queue=[(p['id']+'-'+arm,p['condition'],arm) for p in b['pairs'] for arm in p['order']];assert len(queue)==12
assert not list((E/'attempts').glob('racer-stage65-*.json')),'Inspect existing attempt; never relaunch'
log('Uniform new12 cohort, no inherited performance arms; original actualG01 is historical control gate, new passive measurement differs explicitly')
for index,(tag,condition,arm) in enumerate(queue):
 free=int(subprocess.check_output(D+['exec',C,'python3','-c','import shutil;print(shutil.disk_usage("/tmp").free)'],text=True));required=(len(queue)-index)*2284793818+500000000
 if free<required:save(phase='remaining_capacity_stop',active_simulation=False,batch_closed=True,free_bytes=free,required_bytes=required);break
 log('START '+tag+' '+condition+' '+arm+' once')
 with (E/'diagnostics'/(tag+'-controller.log')).open('x') as f:p=subprocess.run([sys.executable,str(E/'scripts/run_one.py'),tag,condition,arm],stdout=f,stderr=subprocess.STDOUT)
 if p.returncode:save(phase='controller_or_preservation_error_inspect_no_relaunch',active_simulation='inspect',batch_closed=True,error_run=tag);raise RuntimeError('Inspect actual runtime; no relaunch')
 run='racer-stage65-'+tag.lower()
 with (E/'diagnostics'/(tag+'-offline-audit.log')).open('x') as f:p=subprocess.run([sys.executable,str(E/'scripts/audit_one.py'),run],stdout=f,stderr=subprocess.STDOUT)
 if p.returncode:save(phase='offline_audit_error_no_next_arm',active_simulation=False,batch_closed=True,error_run=tag);raise RuntimeError('Offline audit failed')
 if json.loads((E/'STATUS.json').read_text()).get('batch_closed'):log('Original core/prospective recording hardfailure; remaining arms not started');break
 log('AUDITED '+tag+' passed required core/recording checks; no-motion/soft outcomes retained')
else:
 save(phase='all12_uniform_measurement_arms_audited',active_simulation=False,batch_closed=True,task_complete_pending_final_report=True,total_actual_paired_arms=12,new_stage65_arms=12,inherited_performance_arms=0);log('Original fixed6pairs collected under uniform Stage65 measurement version')
