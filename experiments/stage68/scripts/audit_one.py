from pathlib import Path
import subprocess,sys,json,datetime,hashlib
E=Path('/Volumes/WuChenjie/Mac-runtime/runtime-stage52-20261018T1315Z/evidence/stage68');run=sys.argv[1];p=Path('/Volumes/ColimaRuntime/runtime/research-stage68/runs')/run
assert (E/(run+'-raw-manifest.json')).exists() and (p/'result.json').exists()
raw_result=json.loads((p/'result.json').read_text())
if not raw_result.get('deferred_graph_started'):
 subprocess.run([sys.executable,str(E/'scripts/startup_failure_metrics.py'),run],check=True)
 sys.exit(0)
sys.path.insert(0,str(E.parent/'stage66/scripts'))
from stage66_capture_audit import validate
capture=validate(p,run);(E/'audits'/(run+'-capture-contract.json')).write_text(json.dumps(capture,indent=2)+'\n')
replay_cmd=['docker','--context','colima-racer-gvp','exec','annual-px4-research','bash','-c','source /tmp/stage45-setup.sh\nPYTHONPATH=/tmp/stage65-measurement:$PYTHONPATH python3 /tmp/stage65-measurement/stage65_rejection_replay.py /tmp/'+run]
rejection=json.loads(subprocess.check_output(replay_cmd,text=True));(E/'audits'/(run+'-rejection-replay.json')).write_text(json.dumps(rejection,indent=2)+'\n')
commands=[['dynamic_truth_audit.py',run,str(E/'audits')],['metrics_audit.py',run],['fair_metrics.py',run]]
for args in commands:
 with (E/'diagnostics'/(run+'-'+args[0]+'.log')).open('x') as f:
  completed=subprocess.run([sys.executable,str(E/'scripts'/args[0]),*args[1:]],stdout=f,stderr=subprocess.STDOUT)
 assert completed.returncode==0,args
result=json.loads((E/'audits'/(run+'-fair-metrics.json')).read_text())
status=json.loads((E/'STATUS.json').read_text());status.update(updated_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),phase='gate_passed_pair_comparison_pending' if run.endswith('g01') and result['strict_gate_pass'] else 'audit_complete',active_simulation=False,last_audited_run=run,last_strict_gate_pass=result['strict_gate_pass'])
hard_keys=['shared_measurement_layer','grounded_initialization_before_bridge','common_real_imu_pacing_complete','common_native_shutdown_guard_verified','shared_cpu_profile_installed','detector_thread_contract_installed','control_observation_no_loss','native_config_matches_arm','same_frozen_runtime_source','no_fixture','full_budget','actual_flight','no_safety_fault','no_px4_protection','no_unexpected_process_exit','no_collision','independent_recorder_closed','probe_current_state_capture_verified','execution_capture_branch_verified','rejection_contexts_replay_consistent','capture_health_no_loss','baseline_guidance_unchanged']
hard_failures=[k for k in hard_keys if not result['checks'][k]]
failed=[k for k,v in result['checks'].items() if not v] if run.endswith('g01') else hard_failures
if failed:status.update(phase='batch_paused_failed_gate_or_run',batch_closed=True,no_retry=True,failed_checks=failed)
else:status.update(phase='gate_passed_pair_comparison_pending' if run.endswith('g01') else 'pair_run_audited',batch_closed=False,failed_checks=[])
(E/'STATUS.json').write_text(json.dumps(status,indent=2)+'\n')
print(json.dumps({k:result[k] for k in ['run','strict_gate_pass','checks','error','sim_s','wall_s','final','surface_failure','surface_error_m']},indent=2),flush=True)

