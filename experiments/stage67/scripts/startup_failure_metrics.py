from pathlib import Path
import json,sys
E=Path(__file__).resolve().parents[1];run=sys.argv[1];p=Path('/Volumes/ColimaRuntime/runtime/research-stage67/runs')/run
r=json.loads((p/'result.json').read_text());end=r['budget_end_sim'];truth=json.loads((p/'evaluation-truth.json').read_text())
events={a['name']:{k:dict(event=False,processing_sim_s=None,capture_sim_s=None,observation_end_sim_s=end,end_reason=r['error'],early_failure=True) for k in ['first_detected','first_visual_confirmed','first_independently_valid_surface']} for a in truth['actors']}
m=dict(run=run,condition=json.loads((E/'attempts'/(run+'.json')).read_text())['condition'],arm='B' if r['bearing_bias'] else 'A',sim_s=end,wall_s=r['wall_s'],rtf=end/r['wall_s'],error=r['error'],strict_gate_pass=False,
checks=dict(full_budget=False,actual_flight=False,deferred_graph_exercised=False,no_safety_effectiveness_claim=None),times=events,true_actor_count=len(truth['actors']),verified_unique_confirmed=0,verified_recall=0. if truth['actors'] else None,algorithm_confirmed_final=None,final=None,peak_duplicate_excess=0,duplicate_duration_s=0.,snapshots_with_duplicate_excess=0,surface_qualified=0,surface_valid=0,surface_failure=0,surface_error_m=dict(n=0),saved_odom_path_m=0.,limits='Startup failed before deferred graph; no detector/native planner/executor validation, no inferred false-positive rate or synthetic perception.')
(E/'audits'/(run+'-fair-metrics.json')).write_text(json.dumps(m,indent=2)+'\n')
p=E/'STATUS.json';s=json.loads(p.read_text());s.update(phase='closed_startup_failure',active_simulation=False,batch_closed=True,no_retry=True,last_audited_run=run,failed_checks=['startup_before_deferred_graph']);p.write_text(json.dumps(s,indent=2)+'\n')
print(json.dumps(m,indent=2))
