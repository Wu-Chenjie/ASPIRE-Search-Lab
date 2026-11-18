"""Post-run metrics and frozen gate; no runtime truth access."""
from pathlib import Path
import json,sys,collections,bisect,math
import numpy as np
B=Path('/Users/wuchenjie/Documents/Codex/2026-10-13');E=Path('/Volumes/WuChenjie/Mac-runtime/runtime-stage52-20261018T1315Z/evidence/stage66')
p=Path('/Volumes/ColimaRuntime/runtime/research-stage66/runs')/sys.argv[1];O=E/'audits';O.mkdir(exist_ok=True)
def read(f):return [json.loads(s) for s in f.read_text().splitlines()] if f.exists() else []
def stats(v):
 v=[float(a) for a in v if a is not None and math.isfinite(a)]
 return dict(n=len(v),median=float(np.median(v)),p95=float(np.percentile(v,95)),max=max(v)) if v else dict(n=0)
def load(n):return json.loads((O/(p.name+'-'+n+'.json')).read_text())
r=json.loads((p/'result.json').read_text());conf=load('confirmation-truth');ident=load('temporal-identity');surf=load('dynamic-surface')
rows=read(p/'perception/observations.jsonl');tel=read(p/'telemetry.jsonl')
sem={int(x['stamp_ns']):x for x in rows if x['kind']=='semantic'}
select=lambda k:[x['data'] for x in tel if x['kind']==k]
records=select('person_records');execs=select('execution');flights=select('flight')
matched=collections.defaultdict(list)
for s in ident['samples']:
 if s.get('actor'):
  report=sem.get(s['capture_ns'])
  if report:matched[s['counter']].append((report['processing_time_s'],s['capture_ns']*1e-9,s['actor']))
for v in matched.values():v.sort()
actors=conf['true_scene_actors'];end=r['budget_end_sim'];failure=r.get('error')
times={}
for actor in actors:
 detections=sorted((processing,capture) for seq in matched.values() for processing,capture,name in seq if name==actor)
 events=sorted((a['processing_s'],a['capture_ns']*1e-9) for a in conf['events'] if a.get('actor')==actor)
 pub={}
 for x in rows:
  if x['kind']=='registry_observations':
   for a in x['batch']['observations']:
    if a['has_position']:pub.setdefault((a['counter'],round(a['capture_s']*1e9)),x['batch']['now_s'])
 reliable=sorted((pub.get((s['source_counter'],s['source_capture_ns'])),s['source_capture_ns']*1e-9) for s in surf['samples'] if s.get('actor')==actor and s['status']=='passed_surface_match' and pub.get((s['source_counter'],s['source_capture_ns'])) is not None)
 def event(v):return dict(event=bool(v),processing_sim_s=v[0][0] if v else None,capture_sim_s=v[0][1] if v else None,observation_end_sim_s=end,end_reason=failure or 'administrative_180sim',early_failure=bool(failure))
 times[actor]=dict(first_detected=event(detections),first_visual_confirmed=event(events),first_independently_valid_surface=event(reliable))
timeline=[]
for d in records:
 now=d['processing_time_s'];labels=[];mixed=[];unknown=[]
 for h in d['hypotheses']:
  if h['visual_state']!='visually_confirmed':continue
  group=set()
  for source in h['source_ids']:
   seq=matched.get(source.rsplit('/',1)[-1],[]);prior=[a for a in seq if a[0]<=now+1e-9]
   if prior:group.add(prior[-1][2])
  if len(group)==1:labels.extend(group)
  elif len(group)>1:mixed.append(h['id'])
  else:unknown.append(h['id'])
 timeline.append(dict(sim_s=now,confirmed_hypotheses=len(labels)+len(mixed)+len(unknown),unique_actors=len(set(labels)),duplicate_excess=len(labels)-len(set(labels)),wrong_merge_ids=mixed,unknown_ids=unknown))
dup_duration=sum(max(0,b['sim_s']-a['sim_s']) for a,b in zip(timeline,timeline[1:]) if a['duplicate_excess']>0)
(O/(p.name+'-hypothesis-timeline.json')).write_text(json.dumps(timeline,indent=2)+'\n')
odom=select('estimated_odometry');unique={a['stamp']:a for a in odom};seq=[unique[k] for k in sorted(unique)]
gap_limit=1.;paths=[];gaps=[]
for a,b in zip(seq,seq[1:]):
 dt=b['stamp']-a['stamp'];distance=math.dist(a['xyz'],b['xyz'])
 if dt>gap_limit:gaps.append(dict(start_s=a['stamp'],end_s=b['stamp'],gap_s=dt,straight_endpoint_distance_m=distance))
 else:paths.append(distance)
airborne=next((a['stamp'] for a in seq if a['xyz'][2]>.7),None);collisions=[];ground_initial=0
for row in read(p/'contacts.jsonl'):
 for c in row['contacts']:
  other=c['b'] if 'annual_x500_0' in c['a'] else c['a']
  if other.startswith('ground::') and (airborne is None or row['stamp']<airborne):ground_initial+=1
  else:collisions.append(dict(stamp=row['stamp'],**c))
fe=read(p/'flight/0/events.jsonl')
planwall=[v.get('compute_wall_s') for v in fe if v['type']=='native_planning_attempt']
decisions=[a for a in select('bearing_decision') if a.get('event')=='decision']
baseline=not r['bearing_bias']
baseline_errors=[a['selection_id'] for a in decisions if baseline and (a.get('changed') or a.get('candidate_pool') or a.get('chosen_goal')!=a.get('baseline_goal') or a.get('reason')!='disabled')]
health={}
for role in ['mapper','executor','gateway']:
 q=p/'map-evidence'/('drone-0-'+role)/'status.json'
 health[role]=json.loads(q.read_text()) if q.exists() else dict(missing=True)
evstatus=json.loads((p/'evaluation-recorder.status.json').read_text())
stop=json.loads((E/(p.name+'-stop.json')).read_text())
safety=json.loads((E/'supplemental-audits'/(p.name+'-safety.json')).read_text())
confirmation_unknown=[a for a in conf['events'] if a['status']!='matched_true_actor']
confirmed_counters={a['source_counter'] for a in conf['events']}
critical_identity_unknown=[a for a in ident['samples'] if a['counter'] in confirmed_counters and a['status']!='matched_true_actor']
final=timeline[-1] if timeline else None
import yaml
params=yaml.safe_load((p/'native-racer/exploration_node_1.yaml').read_text())['exploration_node_1']['ros__parameters']
common=json.loads((E/'configs/common.json').read_text());pre=json.loads((E/'PREFLIGHT.json').read_text())
expected=common['native_expected_parameters']|{'search.bearing_bias':r['bearing_bias']}
param_mismatches={k:dict(expected=v,actual=params.get(k)) for k,v in expected.items() if params.get(k)!=v}
frozen=json.loads((E/'tested-source-manifest.json').read_text())
source_mismatches=[n for n,h in r['code_sha256'].items() if frozen.get(n)!=h]
journal={}
for role in ['executor','bridge']:
 f=p/'timing'/('drone-0-'+role+'-observation.status.json')
 journal[role]=json.loads(f.read_text()) if f.exists() else dict(missing=True)
cpu=r.get('cpu_profile',{});assigned=cpu.get('assigned',{});verified=cpu.get('verified',{})
runtime=[x for x in rows if x.get('kind')=='detector_runtime']
resource_ok=(cpu.get('name')=='render3-perception2' and cpu.get('no_cpu_permission_expansion') is True and len(cpu.get('parent_allowed',[]))>=6 and assigned==pre['cpu_profile']['assigned'] and verified==assigned and cpu.get('other_roles_inherit_parent') is True)
detector_ok=(len(runtime)==1 and runtime[0].get('actual_threads')==2 and runtime[0].get('enforced_threads')==2 and runtime[0].get('warmup_image_discarded') is True)
guard=r.get('gazebo_shutdown_guard') or {};required_guard=pre['gazebo_shutdown_guard']
guard_ok=(guard.get('enabled') is True and guard.get('loaded_verified') is True and isinstance(guard.get('loaded_pid'),int) and guard['loaded_pid']>0 and all(guard.get(k)==required_guard[k] for k in ['binary_path','binary_sha256','source_sha256','sensors_plugin_path','sensors_plugin_sha256']) and pre['guard_production_verified'] is True)
imu=json.loads((p/'audit.json').read_text())
b=r.get('grounded_initialization') or {}
initial_ok=(b.get('physical_sim_us')==4000 and b.get('first_source_wait_wall_s',1e10)<=5 and b.get('public_feedback_wall_s',1e10)<=.5 and len(b.get('commander_guards',[]))==2 and all(x.get('disarmed') and x.get('no_offboard') and x.get('no_failsafe') for x in b.get('commander_guards',[])) and not any(x in b.get('owned_children_before_bridge',[]) for x in ['px4_bridge_node','view_executor_node','racer_adapter','person_search']) and bool(b.get('bridge_started_monotonic_ns')))
capture=json.loads((E/'audits'/(p.name+'-capture-contract.json')).read_text());rejection_replay=json.loads((E/'audits'/(p.name+'-rejection-replay.json')).read_text())
checks=dict(
 shared_measurement_layer=r.get('measurement_stack_sha256')==pre['measurement_stack_hashes'],
 grounded_initialization_before_bridge=initial_ok,
 common_real_imu_pacing_complete=imu['component_passed'],
 common_native_shutdown_guard_verified=guard_ok,
 shared_cpu_profile_installed=resource_ok,
 detector_thread_contract_installed=detector_ok,
 control_observation_no_loss=all(not x.get('missing') and x.get('written')==x.get('enqueued') and not x.get('dropped') and not x.get('write_errors') for x in journal.values()),
 native_config_matches_arm=not param_mismatches,
 same_frozen_runtime_source=bool(r['code_sha256']) and not source_mismatches and r['native_binary_sha256']==pre['hashes']['/tmp/racer-stage39-native/install/racer_core/lib/racer_core/exploration_node'] and r['px4_binary_sha256']==pre['hashes']['/tmp/stage54-precise-public-px4-v2/bin/px4'] and r['segmentation_weights_sha256']==pre['hashes']['/tmp/racer-stage10-assets/yolo11n-seg.pt'],
 no_fixture=r['flight_fixture']=='none' and not r['registry_cue_fixture'] and not r['fault_injection'] and not r['map_delay_fixture'] and not r['no_perception'] and not r['record_video'] and not r['active_review'] and not r['active_confirmation'] and not r['review_continuation'],
 full_budget=not failure and end>=180 and r['wall_s']<1265,
 actual_flight=any(a.get('armed') and a.get('offboard') for a in flights) and any(a.get('ready') for a in execs),
 no_safety_fault=not safety['observed_safety_fault'] and not safety['failed_certificates'] and not safety['telemetry']['violations'],
 no_px4_protection=not any(a.get('flight_session_fault') or a.get('failsafe_latched') for a in flights),
 no_unexpected_process_exit=not r.get('unexpected_exits'),
 no_collision=not collisions,
 independent_recorder_closed=evstatus.get('state')=='closed' and not evstatus.get('errors') and bool(evstatus.get('counts',{}).get('poses')),
 probe_current_state_capture_verified=capture['startup_state_verified'] and stop['startup_probe_states_verified'] and stop['raw_hashes_equal'] and not stop['supervisor_abort'],
 execution_capture_branch_verified=capture['execution_context_verified'],
 rejection_contexts_replay_consistent=rejection_replay['passed'],
 capture_health_no_loss=all(not h.get('missing') and not any(h.get(k,0) for k in ['oversize','queue_dropped','disk_dropped','write_errors','trigger_dropped']) for h in health.values()),
 all_scene_people_confirmed=conf['unique_matched_true_actors']==len(actors),
 final_no_duplicate_or_wrong_merge=bool(final) and not final['duplicate_excess'] and not final['wrong_merge_ids'] and not final['unknown_ids'],
 no_unknown_or_false_confirmation=not confirmation_unknown,
 confirmed_source_capture_identity_determined=not critical_identity_unknown,
 no_known_source_switch=not ident['source_switches'] and not conf['known_temporal_wrong_merge_or_source_switch'],
 surface_all_qualified_valid=surf['failures']==0,
 baseline_guidance_unchanged=not baseline or not baseline_errors)
result=dict(rejection_replay=rejection_replay,capture_contract=capture,legacy_startup_2mapper_4executor_counts=capture['legacy_2mapper_4executor_counts_satisfied'],gazebo_shutdown_guard=guard,cpu_profile=cpu,detector_runtime=runtime,sim_budget_reached=end>=180,failure_phase='cleanup' if failure and end>=180 else 'runtime' if failure else None,run=p.name,condition=json.loads((E/'attempts'/(p.name+'.json')).read_text())['condition'],arm='B' if r['bearing_bias'] else 'A',sim_s=end,wall_s=r['wall_s'],rtf=end/r['wall_s'],error=failure,checks=checks,strict_gate_pass=all(checks.values()),times=times,
 true_actor_count=len(actors),verified_unique_confirmed=conf['unique_matched_true_actors'],verified_recall=conf['verified_recall_under_rule'],
 algorithm_confirmed_final=final['confirmed_hypotheses'] if final else None,final=final,peak_duplicate_excess=max((a['duplicate_excess'] for a in timeline),default=0),duplicate_duration_s=dup_duration,duplicate_duration_interpretation='Duration between observed processing timestamps only; duplicate snapshots sharing the same timestamp can have zero measured interval without proving zero physical duration.',snapshots_with_duplicate_excess=sum(a['duplicate_excess']>0 for a in timeline),
 unknown_confirmation_events=len(confirmation_unknown),critical_identity_unknown=critical_identity_unknown,known_source_switches=len(ident['source_switches']),processed_detection_samples=ident['selected_samples'],matched_detection_samples=ident['matched_samples'],unmatched_or_unknown_detection_samples=ident['selected_samples']-ident['matched_samples'],
 negative_frame_false_detection_samples=ident['selected_samples'] if not actors else None,
 surface_qualified=surf['qualified_publications'],surface_valid=surf['matched'],surface_failure=surf['failures'],surface_error_m=surf['fused_to_current_surface_error'],
 saved_odom_path_m=sum(paths),saved_odom_path_gap_limit_s=gap_limit,odom_samples=len(seq),odom_gap_intervals=gaps,runner_high_rate_path_m=r['estimated_path_m'],
 planning_success_wall_s=stats(planwall),planning_timing_scope='successful native attempt results only, includes rerank+native trajectory planning; missing/failed plan times not inferred',
 native_planning_timing_missing=sum(v is None for v in planwall),planning_failure_events=collections.Counter(v['type'] for v in fe if 'plan' in v['type'] and ('reject' in v['type'] or 'fail' in v['type'])),
 initial_ground_contact_rows=ground_initial,collision_rows=collisions,airborne_threshold_time_s=airborne,
 independent_recorder=evstatus,capture_health=health,control_observation=journal,native_parameter_mismatches=param_mismatches,runtime_source_mismatches=source_mismatches,safety_status=safety['status'],full_flight_certified=False,
 baseline_decisions_checked=len(decisions) if baseline else 0,baseline_violations=baseline_errors,
 raw_both_copies_verified=stop['raw_hashes_equal'],all_started_runs_retained=True,
 limits='Single room, standing actors, fixed seed, GNSS aided EKF SITL, visible-surface point. Not body centre/multi-UAV/Orin/physical acceptance. Unknown labels retained.')
(O/(p.name+'-fair-metrics.json')).write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
print(json.dumps({k:result[k] for k in ['run','error','sim_s','wall_s','strict_gate_pass','checks','final','surface_qualified','surface_failure','surface_error_m']},indent=2))

