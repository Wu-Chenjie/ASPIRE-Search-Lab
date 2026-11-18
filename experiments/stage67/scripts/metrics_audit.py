"""Offline audit of frozen actual logs. No simulation, inference or runtime edits."""
from pathlib import Path
import sys,json,collections,math
import numpy as np
B=Path('/Users/wuchenjie/Documents/Codex/2026-10-13');S=B/'task-3-stage45'
sys.path[:0]=[str(S),str(S/'src'),str(S/'src/annual_planner/src')]
from tools.flight.stage23_safety_audit import audit_run
from tools.flight.flight_localization import interpolate
E=Path('/Volumes/WuChenjie/Mac-runtime/runtime-stage52-20261018T1315Z/evidence/stage67');out=E/'supplemental-audits';out.mkdir(exist_ok=True)
def stats(values):
 v=[float(x) for x in values if x is not None and math.isfinite(x)]
 return dict(n=len(v),median=float(np.median(v)),p95=float(np.percentile(v,95)),maximum=max(v)) if v else dict(n=0)
reports=[]
for p in ([Path('/Volumes/ColimaRuntime/runtime/research-stage67/runs')/sys.argv[1]] if len(sys.argv)>1 else sorted(Path('/Volumes/ColimaRuntime/runtime/research-stage67/runs').iterdir())):
 r=json.loads((p/'result.json').read_text())
 t=[json.loads(s) for s in (p/'telemetry.jsonl').read_text().splitlines()]
 select=lambda k:[x['data'] for x in t if x['kind']==k]
 tracks=select('tracks');records=select('person_records');timing=select('perception_timing')
 events=select('presence_event');confirmed=[x for x in events if x['event']=='visual_confirmed']
 decisions=[d for d in select('bearing_decision') if d.get('event')=='decision']
 memory=[d for d in select('bearing_decision') if d.get('event')=='discovery_memory']
 plans=[d for d in select('bearing_decision') if d.get('event')=='plan_result']
 execs=select('execution');flights=select('flight')
 f=p/'flight/0/events.jsonl';flight_events=[json.loads(s) for s in f.read_text().splitlines()] if f.exists() else []
 points=[];violations=[];seen_confirmed=set();history_invalid=0
 for x in tracks:
  for v in x['tracks']:

   if v['id'] in seen_confirmed and v['visual_state']!='visually_confirmed':violations.append('source_confirmation_revoked')
   if v['visual_state']=='visually_confirmed':seen_confirmed.add(v['id'])
   reliable=v['localization_state']=='reliable'
   if reliable!=(v['position'] is not None) or reliable!=(v['xyz'] is not None):violations.append('raw_position_state_mismatch')
   if reliable:
    age=x['processing_time_s']-v['position']['capture_time_s'];points.append(age)
    if not -.05<=age<=2.000001:violations.append('raw_current_point_stale')
   if not reliable and v.get('last_reliable_location_history'):history_invalid+=1
 for x in records:
  for h in x['hypotheses']:
   if h['position'] is not None:
    age=x['processing_time_s']-h['position']['capture_time_s']
    if not 0<=age<=2.000001:violations.append('cpp_current_point_stale')
 for m in memory:
  if m['mission_evaluations']>8:violations.append('mission_evaluations')
  for g in m['groups']:
   expected=max(0.,1-(m['processing_time_s']-g['epoch_s'])/20) if m['processing_time_s']>=g['epoch_s'] else 0.
   if abs(g['value']-expected)>1e-9:violations.append('decay_formula')
   if g['attempts']>2 or g['selections']>2 or g['epoch_updates']>1 or g['material_retries']>1:violations.append('group_budget')
 if any(d['mission_bias_selections']>4 for d in decisions):violations.append('selection_budget')
 consumed=[x for x in timing if x.get('status')=='rgb_presence_consumed']
 ts=[x for x in timing if x.get('processing_wall_monotonic_s') is not None]
 frequency={}
 if len(ts)>1:
  a,b=ts[0],ts[-1];sim=(b['now_ns']-a['now_ns'])*1e-9;wall=b['processing_wall_monotonic_s']-a['processing_wall_monotonic_s']

  for label,key in [('received','rgb_received_total'),('consumed','rgb_consumed_total')]:
   delta=b[key]-a[key];frequency[label]=dict(counter_delta=delta,sim_window_s=sim,wall_window_s=wall,hz_sim=delta/sim if sim else None,hz_wall=delta/wall if wall else None)
 truthfile=p/'truth-poses.jsonl';truth=[json.loads(s) for s in truthfile.read_text().splitlines()] if truthfile.exists() else []
 truth=sorted({x['stamp']:x for x in truth}.values(),key=lambda x:x['stamp'])
 physical={}
 if truth:
  xyz=np.array([x['xyz'] for x in truth]);physical=dict(samples=len(truth),model_origin_extent_m=np.ptp(xyz,axis=0).tolist(),model_origin_z_min_m=float(xyz[:,2].min()),model_origin_z_max_m=float(xyz[:,2].max()))
 changed=[]
 for d in decisions:
  if not d['changed']:continue
  plan=next((v for v in plans if v['selection_id']==d['selection_id']),None)
  activation=next((x for x in flight_events if x['type']=='native_trajectory_activated' and plan and x.get('traj_id')==plan['native_plan_id']),None)
  completion=next((x for x in flight_events if x['type']=='terminal_observation_completed' and activation and x.get('token')==activation['token']),None)
  component=next((c for e in d.get('candidate_pool',[]) if e['candidate_id']==d['chosen_candidate_id'] for c in e['components'] if c['cue_id']==d['cue_id']),None)
  motion={}
  if activation and completion and truth:
   first=interpolate(truth,activation['time']);last=interpolate(truth,completion['time'])
   if first is not None and last is not None:

    motion=dict(reference='Independent simulator model pose plus rotated base offset',start_xyz=first[0].tolist(),end_xyz=last[0].tolist(),translation_m=float(np.linalg.norm(last[0]-first[0])),rotation_rad=float((first[1].inv()*last[1]).magnitude()))
  changed.append({k:d.get(k) for k in ('selection_id','processing_time_s','goal_kind','cue_id','chosen_goal','baseline_goal','discount_changes_selection','without_discount_candidate_id')}|dict(plan=plan,activation=activation,completion=completion,chosen_cue=component,physical_motion=motion))
 local_packets=[];obfile=p/'perception/observations.jsonl'
 for line in obfile.open():
  x=json.loads(line)
  if x['kind']=='registry_observations':local_packets.append(x['batch'])
 all_obs={};qualified={}
 for packet in local_packets:
  for o in packet['observations']:
   key=(o['robot'],o['session'],o['counter'],o['capture_s']);all_obs[key]=o
   if o['has_position']:qualified.setdefault(key,o|dict(processing_time_s=packet['now_s']))
 per_source={}
 for key,o in all_obs.items():
  source='/'.join(key[:3]);v=per_source.setdefault(source,dict(observations=0,qualified=0,visual_confirmed=False))
  v['observations']+=1;v['qualified']+=key in qualified;v['visual_confirmed']|=o['visual_state']=='visually_confirmed'
 comm=records[-1].get('communication',{}) if records else {}
 kindcounts=collections.Counter(x['type'] for x in flight_events)
 result=dict(run=p.name,error=r['error'],sim_s=r['budget_end_sim'],wall_s=r['wall_s'],estimated_path_m=r['estimated_path_m'],
  valid_full_flight=r['error'] is None and bool(select('estimated_odometry')) and any(x.get('ready') for x in execs),

  execution_safety_fault_samples=sum(bool(x.get('safety_fault')) for x in execs),
  first_executor_ready=next((x['time'] for x in execs if x.get('ready')),None),
  first_reliable_processing_s=next((x['processing_time_s'] for x in records if x['counts']['reliable_localizations_current']),None),
  last_reliable_processing_s=next((x['processing_time_s'] for x in reversed(records) if x['counts']['reliable_localizations_current']),None),
  flight_exercised=any(x.get('armed') and x.get('offboard') for x in flights) and bool(select('estimated_odometry')),
  raw_visual_records=tracks[-1]['counts']['local_visual_track_records'] if tracks else 0,visual_confirmation_events=len(confirmed),first_confirmations=confirmed,
  cpp_final_counts=records[-1]['counts'] if records else None,
  cpp_confirmed_hypotheses_peak=max((x['counts']['confirmed_presence_hypotheses'] for x in records),default=0),
  reliable_localizations_peak=max((x['counts']['reliable_localizations_current'] for x in records),default=0),
  qualified_source_capture_observations=len(qualified),all_source_capture_observations=len(all_obs),
  qualified_capture_fraction=len(qualified)/len(all_obs) if all_obs else None,
  source_localization_coverage=per_source,current_point_age_s=stats(points),history_while_current_invalid_samples=history_invalid,
  semantic_invariant_violations=dict(collections.Counter(violations)),
  frequency=frequency,RGB_consumed_sim_age_s=stats([x['age_s'] for x in consumed]),RGB_consumed_received_wall_age_s=stats([x['received_age_wall_s'] for x in consumed]),
  inference_wall_s=stats([x.get('inference_wall_s') for x in consumed]),
  callback_total_available=any(x.get('process_calls',0)>0 for x in timing),callback_counter_defect=None,
  timing_status_counts=dict(collections.Counter(x['status'] for x in timing)),
  native_decisions=len(decisions),native_changed=changed,numeric_discount_pivotal_count=sum(bool(x['discount_changes_selection']) for x in changed),
  decision_reasons=dict(collections.Counter(d['reason'] for d in decisions)),
  max_mission_evaluations=max((m['mission_evaluations'] for m in memory),default=0),
  native_memory_outcomes=dict(collections.Counter(g['last_outcome'] for m in memory for g in m['groups'])),

  registry_events=dict(collections.Counter(e['kind'] for x in select('target_registry_events') for e in x.get('events',[]))),
  association_elapsed_us=stats([x['association']['elapsed_us'] for x in records]),
  actual_application_communication=comm,communication_limit='Actual String UTF8 counters only, no DDS/network overhead or system-wide savings claim',
  execution_event_counts=dict(kindcounts),last_observation_time=max((x['time'] for x in flight_events if x['type']=='terminal_observation_completed'),default=None),
  any_armed=any(x.get('armed') for x in flights),origin_reasons=dict(collections.Counter(x.get('origin_calibration') for x in flights)),
  origin_samples_max=max((x.get('origin_samples',0) for x in flights),default=0),estimated_odometry_samples=len(select('estimated_odometry')),physical_truth=physical,
  any_latched_fault=any(x.get('flight_session_fault') or x.get('failsafe_latched') for x in flights),
  cleanup=r['cleanup'])
 (out/(p.name+'-metrics.json')).write_text(json.dumps(result,indent=2)+'\n');reports.append(result)
 safety=audit_run(p);(out/(p.name+'-safety.json')).write_text(json.dumps(safety,indent=2,allow_nan=False)+'\n')
 print(p.name,'safety',safety['status'],safety['certificate_counts'])
 print(p.name,'visual',len(confirmed),'CPP',result['cpp_final_counts'],'qualified',len(qualified),'/',len(all_obs),'changed',len(changed),'valid_flight',result['valid_full_flight'])
(out/'METRICS.json').write_text(json.dumps(reports,indent=2)+'\n')

