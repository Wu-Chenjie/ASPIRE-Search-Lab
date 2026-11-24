#!/usr/bin/env python3
"""Post-run truth-only scoring. Never imported by the flight decision chain."""
import argparse,json,collections,math
from pathlib import Path
import numpy as np


def evaluate(folder):
    folder=Path(folder);result=json.loads((folder/'result.json').read_text());truth=json.loads((folder/'evaluation-truth.json').read_text())
    telemetry=[json.loads(x) for x in (folder/'telemetry.jsonl').read_text().splitlines()]
    eventfile=folder/'flight/0/events.jsonl';events=[json.loads(x) for x in eventfile.read_text().splitlines()] if eventfile.exists() else []
    counts=collections.Counter(e['type'] for e in events);confirmed={};first_ready=None;errors=[]
    for row in telemetry:
        if row['kind']=='execution':
            if row['data'].get('ready') and first_ready is None:first_ready=row['wall_s']
            if row['data'].get('reason') in ('tracking_view','tracking_error_hold','observation_dwell'):errors.append(row['data']['tracking_error'])
        if row['kind']=='tracks':
            for t in row['data']['tracks']:
                if t.get('state')=='confirmed':confirmed.setdefault(t['id'],dict(first_wall_s=row['wall_s'],xyz=t['xyz']))
    people = truth.get("people")
    if people is None:
        person = truth.get("person")
        people = [] if person is None else [dict(id="stationary_person", position=person[:3])]
    matched = set()
    correct, false = [], []
    for ident, track in confirmed.items():
        xyz = track["xyz"]
        candidates = []
        for target in people:
            position = target["position"]
            distance = math.dist(xyz[:2], position[:2])
            if "height" in target and not position[2]-.35 <= xyz[2] <= position[2]+target["height"]+.35:
                continue
            if distance <= .65:
                candidates.append((distance, target["id"]))
        if candidates:
            _, target_id = min(candidates)
            matched.add(target_id)
            correct.append(dict(id=ident, target_id=target_id, **track))
        else:
            false.append(dict(id=ident, **track))
    pairs=list(folder.glob('rgbd-*.npz'));pair_checks=[]
    for f in pairs:
        x=np.load(f);pair_checks.append(bool(x['rgb'].shape[:2]==x['depth'].shape==(240,320) and np.array_equal(x['k'],x['rgb_k']) and x['pose_bracket'][1]-x['pose_bracket'][0]<=.0400001))
    truth_rows=(folder/'truth-poses.jsonl').read_text().splitlines() if (folder/'truth-poses.jsonl').exists() else []
    contact_rows=[json.loads(x) for x in (folder/'contacts.jsonl').read_text().splitlines()] if (folder/'contacts.jsonl').exists() else []
    contact_errors=(folder/'evaluation.log').read_text() if (folder/'evaluation.log').exists() else ''
    evaluation_status=dict(truth_pose_samples=len(truth_rows),contact_messages=len(contact_rows),contact_callback_error='Traceback' in contact_errors,non_ground_contact_events=sum('ground' not in c['a'] and 'ground' not in c['b'] for r in contact_rows for c in r['contacts']),contact_limit='No-event intervals cannot prove absence of contact without a validated sensor heartbeat.')
    fusion=[r['data'] for r in telemetry if r['kind']=='estimator_fusion']
    report=dict(layout=result.get('layout'),performance_profile=result.get('performance_profile'),independent_evaluation=evaluation_status,gnss_position_fused=any(r.get('cs_gnss_pos',False) for r in fusion) if any('cs_gnss_pos' in r for r in fusion) else None,external_vision_position_fused=any(r.get('cs_ev_pos',False) for r in fusion),run=folder.name,scope=result['scope'],active_review=result.get('active_review',False),negative=result['negative'],wall_s=result['wall_s'],error=result['error'],cleanup=result['cleanup'],evaluator_ground_truth_people_total=len(people),confirmed_people=len(matched),matched_person_tracks=len(correct),duplicate_confirmed_tracks=len(correct)-len(matched),person_count_scope='Distinct truth targets matched by position',false_confirmed_tracks=len(false),first_true_confirmation_wall_s=min((t['first_wall_s'] for t in correct),default=None),first_ready_wall_s=first_ready,confirmed_tracks=correct+false,activated_curves=counts['native_trajectory_activated'],completed_observations=counts['terminal_observation_completed'],handoffs=counts['handoff_consumed'],rejections=dict(collections.Counter(e['reason'] for e in events if e['type']=='native_proposal_rejected')),estimated_path_m=result['estimated_path_m'],max_estimated_height_m=result['max_estimated_height_m'],tracking_samples=len(errors),tracking_error_p95_m=float(np.percentile(errors,95)) if errors else None,tracking_error_max_m=max(errors,default=None),any_latched_flight_fault=any(bool(r['data'].get('flight_session_fault') or r['data'].get('failsafe_latched')) for r in telemetry if r['kind']=='flight'),captured_pairs=len(pairs),paired_intrinsics_and_pose_brackets_valid=all(pair_checks) if pairs else None)

    if result.get('confirmation_mode') in ('decoupled', 'registry'):
        try:from .presence_audit import audit_presence
        except ImportError:from presence_audit import audit_presence
        report.update(audit_presence(telemetry,truth))
        report['confirmation_policy'] = result['confirmation_mode']
    return report

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('runs',type=Path,nargs='+');p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.write_text(json.dumps([evaluate(f) for f in a.runs],indent=2)+'\n')
