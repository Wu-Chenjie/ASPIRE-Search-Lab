"""Offline return-state audit against executor proofs and measured odometry.

No truth, planning, controls, or changes to a running process.
"""
import argparse
import json
import math
import re
from pathlib import Path


def rows(path):
    return [json.loads(s) for s in path.read_text().splitlines()] if path.exists() else []


def audit(run):
    run = Path(run)
    result = json.loads((run/'result.json').read_text())
    log = (run/'native-racer/exploration_node_1.log').read_text()
    timeline = []
    for number, line in enumerate(log.splitlines(), 1):
        if 'TASK_RETURN ' not in line:
            continue
        row = dict(re.findall(r'(\w+)=([^\s\x1b]+)', line.split('TASK_RETURN ', 1)[1]))
        for k in ('home', 'goal', 'actual'):
            row[k] = [float(v) for v in row[k].split(',')]
        for k in ('attempts', 'active', 'done', 'segment'):
            row[k] = int(row[k])
        row.update(sim=float(row['sim']), line=number)
        row['distance_home_m'] = math.dist(row['home'], row['actual'])
        timeline.append(row)
    events = rows(run/'flight/0/events.jsonl')
    activations = {r['traj_id']: r for r in events if r['type']=='native_trajectory_activated'}
    proofs = {r['token']: r for r in events if r['type']=='terminal_observation_completed'}
    telemetry = rows(run/'telemetry.jsonl')
    poses = [r['data'] for r in telemetry if r['kind']=='estimated_odometry']
    completed = []
    violations = []
    for row in timeline:
        if row['reason'] not in ('segment_complete_resume_original_home', 'home_pose_and_execution_complete'):
            continue
        activation = activations.get(row['segment'])
        completion = proofs.get(activation['token']) if activation else None
        proof = completion['proof'] if completion else None
        pose = min(poses, key=lambda p: abs(p['stamp']-row['sim'])) if poses else None
        valid = bool(proof and proof['completed_at']<=row['sim']+1e-6 and row['done']==1
                     and row['active']==row['segment'])
        returned = row['reason']=='home_pose_and_execution_complete'
        physical = bool(pose and abs(pose['stamp']-row['sim'])<=.5
                        and math.dist(pose['xyz'], row['home'])<=.15)
        if not valid or (returned and (row['distance_home_m']>.15 or not physical)):
            violations.append(dict(line=row['line'], reason='missing_execution_or_physical_completion'))
        completed.append(dict(sim=row['sim'], segment=row['segment'], returned=returned,
            distance_home_m=row['distance_home_m'], execution_proof_valid=valid,
            activation=activation, completion=completion, nearest_measured_pose=pose))
    if timeline:
        home=timeline[0]['home']
        if any(math.dist(r['home'], home)>1e-6 for r in timeline):
            violations.append(dict(reason='home_changed'))
        counts=[r['attempts'] for r in timeline]
        if counts!=sorted(counts):
            violations.append(dict(reason='retry_count_reset'))
        if max(counts)>8 and any(r['state']=='waiting_return_budget' for r in timeline):
            violations.append(dict(reason='retry_budget_reset_or_exceeded'))
    terminal = next((r for r in timeline if r['state'] in
        ('returned','waiting_return_budget','waiting_return_no_goal','execution_lost_latched')), None)
    post_terminal = [r for r in timeline if terminal and r['line']>terminal['line'] and r['reason'] in
        ('planning_attempt','authorized_activation','proposal')]
    if post_terminal:
        violations.append(dict(reason='motion_after_terminal_state', events=post_terminal))
    late_activations = [r for r in activations.values() if terminal and r['time']>terminal['sim']+.001]
    if late_activations:
        violations.append(dict(reason='executor_activation_after_terminal', events=late_activations))
    finish = [dict(line=n, text=line) for n,line in enumerate(log.splitlines(),1)
              if re.search(r'from .* to FINISH|-> FINISH|to FINISH|EXEC_TRAJ.*FINISH',line)]
    if finish and not any(r['reason']=='home_pose_and_execution_complete' for r in timeline):
        violations.append(dict(reason='finish_without_verified_return', lines=finish))
    hold_errors = [r['data']['tracking_error'] for r in telemetry if terminal and r['kind']=='execution'
                   and r['data']['time']>=terminal['sim'] and 'tracking_error' in r['data']]
    waiting = timeline[-1]['state'] if timeline and timeline[-1]['state'] in (
        'waiting_return_retry','waiting_return_candidates') else None
    summary = dict(scope='Post-run actual native FSM + executor observation proof + estimated pose. No truth. No claim of exhaustive scene coverage.',
        fixture=result.get('flight_fixture','none'), return_triggered=bool(timeline),
        outcome=terminal['state'] if terminal else (waiting or ('in_progress' if timeline else 'not_triggered')),
        attempts=max((r['attempts'] for r in timeline), default=0),
        terminal_sim_s=terminal['sim'] if terminal else None,
        terminal_distance_home_m=terminal['distance_home_m'] if terminal else None,
        observed_after_terminal_s=result['budget_end_sim']-terminal['sim'] if terminal else None,
        terminal_tracking_error_samples=len(hold_errors),
        terminal_tracking_error_max_m=max(hold_errors,default=None),
        temporary_activations=sum(r['state']=='executing_temporary' and r['reason']=='authorized_activation' for r in timeline),
        completion_checks=completed, violations=violations, timeline=timeline,
        native_finish_transitions=finish, cleanup=result['cleanup'], run_error=result['error'],
        native_binary_sha256=result['native_binary_sha256'])
    (run/'return-audit.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps({k:v for k,v in summary.items() if k not in ('timeline','completion_checks')},indent=2))
    return summary


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path)
    audit(p.parse_args().run)
