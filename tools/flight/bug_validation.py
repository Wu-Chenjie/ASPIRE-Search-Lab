"""Post-run evidence for native failures, authorization, and actual motion.

Reads logs only. It never publishes controls or supplies scene truth to planning.
"""
import argparse
import hashlib
import json
import math
import re
from pathlib import Path
import numpy as np
from tools.flight.evaluate import evaluate


def rows(path):
    return [json.loads(x) for x in path.read_text().splitlines()] if path.exists() else []


def audit(folder):
    folder = Path(folder)
    result = json.loads((folder/'result.json').read_text())
    events = rows(folder/'flight/0/events.jsonl')
    telemetry = rows(folder/'telemetry.jsonl')
    poses = [r['data'] for r in telemetry if r['kind'] == 'estimated_odometry']
    log = (folder/'native-racer/exploration_node_1.log').read_text()
    significant = [s for s in log.splitlines() if any(x in s for x in
                   ('TASK_FLIGHT_FIXTURE', 'TASK_RECOVERY', 'TASK_OPT', 'TASK_NATIVE_FAIL'))]
    armed = re.search(r'TASK_FLIGHT_FIXTURE armed sim=([\d.]+)', log)
    # Retain the lifetime-budget metric for archived native builds.
    exhausted = re.search(r'TASK_RECOVERY hold sim=([\d.]+) reason=budget_or_unchanged_map attempts=8', log)
    hold_time = float(exhausted[1]) if exhausted else None
    hold_errors = [r['data']['tracking_error'] for r in telemetry
                   if r['kind']=='execution' and hold_time is not None
                   and r['data']['time']>=hold_time]
    records = []
    for activation in (e for e in events if e['type'] == 'native_trajectory_activated'):
        proposal = next((e for e in events if e['type']=='path_proposed'
                         and e.get('token')==activation['token']), None)
        end = next((e for e in events if e['type'] == 'terminal_observation_completed'
                    and e['token'] == activation['token']), None)
        stop = end['time'] if end else result['budget_end_sim']
        samples = [p for p in poses if activation['time'] <= p['stamp'] <= stop]
        records.append(dict(id=activation['traj_id'], token=activation['token'],
                            safety_checked_proposal=proposal,
                            start_sim=activation['time'], end_sim=stop,
                            completed=end is not None, pose_samples=len(samples),
                            actual_start=samples[0]['xyz'] if samples else None,
                            actual_end=samples[-1]['xyz'] if samples else None,
                            displacement_m=math.dist(samples[0]['xyz'], samples[-1]['xyz'])
                            if len(samples)>1 else None,
                            estimated_path_m=sum(math.dist(a['xyz'], b['xyz'])
                                                 for a,b in zip(samples,samples[1:]))))
    t = float(armed[1]) if armed else None
    after = [p for p in poses if t is not None and p['stamp'] >= t]
    stamps = rows(folder/'continuous-rgb-stamps.jsonl')
    gaps = np.diff([r['stamp_ns']*1e-9 for r in stamps])
    video = folder/'continuous-rgb.avi'
    tail = [p for p in poses if p['stamp'] >= result['budget_end_sim']-30]
    return dict(summary=evaluate(folder),fixture=result.get('flight_fixture', 'none'),
                no_path_recovery=result.get('no_path_recovery'),
                native_binary_sha256=result['native_binary_sha256'],
                runtime_sha256=hashlib.sha256((folder/'runtime.json').read_bytes()).hexdigest(),
                source_hashes=result['code_sha256'],
                fixture_armed_sim=t,significant_native_lines=significant,
                budget_hold_sim=hold_time,
                hold_tracking_p95_m=float(np.percentile(hold_errors,95)) if hold_errors else None,
                hold_tracking_max_m=max(hold_errors,default=None),
                recovery_attempts=len(re.findall(r'TASK_RECOVERY attempt ',log)),
                recovery_candidate_holds=len(re.findall(r'TASK_RECOVERY hold .*reason=no_eligible_measured_goal',log)),
                recovery_activations=len(re.findall(r'TASK_RECOVERY activated ',log)),
                recovery_failure_reasons={str(i):len(re.findall(r'TASK_RECOVERY failure .* astar='+str(i)+r' ',log)) for i in range(4)},
                constant_signal_normalizations=log.count('constant_signal_normalization'),
                trajectory_execution=records,
                displacement_since_injection_m=math.dist(after[0]['xyz'],after[-1]['xyz']) if len(after)>1 else None,
                last_30s_max_distance_from_first_m=max((math.dist(tail[0]['xyz'],p['xyz']) for p in tail),default=None),
                continuous_rgb=dict(frames=len(stamps),encoded_nominal_fps=15,
                    first_sim=stamps[0]['stamp_ns']*1e-9 if stamps else None,
                    last_sim=stamps[-1]['stamp_ns']*1e-9 if stamps else None,
                    max_gap_s=float(max(gaps)) if len(gaps) else None,
                    median_gap_s=float(np.median(gaps)) if len(gaps) else None,
                    gaps_over_100ms=int(sum(gaps>.100001)),
                    sha256=hashlib.sha256(video.read_bytes()).hexdigest() if video.exists() else None))


if __name__ == '__main__':
    p=argparse.ArgumentParser();p.add_argument('runs',nargs='+',type=Path)
    p.add_argument('--output',required=True,type=Path);a=p.parse_args()
    a.output.write_text(json.dumps([audit(f) for f in a.runs],indent=2)+'\n')
