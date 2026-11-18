"""Post-run crossing evaluation against a pre-flight region specification.

Truth is read only here, never in the online planner or detector.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from tools.hm3d.flight_audit import rows


def crossing_evidence(samples, spec):
    normal = np.asarray(spec['portal']['normal'], float)
    origin = np.asarray(spec['portal']['point'], float)
    boxes = [np.asarray(spec[k], float) for k in ('source_bounds', 'target_bounds')]
    portal = np.asarray(spec['portal']['bounds'], float)
    if normal.shape != (3,) or origin.shape != (3,) or not np.isfinite([normal, origin]).all():
        raise ValueError('Expected finite 3D portal plane')
    if not np.isclose(np.linalg.norm(normal), 1.): raise ValueError('Portal normal must be unit length')
    for box in [*boxes, portal]:
        if box.shape != (2,3) or not np.isfinite(box).all() or np.any(box[1] < box[0]):
            raise ValueError('Invalid evaluation bounds')
    direction = spec['portal']['direction']
    if direction not in (-1, 1): raise ValueError('Direction must be -1 or 1')
    max_gap = float(spec.get('max_truth_gap_s', .2))
    if not 0 < max_gap <= .5: raise ValueError('Truth gap must be bounded')
    def inside(p, box): return bool(np.all(p >= box[0]) and np.all(p <= box[1]))
    crossings, source_times, target_times, gaps = [], [], [], 0
    previous = None
    for stamp, position in samples:
        p = np.asarray(position, float)
        if p.shape != (3,) or not np.isfinite(p).all() or not np.isfinite(stamp):
            raise ValueError('Invalid truth sample')
        if inside(p, boxes[0]): source_times.append(float(stamp))
        if inside(p, boxes[1]): target_times.append(float(stamp))
        if previous is not None:
            t, a = previous
            if stamp <= t: raise ValueError('Samples must be strictly time ordered')
            da, db = direction * np.dot(a-origin,normal), direction * np.dot(p-origin,normal)
            if stamp-t > max_gap: gaps += 1
            elif da < 0 <= db:
                fraction = -da/(db-da)
                q = a + fraction*(p-a)
                if inside(q, portal):
                    crossings.append(dict(time=float(t+fraction*(stamp-t)),base_position=q.tolist()))
        previous = (stamp, p)
    witnesses = [dict(crossing=c, entered_target_at=next(t for t in target_times if t>c['time']))
        for c in crossings if any(t<c['time'] for t in source_times) and any(t>c['time'] for t in target_times)]
    return dict(crossings=crossings,source_samples=len(source_times),target_samples=len(target_times),
        large_truth_gaps=gaps,witness=witnesses[0] if witnesses else None)


def audit(run, specification):
    run, specification = Path(run), Path(specification)
    spec = json.loads(specification.read_text())
    summary = json.loads((run/'hm3d-flight-audit.json').read_text())
    ready = summary['ready_sim']
    truth = sorted({r['stamp']:r for r in rows(run/'truth-poses.jsonl')
                    if ready is not None and r['stamp'] >= ready}.values(), key=lambda r:r['stamp'])
    samples = [(r['stamp'],np.asarray(r['xyz'])+Rotation.from_quat(r['quat']).apply([0,0,.24])) for r in truth]
    result = crossing_evidence(samples,spec)
    cross = result['witness']['crossing']['time'] if result['witness'] else None
    observations = [e for e in rows(run/'flight/0/events.jsonl') if e['type']=='execution_grid_published' and 'observed_cells' in e]
    before = [e for e in observations if cross is not None and e['time'] <= cross]
    after = [e for e in observations if cross is not None and e['time'] > cross]
    gain = after[-1]['observed_cells']-before[-1]['observed_cells'] if before and after else None
    result.update(scope='Independent post-run evaluation; specification and truth never enter online navigation.',
        specification_sha256=hashlib.sha256(specification.read_bytes()).hexdigest(),specification=spec,
        new_observed_cells_after_crossing=gain,completed_trajectories=summary['completed'],
        max_completion_gap_s=summary['max_completion_gap_s'],stalled_60s=summary['stalled_60s'],
        post_ready_nonempty_contacts=summary['nonempty_contacts_after_ready'],error=summary['error'],
        geometry_and_progress_gate=bool(result['witness'] and gain is not None and gain>0 and summary['completed']>=3
            and summary['nonempty_contacts_after_ready']==0 and summary['error'] is None),
        limitation='This crossing/progress gate alone does not certify heartbeat, ULog failsafe, return state, detector accuracy or cleanup; inspect their separate audits.')
    (run/'region-audit.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


if __name__ == '__main__':
    p=argparse.ArgumentParser();p.add_argument('run');p.add_argument('specification');a=p.parse_args()
    print(json.dumps(audit(a.run,a.specification),indent=2))
