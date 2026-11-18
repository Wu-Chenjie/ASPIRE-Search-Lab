"""Independent flight result/pose audit; never used for planning."""
import argparse,json
from collections import Counter
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from tools.flight.flight_localization import interpolate


def rows(path):
    return [json.loads(s) for s in path.read_text().splitlines()] if path.is_file() else []


def audit(folder):
    folder=Path(folder);result=json.loads((folder/'result.json').read_text())
    telemetry=rows(folder/'telemetry.jsonl');events=rows(folder/'flight/0/events.jsonl')
    executed=[r['data'] for r in telemetry if r['kind']=='execution'];poses=[r['data'] for r in telemetry if r['kind']=='estimated_odometry']
    ready=next((r['time'] for r in executed if r.get('ready')),None)
    completed=[r for r in events if r['type']=='terminal_observation_completed'];activated=[r for r in events if r['type']=='native_trajectory_activated']
    progress=[ready]+[r['time'] for r in completed]+[result['budget_end_sim']] if ready is not None else []
    gaps=np.diff(sorted(progress)) if progress else []
    truth=sorted({r['stamp']:r for r in rows(folder/'truth-poses.jsonl')}.values(),key=lambda r:r['stamp'])
    errors=[]
    for p in poses:
        gt=interpolate(truth,p['stamp'])
        if gt is not None and p['xyz'][2]>.7:
            errors.append(dict(stamp=p['stamp'],position_m=float(np.linalg.norm(np.array(p['xyz'])-gt[0])),
                rotation_deg=float((Rotation.from_quat(p['quaternion']).inv()*gt[1]).magnitude()*180/np.pi)))
    contacts=rows(folder/'contacts.jsonl');collisions=[]
    for r in contacts:
        if ready is not None and r['stamp']>=ready and r['contacts']:collisions.append(r)
    stats={key:dict(median=float(np.median([r[key] for r in errors])),p95=float(np.percentile([r[key] for r in errors],95)),max=max(r[key] for r in errors)) for key in ('position_m','rotation_deg')} if errors else {}
    observed=[r for r in events if r['type']=='execution_grid_published' and 'observed_cells' in r and ready is not None and r['time']>=ready]
    summary=dict(observed_cells_after_ready=observed[0]['observed_cells'] if observed else None,observed_cells_end=observed[-1]['observed_cells'] if observed else None,new_observed_cells=(observed[-1]['observed_cells']-observed[0]['observed_cells']) if observed else None,scope='Actual SITL motion, GNSS-aided EKF2; independent post-run evaluation',ready_sim=ready,
        end_sim=result['budget_end_sim'],activated=len(activated),completed=len(completed),
        max_completion_gap_s=float(max(gaps)) if len(gaps) else None,stalled_60s=bool(len(gaps) and max(gaps)>=60),
        estimated_path_m=result['estimated_path_m'],pose_error=stats,pose_samples=len(errors),
        recorded_contact_messages=len(contacts),nonempty_contacts_total=sum(bool(r['contacts']) for r in contacts),
        nonempty_contacts_after_ready=len(collisions) if ready is not None else None,
        event_counts=dict(Counter(r['type'] for r in events)),completed_events=completed,
        spatial_extent=np.ptp([p['xyz'] for p in poses if ready is not None and p['stamp']>=ready],axis=0).tolist() if ready is not None else None,
        cross_room_verified=False,acceptance='FAIL_OR_PENDING: cross-room and observation gain need independent evaluation',
        cleanup=result['cleanup'],error=result['error'])
    (folder/'hm3d-flight-audit.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary));return summary

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folder');a=p.parse_args();audit(a.folder)
