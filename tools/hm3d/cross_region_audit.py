"""Independent post-run NBg room/vestibule crossing evaluation; no runtime imports."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def audit(run,output):
    run=Path(run);summary=json.loads((run/'hm3d-flight-audit.json').read_text())
    ready=summary['ready_sim']
    rows=[json.loads(s) for s in (run/'truth-poses.jsonl').read_text().splitlines()]
    samples=sorted({r['stamp']:r for r in rows if ready is not None and r['stamp']>=ready}.values(),key=lambda r:r['stamp'])
    t=np.array([r['stamp'] for r in samples]);pos=np.array([np.array(r['xyz'])+Rotation.from_quat(r['quat']).apply([0,0,.24]) for r in samples])
    # Independent scan-plane review: large vertical transverse wall/doorframe
    # surfaces at x=28.55..28.65, before the narrower vestibule at x=27.7..27.9.
    plane=28.6;crossings=[]
    for i in range(1,len(pos)):
        a,b=pos[i-1],pos[i]
        if a[0]>plane>=b[0] and b[0]<a[0]:
            alpha=(a[0]-plane)/(a[0]-b[0]);p=a+alpha*(b-a)
            if 2.<=p[1]<=4.1 and .7<=p[2]<=3.:
                crossings.append(dict(time=float(t[i-1]+alpha*(t[i]-t[i-1])),base_position=p.tolist()))
    source=(pos[:,0]>=29.2)&(pos[:,1]>=1.5)&(pos[:,1]<=4.6)
    target=(pos[:,0]<=27.2)&(pos[:,0]>=25.8)&(pos[:,1]>=2.)&(pos[:,1]<=4.4)
    entered=float(t[np.flatnonzero(target)[0]]) if target.any() else None
    cross=crossings[0]['time'] if crossings else None
    events=[json.loads(s) for s in (run/'flight/0/events.jsonl').read_text().splitlines()]
    observations=[e for e in events if e['type']=='execution_grid_published' and 'observed_cells' in e]
    before=[e for e in observations if cross is not None and e['time']<=cross]
    after=[e for e in observations if cross is not None and e['time']>cross]
    gain=after[-1]['observed_cells']-before[-1]['observed_cells'] if after and before else 0
    passed=bool(source.any() and entered is not None and cross is not None and entered>cross and gain>0
                and summary['completed']>=3 and not summary['stalled_60s']
                and summary['nonempty_contacts_after_ready']==0 and summary['error'] is None)
    result=dict(scope='Independent post-run geometry/ground-truth evaluation only; GNSS-aided actual PX4 motion. Not a preregistered performance comparison.',
        selected_geometry='NBg transverse doorway x=28.6; wall/doorframe area peaks independently located at x=28.55..28.65 and vestibule x=27.7..27.9',
        portal_plane_x=plane,portal_y=[2.,4.1],portal_z=[.7,3.],
        source_region_x_min=29.2,opposite_region_x_max=27.2,crossings=crossings,entered_opposite_region_at=entered,
        base_link_definition='Gazebo model origin + rotated [0,0,0.24]; estimated odometry not used to decide crossing',
        new_observed_cells_after_crossing=gain,completed_trajectories=summary['completed'],max_completion_gap_s=summary['max_completion_gap_s'],
        post_ready_nonempty_contacts=summary['nonempty_contacts_after_ready'],single_case_passed=passed,
        limitations='Initial region differs from stage8; improvement cannot be attributed solely to inflation repair. Other fixed scenes and person search are separate gates.')
    Path(output).write_text(json.dumps(result,indent=2));print(json.dumps(result));return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run');p.add_argument('output');a=p.parse_args();audit(a.run,a.output)
