#!/usr/bin/env python3
"""Offline inspection only: exact polynomial peaks vs dense sampling and limits."""
import argparse,json,collections
from pathlib import Path
import numpy as np
from scipy.interpolate import BSpline
from core.planning.bspline_trajectory import polynomial_spans
from core.planning.continuous_trajectory import ContinuousTrajectory

def inspect(folder):
    cfg=json.loads((folder/'runtime.json').read_text());limits=cfg['motion_limits'];rows=[];counts=collections.Counter();sample_errors=[]
    for line in (folder/'native-racer/trajectories.jsonl').read_text().splitlines():
        p=json.loads(line)
        d,c=polynomial_spans(p['knots'],p['points'],p['order']);y=np.array(p['yaw_points']);yd,yc=polynomial_spans(np.arange(-3,len(y)+1)*p['yaw_dt'],y[:,None],3)
        curve=ContinuousTrajectory(d,c,float(yc[0,0,0]),float(sum(yc[-1,:,0])-yc[0,0,0]),'audit_only',yaw_durations=yd,yaw_segments=yc[:,:,0])
        peaks={**curve.limits(),'yaw_rate':curve.yaw_rate_limit()};bad=[k for k,v in peaks.items() if v>limits[k]+.001];counts.update(bad)
        ts=np.linspace(0,curve.duration,max(100,int(curve.duration/.002)))
        b=BSpline(np.array(p['knots'])-p['knots'][p['order']],np.array(p['points']),p['order'])
        sampled={k:float(np.linalg.norm(b(ts,nu=j),axis=1).max()) for j,k in enumerate(('speed','acceleration','jerk'),1)}
        sample_errors.append(max(abs(peaks[k]-sampled[k]) for k in sampled))
        rows.append(dict(id=p['traj_id'],peaks=peaks,violations=bad,duration=curve.duration,boundary=p['boundary_progress']))
    return dict(run=folder.name,executor_limits=limits,planning_scale=cfg['racer_planning_scale'],proposals=len(rows),over_limit_counts=dict(counts),exact_vs_2ms_sampling_max_difference=max(sample_errors,default=0),peak_percentiles={k:{str(q):float(np.percentile([r['peaks'][k] for r in rows],q)) for q in (50,95,100)} for k in limits},proposals_detail=rows)
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.write_text(json.dumps(inspect(a.run),indent=2)+'\n')
