"""Offline commanded and independent simulator kinematics, never flight control."""
import argparse,json,re
from pathlib import Path
import numpy as np
from scipy.signal import savgol_filter
from scipy.spatial.transform import Rotation
from tools.flight.evaluate import evaluate

def audit(folder):
 r=evaluate(folder);events=[json.loads(x) for x in (folder/'flight/0/events.jsonl').read_text().splitlines()];tokens={x['token'] for x in events if x['type']=='native_trajectory_activated'};accepted=[x for x in events if x['type']=='path_proposed' and x['token'] in tokens]
 r['authorized_continuous_peaks']={k:max((x['trajectory_limits'][k] for x in accepted),default=None) for k in ['speed','acceleration','jerk']};r['authorized_yaw_peak']=max((x['trajectory_yaw_peak'] for x in accepted if 'trajectory_yaw_peak' in x),default=None)
 tel=[json.loads(x) for x in (folder/'telemetry.jsonl').read_text().splitlines()];execs=[x['data'] for x in tel if x['kind']=='execution' and x['data'].get('token') in tokens and x['data'].get('reason') in ('tracking_view','tracking_error_hold','observation_dwell')];intervals={}
 for x in execs:
  t=x['time'];token=x['token'];lo,hi=intervals.get(token,(t,t));intervals[token]=(min(lo,t),max(hi,t))
 truth=[json.loads(x) for x in (folder/'truth-poses.jsonl').read_text().splitlines()] if (folder/'truth-poses.jsonl').exists() else [];truth=sorted({x['stamp']:x for x in truth}.values(),key=lambda x:x['stamp']);peaks={};samples=0
 if len(truth)>20:
  ts=np.array([x['stamp'] for x in truth]);rot=Rotation.from_quat([x['quat'] for x in truth]);xyz=np.array([x['xyz'] for x in truth])+rot.apply(np.tile([0,0,.24],(len(truth),1)));dt=.02;grid=np.arange(ts[0],ts[-1],dt);values=np.stack([np.interp(grid,ts,xyz[:,i]) for i in range(3)],axis=1);mask=np.zeros(len(grid),bool)
  for lo,hi in intervals.values():mask|=(grid>=lo+.12)&(grid<=hi-.12)
  # Reject gaps in independent truth. Derivatives use a fixed 0.22 s window.
  ix=np.clip(np.searchsorted(ts,grid),1,len(ts)-1);mask&=(ts[ix]-ts[ix-1]<.06)
  samples=int(mask.sum())
  if samples:
   for order,key in enumerate(['speed','acceleration','jerk'],1):
    v=savgol_filter(values,11,3,deriv=order,delta=dt,axis=0);peaks[key]=float(np.linalg.norm(v[mask],axis=1).max())
   yaw=np.unwrap(rot.as_euler('xyz')[:,2]);yaw_grid=np.interp(grid,ts,yaw);peaks['yaw_rate']=float(np.abs(savgol_filter(yaw_grid,11,3,deriv=1,delta=dt)[mask]).max())
 r['simulator_motion_peaks']=peaks;r['simulator_motion_samples']=samples;r['motion_peak_scope']='Executed tracking/dwell intervals; world base_link from independent Gazebo pose; 50 Hz interpolation, 11-point cubic Savitzky-Golay derivatives, exclude 0.12 s interval edges. Filtered kinematic estimates, not certified physical/IMU peaks.'
 log=(folder/'native-racer/exploration_node_1.log').read_text();r['native_planning_failure_messages']=log.count('Plan fail');r['native_no_path_messages']=log.count('No path to next viewpoint');r['native_hard_rest_applications']={str(dim):log.count('TASK_HARD_REST dim='+str(dim)) for dim in (1,3)};pattern=r'TASK_TIME id=(\d+) applied=(\d+) factor=([\d.e+-]+) pre_v=([\d.e+-]+) pre_a=([\d.e+-]+) pre_j=([\d.e+-]+) pre_w=([\d.e+-]+) rest=(\d+) uniform=(\d+)'
 details=[dict(zip(['id','applied','factor','pre_v','pre_a','pre_j','pre_w','rest','uniform'],map(float,m))) for m in re.findall(pattern,log)];r['native_time_conditioning']=details;r['conditioning_scaled']=sum(x['applied'] and x['factor']>1.000001 for x in details);r['conditioning_skipped']=sum(not x['applied'] for x in details)
 first={}
 for row in tel:
  if row['kind']=='tracks':
   for track in row['data']['tracks']:
    if track['state']=='confirmed':first.setdefault(track['id'],dict(wall_s=row['wall_s'],attempts=track['attempts'],xyz=track['xyz']))
 r['confirmed_track_first_records']=first;r['review_request_log']=[x for x in (folder/'person_search.log').read_text().splitlines() if 'Review request' in x];r['native_review_log']=[x for x in log.splitlines() if 'Task review planned' in x];r['native_activated_ids']=[x['traj_id'] for x in events if x['type']=='native_trajectory_activated'];return r
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('runs',type=Path,nargs='+');p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.write_text(json.dumps([audit(f) for f in a.runs],indent=2)+'\n')
