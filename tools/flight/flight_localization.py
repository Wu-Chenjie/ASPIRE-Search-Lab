"""Post-run surface error decomposition; simulator truth never enters online nodes."""
import argparse,json,math
from bisect import bisect_left
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
from tools.flight.localization_audit import triangles,raster
from person_search.perception import PeopleDetector,candidates,cloud

def truth_stamp_ns(seconds):
    if type(seconds) not in (int, float) or not math.isfinite(seconds) or seconds < 0:
        raise ValueError('invalid_truth_timestamp')
    return round(seconds*1_000_000_000)


def truth_pose(row):
    try:
        xyz = np.asarray(row['xyz'], dtype=float)
        quat = np.asarray(row['quat'], dtype=float)
        if xyz.shape != (3,) or quat.shape != (4,) or not np.isfinite(xyz).all():
            raise ValueError
        norm = math.hypot(*quat)
        if not 0 < norm < math.inf:
            raise ValueError
        return xyz, quat/norm
    except (KeyError, TypeError, ValueError, OverflowError) as error:
        raise ValueError('invalid_truth_pose') from error


def interpolate_ns(rows, ns):
    if type(ns) is not int or ns < 0:
        raise ValueError('invalid_observation_capture_stamp')
    def stamp_of(row):
        return truth_stamp_ns(row['stamp'])
    i = bisect_left(rows, ns, key=stamp_of)
    if i < len(rows) and stamp_of(rows[i]) == ns:
        xyz, quat = truth_pose(rows[i])
        rotation = Rotation.from_quat(quat)
    elif 0 < i < len(rows):
        a, b = rows[i-1:i+1]
        start, end = stamp_of(a), stamp_of(b)
        if end-start > 60_000_000:
            return None
        fraction = (ns-start)/(end-start)
        xyz0, quat0 = truth_pose(a)
        xyz1, quat1 = truth_pose(b)
        xyz = (1-fraction)*xyz0 + fraction*xyz1
        rotation = Slerp([0., 1.], Rotation.from_quat([quat0, quat1]))([fraction])[0]
    else:
        return None
    return xyz+rotation.apply([0, 0, .24]), rotation


def interpolate(rows, stamp):
    try:
        return interpolate_ns(rows, truth_stamp_ns(stamp))
    except (KeyError, TypeError, ValueError, OverflowError):
        return None

def camera(xyz,rot):
 t=np.eye(4);t[:3,3]=xyz+rot.apply([.13233,0,.02078]);t[:3,:3]=rot.as_matrix()@Rotation.from_quat([-.5,.5,-.5,.5]).as_matrix();return t

def audit(folder):
 rows=[json.loads(s) for s in (folder/'truth-poses.jsonl').read_text().splitlines()];rows=sorted({r['stamp']:r for r in rows}.values(),key=lambda r:r['stamp']);detector=PeopleDetector();mesh=triangles();frames=[];scored=[]
 for path in sorted(folder.glob('rgbd-*.npz')):
  a=np.load(path);stamp=float(a['stamp_ns'])*1e-9;pose=interpolate(rows,stamp)
  if pose is None:frames.append(dict(file=path.name,unscored='no synchronized simulator truth'));continue
  xyz,rot=pose;truthcam=camera(xyz,rot);estimated_rotation=Rotation.from_quat(a['base_quaternion']);estimatedcam=camera(a['base_position'],estimated_rotation);d=a['depth'];k=a['k'];cs=candidates(a['rgb'],d,k,estimatedcam,detector);gt=raster(mesh,k,truthcam,d.shape);truecloud=cloud(d,k,truthcam);modelcloud=cloud(gt,k,truthcam);results=[]
  for c in cs:
   valid=c.pixels&np.isfinite(gt);fraction=float(valid.sum()/c.pixels.sum())
   if fraction<.95:continue
   surface=np.median(modelcloud[valid],axis=0);corrected=np.median(truecloud[c.pixels],axis=0)
   r=dict(semantic=c.semantic,pixels=int(valid.sum()),matched_fraction=fraction,estimated=c.xyz.tolist(),surface_truth=surface.tolist(),total_surface_error_m=float(np.linalg.norm(c.xyz-surface)),pose_contribution_m=float(np.linalg.norm(c.xyz-corrected)),depth_and_raster_residual_m=float(np.linalg.norm(corrected-surface)))
   results.append(r);scored.append(r)
  frames.append(dict(file=path.name,stamp=stamp,aircraft_position_error_m=float(np.linalg.norm(a['base_position']-xyz)),aircraft_rotation_error_deg=float((estimated_rotation.inv()*rot).magnitude()*180/np.pi),candidates=len(cs),semantic_detections=sum(c.semantic for c in cs),matched_person_observations=results))
 keys=['total_surface_error_m','pose_contribution_m','depth_and_raster_residual_m'];summary={k:dict(n=len(scored),median=float(np.median([r[k] for r in scored])),max=max(r[k] for r in scored)) for k in keys} if scored else {}
 return dict(run=folder.name,definition='Visible surface median, exact same candidate interior mask; model ray matching >=95%. Pose and depth magnitudes do not add arithmetically.',truth_pose_samples=len(rows),saved_frame_count=len(frames),surface_observation_count=len(scored),semantic_surface_observations=sum(r['semantic'] for r in scored),summary=summary,frames=frames,limits='Saved frames are correlated and sparse (~5 s); not detection rate or body-center accuracy. Up to 5% mesh-edge unmatched pixels excluded from truth median. GNSS-aided simulation only.')
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('folder',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args();r=audit(a.folder);a.output.write_text(json.dumps(r,indent=2)+'\n');print(json.dumps({k:v for k,v in r.items() if k!='frames'},indent=2))
