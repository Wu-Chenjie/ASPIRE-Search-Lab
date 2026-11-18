"""Offline audit of recorded model masks, actual depth, estimated TF and truth."""
import argparse,json
from pathlib import Path
from zipfile import BadZipFile
from zlib import error as ZlibError
import numpy as np
from scipy.spatial.transform import Rotation
from tools.flight.confirmation_audit import require_capture_stamp
from tools.flight.flight_localization import camera,interpolate_ns,truth_stamp_ns
from tools.flight.search_evidence import rows,stats
from person_search.segmentation import PersonMask,localize_masks
from person_search.perception import cloud


def audit(run,person):
    import trimesh
    run=Path(run);truth_by_stamp={};invalid_truth_stamps=0
    for row in rows(run/'truth-poses.jsonl'):
        try:truth_by_stamp[truth_stamp_ns(row['stamp'])]=row
        except (KeyError,TypeError,ValueError,OverflowError):invalid_truth_stamps+=1
    truth=[truth_by_stamp[ns] for ns in sorted(truth_by_stamp)]
    placement=json.loads((run/'evaluation-truth.json').read_text())['person']
    mesh=trimesh.load_scene(person,process=False,skip_materials=True).to_mesh();mt=np.eye(4);mt[:3,:3]=Rotation.from_euler('xyz',placement[3:]).as_matrix();mt[:3,3]=placement[:3];mesh.apply_transform(mt)
    sem={r['stamp_ns']:r for r in rows(run/'perception/observations.jsonl') if r['kind']=='semantic'}
    selected=[ns for ns,r in sorted(sem.items()) if any(c['state']=='localized' for c in r['candidates'])]
    # Bounded deterministic sample spread across the full positive recording.
    selected=[selected[i] for i in np.unique(np.linspace(0,len(selected)-1,min(40,len(selected))).astype(int))]
    records=[];invalid_evidence_frames=0
    for ns in selected:
        try:
            gt=interpolate_ns(truth,ns)
            if gt is None:records.append(dict(stamp_ns=ns,unscored='missing_truth_bracket'));continue
            with np.load(run/'perception'/f'{ns}-rgb-masks.npz') as a, np.load(run/'perception'/f'{ns}-geometry.npz') as g:
                require_capture_stamp(a,ns,'mask')
                require_capture_stamp(g,ns,'geometry')
                boxes,scores,masks=a['boxes'],a['scores'],a['masks']
                if not len(boxes)==len(scores)==len(masks):raise ValueError('mask_array_lengths_disagree')
                depth,k,estimated=g['depth'],g['k'],g['camera_to_map']
            ds=[PersonMask(tuple(box),float(score),mask) for box,score,mask in zip(boxes,scores,masks)]
            _,cs=localize_masks(ds,depth,k,estimated)
            target=[r['xyz'] for r in sem[ns]['candidates'] if r['state']=='localized']
            if len(cs)!=len(target):raise ValueError('localized_candidate_count_mismatch')
            if not all(np.shape(xyz)==(3,) and np.allclose(c.xyz,xyz,atol=1e-7,rtol=1e-7) for c,xyz in zip(cs,target)):
                raise ValueError('localized_candidate_position_mismatch')
        except (OSError,ValueError,KeyError,TypeError,EOFError,BadZipFile,ZlibError) as error:
            invalid_evidence_frames+=not isinstance(error,FileNotFoundError)
            records.append(dict(stamp_ns=ns,unscored=str(error) or type(error).__name__));continue
        tc=camera(*gt);true_cloud=cloud(depth,k,tc);k=np.asarray(k).reshape(3,3)
        for c in cs:
            yy,xx=np.where(c.pixels);idx=np.unique(np.linspace(0,len(xx)-1,min(128,len(xx))).astype(int));xx=xx[idx];yy=yy[idx]
            rays=np.c_[(xx-k[0,2])/k[0,0],(yy-k[1,2])/k[1,1],np.ones(len(xx))]@tc[:3,:3].T
            expected=np.full(len(xx),np.inf)
            for i in range(0,len(xx),8):
                ray=rays[i:i+8];loc,ids,_=mesh.ray.intersects_location(np.tile(tc[:3,3],(len(ray),1)),ray,multiple_hits=False)
                if len(ids):expected[i+ids]=(loc-tc[:3,3])@tc[:3,2]
            measured=depth[yy,xx];matched=np.isfinite(expected)&(abs(measured-expected)<.04)
            dist=float(trimesh.proximity.closest_point(mesh,c.xyz[None,:])[1][0])
            records.append(dict(stamp_ns=ns,position_reproduced=True,sampled_mask_pixels=len(xx),matched_target_depth_fraction=float(matched.mean()),
                matched_axial_depth_residual_m=float(np.median(abs(measured[matched]-expected[matched]))) if matched.any() else None,
                camera_origin_error_m=float(np.linalg.norm(estimated[:3,3]-tc[:3,3])),
                camera_rotation_error_deg=float((Rotation.from_matrix(estimated[:3,:3]).inv()*Rotation.from_matrix(tc[:3,:3])).magnitude()*180/np.pi),
                pose_contribution_m=float(np.linalg.norm(c.xyz-np.median(true_cloud[c.pixels],axis=0))),
                visible_position_distance_to_person_surface_m=dist))
    keys=['matched_target_depth_fraction','matched_axial_depth_residual_m','camera_origin_error_m','camera_rotation_error_deg','pose_contribution_m','visible_position_distance_to_person_surface_m']
    unscored=sum('unscored' in r for r in records)
    result=dict(scope='Independent post-run evaluation only; true simulator pose and target mesh never enter online decisions',
        limitations='Up to 40 deterministic correlated observations; up to 128 mask pixels each. Not body-center precision or detection recall. Distances use supplied scan mesh.',
        status='complete' if selected and not unscored else 'incomplete',selected_frames=len(selected),unscored_frames=unscored,
        invalid_evidence_frames=invalid_evidence_frames,
        invalid_truth_timestamp_rows=invalid_truth_stamps,
        summary={k:stats(r[k] for r in records if r.get(k) is not None) for k in keys},observations=records)
    (run/'search-geometry-audit.json').write_text(json.dumps(result,indent=2))
    print(json.dumps({k:v for k,v in result.items() if k!='observations'},indent=2))
    return result


def main():
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--person',required=True,type=Path);a=p.parse_args()
    if audit(a.run,a.person)['invalid_evidence_frames']:raise SystemExit(1)


if __name__=='__main__':main()
