"""Offline landing-contact interpretation; never imported by flight control."""
import argparse,collections,hashlib,itertools,json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from tools.flight.search_evidence import rows
from tools.flight.flight_localization import interpolate


def bottom_points(truth,stamp,pose,size):
    # interpolate already converts model origin to base_link exactly once.
    base,rot=interpolate(truth,stamp)
    local=np.array([[x,y,-size[2]/2] for x,y in itertools.product([-size[0]/2,0,size[0]/2],[-size[1]/2,0,size[1]/2])])
    return rot.apply(Rotation.from_euler('xyz',pose[3:]).apply(local)+pose[:3])+base


def audit(run):
    import trimesh
    run=Path(run);ready=json.loads((run/'hm3d-flight-audit.json').read_text())['ready_sim']
    contacts=[r for r in rows(run/'contacts.jsonl') if ready is not None and r['stamp']>=ready and r['contacts']]
    result=dict(scope='Independent post-run recorded contact names + true pose + saved collision geometry. Nearest surface samples are not Gazebo contact-point/normal measurements.',messages=len(contacts),episode_gap_s=.1)
    if contacts:
        ts=[r['stamp'] for r in contacts];result.update(first_sim_s=ts[0],last_sim_s=ts[-1],continuous_contact_episodes=1+sum(b-a>.1 for a,b in zip(ts,ts[1:])),max_message_gap_s=max(np.diff(ts)) if len(ts)>1 else None,
            pairs=dict(collections.Counter(c['a']+' | '+c['b'] for r in contacts for c in r['contacts'])))
        truth=sorted({r['stamp']:r for r in rows(run/'truth-poses.jsonl')}.values(),key=lambda r:r['stamp'])
        scene=ET.parse(run/'scene.sdf');model=scene.find('.//model[@name="hm3d_scan"]');uri=model.findtext('link/collision/geometry/mesh/uri');source=Path(uri.removeprefix('file://'))
        mesh=trimesh.load(source,process=False);pose=list(map(float,model.findtext('pose').split()));matrix=np.eye(4);matrix[:3,:3]=Rotation.from_euler('xyz',pose[3:]).as_matrix();matrix[:3,3]=pose[:3];mesh.apply_transform(matrix)
        base=ET.parse(run/'models/annual_base_0/model.sdf');link=base.find('model/link[@name="base_link"]');checks=[]
        for row in (contacts[0],contacts[-1]):
            gt=interpolate(truth,row['stamp'])
            if gt is None:
                checks.append(dict(sim_s=row['stamp'],unscored='no synchronized truth bracket; no extrapolation'));continue
            base_xyz,rot=gt[0],gt[1];xyz=base_xyz-rot.apply([0,0,.24])
            for name in sorted({c['a'].split('::')[-1] for c in row['contacts']}):
                element=link.find('collision[@name="'+name+'"]')
                if element is None:checks.append(dict(collision=name,unscored='not a recorded base-link collision'));continue
                cp=list(map(float,element.findtext('pose').split()));size=np.array(list(map(float,element.findtext('geometry/box/size').split())))
                world=bottom_points(truth,row['stamp'],cp,size)
                near,dist,tri=trimesh.proximity.closest_point(mesh,world)
                checks.append(dict(sim_s=row['stamp'],collision=name,model_origin_xyz=xyz.tolist(),base_xyz=base_xyz.tolist(),tilt_deg=float(np.degrees(np.arccos(np.clip(rot.as_matrix()[2,2],-1,1)))),sampled_bottom_z_range=[float(world[:,2].min()),float(world[:,2].max())],nearest_surface_z_range=[float(near[:,2].min()),float(near[:,2].max())],nearest_distance_max_m=float(dist.max()),nearest_surface_abs_normal_z_min=float(abs(mesh.face_normals[tri,2]).min())))
        result.update(collision_mesh_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),first_last_bottom_surface_checks=checks,interpretation='Classify from recorded parts, timing and sampled nearby surfaces; do not count repeated messages as separate collisions or infer an unmeasured impact force.')
    text=json.dumps(result,indent=2);(run/'contact-audit.json').write_text(text);print(text);return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);audit(p.parse_args().run)
