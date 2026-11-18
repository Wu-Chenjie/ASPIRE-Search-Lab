"""Independent target-mesh ray evaluation of static captures; no ROS publisher."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def audit(run,person):
    import trimesh
    run=Path(run);fixture=json.loads((run/'viewpoints.json').read_text());pose=fixture['person_pose']
    if pose is None:raise ValueError('No target in negative fixture')
    mesh=trimesh.load_scene(person,process=False,skip_materials=True).to_mesh()
    transform=np.eye(4);transform[:3,:3]=Rotation.from_euler('xyz',pose[3:]).as_matrix();transform[:3,3]=pose[:3];mesh.apply_transform(transform)
    result={}
    for name,view in fixture['views'].items():
        folder=run/name;meta=json.loads((folder/'frame.json').read_text());k=np.array(meta['depth']['k']).reshape(3,3);depth=np.load(folder/'depth.npy')
        rotation=Rotation.from_euler('z',view['lens_yaw']).as_matrix()@np.array([[0,0,1],[-1,0,0],[0,-1,0]])
        origin=np.array(view['lens_xyz']);cam=(mesh.vertices-origin)@rotation
        uv=cam[:,:2]/cam[:,2,None]*[k[0,0],k[1,1]]+[k[0,2],k[1,2]]
        lo=np.floor(uv.min(0)).astype(int);hi=np.ceil(uv.max(0)).astype(int)
        xx,yy=np.meshgrid(np.arange(max(0,lo[0]),min(320,hi[0]+1)),np.arange(max(0,lo[1]),min(240,hi[1]+1)))
        coords=np.c_[xx.ravel(),yy.ravel()];rays=np.c_[(coords-[k[0,2],k[1,2]])/[k[0,0],k[1,1]],np.ones(len(coords))]@rotation.T
        expected=np.full(depth.shape,np.inf)
        for start in range(0,len(rays),8):
            ray=rays[start:start+8];loc,indices,_=mesh.ray.intersects_location(np.tile(origin,(len(ray),1)),ray,multiple_hits=False)
            if len(indices):
                pixels=coords[start+indices];expected[pixels[:,1],pixels[:,0]]=(loc-origin)@rotation[:,2]
        target=np.isfinite(expected);visible=target&np.isfinite(depth)&(np.abs(depth-expected)<.04)
        masks=np.load(folder/'masks.npz')['masks'];mask_rows=[]
        for mask in masks:
            from scipy.ndimage import binary_erosion
            support=binary_erosion(mask)&np.isfinite(depth)&(depth>.25)&(depth<8)
            mask_rows.append(dict(mask_pixels=int(mask.sum()),valid_interior_pixels=int(support.sum()),
                target_depth_support_fraction=float((support&visible).sum()/max(1,support.sum()))))
        result[name]=dict(projected_target_pixels=int(target.sum()),visible_target_pixels=int(visible.sum()),
            visible_fraction=float(visible.sum()/max(1,target.sum())),projected_bounds=[lo.tolist(),hi.tolist()],
            clipped_by_image=bool((lo<0).any() or hi[0]>=320 or hi[1]>=240),masks=mask_rows)
    report=dict(scope='Independent target truth used only for static evaluation. Axial rendered depth compared with target-only ray intersections within 4 cm.',views=result)
    (run/'person-visibility-audit.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--person',type=Path,required=True);a=p.parse_args();audit(a.run,a.person)
