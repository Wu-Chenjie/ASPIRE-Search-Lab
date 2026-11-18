"""Independent triangle/AABB SAT audit of a saved offline witness."""
import argparse,json
from pathlib import Path
import numpy as np


def intersects(triangles,center,half):
    triangles=np.asarray(triangles,float)-np.asarray(center,float)
    if not len(triangles):return np.zeros(0,bool)
    edges=np.roll(triangles,-1,axis=1)-triangles
    axes=np.concatenate([np.broadcast_to(np.eye(3),(len(triangles),3,3)),
         np.cross(edges[:,0],edges[:,1])[:,None,:],
         np.cross(edges[:,:,None,:],np.eye(3)[None,None,:,:]).reshape(-1,9,3)],axis=1)
    projections=np.einsum('nvi,nai->nav',triangles,axes)
    radius=np.sum(np.abs(axes)*np.asarray(half),axis=-1)
    separated=(projections.min(axis=-1)>radius+1e-10)|(projections.max(axis=-1)<-radius-1e-10)
    return ~np.any(separated,axis=1)


def audit(scene,witness,output):
    import trimesh
    scene=Path(scene);meta=json.loads((scene/'scene-audit.json').read_text())
    mesh=trimesh.load(scene/'assets/collision.stl',process=False);mesh.apply_translation(meta['translation'])
    path=np.array(json.loads(Path(witness).read_text())['path']);points=[]
    for a,b in zip(path[:-1],path[1:]):points.extend(np.linspace(a,b,max(2,int(np.ceil(np.linalg.norm(b-a)/.025))+1)))
    tree=mesh.triangles_tree;triangles=mesh.triangles
    def blocked(c,h):
        ids=list(tree.intersection([*(c-h),*(c+h)]))
        return bool(intersects(triangles[ids],c,h).any())
    cube_bad=[];grid_bad=[]
    for point in points:
        if blocked(point,np.full(3,.6125)):cube_bad.append(point.tolist())
        center=(np.floor(point/.3)+.5)*.3
        if blocked(center,np.full(3,.75)):grid_bad.append(point.tolist())
    result=dict(scope='Independent geometry only; not fed into online mapper',samples=len(points),
       continuous_point6_cube_verified=not cube_bad,cube_failed_samples=len(cube_bad),cube_first_failure=cube_bad[:1],
       conservative_surface_voxel_point3_grid_verified=not grid_bad,grid_failed_samples=len(grid_bad),grid_first_failure=grid_bad[:1],
       explanation='SAT tests all nearby triangles. 0.6125 m cube at samples <=0.025 m apart certifies continuous 0.6 m cube. Grid audit uses union of 5x5x5 cells at 0.3 m; a failure is conservative geometry overlap, not a measured-map occupancy claim.')
    Path(output).write_text(json.dumps(result,indent=2));print(json.dumps(result))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('scene');p.add_argument('witness');p.add_argument('output');a=p.parse_args();audit(a.scene,a.witness,a.output)
