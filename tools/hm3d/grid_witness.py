"""Offline conservative native-lattice geometric witness, not an online planner."""
import argparse,json,math
from pathlib import Path
from collections import deque
import numpy as np
from tools.hm3d.box_audit import intersects


def audit(scene,out,resolution=.3):
    import trimesh
    scene=Path(scene);out=Path(out);out.mkdir(exist_ok=False)
    meta=json.loads((scene/'scene-audit.json').read_text());mesh=trimesh.load(scene/'assets/collision.stl',process=False);mesh.apply_translation(meta['translation'])
    tree=mesh.triangles_tree;triangles=mesh.triangles;r=resolution
    # Region chosen solely for independent evaluation, never exported to runtime.
    axes=[(np.arange(math.floor(lo/r),math.ceil(hi/r))+.5)*r for lo,hi in [(25.5,36.),(1.5,6.5),(1.2,1.9)]]
    shape=tuple(map(len,axes));valid=np.zeros(shape,bool);h=np.full(3,(math.ceil(.6/r)+.5)*r)
    for i in np.ndindex(shape):
        c=np.array([axis[k] for axis,k in zip(axes,i)]);ids=list(tree.intersection([*(c-h),*(c+h)]))
        valid[i]=not intersects(triangles[ids],c,h).any()
    def nearest(p):return tuple(int(np.argmin(abs(a-v))) for a,v in zip(axes,p))
    start=nearest([33.1,3.1,1.5]);target=nearest([26.55,5.55,1.5]);parent={start:None} if valid[start] else {};queue=deque(parent)
    while queue and target not in parent:
        i=queue.popleft()
        for d in range(3):
            for sign in [-1,1]:
                j=list(i);j[d]+=sign;j=tuple(j)
                if 0<=j[d]<shape[d] and valid[j] and j not in parent:parent[j]=i;queue.append(j)
    path=[];i=target
    if i in parent:
        while i is not None:path.append([float(a[k]) for a,k in zip(axes,i)]);i=parent[i]
        path=path[::-1]
    result=dict(scope='Independent triangle/voxel union test in selected known geometric region. Never used as runtime map/goal.',resolution=r,clearance=.6,kernel_half_width_m=h[0],tested_cells=int(np.prod(shape)),free_cells=int(valid.sum()),start_valid=bool(valid[start]),target_valid=bool(valid[target]),connected=bool(path),path=path,
        limitation='False means no witness in this bounded conservative lattice, not global scene impossibility; nearby alternative endpoints may exist.')
    (out/'summary.json').write_text(json.dumps(result,indent=2));np.savez_compressed(out/'grid.npz',valid=valid,xs=axes[0],ys=axes[1],zs=axes[2]);print(json.dumps({k:v for k,v in result.items() if k!='path'}))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('scene');p.add_argument('output');p.add_argument('--resolution',type=float,default=.3);a=p.parse_args();audit(a.scene,a.output,a.resolution)
