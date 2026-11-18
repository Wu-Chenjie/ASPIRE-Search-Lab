"""Offline 3-D native-inflation compatibility; no output is an online map."""
import argparse,json,math
from pathlib import Path
import numpy as np
from scipy.ndimage import label
from tools.hm3d.box_audit import intersects
from tools.hm3d.prepare import distances


def audit(scene,out):
    import trimesh
    scene=Path(scene);out=Path(out);out.mkdir(exist_ok=False)
    meta=json.loads((scene/'scene-audit.json').read_text())
    mesh=trimesh.load(scene/'assets/collision.stl',process=False);mesh.apply_translation(meta['translation'])
    axes=[(np.arange(math.floor(lo/.3),math.ceil(hi/.3))+.5)*.3 for lo,hi in
          [(mesh.bounds[0,0],mesh.bounds[1,0]),(mesh.bounds[0,1],mesh.bounds[1,1]),(.9,2.7)]]
    shape=tuple(map(len,axes));valid=np.zeros(shape,bool);tree=mesh.triangles_tree;triangles=mesh.triangles;half=np.full(3,.75)
    for n,i in enumerate(np.ndindex(shape)):
        c=np.array([a[k] for a,k in zip(axes,i)]);ids=list(tree.intersection([*(c-half),*(c+half)]))
        valid[i]=not intersects(triangles[ids],c,half).any()
        if n%4096==0:print('SAT',n,int(np.prod(shape)),flush=True)
    inds=np.array(np.where(valid)).T;points=np.array([[a[k] for a,k in zip(axes,i)] for i in inds])
    if len(points):
        down=distances(mesh,points,np.tile([0,0,-1],(len(points),1)))
        up=distances(mesh,points,np.tile([0,0,1],(len(points),1)))
        interior=(np.abs(down-points[:,2])<.15)&np.isfinite(up)&(up<5)&(up>.6)
        for i in inds[~interior]:valid[tuple(i)]=False
    labels,n=label(valid);components=[]
    for cid in range(1,n+1):
        ii=np.array(np.where(labels==cid)).T;p=np.array([[a[k] for a,k in zip(axes,i)] for i in ii])
        components.append(dict(id=cid,cells=len(p),bounds=[p.min(0).tolist(),p.max(0).tolist()],seed=p[len(p)//2].tolist()))
    components.sort(key=lambda c:-c['cells']);start=np.array(meta['spawn']['position'])+meta['translation']
    idx=tuple(np.argmin(abs(a-v)) for a,v in zip(axes,start))
    report=dict(scene=meta['scene'],source_sha256=meta['source_sha256'],resolution=.3,inflation=.6,cube_half_width=.75,
        start=start.tolist(),start_lattice=[float(a[k]) for a,k in zip(axes,idx)],start_component=int(labels[idx]),
        tested_cells=int(np.prod(shape)),compatible_cells=int(valid.sum()),components=components,
        tested_height_centers=[float(axes[2][0]),float(axes[2][-1])],
        scope='Independent geometry-only 3-D compatibility audit, bounded sampled heights, native cell-union cube. Never used for planning.',
        limitation='A component may lie within one room. Disconnection is only for this conservative finite lattice and floor filter, not all continuous motions.')
    np.savez_compressed(out/'lattice.npz',xs=axes[0],ys=axes[1],zs=axes[2],labels=labels)
    (out/'summary.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('scene',type=Path);p.add_argument('output',type=Path);a=p.parse_args();audit(a.scene,a.output)
