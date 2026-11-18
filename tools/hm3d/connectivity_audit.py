"""Offline mesh clearance witnesses only; never imported by navigation nodes.

A failed slice is not a proof that the full 3-D scene is disconnected.
"""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.ndimage import label
from tools.hm3d.prepare import distances


def audit(scene,output,height=1.5,step=.25):
    import trimesh
    scene=Path(scene);output=Path(output);output.mkdir(exist_ok=False)
    meta=json.loads((scene/'scene-audit.json').read_text())
    mesh=trimesh.load(scene/'assets/collision.stl',process=False)
    mesh.apply_translation(meta['translation'])
    lo,hi=mesh.bounds
    xs=np.arange(lo[0],hi[0],step);ys=np.arange(lo[1],hi[1],step)
    xx,yy=np.meshgrid(xs,ys,indexing='ij')
    points=np.c_[xx.ravel(),yy.ravel(),np.full(xx.size,height)]
    clearance=np.empty(len(points))
    for i in range(0,len(points),64):
        clearance[i:i+64]=trimesh.proximity.closest_point(mesh,points[i:i+64])[1]
        if i%2048==0:print('nearest',i,len(points),flush=True)
    # For four-neighbor graph edges, endpoint distance - half edge length
    # certifies every center along that edge, by 1-Lipschitz distance.
    geometric=clearance>=.6+step/2
    indices=np.flatnonzero(geometric);p=points[indices]
    down=distances(mesh,p,np.tile([0,0,-1],(len(p),1)))
    up=distances(mesh,p,np.tile([0,0,1],(len(p),1)))
    enclosed=(np.abs(down-height)<.15)&np.isfinite(up)&(up<5)&(up>.6)
    valid=np.zeros(len(points),bool);valid[indices[enclosed]]=True
    labels,n=label(valid.reshape(xx.shape))
    components=[]
    for c in range(1,n+1):
        coords=points[labels.ravel()==c]
        components.append(dict(id=c,cells=len(coords),bounds=[coords.min(axis=0).tolist(),coords.max(axis=0).tolist()],
                               seed=coords[len(coords)//2].tolist()))
    components.sort(key=lambda x:-x['cells'])
    start=np.array(meta['spawn']['position'])+meta['translation'];ix=np.argmin(abs(xs-start[0]));iy=np.argmin(abs(ys-start[1]))
    summary=dict(scope='Offline independent 2-D clearance graph; positive continuous witnesses only, negative is NOT full 3-D disconnection proof',
        scene=meta['scene'],height=height,step=step,required_center_clearance=.6,
        edge_lower_bound=.6,source_sha256=meta['source_sha256'],start=start.tolist(),start_component=int(labels[ix,iy]),components=components)
    np.savez_compressed(output/'slice.npz',xs=xs,ys=ys,clearance=clearance.reshape(xx.shape),valid=valid.reshape(xx.shape),labels=labels)
    (output/'summary.json').write_text(json.dumps(summary,indent=2))
    print(json.dumps(summary),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('scene');p.add_argument('output');p.add_argument('--height',type=float,default=1.5);p.add_argument('--step',type=float,default=.25);a=p.parse_args();audit(a.scene,a.output,a.height,a.step)
