"""Offline whole-floor geometry screening; never imported by online navigation.

Positive components need separate structural region/portal and spawn review.
Negative results describe this finite conservative lattice, not impossibility.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.ndimage import label
from scipy.spatial.distance import pdist, squareform
from tools.hm3d.box_audit import intersects
from tools.hm3d.prepare import distances, LOCKS


def floors(bins):
    selected=[]
    for height,area in sorted(bins,key=lambda p:(-p[1],p[0])):
        if area>=12. and all(abs(height-z)>.3 for z in selected):
            selected.append(height)
    return sorted(selected)


def components(valid,axes):
    labels,count=label(valid)
    result=[]
    for cid in range(1,count+1):
        indices=np.array(np.where(labels==cid)).T
        points=np.column_stack([a[indices[:,d]] for d,a in enumerate(axes)])
        xy=np.unique(points[:,:2],axis=0)
        span=float(pdist(xy).max()) if len(xy)>1 else 0.
        cruise=points[points[:,2]>=2.]
        endpoint_span=0.;endpoints=[]
        if len(cruise)>1:
            d=squareform(pdist(cruise[:,:2]));i,j=np.unravel_index(d.argmax(),d.shape)
            endpoint_span=float(d[i,j]);endpoints=[cruise[i].tolist(),cruise[j].tolist()]
        result.append(dict(id=cid,cells=len(points),bounds=[points.min(0).tolist(),points.max(0).tolist()],
            horizontal_diameter_m=span,cruise_cells=len(cruise),cruise_endpoint_separation_m=endpoint_span,
            endpoints=endpoints,candidate_for_region_review=endpoint_span>=6.))
    return labels,sorted(result,key=lambda c:(-c['cruise_endpoint_separation_m'],-c['cells']))


def screen(source,floor,out):
    import trimesh
    source,out=Path(source),Path(out);out.mkdir(parents=True,exist_ok=False)
    digest=hashlib.sha256(source.read_bytes()).hexdigest()
    if digest!=LOCKS[source.stem]:raise ValueError('Unverified source geometry')
    mesh=trimesh.load_scene(source,process=False,skip_materials=True).to_mesh()
    lo=mesh.bounds[0].copy();translation=np.array([.6-lo[0],.6-lo[1],-floor])
    mesh.apply_translation(translation)
    axes=[(np.arange(np.floor(mesh.bounds[0,d]/.3),np.ceil(mesh.bounds[1,d]/.3))+.5)*.3 for d in (0,1)]
    axes.append(np.arange(3,10)*.3+.15)  # 1.05 through 2.85, unchanged native centers.
    shape=tuple(map(len,axes));base=np.zeros(shape,bool);wide=np.zeros(shape,bool)
    tree=mesh.triangles_tree;triangles=mesh.triangles
    for n,index in enumerate(np.ndindex(shape)):
        c=np.array([a[i] for a,i in zip(axes,index)])
        ids=list(tree.intersection([*(c-.9),*(c+.9)]))
        subset=triangles[ids]
        base[index]=not intersects(subset,c,np.full(3,.75)).any()
        if base[index]:wide[index]=not intersects(subset,c,np.full(3,.9)).any()
        if n%4096==0:print(source.stem,floor,n,int(np.prod(shape)),flush=True)
    indices=np.array(np.where(base)).T
    if len(indices):
        points=np.column_stack([a[indices[:,d]] for d,a in enumerate(axes)])
        down=distances(mesh,points,np.tile([0,0,-1],(len(points),1)))
        up=distances(mesh,points,np.tile([0,0,1],(len(points),1)))
        enclosed=(np.abs(down-points[:,2])<.15)&np.isfinite(up)&(up<5)&(up>.6)
        for i in indices[~enclosed]:base[tuple(i)]=False;wide[tuple(i)]=False
    base_labels,bc=components(base,axes);wide_labels,wc=components(wide,axes)
    report=dict(scene=source.stem,source=str(source),source_sha256=digest,floor_source_z=floor,
        translation=translation.tolist(),resolution_m=.3,native_half_width_m=.75,preferred_half_width_m=.9,
        tested_cells=int(np.prod(shape)),native_compatible_cells=int(base.sum()),preferred_cells=int(wide.sum()),
        native_components=bc,preferred_components=wc,
        candidate_for_region_review=any(c['candidate_for_region_review'] for c in wc),
        coordinate_frame='Original scan translated rigidly as in prepare.py; no scale, triangle removal or geometry repair.',
        scope='Offline finite lattice SAT and enclosed-floor ray filter only. Not an online free-space map or route.',
        limitation='A component can be one room. Must independently verify a real inter-region portal and safe spawn before any flight. Failure is not proof of global continuous-space impossibility.')
    np.savez_compressed(out/'lattice.npz',xs=axes[0],ys=axes[1],zs=axes[2],native_labels=base_labels,preferred_labels=wide_labels)
    (out/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if not k.endswith('components')}),flush=True)
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--inventory',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    reports=[]
    for row in json.loads(a.inventory.read_text()):
        if row['scene']=='NBg5UqG3di3':continue
        for floor in floors(row['upward_surface_height_area_bins']):
            reports.append(screen(row['path'],floor,a.output/(row['scene']+'-floor-'+str(round(floor,3)))))
    (a.output/'all-floors.json').write_text(json.dumps(reports,indent=2)+'\n')
