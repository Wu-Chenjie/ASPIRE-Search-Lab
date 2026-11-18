"""Independent door cross-section clearance bound; never supplies navigation goals."""
import argparse,json,math
from pathlib import Path
import numpy as np


def audit(scene,output,y=6.75,x_range=(3.8,6.),z_range=(.7,3.),fixed_axis=1):
    import trimesh
    scene=Path(scene);m=trimesh.load(scene/'assets/collision.stl',process=False)
    m.apply_translation(json.loads((scene/'scene-audit.json').read_text())['translation'])
    # Refine only cells whose Lipschitz upper bound could still admit the sphere.
    pending=[((x_range[0]+x_range[1])/2,(z_range[0]+z_range[1])/2,
              (x_range[1]-x_range[0])/2,(z_range[1]-z_range[0])/2,0)]
    leaf_bounds=[];max_distance=0.;max_point=None;samples=0;unresolved=0
    while pending:
        batch=pending[:32];pending=pending[32:]
        points=np.array([[x,y,z] if fixed_axis==1 else [y,x,z] for x,z,hx,hz,level in batch])
        _,ds,_=trimesh.proximity.closest_point(m,points);samples+=len(ds)
        for cell,point,d in zip(batch,points,ds):
            x,z,hx,hz,level=cell;bound=float(d+math.hypot(hx,hz))
            if d>max_distance:max_distance=float(d);max_point=point.tolist()
            if bound < .6:leaf_bounds.append(bound)
            elif level>=10 or d>=.6:
                leaf_bounds.append(bound);unresolved+=1
            else:
                pending.extend((x+dx*hx/2,z+dz*hz/2,hx/2,hz/2,level+1)
                               for dx in (-1,1) for dz in (-1,1))
    upper=max(leaf_bounds)
    result=dict(scope='Independent simulator geometry clearance; selected door only, not whole-scene connectivity',
        plane_axis='y' if fixed_axis==1 else 'x',plane_coordinate=y,horizontal_range=x_range,z_range=z_range,samples=samples,max_surface_distance_m=max_distance,
        maximizing_sample=max_point,unresolved_cells=unresolved,
        continuous_upper_bound_m=upper,required_clearance_m=.6,
        blocks_point6m_sphere=bool(upper<.6),
        bound='Distance to closed triangle set is 1-Lipschitz: each adaptively subdivided rectangle has upper bound center distance + rectangle circumradius; maximum over all leaves covers the full selected cross-section.')
    Path(output).write_text(json.dumps(result,indent=2));print(json.dumps(result));return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('scene');p.add_argument('output');p.add_argument('--axis',choices=['x','y'],default='y')
    p.add_argument('--coordinate',type=float,default=6.75);p.add_argument('--horizontal-range',type=float,nargs=2,default=[3.8,6.]);p.add_argument('--z-range',type=float,nargs=2,default=[.7,3.])
    a=p.parse_args();audit(a.scene,a.output,a.coordinate,a.horizontal_range,a.z_range,0 if a.axis=='x' else 1)
