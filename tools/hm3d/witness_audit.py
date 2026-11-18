"""Save an independently verified geometry witness, never a planner input."""
import argparse,json
from collections import deque
from pathlib import Path
import numpy as np


def witness(scene,slice_dir,output,start,end):
    import trimesh
    from tools.hm3d.prepare import path_clearance
    scene=Path(scene);data=np.load(Path(slice_dir)/'slice.npz');xs,ys=data['xs'],data['ys'];valid=data['valid']
    metadata=json.loads((Path(slice_dir)/'summary.json').read_text());z=metadata['height']
    def index(p):return (int(np.argmin(abs(xs-p[0]))),int(np.argmin(abs(ys-p[1]))))
    a,b=index(start),index(end)
    if not valid[a] or not valid[b]:raise ValueError('Requested endpoint not in certified graph')
    queue=deque([a]);parent={a:None}
    while queue and b not in parent:
        q=queue.popleft()
        for delta in [(1,0),(-1,0),(0,1),(0,-1)]:
            n=(q[0]+delta[0],q[1]+delta[1])
            if 0<=n[0]<len(xs) and 0<=n[1]<len(ys) and valid[n] and n not in parent:
                parent[n]=q;queue.append(n)
    if b not in parent:raise ValueError('No certified slice path; not a full 3-D impossibility proof')
    cells=[];q=b
    while q is not None:cells.append(q);q=parent[q]
    path=np.array([[xs[i],ys[j],z] for i,j in cells[::-1]])
    meta=json.loads((scene/'scene-audit.json').read_text())
    mesh=trimesh.load(scene/'assets/collision.stl',process=False);mesh.apply_translation(meta['translation'])
    proofs=[path_clearance(lambda p:trimesh.proximity.closest_point(mesh,p)[1],a,b) for a,b in zip(path[:-1],path[1:])]
    result=dict(scope='Independent offline cross-region candidate; geometry only, not autonomous traversal. Never consumed by runtime.',source_sha256=meta['source_sha256'],
                path=path.tolist(),length_m=float(np.linalg.norm(np.diff(path,axis=0),axis=1).sum()),required_center_clearance_m=.6,
                continuous_surface_distance_lower_bound_m=min(p['lower_bound_m'] for p in proofs),verified=all(p['verified'] for p in proofs),
                room_boundary_review='Requires independent visual review of regions; native voxel dilation compatibility audited separately')
    Path(output).write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k!='path'}));return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('scene');p.add_argument('slice');p.add_argument('output');p.add_argument('--start',nargs=2,type=float,required=True);p.add_argument('--end',nargs=2,type=float,required=True)
    a=p.parse_args();witness(a.scene,a.slice,a.output,a.start,a.end)
