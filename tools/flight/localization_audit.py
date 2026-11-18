"""Independent model-ray surface truth; never imported by online decisions."""
import argparse,json,sys,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[2];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'src/annual_planner/src')]
from person_search.perception import PeopleDetector,candidates,cloud

def triangles():
 tree=ET.parse(ROOT/'external/rgbd_scene_assets/person_standing/meshes/standing.dae')
 for n in tree.iter():n.tag=n.tag.split('}')[-1]
 geometries={}
 for g in tree.findall('.//library_geometries/geometry'):
  mesh=g.find('mesh');sources={s.get('id'):np.fromstring(s.findtext('float_array'),sep=' ').reshape(-1,int(s.find('technique_common/accessor').get('stride'))) for s in mesh.findall('source')};vertices={v.get('id'):v.find('input[@semantic="POSITION"]').get('source')[1:] for v in mesh.findall('vertices')};parts=[]
  for poly in mesh.findall('polylist'):
   stride=max(int(x.get('offset')) for x in poly.findall('input'))+1;inp=poly.find('input[@semantic="VERTEX"]');indices=np.fromstring(poly.findtext('p'),sep=' ',dtype=int).reshape(-1,stride)[:,int(inp.get('offset'))];v=sources[vertices[inp.get('source')[1:]]][:,:3];start=0
   for count in np.fromstring(poly.findtext('vcount'),sep=' ',dtype=int):
    face=indices[start:start+count];start+=count
    for j in range(1,count-1):parts.append(v[face[[0,j,j+1]]])
  geometries[g.get('id')]=np.array(parts)
 result=[]
 def visit(n,parent):
  m=n.find('matrix');t=parent@(np.fromstring(m.text,sep=' ').reshape(4,4) if m is not None else np.eye(4))
  for instance in n.findall('instance_geometry'):
   v=geometries[instance.get('url')[1:]];result.append(v@t[:3,:3].T+t[:3,3])
  for child in n.findall('node'):visit(child,t)
 for node in tree.findall('library_visual_scenes/visual_scene/node'):visit(node,np.eye(4))
 v=np.concatenate(result);v=v@Rotation.from_euler('x',.04).as_matrix().T+[0,0,.02]
 return v@Rotation.from_euler('z',-np.pi/2).as_matrix().T+[4,0,0]

def raster(mesh,k,t,shape):
 cam=(mesh-t[:3,3])@t[:3,:3];cam=cam[np.all(cam[:,:,2]>.25,axis=1)];k=np.asarray(k).reshape(3,3);uv=cam[:,:,:2]/cam[:,:,2,None]*[k[0,0],k[1,1]]+[k[0,2],k[1,2]];depth=np.full(shape,np.inf)
 for points,z in zip(uv,cam[:,:,2]):
  lo=np.maximum(np.ceil(points.min(axis=0)).astype(int),[0,0]);hi=np.minimum(np.floor(points.max(axis=0)).astype(int),[shape[1]-1,shape[0]-1])
  if np.any(lo>hi):continue
  a,b,c=points;den=(b[1]-c[1])*(a[0]-c[0])+(c[0]-b[0])*(a[1]-c[1])
  if abs(den)<1e-10:continue
  x,y=np.meshgrid(np.arange(lo[0],hi[0]+1),np.arange(lo[1],hi[1]+1));w0=((b[1]-c[1])*(x-c[0])+(c[0]-b[0])*(y-c[1]))/den;w1=((c[1]-a[1])*(x-c[0])+(a[0]-c[0])*(y-c[1]))/den;w2=1-w0-w1;inside=(w0>=-1e-9)&(w1>=-1e-9)&(w2>=-1e-9)
  if inside.any():
   zz=1/(w0[inside]/z[0]+w1[inside]/z[1]+w2[inside]/z[2]);xx=x[inside];yy=y[inside];depth[yy,xx]=np.minimum(depth[yy,xx],zz)
 return depth

def audit(run):
 import cv2
 fixture=json.loads((run/'viewpoints.json').read_text());mesh=triangles();detector=PeopleDetector();rows=[]
 for name in ('visible','partial','hidden','revealed'):
  folder=run/name;rec=json.loads((folder/'frame.json').read_text());v=fixture['views'][name];t=np.eye(4);t[:3,:3]=Rotation.from_euler('z',v['lens_yaw']).as_matrix()@np.array([[0,0,1],[-1,0,0],[0,-1,0]]);t[:3,3]=v['lens_xyz'];d=np.load(folder/'depth.npy');rgb=cv2.imread(str(folder/'rgb.png'))[:,:,::-1];k=rec['depth']['k'];cs=candidates(rgb,d,k,t,detector);gt=raster(mesh,k,t,d.shape);gtcloud=cloud(gt,k,t);observations=[]
  for c in cs:
   valid=c.pixels&np.isfinite(gt);fraction=float(valid.sum()/c.pixels.sum())
   if fraction<.95:observations.append(dict(semantic=c.semantic,matched_fraction=fraction,unscored='candidate does not match model surface'));continue
   expected=np.median(gtcloud[valid],axis=0);err=float(np.linalg.norm(c.xyz-expected));observations.append(dict(semantic=c.semantic,pixels=int(valid.sum()),matched_fraction=fraction,estimated=c.xyz.tolist(),surface_truth=expected.tolist(),surface_error_3d_m=err,axial_depth_error_median_m=float(np.median(abs(d[valid]-gt[valid]))),axial_depth_error_max_m=float(np.max(abs(d[valid]-gt[valid])))))
  rows.append(dict(view=name,saved_frames=1,candidates=len(cs),semantic_detections=sum(c.semantic for c in cs),observations=observations))
 observations=[x for r in rows for x in r['observations'] if 'surface_error_3d_m' in x];errors=[x['surface_error_3d_m'] for x in observations];xy=[np.array(x['estimated'][:2]) for x in observations];truthxy=[np.array(x['surface_truth'][:2]) for x in observations]
 return dict(definition='Median model-ray intersections at the same candidate interior pixels. Exact fixture camera pose; not body center or flight localization accuracy.',samples=len(observations),median_surface_error_3d_m=float(np.median(errors)),max_surface_error_3d_m=max(errors),cross_view_estimated_xy_max_separation_m=float(max(np.linalg.norm(a-b) for a in xy for b in xy)),cross_view_true_surface_xy_max_separation_m=float(max(np.linalg.norm(a-b) for a in truthxy for b in truthxy)),views=rows,limits='One saved frame per view; 120 stamp checks are not 120 localization samples. Pixel-center/rasterization and mesh tessellation affect this audit. Aircraft estimator error excluded.')
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args();r=audit(a.run);a.output.write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
