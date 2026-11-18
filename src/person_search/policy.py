"""Select optional review viewpoints using measured free-space only.

RACER remains responsible for baseline exploration and all trajectory generation.
This filter is conservative and rejects unknown space, including behind the camera.
"""
from itertools import product
import numpy as np
from scipy.spatial import cKDTree
from .perception import cloud, validate_transform


class ObservedVolume:
    def __init__(self, clearance=.35):
        self.frames=[];self.clearance=clearance;self.surfaces=[];self._tree=None

    @property
    def tree(self):
        if self._tree is None:
            self._tree=cKDTree(np.concatenate(self.surfaces))
        return self._tree
    def add(self, depth, k, transform, *, points=None):
        """Use an existing pixel-aligned projection of this capture when supplied."""
        if points is None:
            points = cloud(depth, k, transform)
        else:
            transform = validate_transform(transform)
        self.frames.append((depth.copy(), np.asarray(k).reshape(3,3), np.array(transform, dtype=float)))
        self.frames=self.frames[-30:]
        pts = points[::3, ::3]
        pts = pts[np.isfinite(pts).all(axis=-1)]
        self.surfaces.append(pts);self.surfaces=self.surfaces[-30:]
        self._tree=None
    def free(self, point):
        # Remaining cube samples must each be in front of a measured 3×3 patch.
        samples=np.asarray(point)+np.array(list(product((-self.clearance,0,self.clearance),repeat=3)))
        patch_y,patch_x=np.indices((3,3)).reshape(2,9)-1
        for depth,k,t in self.frames:
            cam=(samples-t[:3,3])@t[:3,:3]
            with np.errstate(divide='ignore',invalid='ignore'):
                uv=np.rint(cam[:,:2]/cam[:,2,None]*[k[0,0],k[1,1]]+[k[0,2],k[1,2]])
            inside=(cam[:,2]>.25)&np.all((uv>=1)&(uv<np.asarray(depth.shape[::-1])-1),axis=1)
            u,v=uv[inside].astype(int).T
            patches=depth[v[:,None]+patch_y,u[:,None]+patch_x]
            known=inside.copy()
            known[inside]=np.isfinite(patches).all(axis=1)&(patches.min(axis=1)>cam[inside,2]+.10)
            samples=samples[~known]
            if not samples.size:
                return True
        return False

    def path_free(self,start,end):
        distance=np.linalg.norm(np.asarray(end)-start)
        if distance==0:return self.free(start)
        return all(self.free(p) for p in np.linspace(start,end,max(2,int(distance/.05)+1)))


def choose_view(track, origin, volume, previous=(), strategy='active', min_horizontal=0.):
    if track.state in ('confirmed','abandoned','unresolved'):return None
    origin=np.asarray(origin);direction=track.xyz-origin;direction[2]=0
    direction/=max(np.linalg.norm(direction),1e-6)
    side=np.array([-direction[1],direction[0],0.])
    old=track.xyz-origin
    old/=max(np.linalg.norm(old),1e-6)
    options=[]
    for forward,lateral,dz in product((-.7,0.,.7,1.1,1.5),(-1.5,-1.,-.6,-.3,.3,.6,1.,1.5),(0.,.4,-.3)):
        p=origin+forward*direction+lateral*side+[0,0,dz]
        displacement=p-origin
        if np.linalg.norm(displacement[:2])<min_horizontal:continue
        if not .7<=p[2]<=3.0 or any(np.linalg.norm(p-q)<.4 for q in previous):continue
        ray=track.xyz-p
        distance=np.linalg.norm(ray[:2])
        # Keep a complete plausible standing person inside vertical FOV.
        if not 2.8<=distance<=5.5 or max(p[2],2.-p[2])>distance*.48 or not volume.free(p):continue
        if strategy=='lateral':
            score=-abs(lateral+.6)-forward-abs(dz)
        else:
            # RACER certifies the path after this endpoint check.
            length=np.linalg.norm(ray)
            parallax=np.arccos(np.clip((ray/length)@old,-1,1))
            samples=np.linspace(p,track.xyz,max(2,int(length/.05)))[:-8]
            occluded=bool(len(samples) and volume.surfaces and np.any(volume.tree.query(samples)[0]<.12))
            score=parallax-.03*np.linalg.norm(displacement)-(2. if occluded else 0.)-.1*abs(p[2]-1.1)
        options.append((score,p))
    return max(options,key=lambda v:v[0])[1] if options else None
