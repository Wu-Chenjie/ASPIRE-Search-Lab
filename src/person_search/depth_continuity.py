"""Corroborate short visual-track reacquisition with measured RGB-D support."""
import math
import numpy as np


def measured_support(mask, depth, view):
    if mask is None or mask.shape != depth.shape:
        return None
    y,x=np.nonzero(mask)
    observed=depth[y,x]
    positive=(observed>0)&(observed<np.inf)
    count=int(np.count_nonzero(positive))
    if count<120 or count<.8*len(x):
        return None
    step=math.ceil(count/512)
    x,y=x[positive][::step],y[positive][::step]
    rays = np.linalg.solve(view['k'], np.stack([x,y,np.ones(len(x))]))
    return (view['rotation'] @ (rays*depth[y,x])) + view['origin'][:,None]


def depth_support_matches(points, mask, depth, view):
    if points is None or mask is None or mask.shape != depth.shape:
        return False
    camera = view['rotation'].T @ (points-view['origin'][:,None])
    required=max(80,.60*points.shape[1])
    camera=camera[:,camera[2]>1e-6]
    if camera.shape[1]<required:return False
    pixels=view['k']@camera
    with np.errstate(invalid='ignore',divide='ignore',over='ignore'):
        x,y=np.rint(pixels[:2]/pixels[2])
    inside=(x>=0)&(x<depth.shape[1])&(y>=0)&(y<depth.shape[0])
    x,y=x[inside].astype(int),y[inside].astype(int)
    masked=mask[y,x]
    if np.count_nonzero(masked)<required:return False
    observed=depth[y[masked],x[masked]]
    expected=camera[2,inside][masked]
    return np.count_nonzero((observed>0)&(abs(observed-expected)<=.20))>=required
