"""Planning hint from the executor's current private observed voxel map.

This raster is not an execution certificate. The common continuous path, actual
sensor, unanimous lease, and C2 checks remain authoritative.
"""
import numpy as np
from scipy.spatial import cKDTree
from core.exploration.decentralized import sample_path


def execution_grid_array(runtime, stamp, traffic_paths=()):
    runtime.rebuild()
    distance=np.asarray(runtime.distance,float)
    safe=np.asarray(runtime.safe,bool).copy()
    # Native optimization must see the hard flight-height boundaries, including
    # half a voxel of planning margin. The common executor's limits stay intact.
    z=runtime.origin[2]+(np.arange(runtime.shape[2])+.5)*runtime.resolution
    low,high=runtime.flight_limits; margin=.5*runtime.resolution
    height_distance=runtime.clearance+runtime.metric[2]*np.minimum(z-low-margin,high-margin-z)
    height_distance[np.abs(height_distance-runtime.clearance)<1e-9]=runtime.clearance
    distance=np.minimum(distance,height_distance[None,None,:])
    safe &= height_distance[None,None,:]>=runtime.clearance
    # Match the authorization ledger's XY exclusions, before native route
    # search. This changes only the read-only hint, never sensed occupancy.
    samples=[]
    for path in traffic_paths:
        p=np.asarray(path,float).reshape(-1,3)
        if not np.isfinite(p).all():raise ValueError('Non-finite traffic path')
        if len(p):samples.append(sample_path(p)[:,:2])
    if samples:
        xy=runtime.origin[:2]+(np.indices(runtime.shape[:2]).reshape(2,-1).T+.5)*runtime.resolution
        separation=cKDTree(np.vstack(samples)).query(xy)[0].reshape(runtime.shape[:2])
        # 5 cm accounts for the sampled polyline; final leases remain exact.
        traffic_distance=runtime.clearance+separation-1.25
        distance=np.minimum(distance,traffic_distance[:,:,None])
        safe &= separation[:,:,None]>=1.25
    if not np.isfinite(stamp) or not np.isfinite(distance).all():
        raise ValueError('Non-finite execution grid')
    header=[float(stamp),*runtime.origin.tolist(),float(runtime.resolution),
            *map(float,runtime.shape),float(runtime.clearance),*map(float,runtime.flight_limits),float(runtime.version)]
    return np.concatenate((header,distance.ravel(),safe.ravel()))
