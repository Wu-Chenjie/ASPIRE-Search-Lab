"""Three-dimensional ray integration and known-free voxel routing.

The mapper accepts sensor-frame endpoints plus a pose, never scenario geometry.
Ray misses are integrated as free until max range; hits update occupied voxels.
"""
import numpy as np
from scipy.spatial import cKDTree
from core.obstacles import OccupancyGrid
from core.exploration.collision_geometry import (collision_metric, BODY_HALF_XY,
    BODY_CENTER_Z, BODY_HALF_Z, CLEARANCE_TOLERANCE_M)
from core.exploration.voxel_config import voxel_config


class VoxelMap:
    ndim = 3
    def __init__(self, bounds, resolution=None, clearance=None, flight_limits=None):
        configured=voxel_config()
        resolution=configured.resolution if resolution is None else resolution
        clearance=configured.clearance if clearance is None else clearance
        self.collision_profile=configured.collision_profile
        self.bounds = np.asarray(bounds, float); self.origin = self.bounds[0].copy()
        self.flight_limits = (.7, self.bounds[1, 2]-.3) if flight_limits is None else tuple(flight_limits)
        self.resolution = resolution; self.clearance = clearance; self.altitude = 1.5
        self.shape = tuple(np.ceil((self.bounds[1]-self.origin)/resolution).astype(int))
        self.state = np.full(self.shape, -1, np.int8); self.version = 0; self.field = self
        self.recovery_yaw = None
        self._built_signature = None
        self.rebuild()

    def indices(self, points):
        return np.floor((np.asarray(points)-self.origin)/self.resolution).astype(int)

    def points(self, indices):
        return self.origin+(np.asarray(indices)+.5)*self.resolution

    def update(self, indices, values):
        indices = np.asarray(indices, int).reshape(-1, 3); key = tuple(indices.T)
        changed = bool(np.any(self.state[key] != values))
        self.state[key] = values
        if changed:
            self.version += 1
        return changed

    def integrate(self, sensor_origin, endpoints, hits):
        origin = np.asarray(sensor_origin); endpoints = np.asarray(endpoints)
        hits = np.asarray(hits, bool); finite = np.isfinite(endpoints).all(axis=1)
        endpoints = endpoints[finite]; hits = hits[finite]
        if not len(endpoints):
            return np.empty((0, 3), int), np.empty(0, np.int8)
        count = max(2, int(np.ceil(np.max(np.linalg.norm(endpoints-origin, axis=1))/(self.resolution*.4)))+1)
        rays = origin+(endpoints-origin)[:, None, :]*np.linspace(0., 1., count)[None, :, None]
        cells = self.indices(rays)
        valid = np.all((cells >= 0) & (cells < self.shape), axis=2)
        # Surface returns within one boundary voxel are clamped to that voxel.
        # This handles range noise around the floor/map boundary conservatively.
        near = np.all((cells[:, -1] >= -1) & (cells[:, -1] <= self.shape), axis=1)
        occupied = np.clip(cells[:, -1][hits & near], 0, np.array(self.shape)-1)
        ids = np.unique(np.ravel_multi_index(tuple(cells[valid].T), self.shape))
        occupied_ids = np.unique(np.ravel_multi_index(tuple(occupied.T), self.shape)) if len(occupied) else np.empty(0, int)
        values = np.isin(ids, occupied_ids).astype(np.int8)
        indices = np.column_stack(np.unravel_index(ids, self.shape))
        self.update(indices, values)
        return indices, values

    def rebuild(self):
        # Copies sent to the planner already contain a valid collision field.
        # Compare bytes as well as configuration: tests/imports may edit state
        # directly without incrementing the sensor version counter.
        signature = (self.state.tobytes(), self.clearance, self.flight_limits, self.collision_profile)
        if self._built_signature == signature:
            return
        self._built_signature = signature
        self.reserved_points = None
        # Locked X500 + OakD collision geometry, including swept propellers
        # and landing gear; share this metric with routing and path checks.
        self.metric = collision_metric(self.clearance,self.collision_profile)
        blocked=np.pad(self.state != 0,1,constant_values=True)
        if self.collision_profile=='x500_swept_sphere':
            # For free-center queries, only boundary boxes of the obstacle
            # union can be first intersected. Retain all six-face neighbors of
            # free space, including unknown and the padded map boundary. This
            # avoids indexing millions of interior unknown boxes; it erases
            # no occupancy. Occupied-center queries below fail closed.
            adjacent=np.zeros(blocked.shape,bool)
            for axis in range(3):
                a=[slice(None)]*3;b=a.copy();a[axis]=slice(1,None);b[axis]=slice(None,-1)
                adjacent[tuple(a)] |= ~blocked[tuple(b)]
                adjacent[tuple(b)] |= ~blocked[tuple(a)]
            boundary=blocked & adjacent
            occupied=np.argwhere(boundary)-1
            if not len(occupied):occupied=np.array([[-1,-1,-1]])
        else:
            occupied=np.argwhere(blocked)-1
        self.obstacle_centers = self.points(occupied)
        self.obstacle_tree = cKDTree(self.obstacle_centers*self.metric)
        # Online envelope checks need the obstacle tree, not a distance for
        # every free voxel. Materialize the navigation grid only on demand.
        self._physical_distance = None; self._navigation_dirty = True
        self._navigation_reservations = []
        for name in ('_distance','_safe','_grid'):
            self.__dict__.pop(name,None)

    def _ensure_navigation(self):
        if not self._navigation_dirty:
            return
        if self._physical_distance is None:
            self._physical_distance = np.full(self.shape, -1.)
            free = np.argwhere(self.state == 0)
            if len(free):
                points = self.points(free)
                if self.collision_profile=='x500_swept_sphere':
                    centers,indices=self.obstacle_tree.query(points*self.metric)
                    half_diagonal=np.linalg.norm(self.metric*self.resolution/2)
                    lower=np.maximum(centers-half_diagonal,0.)
                    delta=np.maximum(abs(points-self.obstacle_centers[indices])-self.resolution/2,0.)
                    upper=np.linalg.norm(delta*self.metric,axis=1)
                    # Far cells retain a conservative lower bound. Only the
                    # narrow band whose safety is undecided needs exact boxes.
                    ambiguous=(lower<self.clearance)&(upper>=self.clearance)
                    p=points[ambiguous];radii=upper[ambiguous]+half_diagonal+1e-12
                    neighbors=self.obstacle_tree.query_ball_point(p*self.metric,radii)
                    counts=np.array([len(v) for v in neighbors],int)
                    if counts.sum():
                        ids=np.concatenate([v for v in neighbors if len(v)]).astype(int)
                        owner=np.repeat(np.arange(len(p)),counts)
                        delta=np.maximum(abs(p[owner]-self.obstacle_centers[ids])-self.resolution/2,0.)
                        exact=upper[ambiguous].copy()
                        np.minimum.at(exact,owner,np.linalg.norm(delta*self.metric,axis=1))
                        lower[ambiguous]=exact
                    self._physical_distance[tuple(free.T)]=lower
                else:
                    _, indices = self.obstacle_tree.query(points*self.metric, k=min(16, len(self.obstacle_centers)))
                    delta = np.maximum(np.abs(points[:, None]-self.obstacle_centers[indices])-self.resolution/2, 0.)
                    self._physical_distance[tuple(free.T)] = np.min(np.linalg.norm(delta*self.metric, axis=2), axis=1)
        self._distance = self._physical_distance.copy()
        self._safe = self._distance >= self.clearance
        heights = self.origin[2]+(np.arange(self.shape[2])+.5)*self.resolution
        self._safe &= (heights[None, None, :] >= self.flight_limits[0]) & (heights[None, None, :] <= self.flight_limits[1])
        for tree, radius in self._navigation_reservations:
            cells = np.argwhere(self._safe)
            if len(cells):
                blocked = tree.query(self.points(cells))[0] < radius
                key = tuple(cells[blocked].T); self._safe[key] = False; self._distance[key] = 0.
        self.__dict__.pop('_grid', None)
        self._navigation_dirty = False

    @property
    def distance(self):
        self._ensure_navigation(); return self._distance

    @property
    def safe(self):
        self._ensure_navigation(); return self._safe

    @property
    def grid(self):
        self._ensure_navigation()
        if '_grid' not in self.__dict__:
            grid = OccupancyGrid(self.origin+self.resolution/2, self.resolution, self.shape)
            np.logical_not(self._safe, out=grid.data)
            self._grid = grid
        return self._grid

    def signed_distance(self, p):
        idx = self.indices(p)
        if np.any(idx < 0) or np.any(idx >= self.shape):
            return -1.
        if self.collision_profile=='x500_swept_sphere' and self.state[tuple(idx)]!=0:return -1.
        _, ids = self.obstacle_tree.query(np.asarray(p)*self.metric, k=min(16, len(self.obstacle_centers)))
        delta = np.maximum(np.abs(np.asarray(p)-self.obstacle_centers[ids])-self.resolution/2, 0.)
        return float(np.min(np.linalg.norm(delta*self.metric, axis=1)))

    def signed_distances(self, points):
        """Batch the scalar obstacle-box distance; retain the same 16 neighbors."""
        p=np.asarray(points,float).reshape(-1,3);idx=self.indices(p)
        valid=np.all((idx>=0)&(idx<self.shape),axis=1)
        if self.collision_profile=='x500_swept_sphere':
            valid_ids=np.flatnonzero(valid)
            valid[valid_ids] &= self.state[tuple(idx[valid_ids].T)]==0
        result=np.full(len(p),-1.)
        if valid.any():
            k=min(16,len(self.obstacle_centers))
            _,ids=self.obstacle_tree.query(p[valid]*self.metric,k=k)
            if k==1:ids=ids[:,None]
            delta=np.maximum(np.abs(p[valid,None]-self.obstacle_centers[ids])-self.resolution/2,0.)
            result[valid]=np.min(np.linalg.norm(delta*self.metric,axis=2),axis=1)
        return result

    def safe_path(self, path):
        p = np.asarray(path, float)
        if p.ndim != 2 or p.shape[1] != 3 or len(p) == 0 or not np.isfinite(p).all():
            return False
        chunks = [p]
        for a, b in zip(p[:-1], p[1:]):
            steps = int(np.ceil(np.linalg.norm(b-a)/.06))
            if steps > 1:
                chunks.append(np.linspace(a, b, steps+1)[1:-1])
        points = np.vstack(chunks); idx = self.indices(points)
        if np.any(points[:, 2] < self.flight_limits[0]) or np.any(points[:, 2] > self.flight_limits[1]):
            return False
        if not (np.all(idx >= 0) and np.all(idx < self.shape)):
            return False
        # A centerline inside an occupied/unknown box cannot pass any positive
        # body-clearance check. Reject it before the costly neighborhood query;
        # free centerlines still undergo the full envelope and reservation test.
        if np.any(self.state[tuple(idx.T)] != 0):
            return False
        # Every voxel box potentially intersecting the ellipsoid is checked;
        # nearest-center interpolation alone is not a safety certificate.
        radius = self.clearance+np.linalg.norm(self.metric*self.resolution/2)
        recovery_yaw = self.recovery_yaw
        if recovery_yaw is not None:
            # Recovery is a rotated box, not the ordinary ellipsoid. Its
            # corners must be included in the broad phase at every yaw.
            radius = max(radius, np.linalg.norm(self.metric*np.array([
                np.sqrt(2)*BODY_HALF_XY, np.sqrt(2)*BODY_HALF_XY,
                abs(BODY_CENTER_Z)+BODY_HALF_Z]))+np.linalg.norm(self.metric*self.resolution/2))
        neighbours = self.obstacle_tree.query_ball_point(points*self.metric, radius)
        counts = np.array([len(v) for v in neighbours]); total = int(counts.sum())
        if total:
            indices = np.concatenate([v for v in neighbours if len(v)]).astype(int)
            repeated = np.repeat(points, counts, axis=0)
            delta = np.maximum(np.abs(repeated-self.obstacle_centers[indices])-self.resolution/2, 0.)
            if recovery_yaw is None:
                if np.any(np.linalg.norm(delta*self.metric, axis=1) < self.clearance-CLEARANCE_TOLERANCE_M):
                    return False
            else:
                # Short, fixed-yaw retreat uses the actual collision hull.
                # Test all separating axes of the oriented hull
                # and occupied/unknown voxel boxes; do not erase unknown space.
                offset = self.obstacle_centers[indices]-repeated
                c, s = np.cos(recovery_yaw), np.sin(recovery_yaw)
                axes = np.array([[c, s], [-s, c]])
                offset[:, 2] -= BODY_CENTER_Z
                half = self.resolution/2; body = BODY_HALF_XY
                xy_overlap = np.all(np.abs(offset[:, :2]) <= half+body*(abs(c)+abs(s)), axis=1)
                body_overlap = np.all(np.abs(offset[:, :2]@axes.T) <= body+half*(abs(c)+abs(s)), axis=1)
                if np.any(xy_overlap & body_overlap & (np.abs(offset[:, 2]) <= half+BODY_HALF_Z)):
                    return False
        if self.reserved_points is not None:
            if np.any(self.reserved_points.query(points)[0] < self.reserved_radius):
                return False
        return True

    def block_paths(self, paths, radius=1.25):
        chunks = []
        for path in paths:
            p = np.asarray(path, float); chunks.append(p)
            chunks.extend(np.linspace(a, b, max(2, int(np.linalg.norm(b-a)/.15)+1)) for a, b in zip(p[:-1], p[1:]))
        if not chunks:
            return
        self._built_signature = None
        # Full 3D reservation distance, conservative isotropic downwash margin.
        self.reserved_points = cKDTree(np.vstack(chunks)); self.reserved_radius = radius
        self._navigation_reservations.append((self.reserved_points, radius))
        self._navigation_dirty = True
