"""Offline scene checks must preserve missing geometry as unknown."""
from types import SimpleNamespace
import numpy as np
from tools.hm3d.prepare import distances


def test_missing_mesh_hits_remain_infinite_not_free_distance():
    ray=SimpleNamespace(intersects_location=lambda *a,**kw:(np.array([]),np.array([],dtype=int),np.array([],dtype=int)))
    mesh=SimpleNamespace(ray=ray)
    d=distances(mesh,[[0,0,1],[1,0,1]],[[0,0,-1],[0,0,-1]])
    assert np.isinf(d).all()


def test_sparse_ray_hits_keep_original_ray_indices():
    def intersect(origins,directions,**kw):
        return np.array([[2.,0.,0.]]),np.array([1]),np.array([0])
    d=distances(SimpleNamespace(ray=SimpleNamespace(intersects_location=intersect)),
                [[0,0,1],[2,0,1]],[[0,0,-1],[0,0,-1]])
    assert np.isinf(d[0]) and d[1]==1.


def test_bootstrap_audit_rejects_obstacle_between_clear_endpoint_heights():
    from tools.hm3d.prepare import path_clearance
    # Endpoints can be clear while an overhang cuts through the vertical scan.
    def surface_distance(points):return np.abs(points[:,2]-1.25)+.1
    assert surface_distance(np.array([[0,0,.5],[0,0,2.]] )).min()>.6
    assert not path_clearance(surface_distance,[0,0,.5],[0,0,2.])['verified']
    assert path_clearance(lambda p:np.full(len(p),.7),[0,0,.7],[0,0,1.8])['verified']


def test_measured_bootstrap_guard_requires_persistent_deviation():
    from tools.flight.run_safety import BootstrapTrackingWatchdog
    w=BootstrapTrackingWatchdog()
    def message(t,error,reason='initial_yaw_scan'):
        return dict(time=t,tracking_error=error,reason=reason,ready=False)
    assert w.observe(message(1,1.,'initial_scan_ascent')) is None
    assert w.observe(message(2,.8)) is None
    assert w.observe(message(4,.1)) is None
    assert w.observe(message(5,.8)) is None
    assert w.observe(message(7.99,.8)) is None
    assert 'tracking deviation persisted' in w.observe(message(8,.8))


def test_triangle_box_audit_handles_crossing_without_inside_vertices():
    from tools.hm3d.box_audit import intersects
    triangles=np.array([[[-2,0,0],[2,-2,0],[2,2,0]],[[2,2,2],[3,2,2],[2,3,2]],[[.9,.9,0],[.9,0,.9],[0,.9,.9]]])
    assert intersects(triangles,[0,0,0],[.5,.5,.5]).tolist()==[True,False,False]
    # Its AABB overlaps the box, but plane x+y+z=1.8 separates it.
    assert intersects(triangles[-1:],[0,0,0],[.7,.7,.7])[0]
    assert not intersects(triangles[-1:],[0,0,0],[.2,.2,.2])[0]
