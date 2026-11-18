import numpy as np
from tools.flight.localization_audit import raster
from tools.flight.flight_localization import interpolate

def test_model_ray_depth_on_known_slanted_plane():
    # Triangle vertices project to (0,0), (2,0), (0,2).
    mesh=np.array([[[0,0,2],[8,0,4],[0,4,2]]],float)
    depth=raster(mesh,np.eye(3),np.eye(4),(3,3))
    assert np.isclose(depth[0,1],8/3)
    assert np.isclose(depth[1,0],2)
    assert np.isinf(depth[2,2])

def test_truth_interpolation_requires_bounded_bracket_and_base_offset():
    rows=[dict(stamp=t,xyz=[t,0,0],quat=[0,0,0,1]) for t in [1,1.02,2]]
    assert np.allclose(interpolate(rows,1.01)[0],[1.01,0,.24])
    assert interpolate(rows,.9) is None
    assert interpolate(rows,1.5) is None
