import numpy as np
from tools.flight.contact_audit import bottom_points


def test_contact_audit_does_not_apply_base_offset_twice():
    rows=[dict(stamp=t,xyz=[0,0,-.013],quat=[0,0,0,1]) for t in (1.,1.02)]
    # Model-origin -13 mm plus 240 mm base offset puts the skid on z=0.
    points=bottom_points(rows,1.01,[0,.132,-.2195,0,0,0],[.25,.015,.015])
    np.testing.assert_allclose(points[:,2],0,atol=1e-12)
    np.testing.assert_allclose(points.mean(0),[0,.132,0],atol=1e-12)
