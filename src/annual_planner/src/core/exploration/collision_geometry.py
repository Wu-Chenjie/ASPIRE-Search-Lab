"""Collision envelopes of the locked PX4 x500_depth collision primitives.

Model commit: e05f4312d3f28aa621157610584a4870406cb6d3.
Coordinates are relative to base_link, not the model's +0.24m spawn pose.
The rotating blade boxes are enclosed over every rotor angle. Recovery uses
the resulting body-axis box; ordinary routing uses a yaw-invariant ellipsoid
containing every primitive, with additional clearance. Neither is visual mesh
geometry. The opt-in swept sphere encloses all vehicle attitudes, but its native
flight validation has not passed; geometry containment alone does not certify
localization or tracking. See tests/test_x500_collision.py for SDF verification.
"""
import numpy as np

ROTOR_RADIUS = np.hypot(.2792307692307692, .016923076923076923)/2
BODY_HALF_XY = .174 + ROTOR_RADIUS
BODY_BOTTOM = -.227
BODY_TOP = .270
BODY_CENTER_Z = (BODY_BOTTOM + BODY_TOP)/2
BODY_HALF_Z = (BODY_TOP - BODY_BOTTOM)/2
# Voxel boxes are axis-aligned and can sit a few micrometres inside the modelled
# clearance (round 28: 0.49998 m against 0.50 m). Motion admission ignores that
# numerical residue; unknown and occupied space still block.
CLEARANCE_TOLERANCE_M = 1e-4


def collision_metric(clearance, profile='legacy'):
    if profile=='x500_swept_sphere':
        # Independently audited continuous rotor bound <0.40 m. A sphere
        # contains the locked primitives under every vehicle attitude, with
        # at least 3 cm of reference-path margin. This is not a tracking proof.
        radius=max(float(clearance),.43)
        return np.full(3,clearance/radius)
    if profile!='legacy':raise ValueError('Unknown collision profile')
    # Even a reduced routing clearance must still contain the physical model.
    # 0.50 x 0.50 x 0.30 encloses all swept primitives at every body yaw.
    horizontal = max(float(clearance), .50)
    return np.array([clearance/horizontal, clearance/horizontal, clearance/.30])
