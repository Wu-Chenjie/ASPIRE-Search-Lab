"""Geometry of the locked X500 / OakD-Lite depth sensor (body FLU).

Predicted views share the actual depth mapper's mount. The conservative pitch
extent stays inside the rectangular image even at its horizontal edges.
"""
import math

# The camera include is in x500_depth's model frame. The merged x500_base
# model places base_link at z=.24 in that frame. Joint poses are anchors, not
# child-link transforms: subtract the base frame before adding sensor pose.
# (.12,.03,.242) + (.01233,-.03,.01878) - (0,0,.24).
DEPTH_TRANSLATION = (.13233, 0., .02078)
DEPTH_HORIZONTAL_FOV = 1.274
DEPTH_ASPECT_RATIO = 240. / 320.
DEPTH_PITCH_LIMIT = math.atan(DEPTH_ASPECT_RATIO * math.sin(DEPTH_HORIZONTAL_FOV / 2.))
