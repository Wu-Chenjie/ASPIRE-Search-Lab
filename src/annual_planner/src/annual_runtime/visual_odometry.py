"""Fail-closed checks for measured external odometry."""


def depth_fusion_flags_ready(flags, fuse_height=True):
    """Require the enabled PX4 EV sources; barometer height need not fuse EV z."""
    return bool(flags and flags.cs_ev_pos and flags.cs_ev_yaw and
                (not fuse_height or flags.cs_ev_hgt))
