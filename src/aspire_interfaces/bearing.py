"""Vision to RACER angular observation wire format."""


def bearing_packet(report):
    """Version1, fixed map frame; numeric layout is shared with native header."""
    return [1.,float(report['source_uav']),float(report['cue_id']),report['capture_time_s'],*report['origin'],*report['direction'],
        report['half_angle_rad'],report['confidence'],float(report['support_captures']),float(report['observation_coverage']),
        float(report['localization_state']=='localized_surface'),report['support_first_s'],report['camera_half_fov_rad']]
