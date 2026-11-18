from pathlib import Path
from tools.racer_ros2.configuration import native_launch_nodes

ROOT = Path(__file__).resolve().parents[2]

def configuration(output,k):
    nodes = list(native_launch_nodes(output, [[-3,-4,0],[7,4,4]], k,
        dict(speed=.6, acceleration=.8, jerk=2., yaw_rate=.65),
        ROOT/'external/racer_ros2_ws/src/racer_core/config/single_drone_planner.xml',
        1, voxel_resolution=.15, body_clearance=.35))
    for node in nodes:
        if node['executable'] == 'exploration_node':
            node['executable'] = 'search_racer'
            node['parameters'].update({'sdf_map.box_min_z': .5, 'sdf_map.box_max_z': 3.2})
    return nodes
