import math

from core.exploration.voxel_config import VoxelConfig
from core.planning.execution_safety import TRACKING_MARGIN
from tools.scenarios.depot07 import south_map_bound


def test_south_wall_cell_ends_at_launch_clearance():
    grid = VoxelConfig()
    start_y, floor_edge = -5.96, -7.14
    origin = south_map_bound(floor_edge, start_y, grid)
    assert floor_edge <= origin < floor_edge + grid.resolution

    def cell(y):
        return math.floor((y - origin) / grid.resolution)

    # Reprojected south-wall return from the failed launch at 52.128 s.
    wall = cell(-6.8396)
    assert cell(start_y) - wall > math.ceil(grid.clearance / grid.resolution)
    hover_edge = start_y - TRACKING_MARGIN - grid.clearance
    assert math.isclose(origin + (wall + 1) * grid.resolution, hover_edge)
    assert cell(hover_edge + .001) > wall
