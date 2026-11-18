import numpy as np

from core.exploration.voxel_mapping import VoxelMap
from core.obstacles import OccupancyGrid


def test_inflate_keeps_box_ellipsoid_and_label_semantics():
    data = np.zeros((4, 5, 3), dtype=np.uint8)
    data[0, 0, 1], data[3, 4, 2] = 1, 255
    original = data.copy()
    grid = OccupancyGrid([0, 0, 0], 1., data.shape, data)

    box = grid.inflate(0.)
    assert box.data[1, 1, 0] == 2
    assert box.data[2, 1, 0] == 0

    ellipsoid = grid.inflate((2., 1., 0.))
    assert ellipsoid.data[2, 0, 1] == 2
    assert ellipsoid.data[1, 1, 1] == 0
    assert ellipsoid.data[0, 0, 2] == 0
    assert ellipsoid.data[0, 0, 1] == 1
    assert ellipsoid.data[3, 4, 2] == 255

    np.testing.assert_array_equal(grid.inflate((1e10, 1e10, 1e10)).data,
                                  np.where(original == 0, 2, original))
    np.testing.assert_array_equal(data, original)


def test_navigation_grid_is_lazy_and_preserves_reservations():
    voxels = VoxelMap([[0., 0., 0.], [5., 5., 5.]], resolution=.5)
    voxels.state[:] = 0
    voxels.rebuild()
    center = (4, 4, 4)
    assert voxels.safe[center]
    assert '_grid' not in vars(voxels)

    original = voxels.grid
    assert voxels.grid is original
    assert original.data[center] == 0
    voxels.block_paths([[voxels.points(center)]])
    assert not voxels.safe[center]
    assert '_grid' not in vars(voxels)
    assert voxels.grid is not original
    assert voxels.grid.data[center] == 1
    assert original.data[center] == 0
