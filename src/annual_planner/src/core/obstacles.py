"""Occupancy-grid helpers adapted from Annual-Planner."""
from __future__ import annotations
from typing import Iterable
from dataclasses import dataclass
import numpy as np
from scipy.ndimage import maximum_filter


@dataclass
class OccupancyGrid:
    """三维占据栅格地图。

    用途
    ----
    将连续空间离散化为均匀体素，供路径规划算法查询。

    原理
    ----
    data 为 uint8 数组：0=自由空间, 1=占据, 2=膨胀区域。
    """

    origin: np.ndarray      # (3,) 世界坐标系原点
    resolution: float       # 体素边长 m
    shape: tuple[int, int, int]
    data: np.ndarray | None = None  # uint8 (nx, ny, nz)

    def __post_init__(self):
        self.origin = np.asarray(self.origin, dtype=float)
        self.resolution = float(self.resolution)
        if self.data is None:
            self.data = np.zeros(self.shape, dtype=np.uint8)

    def world_to_index(self, p: np.ndarray) -> tuple[int, int, int]:
        p = np.asarray(p, dtype=float)
        idx = np.floor((p - self.origin) / self.resolution + 1e-9).astype(int)
        return tuple(np.clip(idx, 0, np.array(self.shape) - 1))

    def contains_world(self, p: np.ndarray) -> bool:
        p = np.asarray(p, dtype=float)
        idx_float = (p - self.origin) / self.resolution
        upper = np.asarray(self.shape, dtype=float) - 1.0
        return bool(np.all(idx_float >= 0.0) and np.all(idx_float <= upper + 1e-9))

    def index_to_world(self, idx: tuple[int, int, int]) -> np.ndarray:
        return self.origin + np.array(idx, dtype=float) * self.resolution

    def is_occupied(self, idx: tuple[int, int, int]) -> bool:
        if any(i < 0 or i >= s for i, s in zip(idx, self.shape)):
            return True  # 越界视为占据
        return self.data[idx] >= 1

    def is_truly_occupied(self, idx: tuple[int, int, int]) -> bool:
        """仅判断体素是否被障碍物实体占据（值=1），不含膨胀层（值=2）。"""
        if any(i < 0 or i >= s for i, s in zip(idx, self.shape)):
            return True
        return self.data[idx] == 1

    def is_free(self, idx: tuple[int, int, int]) -> bool:
        return not self.is_occupied(idx)

    def _normalize_inflate_radius(
        self, radius: float | Iterable[float]
    ) -> tuple[tuple[int, int, int], tuple[float, float, float], bool]:
        """将膨胀半径统一为体素半径与世界坐标半径。"""
        if np.isscalar(radius):
            scalar = max(0.0, float(radius))
            r_voxels = max(1, int(np.ceil(scalar / self.resolution)))
            return (r_voxels, r_voxels, r_voxels), (scalar, scalar, scalar), False

        values = tuple(float(v) for v in radius)
        if len(values) != 3:
            raise ValueError("inflate radius must be a scalar or length-3 iterable")
        clamped = tuple(max(0.0, v) for v in values)
        voxels = tuple(int(np.ceil(v / self.resolution)) for v in clamped)
        return voxels, clamped, True

    def inflate(self, radius: float | Iterable[float]) -> OccupancyGrid:
        """膨胀占据区域。

        标量半径保留旧版立方体膨胀语义；三轴半径 `(rx, ry, rz)` 使用椭球结构元，
        以避免编队纵向长度错误挤占横向通道。
        """
        voxels, radii, anisotropic = self._normalize_inflate_radius(radius)
        result = OccupancyGrid(self.origin.copy(), self.resolution, self.shape, self.data.copy())
        occupied = self.data >= 1
        if min(voxels) < 0 or not occupied.any() or occupied.all():
            return result
        # Only displacements between two cells in this map can contribute.
        voxels = tuple(min(r, length-1) for r, length in zip(voxels, self.shape))
        if not anisotropic:
            size = tuple(2*r+1 for r in voxels)
            expanded = maximum_filter(occupied, size=size, mode='constant')
            result.data[expanded & (self.data == 0)] = 2
            return result

        squared_offsets = [np.square(np.arange(-r, r+1, dtype=float)*self.resolution/limit)
                           if limit else np.zeros(1) for r, limit in zip(voxels, radii)]
        for center in np.argwhere(occupied):
            bounds = [(max(0, c-r), min(n, c+r+1)) for c, r, n in zip(center, voxels, self.shape)]
            block = result.data[tuple(slice(lo, hi) for lo, hi in bounds)]
            dx, dy, dz = (axis[lo-c+r:hi-c+r] for axis, (lo, hi), c, r in
                          zip(squared_offsets, bounds, center, voxels))
            mask = dx[:, None, None]+dy[None, :, None]+dz[None, None, :] <= 1.+1e-9
            block[(block == 0) & mask] = 2
        return result
