"""Measure thin-bar occupancy against a wall-only control on synthetic depth frames."""
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT / 'src'), str(ROOT / 'src/annual_planner/src')]

import numpy as np
from aspire_camera.depth_projection import project_depth, SENSOR_TRANSLATION
from core.exploration.voxel_mapping import VoxelMap

WALL_DEPTH_M = 4.4  # Behind both bars and within the central 4.5 m ray range.


def focal(width):
    return width * 0.866  # Same horizontal field of view at both resolutions.


def scene(width, height, bar_px, phase, distance):
    depth = np.full((height, width), WALL_DEPTH_M, np.float32)
    bar = np.zeros_like(depth, dtype=bool)
    left = width // 2 + phase
    bar[:, left:left + bar_px] = True
    depth[bar] = distance
    k = np.array([[focal(width), 0., width / 2 - .5],
                  [0., focal(width), height / 2 - .5], [0., 0., 1.]])
    return depth, k, bar


def run(width, height, stride, bar_px, phase, distance, resolution=0.3):
    depth, k, bar = scene(width, height, bar_px, phase, distance)
    points, hits = project_depth(depth, k, stride=stride)
    voxel = VoxelMap(np.array([[-4., -6., -1.], [8., 6., 4.5]]), resolution=resolution)
    origin = np.asarray(SENSOR_TRANSLATION)
    world = points + origin
    started = time.perf_counter()
    cells, values = voxel.integrate(origin, world, hits)
    elapsed = time.perf_counter() - started

    # In this fronto-parallel scene the bar and wall occupy distinct x layers.
    occupied = cells[values == 1]
    bar_layer = voxel.indices(origin + [distance, 0., 0.])[0]
    bar_pixels = bar[::stride, ::stride].ravel()
    wall_depth = points[hits & ~bar_pixels, 0]
    return dict(width=width, height=height, stride=stride, bar_px=bar_px, phase=phase,
                distance=distance, wall_depth_m=WALL_DEPTH_M, samples=len(points),
                obstacle_samples=int(np.count_nonzero(hits & bar_pixels)),
                obstacle_cells=int(np.count_nonzero(occupied[:, 0] == bar_layer)),
                occupied_cells=len(occupied), wall_hits=len(wall_depth),
                wall_range_p95_error_m=float(np.quantile(np.abs(wall_depth - WALL_DEPTH_M), .95)),
                integrate_s=elapsed)


if __name__ == '__main__':
    rows = []
    for width, height in ((320, 240), (640, 480)):
        for distance in (1.5, 4.):
            for stride in (8, 4, 2):
                cases = [run(width, height, stride, bar_px, 1, distance)
                         for bar_px in (0, 1, 2, 3, 4, 6, 8, 12, 16)]
                rows.extend(cases)
                control, *bars = cases
                assert control['obstacle_cells'] == 0
                thinnest = min(row['bar_px'] for row in bars if row['obstacle_cells'] > 0)
                width_mm = thinnest * distance / focal(width) * 1000
                median_ms = np.median([row['integrate_s'] for row in bars]) * 1000
                print(f'{width}x{height}  d={distance} m  stride={stride}  '
                      f'thinnest={width_mm:.1f} mm  integrate_median={median_ms:.1f} ms')
    Path(__file__).with_name('stride_sweep.json').write_text(json.dumps(rows, indent=2) + '\n')
