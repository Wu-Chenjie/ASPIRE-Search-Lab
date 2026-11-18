"""Exact required helper subset from original Annual-Planner; no alternate planner."""
import math
import numpy as np
from .sensor_geometry import DEPTH_TRANSLATION, DEPTH_HORIZONTAL_FOV, DEPTH_PITCH_LIMIT


def visible_cells(runtime, position, yaw, fov=None, radius=4.5, *, azimuth_samples=None, pitch_samples=13):
    """Predicted view: count the first unknown surface on each ray; do not see through it.

    This function never receives simulator truth. Its predicted gain is corrected
    by the next actual observation, including previously unknown occluders.
    """
    count = azimuth_samples if azimuth_samples is not None else (61 if runtime.state.ndim == 3 else 121)
    if fov is None: fov = DEPTH_HORIZONTAL_FOV if runtime.state.ndim == 3 else 2.
    angles = np.linspace(yaw-fov/2, yaw+fov/2, count)
    ranges = np.arange(0., radius+runtime.resolution/2, runtime.resolution/2)
    if runtime.state.ndim == 3:
        x, y, z = DEPTH_TRANSLATION
        position = np.asarray(position)+np.array([
            x*math.cos(yaw)-y*math.sin(yaw),
            x*math.sin(yaw)+y*math.cos(yaw), z])
        headings, pitch = np.meshgrid(angles, np.linspace(-DEPTH_PITCH_LIMIT, DEPTH_PITCH_LIMIT, pitch_samples))
        directions = np.column_stack([np.cos(pitch.ravel())*np.cos(headings.ravel()),
                                      np.cos(pitch.ravel())*np.sin(headings.ravel()), np.sin(pitch.ravel())])
        coordinates = np.asarray(position)+directions[:, None, :]*ranges[None, :, None]
    else:
        coordinates = np.asarray(position)[:2]+np.stack([np.cos(angles), np.sin(angles)], axis=1)[:, None, :]*ranges[None, :, None]
    cells = np.floor((coordinates-runtime.origin)/runtime.resolution).astype(int)
    valid = np.all((cells >= 0) & (cells < runtime.shape), axis=2)
    clipped = np.clip(cells, 0, np.array(runtime.shape)-1)
    values = runtime.state[tuple(np.moveaxis(clipped, -1, 0))]
    hit = (values != 0) | ~valid
    # Only the first blocking sample can be an unknown surface. Previously we
    # collected every preceding free sample, sorted them, then discarded them.
    # No-hit rays contribute nothing; an out-of-map sample stops the ray.
    first = np.argmax(hit, axis=1)
    rays = np.arange(len(first))
    keep = hit[rays,first] & valid[rays,first] & (values[rays,first] == -1)
    cells = clipped[rays[keep],first[keep]]
    ids = np.unique(np.ravel_multi_index(tuple(cells.T), runtime.shape))
    return frozenset(map(int, ids))
