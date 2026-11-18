"""Independent post-flight clearance to scene box collisions; never control input."""
import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def signed_box_distance(points, center, size):
    q = np.abs(points - center) - size / 2
    return np.linalg.norm(np.maximum(q, 0), axis=1) + np.minimum(q.max(axis=1), 0)


def audit(folder):
    rows = [json.loads(s) for s in (folder / 'truth-poses.jsonl').read_text().splitlines()]
    rot = Rotation.from_quat([r['quat'] for r in rows])
    points = np.array([r['xyz'] for r in rows]) + rot.apply(np.tile([0, 0, .24], (len(rows), 1)))
    # Exclude landed/takeoff ground regime; every recorded point above 0.7 m remains.
    points = points[points[:, 2] >= .7]
    boxes = []
    for model in ET.parse(folder / 'scene.sdf').findall('world/model'):
        name = model.get('name')
        if name != 'board' and not name.startswith('room_'):
            continue
        pose = np.fromstring(model.findtext('pose'), sep=' ')
        size = np.fromstring(model.findtext('link/collision/geometry/box/size'), sep=' ')
        if not (np.allclose(pose[3:], 0) and len(size) == 3):
            raise ValueError('Scene box requires axis-aligned pose and 3D size')
        clearance = float(signed_box_distance(points, pose[:3], size).min())
        boxes.append(dict(obstacle=name, base_center_min_m=clearance,
                          margin_beyond_043m_sphere_m=clearance - .43))
    return dict(run=folder.name, samples=len(points), boxes=boxes,
                scope='Independent recorded true base_link centers above 0.7 m; scene board and room boxes only. Distance minus 0.43 m is an analysis envelope, not full collision certification; excludes person mesh and floor, discrete samples only.')


if __name__ == '__main__':
    p = argparse.ArgumentParser();p.add_argument('runs', type=Path, nargs='+');p.add_argument('--output', type=Path, required=True)
    a = p.parse_args();a.output.write_text(json.dumps([audit(f) for f in a.runs], indent=2) + '\n')
