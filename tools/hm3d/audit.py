"""Independent post-run mesh/depth audit. Never loaded by flight or decision code."""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from tools.hm3d.prepare import distances


def audit(scene, capture):
    import trimesh
    scene, capture = Path(scene), Path(capture)
    mesh = trimesh.load(scene/'assets/collision.stl', process=False)
    metadata = json.loads((scene/'scene-audit.json').read_text())
    mesh.apply_translation(metadata['translation'])
    records = {}
    for directory in sorted(capture.iterdir()):
        if not (directory/'frame.json').is_file(): continue
        f = json.loads((directory/'frame.json').read_text())
        depth = np.load(directory/'depth.npy'); k = np.array(f['depth']['k']).reshape(3, 3)
        pose = np.array(f['switch']['pose']); base_r = Rotation.from_euler('xyz', pose[3:])
        ext = f['depth_to_base']; r = base_r*Rotation.from_quat(ext['xyzw'])
        lens = pose[:3]+base_r.apply(np.array(ext['xyz'])+[0, 0, .24])
        u, v = np.meshgrid(np.arange(4, depth.shape[1]-4, 8), np.arange(4, depth.shape[0]-4, 8))
        z = depth[v, u].ravel(); local = np.c_[(u.ravel()-k[0, 2])/k[0, 0], (v.ravel()-k[1, 2])/k[1, 1], np.ones(u.size)]
        lengths = np.linalg.norm(local, axis=1); rays = r.apply(local/lengths[:, None])
        expected = distances(mesh, np.tile(lens, (len(rays), 1)), rays)/lengths
        valid = np.isfinite(z) & np.isfinite(expected) & (z > .1) & (expected > .1) & (expected < 10)
        errors = np.abs(z[valid]-expected[valid]); med = float(np.median(errors)) if len(errors) else None
        fraction = float(np.mean(errors < .2)) if len(errors) else 0
        records[directory.name] = dict(n=int(valid.sum()), median_abs_axial_m=med,
            p95_abs_axial_m=float(np.quantile(errors, .95)) if len(errors) else None,
            fraction_within_20cm=fraction, pass_alignment=bool(len(errors)>100 and med<=.05 and fraction>=.95), lens_map=lens.tolist())
        print(json.dumps({directory.name: records[directory.name]}), flush=True)
    result = dict(scope='Static known-pose sensor projection only, not flight localization', views=records,
                  passed=bool(len(records)==4 and all(v['pass_alignment'] for v in records.values())))
    (capture/'mesh-depth-audit.json').write_text(json.dumps(result, indent=2)); print(json.dumps(result))
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('scene');p.add_argument('capture');a=p.parse_args();audit(a.scene,a.capture)
