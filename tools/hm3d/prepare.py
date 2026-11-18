"""HM3D simulator assets and offline placement; never imported by decision nodes."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from tools.prepare_sim import prepare, model, landing_support
from tools.rgbd_scene.cameras import set_text

LOCKS = {
    # Official minival v0.2, completed archive bfe2300d...e07af2d6; local integrity locks.
    'TEEsavR23oF': '355befa3bc2802069587e26de25903dda96d9ce4f872274e5a33dfda217c5e83',
    'HaxA7YrQdEC': 'daa2fc89b5f6623842e606612e1c5f4958178c4a487d3fe6930193ef5d03ea7c',
    'wcojb4TFT35': 'fd7ebbfc7de4bd682221d4c434d1fc2a889d42cbe72da6b7d791c62bc08731af',
    'k1cupFYWXJ6': '1b71abdeb355392bffb8cc10eeba587ad83b46a5d6b5d332cc2dd760f0119663',
    'BHXhpBwSMLh': '1a6eb39aecd7dca5f6a75472c9d422178d99b34ec54f3f669dd6c67502aefdcf',
    'SUHsP6z2gcJ': 'dc384fed0a4c0d6f7e6a0fdb67b023ecd26d98c9c27d06481f22506746442957',
    'tQ5s4ShP627': '3f4e8baa7eb426c1eccedd5c226d17092972d1020e777a85d578a67d2814461a',
    'rsggHU7g7dh': 'e332fec2f8d6af1521064807154e5fdf1a02adbcc952626e7a9ee25abfdb80c5',
    'y9hTuugGdiq': '75953891cc8644441dca2271f1c10038d67a9940e2b44b430e57de1eacf66840',
    'Qpor2mEya8F': '849600006b406015bef3beece685ee0032fe5d8048713c7f45c90a614c7139ae',
    'GLAQ4DNUx5U': 'c3bb83e0145eb135c6bb7f6e5b19135ee6f2cc9ced065c1bb8afc6fb88466a7a',
    'CFVBbU9Rsyb': 'f394a073f873254c1eec5b77ce06724f96259acf3e97da6c1314a24563c10671',
    'NBg5UqG3di3': '9c78e2f3f4e993baaaf3dd76a9ac123ef6e3d26082e7f7e80bee5639bad3430d',
}

def distances(mesh, origins, directions):
    origins, directions = np.asarray(origins), np.asarray(directions)
    result = np.full(len(origins), np.inf)
    for start in range(0, len(origins), 8):
        o, d = origins[start:start+8], directions[start:start+8]
        locations, ray, _ = mesh.ray.intersects_location(o, d, multiple_hits=False)
        if len(ray):result[start+ray] = np.linalg.norm(locations-o[ray], axis=1)
    return result


def path_clearance(distance_query, a, b, step=.025):
    """A distance-to-surface query is 1-Lipschitz; certify the entire segment."""
    a,b=np.asarray(a,float),np.asarray(b,float)
    length=float(np.linalg.norm(b-a));intervals=max(1,math.ceil(length/step))
    points=np.linspace(a,b,intervals+1);d=np.asarray(distance_query(points),float)
    if d.shape!=(len(points),) or not np.isfinite(d).all():
        return dict(verified=False,reason='invalid_distance_query')
    lower=float(d.min()-length/(2*intervals))
    return dict(verified=bool(lower>=.6),lower_bound_m=lower,sampled_min_m=float(d.min()),samples=len(points))


def placement(mesh, source_start=None):
    centers, areas = mesh.triangles_center, mesh.area_faces
    mask = (mesh.face_normals[:, 2] > .95) & (areas > 1e-6)
    levels = np.round(centers[mask, 2]/.05).astype(int)
    ranked = sorted(np.unique(levels), key=lambda z: (-areas[mask][levels == z].sum(), z))[:8]
    lo, hi = mesh.bounds
    if source_start is not None:
        source_start=np.asarray(source_start,float)
        if source_start.shape!=(3,) or not np.isfinite(source_start).all():raise ValueError('Expected finite source-frame initial xyz')
        ranked=[(source_start[2]-1.5)/.05]
    xy = np.array([(x, y) for x in np.arange(lo[0]+1, hi[0]-1, .9)
                   for y in np.arange(lo[1]+1, hi[1]-1, .9)])
    if source_start is not None:xy=source_start[None,:2].copy()
    angles = np.arange(16)*2*np.pi/16
    directions = np.c_[np.cos(angles), np.sin(angles), np.zeros(16)]
    attempts = []
    for level in ranked:
        floor = float(level*.05)
        origins = np.c_[xy, np.full(len(xy), floor+1)]
        down = distances(mesh, origins, np.tile([0, 0, -1], (len(xy), 1)))
        candidates = origins[np.abs(down-1) < .04]
        passed = []
        for center in candidates:
            center[2] = floor+1.5
            up = distances(mesh, [center], [[0, 0, 1]])[0]
            if not .8 < up < 4: continue
            hd = np.concatenate([distances(mesh, np.tile(center+[0, 0, h], (16, 1)), directions)
                                 for h in (-1., -.5, 0., .3)])
            if not np.isfinite(hd).all() or min(hd) < 1.1: continue
            ring = np.c_[center[0]+.5*np.cos(angles), center[1]+.5*np.sin(angles), np.full(16, floor+.5)]
            fd = distances(mesh, ring, np.tile([0, 0, -1], (16, 1)))
            if not np.all(np.abs(fd-.5) < .04): continue
            passed.append(dict(position=center.tolist(), horizontal_clearance=float(min(hd)), ceiling_clearance=float(up)))
        attempts.append(dict(floor=floor, candidates=len(candidates), passed=len(passed)))
        print(json.dumps(attempts[-1]), flush=True)
        if passed:
            passed.sort(key=lambda p: (-p['horizontal_clearance'], p['position']))
            import trimesh
            for p in passed:
                x,y,_=p['position']
                check=path_clearance(lambda points:trimesh.proximity.closest_point(mesh,points)[1],
                                     [x,y,floor+.7],[x,y,floor+1.8])
                p['bootstrap_clearance']=check
                if check['verified']:return floor,p,attempts
            attempts[-1]['bootstrap_tube_rejected']=len(passed)
            print(json.dumps(attempts[-1]),flush=True)
    raise ValueError('No enclosed spawn with unchanged aircraft clearance: '+repr(attempts))


def convert(source, out, models, static=False, source_start=None):
    import trimesh
    source, out = source.resolve(), out.resolve()
    if ROOT == out or ROOT in out.parents: raise ValueError('Assets/runs must be outside code tree')
    if hashlib.sha256(source.read_bytes()).hexdigest() != LOCKS[source.stem]: raise ValueError('Source hash mismatch')
    out.mkdir(parents=True, exist_ok=False)
    mesh = trimesh.load_scene(source, process=False, skip_materials=True).to_mesh()
    floor, spawn, attempts = placement(mesh,source_start)
    lo, hi = mesh.bounds
    offset = np.array([-lo[0]+.6, -lo[1]+.6, -floor])
    start = np.array(spawn['position'])+offset
    # Keep raw visual asset byte-for-byte. Both visual and collision receive the same rigid translation.
    assets = out/'assets'; assets.mkdir()
    shutil.copyfile(source, assets/'scene.glb')
    mesh.export(assets/'collision.stl')
    probes = landing_support(models)
    origins = probes.copy(); origins[:, :2] += np.array(spawn['position'])[:2]; origins[:, 2] = floor+1.5
    ground = origins[:, 2]-distances(mesh, origins, np.tile([0, 0, -1], (len(origins), 1)))
    if not np.isfinite(ground).all(): raise ValueError('Missing skid support')
    initial = [*start[:2], float(np.max(ground-probes[:, 2])-floor)]
    upper = min(3., float(spawn['ceiling_clearance']+1.5-.7))
    definition = dict(bounds=[[0, 0, 0], [float(hi[0]-lo[0]+1.2), float(hi[1]-lo[1]+1.2), max(3., upper+.7)]],
                      search_starts=[start.tolist()], initial_positions=[initial], obstacles=[])
    (out/'placement.json').write_text(json.dumps(definition, indent=2))
    prepare(out/'placement.json', out, models=models, motion_profile='legacy')
    tree = ET.parse(out/'scene.sdf'); world = tree.find('world')
    world.remove(world.find('model[@name="ground"]'))
    model(world, 'hm3d_scan', offset, ('mesh', {'uri': (assets/'collision.stl').as_uri()}))
    world.find('model[@name="hm3d_scan"]/link/visual/geometry/mesh/uri').text = (assets/'scene.glb').as_uri()
    include = world.find('include[name="annual_x500_0"]')
    include.find('pose').text = ' '.join(map(str, [*start[:2], initial[2]-.24+.02, 0, 0, 0]))
    ET.SubElement(world, 'plugin', filename='gz-sim-contact-system', name='gz::sim::systems::Contact')
    basepath = out/'models/annual_base_0/model.sdf'; base = ET.parse(basepath)
    link = base.find('model/link[@name="base_link"]')
    sensor = ET.SubElement(link, 'sensor', name='evaluation_contact', type='contact')
    for key, value in [('always_on', 'true'), ('update_rate', '20'), ('topic', '/evaluation/contacts')]: set_text(sensor, key, value)
    contact = ET.SubElement(sensor, 'contact')
    for collision in link.findall('collision'): ET.SubElement(contact, 'collision').text = collision.get('name')
    base.write(basepath)
    cfg = json.loads((out/'runtime.json').read_text())
    cfg['vehicles'][0]['spawn'] = [*start[:2], initial[2]+.02]
    cfg.update(research_planner='racer_ros2', native_frame_source='mapper_observation', racer_planning_scale=.5,
               racer_flight_limits=[.7, upper], bootstrap_altitudes=[1.2, min(1.8, upper), 1.5],
               control_timing_diagnostics=True, performance_profile='consistent', layout=source.stem,
               motion_limits=dict(speed=1., acceleration=1.2, jerk=4., yaw_rate=.9),
               racer_planning_limits=dict(speed=.3, acceleration=.4, jerk=4., yaw_rate=.325), racer_min_time=False, recover_frontiers=True)
    if static:
        # A falling sphere verifies physical floor contact independently of the static cameras.
        probe=ET.SubElement(world,'model',name='hm3d_collision_probe')
        set_text(probe,'pose',' '.join(map(str,[*start[:2],.8,0,0,0])))
        link=ET.SubElement(probe,'link',name='body')
        set_text(link,'inertial/mass',.1)
        for key in ('ixx','iyy','izz'):set_text(link,'inertial/inertia/'+key,.0004)
        collision=ET.SubElement(link,'collision',name='sphere');set_text(collision,'geometry/sphere/radius',.1)
        sensor=ET.SubElement(link,'sensor',name='contact',type='contact')
        set_text(sensor,'always_on','true');set_text(sensor,'update_rate',20)
        set_text(sensor,'topic','/evaluation/hm3d_probe');set_text(sensor,'contact/collision','sphere')
        path = out/'models/annual_x500_0/model.sdf'; vehicle = ET.parse(path)
        set_text(vehicle.find('model'), 'static', 'true'); vehicle.write(path)
        views = {}
        for name, yaw in [('visible', 0.), ('partial', math.pi/2), ('hidden', math.pi), ('revealed', -math.pi/2)]:
            lens = start.copy(); lens[2] += .02078
            views[name] = dict(lens_xyz=lens.tolist(), lens_yaw=yaw,
                               model_pose=[lens[0]-.13233*math.cos(yaw), lens[1]-.13233*math.sin(yaw), lens[2]-.26078, 0, 0, yaw])
        include.find('pose').text = ' '.join(map(str, views['visible']['model_pose']))
        cfg.update(manual_camera_test=True, additional_model_paths=[])
        (out/'viewpoints.json').write_text(json.dumps(dict(world='annual', model='annual_x500_0', views=views,
            scope='Manual four headings for sensor validation; names retain runner compatibility, no person present')))
    tree.write(out/'scene.sdf')
    (out/'runtime.json').write_text(json.dumps(cfg, indent=2))
    report = dict(scene=source.stem, source_sha256=LOCKS[source.stem], translation=offset.tolist(), source_up=[0, 0, 1],
                  floor_source_z=floor, spawn=spawn, placement_attempts=attempts, faces=len(mesh.faces),
                  visual='raw textured GLB unchanged', collision='all GLB triangles exported to STL, no decimation or hole repair',
                  assets_sha256={p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in assets.iterdir()},
                  license='https://matterport.com/legal/matterport-end-user-license-agreement-academic-use-model-data',
                  attribution='HM3D: Habitat-Matterport 3D Research Dataset, Matterport',
                  use='scene generation and independent evaluation only; no source geometry in runtime.json',
                  static=static, flight_validated=False, requested_source_start=source_start)
    (out/'scene-audit.json').write_text(json.dumps(report, indent=2))
    (out/'evaluation-truth.json').write_text(json.dumps(dict(person=None, board=None, use='independent evaluation only')))
    return cfg

if __name__ == '__main__':
    p = argparse.ArgumentParser(); p.add_argument('--source', type=Path, required=True); p.add_argument('--output', type=Path, required=True)
    p.add_argument('--models', type=Path, default=ROOT/'external/PX4-gazebo-models/models'); p.add_argument('--static', action='store_true')
    p.add_argument('--source-start',nargs=3,type=float,help='Offline initial placement in source coordinates; same full clearance checks apply, never a route')
    a = p.parse_args(); convert(a.source, a.output, a.models, a.static,a.source_start)
