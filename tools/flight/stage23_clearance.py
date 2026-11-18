#!/usr/bin/env python3
"""Offline 0.43m sphere surface bounds from actual poses; never control input.

python3 tools/flight/stage23_clearance.py RUN --output /tmp/clearance.json
"""
import os
for _key in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[_key] = '1'
import argparse
import hashlib
import json
import gc
import resource
import sys
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
if __package__:
    from .clearance_audit import signed_box_distance
else:
    from clearance_audit import signed_box_distance

RADIUS = .43


def read_rows(path):
    if not path.is_file(): return []
    with path.open() as stream:
        return [json.loads(line) for line in stream]


def transform(element):
    p = element.find('pose')
    if p is not None and (p.get('relative_to') or p.get('rotation_format', 'euler_rpy') != 'euler_rpy'):
        raise ValueError('named-frame / non-RPY pose unsupported')
    values = np.fromstring(p.text or '', sep=' ') if p is not None else np.zeros(6)
    if values.shape != (6,) or not np.isfinite(values).all(): raise ValueError('invalid pose')
    matrix = np.eye(4)
    matrix[:3,:3] = Rotation.from_euler('xyz', values[3:], degrees=p is not None and p.get('degrees') == 'true').as_matrix()
    matrix[:3,3] = values[:3]
    return matrix


class Scene:
    def __init__(self, run, vehicle_names, mesh_exact_radius=.8):
        self.vehicle_names = vehicle_names
        self.geometries = []; self.missing = []; self.excluded = []
        self.exact_radius = mesh_exact_radius
        self.paths = [run/'models']
        config = json.loads((run/'runtime.json').read_text()) if (run/'runtime.json').is_file() else {}
        if config.get('models'): self.paths.append(Path(config['models']))
        self.paths.extend(Path(x) for x in config.get('additional_model_paths', []))
        self.paths.extend(Path(x) for x in os.environ.get('GZ_SIM_RESOURCE_PATH','').split(os.pathsep) if x)
        scene_bytes = (run/'scene.sdf').read_bytes()
        self.sha256 = hashlib.sha256(scene_bytes).hexdigest()
        root = ET.fromstring(scene_bytes).find('world')
        if root is None: raise ValueError('scene has no world')
        self.walk(root, np.eye(4), run, '', False, set())

    def resolve(self, uri, base):
        if uri.startswith('file://'):
            path = Path(uri[7:]); return path if path.is_absolute() else base/path
        if uri.startswith('model://'):
            for root in self.paths:
                if (root/uri[8:]).exists(): return root/uri[8:]
            raise ValueError(f'unresolved model URI: {uri}')
        if '://' in uri: raise ValueError(f'nonlocal URI not fetched: {uri}')
        return base/uri

    def walk(self, parent, matrix, base, prefix, inherited_static, chain):
        for item in parent:
            if item.tag not in ('model','include'): continue
            name = item.get('name') if item.tag == 'model' else item.findtext('name')
            name = name or item.findtext('uri','').rstrip('/').split('/')[-1]
            label = prefix+'/'+name
            if name in self.vehicle_names:
                self.excluded.append(dict(model=label,reason='recorded moving vehicle, not a static obstacle')); continue
            try:
                placement = matrix@transform(item)
                if item.tag == 'include':
                    if item.find('placement_frame') is not None: raise ValueError('include placement_frame unsupported')
                    path = self.resolve(item.findtext('uri',''),base)
                    if path.is_dir(): path = path/'model.sdf'
                    if path.resolve() in chain: raise ValueError('recursive model include')
                    model = ET.parse(path).getroot().find('model')
                    if model is None: raise ValueError('included SDF has no model')
                    if item.find('pose') is None: placement = matrix@transform(model)
                    static_text = item.findtext('static',model.findtext('static','true' if inherited_static else 'false'))
                    self.model(model,placement,path.parent,label,static_text=='true',chain|{path.resolve()})
                else:
                    self.model(item,placement,base,label,item.findtext('static','true' if inherited_static else 'false')=='true',chain)
            except (ValueError, OSError, ET.ParseError) as error:
                self.missing.append(dict(model=label,reason=f'{type(error).__name__}: {error}'))

    def model(self, model, matrix, base, label, static, chain):
        if not static:
            self.excluded.append(dict(model=label,reason='nonstatic model; time-varying geometry not evaluated')); return
        for link in model.findall('link'):
            lname = label+'/'+link.get('name','link')
            try: lm = matrix@transform(link)
            except ValueError as error:
                self.missing.append(dict(model=lname,reason=str(error)));continue
            for collision in link.findall('collision'):
                cname = lname+'/'+collision.get('name','collision')
                try:
                    geometry = collision.find('geometry')
                    if geometry is None or len(geometry) != 1: raise ValueError('missing or ambiguous geometry')
                    self.add(cname,geometry[0],lm@transform(collision),base)
                except (ValueError, OSError, ET.ParseError) as error:
                    self.missing.append(dict(model=cname,reason=f'{type(error).__name__}: {error}'))
        self.walk(model,matrix,base,label,True,chain)

    def add(self, name, shape, matrix, base):
        g = dict(name=name,kind=shape.tag,matrix=matrix)
        if shape.tag == 'mesh':
            if shape.find('submesh') is not None: raise ValueError('mesh submesh selection unsupported')
            path = self.resolve(shape.findtext('uri',''),base)
            scale = np.fromstring(shape.findtext('scale','1 1 1'),sep=' ')
            if scale.shape != (3,) or not np.isfinite(scale).all() or np.any(scale<=0): raise ValueError('invalid mesh scale')
            digest=hashlib.sha256()
            with path.open('rb') as stream:
                for block in iter(lambda:stream.read(1024*1024),b''): digest.update(block)
            # Do not hold every large scene mesh in memory at once.
            g.update(source=str(path),sha256=digest.hexdigest(),scale=scale)
        elif shape.tag == 'box':
            g['size'] = np.fromstring(shape.findtext('size',''),sep=' ')
            if g['size'].shape != (3,) or not np.isfinite(g['size']).all() or np.any(g['size']<=0):
                raise ValueError('invalid box')
        elif shape.tag in ('sphere','cylinder','capsule'):
            g['radius'] = float(shape.findtext('radius','nan'))
            if not np.isfinite(g['radius']) or g['radius']<=0: raise ValueError('invalid radius')
            if shape.tag != 'sphere':
                g['length'] = float(shape.findtext('length','nan'))
                if not np.isfinite(g['length']) or g['length']<=0: raise ValueError('invalid length')
        elif shape.tag == 'plane':
            normal = np.fromstring(shape.findtext('normal','0 0 1'),sep=' ');norm=np.linalg.norm(normal)
            if normal.shape != (3,) or not np.isfinite(norm) or norm<1e-9:
                raise ValueError('invalid plane normal')
            g['normal'] = normal/norm
        else: raise ValueError(f'unsupported collision geometry: {shape.tag}')
        self.geometries.append(g)

    def distance(self, g, points, batch):
        if g['kind']=='mesh':
            import trimesh
            mesh=trimesh.load(g['source'],process=False)
            if isinstance(mesh,trimesh.Scene): mesh=mesh.dump(concatenate=True)
            if not isinstance(mesh,trimesh.Trimesh) or not len(mesh.faces): raise ValueError('mesh has no triangles')
            mesh.vertices*=g['scale'];mesh.apply_transform(g['matrix'])
            g.update(faces=len(mesh.faces),bounds=mesh.bounds.tolist())
            lo,hi = np.asarray(g['bounds'])
            distance = signed_box_distance(points,(lo+hi)/2,hi-lo)
            exact = np.flatnonzero(distance < self.exact_radius)
            for start in range(0,len(exact),batch):
                ids = exact[start:start+batch]
                _,dist,_ = trimesh.proximity.closest_point(mesh,points[ids]); distance[ids] = dist
            g['triangle_surface_queries'] = int(len(exact));g['bbox_lower_bound_queries'] = int(len(points)-len(exact))
            return distance
        local = (points-g['matrix'][:3,3])@g['matrix'][:3,:3]
        if g['kind']=='box': return signed_box_distance(local,0.,g['size'])
        if g['kind']=='sphere': return np.linalg.norm(local,axis=1)-g['radius']
        if g['kind']=='plane': return np.abs(local@g['normal'])
        if g['kind']=='capsule':
            q=local.copy();q[:,2]-=np.clip(q[:,2],-g['length']/2,g['length']/2)
            return np.linalg.norm(q,axis=1)-g['radius']
        q=np.column_stack([np.linalg.norm(local[:,:2],axis=1)-g['radius'],np.abs(local[:,2])-g['length']/2])
        return np.linalg.norm(np.maximum(q,0),axis=1)+np.minimum(q.max(axis=1),0)


def summarize(t, points, clearance, mask, maximum_gap):
    ids=np.flatnonzero(mask)
    if not len(ids): return None
    if clearance is None: return dict(samples=int(len(ids)),minimum_sampled_surface_bound_m=None)
    selected=clearance[ids];at=ids[np.argmin(selected)]
    dt=np.diff(t);step=np.linalg.norm(np.diff(points,axis=0),axis=1)
    pair=mask[:-1]&mask[1:]&(dt>0)&(dt<=maximum_gap)
    # Every point of the straight chord is <= length/2 from an endpoint.
    lower=np.minimum(clearance[:-1],clearance[1:])-step/2
    pi=np.flatnonzero(pair);worst=pi[np.argmin(lower[pi])] if len(pi) else None
    return dict(samples=int(len(ids)),interval_sim_s=[float(t[ids[0]]),float(t[ids[-1]])],
        minimum_sampled_surface_bound_m=float(clearance[at]),minimum_sample_sim_s=float(t[at]),minimum_sample_center=points[at].tolist(),
        nonpositive_sample_count=int(np.sum(selected<=0)),positive_sample_count=int(np.sum(selected>0)),
        chord_intervals=int(pair.sum()),minimum_chord_lower_bound_m=float(lower[worst]) if worst is not None else None,
        minimum_chord_interval_sim_s=[float(t[worst]),float(t[worst+1])] if worst is not None else None,
        maximum_accepted_chord_m=float(step[pair].max()) if pair.any() else None,
        maximum_recorded_interval_s=float(dt[mask[:-1]&mask[1:]].max()) if np.any(mask[:-1]&mask[1:]) else None,
        excluded_long_time_gaps=int(np.sum(mask[:-1]&mask[1:]&(dt>maximum_gap))),
        interpretation='Nonpositive bound is conservative uncertainty, not physical collision proof. Chord bound is not an observed inter-frame curve sweep.')


def audit(run, batch=32, maximum_gap=.1, mesh_exact_radius=.8):
    run=Path(run);truth=read_rows(run/'truth-poses.jsonl');tel=read_rows(run/'telemetry.jsonl')
    missing=[]
    if not truth: raise ValueError('Missing actual Gazebo truth poses')
    names={r.get('child') for r in truth}
    if len(names)!=1: raise ValueError('Expected exactly one recorded vehicle pose stream')
    if any('model origin' not in r.get('definition','') for r in truth): raise ValueError('Unverified truth pose origin definition')
    base=ET.parse(run/'models/annual_base_0/model.sdf').getroot().find('model')
    offset=transform(base)
    if not np.allclose(offset[:3,:3],np.eye(3)) or not np.allclose(offset[:3,3],[0,0,.24]): raise ValueError('Recorded base_link offset differs from locked contract')
    truth=sorted({float(r['stamp']):r for r in truth}.values(),key=lambda r:r['stamp'])
    t=np.array([r['stamp'] for r in truth]);q=np.array([r['quat'] for r in truth]);p=np.array([r['xyz'] for r in truth])
    if not np.isfinite(t).all() or not np.isfinite(p).all() or not np.isfinite(q).all(): raise ValueError('Nonfinite true pose')
    p=p+Rotation.from_quat(q).apply(np.tile(offset[:3,3],(len(p),1)))
    scene=Scene(run,names,mesh_exact_radius);distance=np.full(len(p),np.inf);per_model=[]
    for g in scene.geometries:
        try:
            d=scene.distance(g,p,batch)
            if not np.isfinite(d).all(): raise ValueError('Nonfinite geometry distance')
            distance=np.minimum(distance,d)
            metadata={k:v for k,v in g.items() if k not in ('mesh','matrix','size','normal','scale')}
            metadata.update(minimum_sampled_sphere_surface_bound_m=float(d.min()-RADIUS),minimum_sample_sim_s=float(t[np.argmin(d)]))
            per_model.append(metadata)
        except Exception as error: missing.append(dict(model=g['name'],reason=f'distance query failed: {type(error).__name__}: {error}'))
        finally:
            if g['kind']=='mesh': gc.collect()
    missing=scene.missing+missing;clearance=distance-RADIUS if np.isfinite(distance).all() else None
    ex=[r['data'] for r in tel if r.get('kind')=='execution']
    ready=min((r['time'] for r in ex if r.get('ready')),default=None)
    intervals=dict(all_recorded=np.ones(len(t),bool),post_ready=t>=ready if ready is not None else np.zeros(len(t),bool))
    revisions={}
    for r in ex:
        rev=r.get('safety_revision')
        if rev and r.get('token'): revisions.setdefault((r['token'],rev['number']),(rev,[]))[1].append(r)
    safety=[]
    for (token,number),(rev,records) in revisions.items():
        start=float(rev['time']);duration=float(rev['duration'])
        records=[r for r in records if r['time']>=start]
        done=[r['time'] for r in records if r.get('trajectory_time',-1)>=duration-1e-5]
        end=min(done) if done else max((r['time'] for r in records),default=start)
        safety.append(dict(token=token,revision=number,start_sim_s=start,end_sim_s=end,completion_observed=bool(done),
                           result=summarize(t,p,clearance,(t>=start)&(t<=end),maximum_gap)))
    telemetry_available=(run/'telemetry.jsonl').is_file()
    return dict(schema='racer.stage23.clearance/1',run=str(run),sphere_radius_m=RADIUS,
        scene_sha256=scene.sha256,
        safety_revision_evidence=dict(telemetry_available=telemetry_available,
            revision_count=len(revisions) if telemetry_available else None),
        pose=dict(samples=len(t),source='Actual Gazebo model poses + rotated verified base_link offset',base_offset_m=offset[:3,3].tolist(),vehicle=next(iter(names))),
        static_collision_coverage=dict(parsed_and_evaluated=per_model,missing=missing,excluded_models=scene.excluded,
            all_discovered_static_collisions_evaluated=not missing and bool(per_model)),
        scopes={k:summarize(t,p,clearance,v,maximum_gap) for k,v in intervals.items()},safety_revision_intervals=safety,
        full_static_scene_bound_available=not missing and bool(per_model),
        method=dict(mesh='Triangle-surface nearest distance in batches; far points use enclosing world AABB signed distance as a conservative lower bound',
            mesh_exact_radius_m=mesh_exact_radius,batch_size=batch,
            primitives='Analytic signed solid box/sphere/cylinder/capsule distance; infinite plane surface distance is conservative for finite plane',
            sphere='Center-to-surface lower bound minus 0.43m; attitude affects rotated center offset, not the sphere radius',
            chord='min(endpoint surface bounds) - center endpoint chord length/2; only adjacent recorded poses with positive gap <= maximum_gap_s',maximum_gap_s=maximum_gap),
        limits=['Missing collision geometry means values cover the parsed subset only, not the full scene.',
            'Nonpositive sphere bounds mean the enclosing sphere may overlap; they do not prove vehicle collision.',
            'Positive surface distance does not establish that a center is outside a closed solid: mesh interior classification is not computed.',
            'Chord lower bound applies to straight interpolation only. Actual inter-frame curves are unobserved, so this is not a continuous physical sweep certificate.',
            'Referenced static SDF geometry is evaluated; engine-specific mesh cooking differences are not independently reconstructed.',
            'Referenced geometry files are read as present at audit time; their hashes are recorded, without assuming historic assets were immutable.',
            'No online control input, no planning decisions, no new simulation.'])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--batch',type=int,default=32);parser.add_argument('--maximum-gap',type=float,default=.1)
    parser.add_argument('--memory-limit-mib',type=int,
                        help='Require this virtual address-space limit (default: try 1536 MiB)')
    args=parser.parse_args()
    if args.batch<1 or args.maximum_gap<=0: parser.error('batch and maximum-gap must be positive')
    requested_limit=1536 if args.memory_limit_mib is None else args.memory_limit_mib
    if requested_limit<512: parser.error('memory-limit-mib must be at least 512')
    if args.output.exists() or args.output.resolve().is_relative_to(args.run.resolve()): parser.error('Choose a new output outside the original run')
    try:
        os.nice(10)
        if hasattr(os,'sched_getaffinity'): os.sched_setaffinity(0,{min(os.sched_getaffinity(0))})
    except (OSError,AttributeError): pass
    applied_limit=None;limit_error=None
    try:
        bounds=resource.getrlimit(resource.RLIMIT_AS)
        limit=min([requested_limit*1024**2]+[x for x in bounds if x!=resource.RLIM_INFINITY])
        if bounds[0]!=limit:
            resource.setrlimit(resource.RLIMIT_AS,(limit,limit))
        installed=resource.getrlimit(resource.RLIMIT_AS)[0]
        if installed==resource.RLIM_INFINITY or installed>limit:
            raise ValueError('Requested address-space limit was not applied')
        applied_limit=installed / 1024**2
    except (AttributeError,OSError,ValueError,OverflowError) as error:
        limit_error=str(error)
        if args.memory_limit_mib is not None:
            parser.error(f'Cannot enforce memory-limit-mib: {limit_error}')
    result=audit(args.run,args.batch,args.maximum_gap)
    result['resources']=dict(single_cpu_requested=True,nice_increment_requested=10,
        address_space_limit_requested_mib=requested_limit,address_space_limit_mib=applied_limit,
        address_space_limit_error=limit_error,
        peak_rss_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/(1024**2 if sys.platform=='darwin' else 1024))
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(output=str(args.output),coverage=result['static_collision_coverage'],scopes=result['scopes'],
                          resources=result['resources']),indent=2))


if __name__=='__main__': main()
