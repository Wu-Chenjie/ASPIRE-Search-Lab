"""Add one stationary scan to unchanged HM3D geometry; simulator/evaluation only."""
import argparse,hashlib,json,math,shutil
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from tools.rgbd_scene.cameras import set_text


def prepare(base,out,person=None,static=False,negative=False,person_pose=None,static_views=None):
    base,out=Path(base),Path(out)
    metadata=json.loads((base/'scene-audit.json').read_text())
    pose=None
    if person_pose is not None or static or not negative:
        if person_pose is None and metadata.get('scene')!='NBg5UqG3di3':
            raise ValueError('New scenes require an explicit offline person pose')
        pose=np.asarray(person_pose if person_pose is not None else [26.6,5.3,0.,0.,0.,0.],float)
        if pose.shape!=(6,) or not np.isfinite(pose).all():raise ValueError('Expected finite person xyz/rpy')
        pose=pose.tolist()
    if static and static_views is None and metadata.get('scene')!='NBg5UqG3di3':
        raise ValueError('New scenes require explicit static sensor viewpoints')
    person_hash=None
    if person is None and not negative:
        raise ValueError('Positive scenes require a person mesh')
    if person is not None:
        person=Path(person)
        try:
            person_hash=hashlib.sha256(person.read_bytes()).hexdigest()
        except OSError:
            if not negative:
                raise
    out.mkdir(parents=True,exist_ok=False)
    for name in ('models','scene.sdf','runtime.json','bridge.yaml','scene-audit.json'):
        p=base/name
        if p.is_dir():shutil.copytree(p,out/name)
        else:shutil.copy2(p,out/name)
    tree=ET.parse(out/'scene.sdf');world=tree.find('world')
    # Only sibling models and the camera vehicle include are edited below.
    scan_xml=ET.tostring(world.find('model[@name="hm3d_scan"]'))
    # This pose is only simulator generation and independent evaluation data.
    if not negative:
        m=ET.SubElement(world,'model',name='stationary_person');set_text(m,'static','true');set_text(m,'pose',' '.join(map(str,pose)))
        link=ET.SubElement(m,'link',name='body')
        for tag in ('visual','collision'):
            part=ET.SubElement(link,tag,name='body_'+tag);set_text(part,'geometry/mesh/uri',person.resolve().as_uri())
    cfg=json.loads((out/'runtime.json').read_text());cfg['additional_model_paths']=[]
    if static:
        setup={}
        # Manually chosen offline sensor fixtures; never used by flight control.
        views=static_views if static_views is not None else dict(visible=[26.4,2.1,1.35],partial=[28.5,2.8,1.35],
                          hidden=[33.1,3.1,1.52078],revealed=[27.1,2.3,1.35])
        if set(views)!={'visible','partial','hidden','revealed'}:raise ValueError('Expected four named static views')
        for name,definition in views.items():
            lens=np.asarray(definition['lens_xyz'] if isinstance(definition,dict) else definition,float)
            if lens.shape!=(3,) or not np.isfinite(lens).all():raise ValueError('Expected finite static lens xyz')
            lens=lens.tolist()
            yaw=definition.get('lens_yaw',math.atan2(pose[1]-lens[1],pose[0]-lens[0])) if isinstance(definition,dict) else math.atan2(pose[1]-lens[1],pose[0]-lens[0])
            if not math.isfinite(yaw):raise ValueError('Expected finite static yaw')
            setup[name]=dict(lens_xyz=lens,lens_yaw=yaw,model_pose=[lens[0]-.13233*math.cos(yaw),lens[1]-.13233*math.sin(yaw),lens[2]-.26078,0.,0.,yaw])
        p=out/'models/annual_x500_0/model.sdf';vehicle=ET.parse(p);set_text(vehicle.find('model'),'static','true');vehicle.write(p)
        world.find('include[name="annual_x500_0"]/pose').text=' '.join(map(str,setup['visible']['model_pose']))
        cfg['manual_camera_test']=True
        (out/'viewpoints.json').write_text(json.dumps(dict(world='annual',model='annual_x500_0',views=setup,person_pose=None if negative else pose,
            negative=negative,scope='Static hand-positioned camera in unchanged '+metadata['scene']+'; NOT autonomous search'),indent=2))
    tree.write(out/'scene.sdf');(out/'runtime.json').write_text(json.dumps(cfg,indent=2))
    report=dict(person=None if negative else pose,person_mesh_sha256=person_hash,board=None,
        hm3d_model_xml_sha256=hashlib.sha256(scan_xml).hexdigest(),base_scene=str(base),geometry_unchanged=True,
        use='Scene generation and independent evaluation only; target is absent from runtime.json')
    (out/'evaluation-truth.json').write_text(json.dumps(report,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--base',required=True,type=Path);p.add_argument('--output',required=True,type=Path)
    p.add_argument('--person',type=Path,help='Person mesh required for positive scenes');p.add_argument('--static',action='store_true');p.add_argument('--negative',action='store_true')
    p.add_argument('--pose',nargs=6,type=float,help='Offline simulator/evaluation person xyz/rpy; never runtime goals')
    p.add_argument('--views',type=Path,help='Offline JSON with four hand-positioned static lens views');a=p.parse_args()
    if a.person is None and not a.negative:p.error('--person is required for positive scenes')
    prepare(a.base,a.output,a.person,a.static,a.negative,a.pose,json.loads(a.views.read_text()) if a.views else None)
