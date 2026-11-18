"""Simulator-only Artec scan preparation; target poses never enter online nodes."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import zipfile
import xml.etree.ElementTree as ET
import numpy as np
from tools.rgbd_scene.cameras import set_text
from tools.prepare_sim import prepare
from tools.rgbd_scene.scene import views

ARCHIVE_SHA='a931d512a7ee87970d61d6731f556ed1d841af2140c7bead6c3ff38909f304b2'
SOURCE='https://www.artec3d.com/3d-models/body-scan'
DOWNLOAD='https://cdn.artec3d.com/content-hub-3dmodels/3d-body-scan.zip?VersionId=EA9mxOXJFoZeMpLc_IMh0QR1914D8WG1'


def convert(archive,out):
    import trimesh
    archive=Path(archive);out=Path(out);out.mkdir(parents=True,exist_ok=False)
    if hashlib.sha256(archive.read_bytes()).hexdigest()!=ARCHIVE_SHA:raise ValueError('Scan archive hash mismatch')
    with zipfile.ZipFile(archive) as z:
        for n in z.namelist():
            if n.lower().endswith(('.obj','.mtl','.png','license.txt')):
                dst=(out/'original'/n).resolve()
                if not dst.is_relative_to((out/'original').resolve()):raise ValueError('Archive path traversal')
                z.extract(n,out/'original')
    obj=next((out/'original').rglob('*.obj'));scene=trimesh.load_scene(obj,process=False)
    bounds=scene.bounds;transform=np.eye(4);transform[:3,:3]*=.001
    transform[:3,3]=[-(bounds[0,0]+bounds[1,0])*.0005,-(bounds[0,1]+bounds[1,1])*.0005,-bounds[0,2]*.001]
    scene.apply_transform(transform)
    for mesh in scene.geometry.values():
        # OBJ has no metallic parameter. glTF defaults an omitted value to 1,
        # which rendered this scanned clothed person as black reflective metal.
        image=mesh.visual.material.image
        mesh.visual.material=trimesh.visual.material.PBRMaterial(baseColorTexture=image,
            baseColorFactor=[255,255,255,255],metallicFactor=0.,roughnessFactor=1.)
    scene.export(out/'person.glb')
    shutil.copy2(next((out/'original').rglob('license.txt')),out/'LICENSE.txt')
    report=dict(source=SOURCE,download=DOWNLOAD,archive_sha256=ARCHIVE_SHA,attribution='Artec Group inc.',
        license='CC BY 3.0 Unported, bundled license retained; website currently links CC BY 4.0',
        modifications='Coordinates converted from millimetres to metres, XY bounds centered, feet at Z=0; triangles and texture preserved; explicit nonmetallic rough PBR material for scanned albedo',
        source_bounds=bounds.tolist(),transform=transform.tolist(),bounds=scene.bounds.tolist(),
        vertices=sum(len(m.vertices) for m in scene.geometry.values()),faces=sum(len(m.faces) for m in scene.geometry.values()),
        model_sha256=hashlib.sha256((out/'person.glb').read_bytes()).hexdigest())
    (out/'asset-lock.json').write_text(json.dumps(report,indent=2));return report


def fixture(mesh,out,base=None,anchor=(0,0,0),negative=False):
    out=Path(out);out.mkdir(parents=True,exist_ok=False);anchor=np.asarray(anchor,float)
    root=Path(__file__).resolve().parents[2]
    if base:
        for name in ('models','scene.sdf','runtime.json','bridge.yaml','scene-audit.json'):
            src=Path(base)/name
            if src.is_dir():shutil.copytree(src,out/name)
            else:shutil.copy2(src,out/name)
    else:
        prepare(root/'tools/rgbd_scene/map.json',out)
        (out/'scene-audit.json').write_text(json.dumps(dict(scope='Static scanned-human fixture, no autonomous motion')))
    tree=ET.parse(out/'scene.sdf');world=tree.find('world')
    for name in ('stationary_person','occlusion_board'):
        for m in world.findall('model')+world.findall('include'):
            if m.get('name')==name or m.findtext('name')==name:world.remove(m)
    if not negative:
        m=ET.SubElement(world,'model',name='stationary_person');set_text(m,'static','true')
        set_text(m,'pose',' '.join(map(str,[*(anchor+[4,0,0]),0,0,-math.pi/2])))
        link=ET.SubElement(m,'link',name='body')
        for tag in ('visual','collision'):
            part=ET.SubElement(link,tag,name='body_'+tag);set_text(part,'geometry/mesh/uri',Path(mesh).resolve().as_uri())
    board=ET.SubElement(world,'model',name='occlusion_board');set_text(board,'static','true')
    set_text(board,'pose',' '.join(map(str,[*(anchor+[2,0,1.1]),0,0,0])))
    link=ET.SubElement(board,'link',name='board')
    for tag in ('visual','collision'):
        part=ET.SubElement(link,tag,name='board_'+tag);set_text(part,'geometry/box/size','.08 .9 2.2')
        if tag=='visual':set_text(part,'material/diffuse','.12 .25 .45 1');set_text(part,'material/ambient','.12 .25 .45 1')
    v=views()
    for view in v.values():
        view['lens_xyz']=(np.array(view['lens_xyz'])+anchor).tolist()
        view['model_pose'][:3]=(np.array(view['model_pose'][:3])+anchor).tolist()
    path=out/'models/annual_x500_0/model.sdf';vehicle=ET.parse(path);set_text(vehicle.find('model'),'static','true');vehicle.write(path)
    world.find('include[name="annual_x500_0"]/pose').text=' '.join(map(str,v['visible']['model_pose']))
    tree.write(out/'scene.sdf');cfg=json.loads((out/'runtime.json').read_text());cfg.update(manual_camera_test=True,additional_model_paths=[])
    (out/'runtime.json').write_text(json.dumps(cfg,indent=2))
    truth=dict(world='annual',model='annual_x500_0',views=v,person_pose=None if negative else [*(anchor+[4,0,0]),0,0,-math.pi/2],
               board_pose=[*(anchor+[2,0,1.1]),0,0,0],negative=negative,scope='Manual static scanned person, not autonomous search')
    (out/'viewpoints.json').write_text(json.dumps(truth,indent=2));(out/'evaluation-truth.json').write_text(json.dumps(truth,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--archive',type=Path);p.add_argument('--mesh',type=Path);p.add_argument('--base',type=Path)
    p.add_argument('--anchor',type=float,nargs=3,default=[0,0,0]);p.add_argument('--negative',action='store_true');p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.archive:print(json.dumps(convert(a.archive,a.output),indent=2))
    elif a.mesh:fixture(a.mesh,a.output,a.base,a.anchor,a.negative)
    else:p.error('Specify --archive to convert or --mesh to generate a static fixture')
