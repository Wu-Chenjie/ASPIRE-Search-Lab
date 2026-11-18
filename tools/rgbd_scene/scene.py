"""Small static appearance/occlusion fixture; evaluator geometry never enters planner."""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
from tools.rgbd_scene.cameras import set_text
ROOT=Path(__file__).resolve().parents[2]


def views():
    result={}
    for name,y in [('visible',-2.),('partial',-.9),('hidden',0.),('revealed',2.)]:
        # Camera model origin and base_link are offset; compute lens pose exactly.
        yaw=math.atan2(-y,4.)
        lens_offset=[.13233,0.,.26078]
        result[name]=dict(lens_xyz=[0.,y,1.05],lens_yaw=yaw,
            model_pose=[-lens_offset[0]*math.cos(yaw),y-lens_offset[0]*math.sin(yaw),1.05-lens_offset[2],0.,0.,yaw])
    return result


def add_test_scene(world,output,runtime):
    assets=ROOT/'external/rgbd_scene_assets'
    if not (assets/'person_standing/model.sdf').exists():
        raise ValueError('Run tools/rgbd_scene/prepare_assets.py before generating the RGBD test scene')
    if world.findall('model[@name="obstacle_0"]'):
        raise ValueError('RGBD fixture map must have no other obstacles')
    include=ET.SubElement(world,'include')
    for key,value in [('uri','model://person_standing'),('name','stationary_person'),('static','true'),('pose','4 0 0 0 0 -1.5707963267948966')]:
        ET.SubElement(include,key).text=value
    board=ET.SubElement(world,'model',name='occlusion_board')
    set_text(board,'static','true');set_text(board,'pose','2 0 1.1 0 0 0')
    link=ET.SubElement(board,'link',name='board')
    for tag in ('visual','collision'):
        part=ET.SubElement(link,tag,name='board_'+tag)
        set_text(part,'geometry/box/size','.08 .9 2.2')
        if tag=='visual':
            set_text(part,'material/ambient','.12 .25 .45 1')
            set_text(part,'material/diffuse','.12 .25 .45 1')
    # Generated model variant: fixed test rig; not a flyable/authorized flight setup.
    path=output/'models/annual_x500_0/model.sdf';vehicle=ET.parse(path)
    set_text(vehicle.find('model'),'static','true');vehicle.write(path,encoding='unicode')
    viewpoints=views()
    camera=world.find('include[name="annual_x500_0"]')
    camera.find('pose').text=' '.join(map(str,viewpoints['visible']['model_pose']))
    runtime.update(manual_camera_test=True,additional_model_paths=[str(assets)])
    # Deliberately separate from planner runtime: known target geometry and test poses.
    (output/'viewpoints.json').write_text(json.dumps(dict(world='annual',model='annual_x500_0',
        views=viewpoints,person_pose=[4,0,0,0,0,-math.pi/2],board_pose=[2,0,1.1,0,0,0],
        scope='Manual camera acceptance only; no controller, target positions not sent to planner'),indent=2)+'\n')
