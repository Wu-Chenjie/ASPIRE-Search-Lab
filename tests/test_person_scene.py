import hashlib
import json
import xml.etree.ElementTree as ET
import pytest
from tools.hm3d.person_scene import prepare


def base_scene(tmp_path):
    base=tmp_path/'base';(base/'models').mkdir(parents=True)
    (base/'scene-audit.json').write_text(json.dumps(dict(scene='new_scene')))
    (base/'runtime.json').write_text(json.dumps(dict(additional_model_paths=[])))
    (base/'bridge.yaml').write_text('[]')
    (base/'scene.sdf').write_text(
        '<sdf><world><model name="hm3d_scan"><static>true</static><pose>1 2 0 0 0 0</pose>'
        '<link name="scan"><visual name="colour"/><collision name="mesh"/></link></model>'
        '<include><name>annual_x500_0</name><pose>0 0 0 0 0 0</pose></include></world></sdf>')
    vehicle = base/'models/annual_x500_0'
    vehicle.mkdir()
    (vehicle/'model.sdf').write_text('<sdf><model name="annual_x500_0"/></sdf>')
    person=tmp_path/'person.glb';person.write_bytes(b'fixture only')
    return base,person


@pytest.mark.parametrize('static,negative',[(False,False),(True,True)])
def test_new_scene_requires_explicit_offline_pose(tmp_path,static,negative):
    base,person=base_scene(tmp_path)
    with pytest.raises(ValueError,match='explicit offline'):
        prepare(base,tmp_path/'out',person,static=static,negative=negative)
    assert not (tmp_path/'out').exists()


@pytest.mark.parametrize('negative',[False,True])
@pytest.mark.parametrize('static',[False,True])
def test_target_pose_stays_out_of_runtime_and_scan_is_unchanged(tmp_path,negative,static):
    base,person=base_scene(tmp_path);out=tmp_path/'out';pose=[1,2,0,0,0,.5]
    views = {name: [3., 4., 1.] for name in ('visible', 'partial', 'hidden', 'revealed')}
    prepare(base,out,person,negative=negative,static=static,
            person_pose=pose if static or not negative else None,static_views=views if static else None)
    expected_runtime = json.loads((base/'runtime.json').read_text())
    if static:
        expected_runtime['manual_camera_test'] = True
    assert json.loads((out/'runtime.json').read_text()) == expected_runtime
    original_world = ET.parse(base/'scene.sdf').find('world')
    generated_world = ET.parse(out/'scene.sdf').find('world')
    scan = 'model[@name="hm3d_scan"]'
    scan_xml = ET.tostring(original_world.find(scan))
    assert scan_xml == ET.tostring(generated_world.find(scan))
    camera = 'include[name="annual_x500_0"]/pose'
    assert (original_world.findtext(camera) != generated_world.findtext(camera)) is static
    assert (generated_world.find('model[@name="stationary_person"]') is None) == negative
    proof=json.loads((out/'evaluation-truth.json').read_text())
    assert proof['person']==(None if negative else pose)
    assert proof['person_mesh_sha256']==hashlib.sha256(person.read_bytes()).hexdigest()
    assert proof['hm3d_model_xml_sha256'] == hashlib.sha256(scan_xml).hexdigest()


@pytest.mark.parametrize('person_arg', [None, 'missing.glb'])
def test_negative_scene_needs_no_person_mesh(tmp_path, person_arg):
    base,_=base_scene(tmp_path);out=tmp_path/'out'
    prepare(base,out,person_arg,negative=True)
    proof=json.loads((out/'evaluation-truth.json').read_text())
    assert proof['person_mesh_sha256'] is None
    assert proof['person'] is None
