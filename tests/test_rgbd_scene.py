"""Generated optical contract and real occlusion geometry, no flight claims."""
import json
from pathlib import Path
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
import xml.etree.ElementTree as ET
from tools.rgbd_scene.scene import views
from tools.rgbd_scene.cameras import transforms
from tools.prepare_sim import prepare
ROOT=Path(__file__).resolve().parents[1]


def test_manual_view_geometry_crosses_board_without_changing_scene():
    for name,view in views().items():
        origin=np.array(view['lens_xyz']);target=np.array([4,0,1.05])
        y_at_board=(origin+(target-origin)*.5)[1]
        if name=='hidden':assert abs(y_at_board)<.45
        if name in ('visible','revealed'):assert abs(y_at_board)>.45+.2
        if name=='partial':assert abs(y_at_board)==pytest.approx(.45)
        pose=view['model_pose'];r=Rotation.from_euler('z',pose[-1])
        np.testing.assert_allclose(np.array(pose[:3])+r.apply([.13233,0,.26078]),origin,atol=1e-12)


def test_optical_tf_axes_match_depth_projection():
    tf=transforms(0)
    assert tf[0]['xyz']==tf[1]['xyz'] and tf[0]['xyzw']==tf[1]['xyzw']
    r=Rotation.from_quat(tf[0]['xyzw'])
    np.testing.assert_allclose(r.apply([0,0,1]),[1,0,0],atol=1e-12)
    np.testing.assert_allclose(r.apply([1,0,0]),[0,-1,0],atol=1e-12)
    np.testing.assert_allclose(r.apply([0,1,0]),[0,0,-1],atol=1e-12)


def test_regeneration_preserves_rgb_depth_and_static_test_fixture(tmp_path):
    if not (ROOT/'external/rgbd_scene_assets/person_standing/model.sdf').exists():pytest.skip('optional pinned appearance assets unavailable')
    if not (ROOT/'external/PX4-gazebo-models/models').exists():pytest.skip('locked vehicle models unavailable')
    for _ in range(2):
        path=prepare(ROOT/'tools/rgbd_scene/map.json',tmp_path,rgbd_test_scene=True)
        cfg=json.loads(path.read_text());assert cfg['manual_camera_test']
        camera=ET.parse(tmp_path/'models/annual_camera_0/model.sdf')
        rgb=camera.find('./model/link/sensor[@type="camera"]');depth=camera.find('./model/link/sensor[@type="depth_camera"]')
        for key in ('pose','update_rate','camera/horizontal_fov','camera/image/width','camera/image/height','camera/clip/near','camera/clip/far'):
            assert rgb.findtext(key)==depth.findtext(key)
        assert depth.findtext('camera/image/format')=='R_FLOAT32'
        world=ET.parse(tmp_path/'scene.sdf')
        assert world.findtext('./world/include[name="stationary_person"]/static')=='true'
        assert world.find('./world/model[@name="occlusion_board"]/link/collision') is not None
        assert 'person_pose' not in cfg and 'board_pose' not in cfg


@pytest.mark.parametrize('value',[None,'unused',9,121])
def test_heading_calibration_duration_only_applies_to_ground_calibration(tmp_path,value):
    missing_map=tmp_path/'missing.json'
    for mode in ('init','auto'):
        with pytest.raises(FileNotFoundError):
            prepare(missing_map,tmp_path,magnetometer_fusion=mode,heading_calibration_s=value)
    with pytest.raises(ValueError,match='Heading calibration'):
        prepare(missing_map,tmp_path,magnetometer_fusion='ground_calibrated_init',heading_calibration_s=value)


def test_heading_calibration_cli_validates_only_the_enabled_option(monkeypatch,capsys):
    from tools import prepare_sim

    parameters=[]
    monkeypatch.setattr(prepare_sim,'prepare',lambda *args,**kwargs:parameters.append(kwargs))
    monkeypatch.setattr('sys.argv',['prepare_sim.py','--heading-calibration-seconds','unused'])
    prepare_sim.main()
    assert parameters[-1]['magnetometer_fusion']=='init'
    monkeypatch.setattr('sys.argv',['prepare_sim.py','--magnetometer-fusion','ground_calibrated_init',
                                  '--heading-calibration-seconds','120'])
    prepare_sim.main()
    assert parameters[-1]['heading_calibration_s']==120
    monkeypatch.setattr('sys.argv',['prepare_sim.py','--magnetometer-fusion','ground_calibrated_init',
                                  '--heading-calibration-seconds','unused'])
    with pytest.raises(SystemExit,match='2'):
        prepare_sim.main()
    assert 'requires an integer with ground calibration' in capsys.readouterr().err
