import json
import xml.etree.ElementTree as ET

from tools.scenarios.depot07_labelled import prepare


def test_labelled_scene_keeps_warehouse_and_registers_each_person(tmp_path):
    assets = tmp_path / "assets"
    (assets / "worlds").mkdir(parents=True)
    source = assets / "worlds/depot07.sdf"
    original = """<sdf version="1.9"><world name="depot07">
      <include><uri>model://aspire_depot07</uri></include>
      <include><uri>model://aspire_person_p01</uri></include>
      <include><uri>model://aspire_r01</uri></include>
      </world></sdf>"""
    source.write_text(original)
    out = tmp_path / "evaluation"
    prepare(assets, out)
    world = ET.parse(out / "scene.sdf").find("world")
    people = world.findall('./include[uri="model://aspire_person_p01"]')
    assert {int(p.findtext("plugin/label")) for p in people} == {11, 12, 21, 22}
    assert len(world.findall('./include[uri="model://aspire_depot07"]')) == 1
    assert not world.findall('./include[uri="model://aspire_r01"]')
    cameras = world.findall('./model[@name="evaluation_camera"]/link/sensor')
    assert {c.get("type") for c in cameras} == {"rgbd_camera", "segmentation"}
    assert {c.findtext("camera/horizontal_fov") for c in cameras} == {"1.274"}
    truth = json.loads((out / "scene-truth.json").read_text())
    poses = truth["camera_path"]
    assert len(poses) == len({tuple(p["pose"]) for p in poses}) == 3125
    assert [p["split"] for p in poses] == ["calibration"] * 625 + ["test"] * 2500
    assert source.read_text() == original
