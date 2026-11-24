import xml.etree.ElementTree as ET

import pytest

from tools.scenarios.depot07 import prepare


@pytest.mark.parametrize("options, expected_hz", [({}, 60), ({"camera_fps": 8}, 8)])
def test_depot07_camera_cadence(tmp_path, options, expected_hz):
    assets = tmp_path / "assets"
    inputs = {
        "worlds/depot07.sdf": """<sdf><world>
          <physics><max_step_size>.004</max_step_size></physics>
          <include><uri>model://aspire_r01</uri><pose>0 0 1 0 0 0</pose></include>
          <include><uri>model://aspire_person_p01</uri><pose>1 1 0 0 0 0</pose></include>
        </world></sdf>""",
        "models/aspire_r01/model.sdf": """<sdf><model><link name="base_link">
          <collision name="skid_1"><pose>0 0 0 0 0 0</pose>
          <geometry><box><size>1 1 0.1</size></box></geometry></collision>
        </link></model></sdf>""",
        "models/aspire_depot07/model.sdf": """<sdf><model><link>
          <collision name="concrete_floor"><geometry><box><size>12 12 1</size></box></geometry></collision>
        </link></model></sdf>""",
        "start.sh": "",
        "SHA256SUMS": "",
    }
    for name, contents in inputs.items():
        path = assets / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(contents)

    output = tmp_path / "derived"
    prepare(assets, output, **options)
    vehicle = ET.parse(output / "models/aspire_r01/model.sdf")
    rates = {s.get("type"): float(s.findtext("update_rate")) for s in vehicle.findall(".//sensor")}
    assert rates["camera"] == rates["depth_camera"] == expected_hz
    step_ns = round(float(ET.parse(output / "scene.sdf").findtext("world/physics/max_step_size")) * 1e9)
    if expected_hz == 60:
        # A whole number of physics ticks must reach the sampling gate each camera frame.
        assert 16_666_666 <= 5 * step_ns <= 16_666_670
    else:
        assert step_ns == 4_000_000
