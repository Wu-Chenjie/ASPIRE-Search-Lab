"""Exercise the independent team entry points and their shared on-disk / ROS contracts."""
import json
import os
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import yaml

from aspire_interfaces.topics import camera_topic
from tools.flight.run import validate_flight_scene
from tools.scenarios.room import DEFAULT_SPEC

ROOT = Path(__file__).resolve().parents[1]


def test_scene_author_can_deliver_a_custom_start_and_an_empty_control(tmp_path):
    spec = json.loads(DEFAULT_SPEC.read_text())
    spec["starts"] = {"doorway": [0.3, 1.0, 2.2]}
    spec["person_pose"][:3] = [4.5, 0.5, 0]
    path = tmp_path / "map.json"
    path.write_text(json.dumps(spec))
    runs = []
    for name, options in (("positive", []), ("negative", ["--negative"])):
        out = tmp_path / name
        subprocess.run([sys.executable, "-m", "tools.flight.scene_bundle", "--spec", str(path),
                        "--layout", "doorway", "--output", str(out), *options], cwd=ROOT, check=True)
        config = json.loads((out / "runtime.json").read_text())
        validate_flight_scene(out, config)
        assert config["fleet_starts"] == [[0.3, 1.0, 2.2]]
        world = ET.parse(out / "scene.sdf")
        pose = world.findtext('./world/include[name="stationary_person"]/pose')
        assert (pose is None) == bool(options)
        truth = json.loads((out / "evaluation-truth.json").read_text())
        assert truth["person"] == (None if options else [4.5, 0.5, 0])
        topics = {row["ros_topic_name"] for row in yaml.safe_load((out / "bridge.yaml").read_text())}
        for stream in ("depth", "rgb"):
            for channel in ("image", "camera_info"):
                assert camera_topic(0, stream, channel) in topics
        runs.append(config)
    assert runs[0] == runs[1]  # Person presence must not enter the planning configuration.


def test_vision_import_with_only_camera_and_contracts(tmp_path):
    code = """
import sys
class BlockPlanner:
    def find_spec(self, name, path=None, target=None):
        if name.split('.')[0] in {'core', 'annual_runtime', 'tools', 'aspire_planning'}:
            raise AssertionError('visual processing imports ' + name)
sys.meta_path.insert(0, BlockPlanner())
from person_search import node, presence_adapter, registry_evidence
assert node.confirmation_options('registry', False, False)[1] is False
"""
    subprocess.run([sys.executable, "-c", code], cwd=tmp_path, check=True,
                   env={**os.environ, "PYTHONPATH": str(ROOT / "src")})
