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


def test_prepared_multilevel_market_and_negative_share_navigation_config(tmp_path):
    spec = dict(bounds=[[-18,-13,-3.6],[18,13,8.8]], flight_limits=[-2.85,7.1],
                ground_plane_z=-3.6, starts={"original":[0,-11,2.2]},
                environment_model="hall", boxes=[], people=[
                    dict(model="worker", pose=[1,2,z,0,0,0], height=1.7) for z in (-3.6,3.6)])
    for name in ("hall","worker"):
        model = tmp_path/"models"/name
        model.mkdir(parents=True)
        (model/"model.sdf").write_text(f'<sdf version="1.9"><model name="{name}"><link name="body"/></model></sdf>')
    source = tmp_path/"scene-spec.json"
    source.write_text(json.dumps(spec))
    configs = []
    for label,flags in [("occupied", []),("control", ["--negative"])]:
        out = tmp_path/label
        subprocess.run([sys.executable,"-m","tools.flight.scene_bundle","--spec",str(source),
                        "--output",str(out),*flags],cwd=ROOT,check=True)
        cfg = json.loads((out/"runtime.json").read_text())
        configs.append(cfg)
        validate_flight_scene(out,cfg)
        world = ET.parse(out/"scene.sdf").find("world")
        floor_z = float(world.findtext('model[@name="ground"]/pose').split()[2])+.1
        assert floor_z == spec["ground_plane_z"]
        assert cfg["racer_flight_limits"] == spec["flight_limits"]
        assert (out/"models/hall/model.sdf").exists()
        actors = world.findall('include[uri="model://worker"]')
        truth = json.loads((out/"evaluation-truth.json").read_text())["people"]
        assert len(actors) == len(truth) == (0 if flags else 2)
        assert [a.findtext("name") for a in actors] == [t["id"] for t in truth]
        assert "people" not in cfg and "environment_model" not in cfg
    assert configs[0] == configs[1]
