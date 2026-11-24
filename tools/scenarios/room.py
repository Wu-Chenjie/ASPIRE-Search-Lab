"""Build the Gazebo room from D's description. Truth stays in the offline file."""
import json
import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

from tools.prepare_sim import model, prepare

DEFAULT_SPEC = Path(__file__).with_name("room.json")


def vector(values, length):
    if (not isinstance(values, list) or len(values) != length or
            any(type(x) not in (int, float) or not math.isfinite(x) for x in values)):
        raise ValueError(f"Expected {length} finite numeric coordinates, got {values!r}")
    return values


def build_room(out, spec_path, layout, negative):
    source = Path(spec_path).read_bytes()
    spec = json.loads(source)
    if layout not in spec["starts"]:
        raise ValueError(f"Unknown start {layout!r} in {spec_path}")
    lower, upper = (vector(v, 3) for v in spec["bounds"])
    start = vector(spec["starts"][layout], 3)
    if any(not lo < p < hi for lo, p, hi in zip(lower, start, upper)):
        raise ValueError("Start must lie inside the room bounds")
    map_path = out / "map.json"
    map_path.write_text(json.dumps(dict(bounds=[lower, upper], search_starts=[start], obstacles=[])))
    prepare(map_path, out, motion_profile="legacy")
    path = out / "scene.sdf"
    tree = ET.parse(path)
    world = tree.find("world")
    if "ground_plane_z" in spec:
        z = spec["ground_plane_z"]
        world.find('model[@name="ground"]/pose').text = f"0 0 {z - .1} 0 0 0"

    def include(model_name, instance, pose):
        included = ET.SubElement(world, "include")
        for key, value in (("uri", "model://" + model_name), ("name", instance),
                           ("static", "true"), ("pose", " ".join(map(str, pose)))):
            ET.SubElement(included, key).text = value

    truth = {"use": "independent evaluation only"}
    asset_models = set()
    if "environment_model" in spec:
        building = spec["environment_model"]
        asset_models.add(building)
        include(building, building, [0] * 6)
    if "people" in spec:
        truth["people"] = []
        for index, actor in enumerate([] if negative else spec["people"], 1):
            pose = vector(actor["pose"], 6)
            name = f"survivor_{index:02}"
            asset_models.add(actor["model"])
            include(actor["model"], name, pose)
            truth["people"].append(dict(id=name, position=pose[:3], height=actor["height"]))
    else:
        person = None if negative else vector(spec["person_pose"], 6)
        if person:
            include("person_standing", "stationary_person", person)
        truth["person"] = person[:3] if person else None
    for asset in sorted(asset_models):
        if not asset.isidentifier():
            raise ValueError(f"Invalid scene model directory: {asset!r}")
        shutil.copytree(Path(spec_path).parent / "models" / asset, out / "models" / asset)
    names = {obj.get("name") for obj in world.findall("model")}
    names.update(obj.findtext("name") for obj in world.findall("include"))
    for box in spec["boxes"]:
        name = box["name"]
        center, size = vector(box["center"], 3), vector(box["size"], 3)
        if not isinstance(name, str) or not name or name in names or min(size) <= 0:
            raise ValueError(f"Invalid or duplicate scene box: {name!r}")
        names.add(name)
        model(world, name, center, ("box", {"size": " ".join(map(str, size))}))
        if name == "board":
            truth["board"] = center
    ET.SubElement(world, "plugin", filename="gz-sim-contact-system", name="gz::sim::systems::Contact")
    tree.write(path)
    (out / "evaluation-truth.json").write_text(json.dumps(truth))
    (out / "scene-spec.json").write_bytes(source)
    return vector(spec.get("flight_limits", [.7, 3.]), 2)
