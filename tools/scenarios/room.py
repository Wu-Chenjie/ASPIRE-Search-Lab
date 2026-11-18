"""Build the Gazebo room from D's description. Truth stays in the offline file."""
import json
import math
from pathlib import Path
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
    person = None if negative else vector(spec["person_pose"], 6)
    map_path = out / "map.json"
    map_path.write_text(json.dumps(dict(bounds=[lower, upper], search_starts=[start], obstacles=[])))
    prepare(map_path, out, motion_profile="legacy")
    path = out / "scene.sdf"
    tree = ET.parse(path)
    world = tree.find("world")
    if person is not None:
        included = ET.SubElement(world, "include")
        for key, value in (("uri", "model://person_standing"), ("name", "stationary_person"),
                           ("static", "true"), ("pose", " ".join(map(str, person)))):
            ET.SubElement(included, key).text = value

    truth = {"person": person[:3] if person else None, "use": "independent evaluation only"}
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
