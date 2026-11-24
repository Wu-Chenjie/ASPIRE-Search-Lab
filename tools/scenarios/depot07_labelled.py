"""Generate the sensor-only Depot 07 instance-mask evaluation scene and camera path."""
import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from aspire_camera.gazebo import set_text

# The two pairs occupy the south and north-central aisles.
# Poses are expressed in Gazebo world coordinates. Labels identify individuals.
PEOPLE = [
    (11, "fallen_south", [-.25, -3.20, .181, math.pi / 2, 0, -1.52]),
    (12, "standing_south", [1.50, -2.86, .003, 0, 0, .18]),
    (21, "fallen_north", [-3.05, 4.99, .171, -math.pi / 2, 0, -1.38]),
    (22, "standing_north", [-.72, 4.91, .003, 0, 0, -.27]),
]
CRATES = [
    ("south", [1.47, -3.47, .50], [.89, .38, 1.0]),
    ("north", [-.72, 4.25, .58], [.94, .40, 1.16]),
]


def numbers(values):
    return " ".join(map(str, values))


def camera_path():
    poses = []
    for clip in range(25):
        north = clip % 2
        cx, cy = (-1.75, 4.8) if north else (.16, -3.07)
        variant = clip // 2
        for frame in range(125):
            t = frame / 124
            side = (-1 if variant % 2 else 1) * (2 * t - 1)
            x = cx + side * (1.00 + .12 * (variant % 4)) + .25 * math.sin(variant)
            distance = 2.45 + .38 * (variant % 5)
            if not north and variant % 5 == 4:
                distance = 3.0  # Keep the camera inside the south aisle.
            y = cy - distance - .24 * math.cos(math.pi * t)
            z = 1.27 + .18 * (variant % 5) + .22 * math.sin(math.pi * t + .3 * variant)
            target_x = cx + .35 * math.sin(math.pi * t + variant)
            yaw = math.atan2(cy - y, target_x - x)
            if clip % 5 == 4:  # Background sweeps turn away from these two actors.
                yaw += (-1 if north else 1) * (1.9 + .25 * t)
            pitch = math.atan2(z - .70, math.hypot(cy - y, target_x - x))
            poses.append(dict(clip=clip, frame=frame, split="calibration" if clip < 5 else "test",
                              pose=[x, y, z, 0., pitch, yaw]))
    return poses


def prepare(assets, output):
    tree = ET.parse(assets / "worlds/depot07.sdf")
    world = tree.find("world")
    world.set("name", "labelled_depot")
    for inc in list(world.findall("include")):
        if inc.findtext("uri") != "model://aspire_depot07":
            world.remove(inc)
    for label, name, pose in PEOPLE:
        person = ET.SubElement(world, "include")
        set_text(person, "uri", "model://aspire_person_p01")
        set_text(person, "name", name)
        set_text(person, "pose", numbers(pose))
        plugin = ET.SubElement(person, "plugin", filename="gz-sim-label-system", name="gz::sim::systems::Label")
        set_text(plugin, "label", label)
    for name, pose, size in CRATES:
        model = ET.SubElement(world, "model", name=name + "_crate")
        set_text(model, "static", "true")
        set_text(model, "pose", numbers(pose + [0, 0, 0]))
        link = ET.SubElement(model, "link", name="body")
        visual = ET.SubElement(link, "visual", name="crate")
        set_text(visual, "geometry/box/size", numbers(size))
        set_text(visual, "material/diffuse", ".44 .30 .16 1")
        set_text(visual, "material/ambient", ".32 .22 .13 1")
    path = camera_path()
    rig = ET.SubElement(world, "model", name="evaluation_camera")
    set_text(rig, "pose", numbers(path[0]["pose"]))
    set_text(rig, "static", "true")
    body = ET.SubElement(rig, "link", name="lens")
    for name, kind in [("rgbd", "rgbd_camera"), ("truth", "segmentation")]:
        sensor = ET.SubElement(body, "sensor", name=name, type=kind)
        set_text(sensor, "topic", "/labelled/" + name)
        set_text(sensor, "update_rate", 120)
        set_text(sensor, "always_on", "true")
        set_text(sensor, "camera/horizontal_fov", 1.274)
        set_text(sensor, "camera/image/width", 320)
        set_text(sensor, "camera/image/height", 240)
        set_text(sensor, "camera/clip/near", .2)
        set_text(sensor, "camera/clip/far", 19.1)
        if name == "truth":
            set_text(sensor, "camera/segmentation_type", "semantic")
    set_text(world, "physics/max_step_size", ".008333333")
    output.mkdir(parents=True, exist_ok=False)
    tree.write(output / "scene.sdf", encoding="utf-8", xml_declaration=True)
    truth = dict(people=[dict(id=i, name=n, pose=p) for i, n, p in PEOPLE],
                 crates=CRATES, camera_path=path)
    (output / "scene-truth.json").write_text(json.dumps(truth) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--assets", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    prepare(args.assets, args.output)
