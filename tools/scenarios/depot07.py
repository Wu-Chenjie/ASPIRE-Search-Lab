"""Prepare the external Depot 07 asset pack for the Search Lab flight runner."""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import shutil
from tempfile import TemporaryDirectory
import xml.etree.ElementTree as ET

from tools.flight.scene_bundle import scene
from aspire_camera.gazebo import set_text
from aspire_camera.mount import DEPTH_TRANSLATION


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def prepare(assets, output, camera_fps=8.):
    if not math.isfinite(camera_fps) or camera_fps <= 0:
        raise ValueError("camera_fps must be finite and positive")
    assets = assets.resolve()
    output = output.resolve()
    repo = Path(__file__).resolve().parents[2]
    if output.is_relative_to(repo) or output.is_relative_to(assets):
        raise ValueError("Prepare Depot 07 outside the repository and asset pack")
    if output.exists():
        raise FileExistsError(output)

    world_src = assets / "worlds/depot07.sdf"
    vehicle_src = assets / "models/aspire_r01/model.sdf"
    world_doc = ET.parse(world_src)
    world = world_doc.find("world")
    world.set("name", "annual")
    drone = world.find('./include[uri="model://aspire_r01"]')
    person = world.find('./include[uri="model://aspire_person_p01"]')
    spawn = list(map(float, drone.findtext("pose").split()))
    person_pose = list(map(float, person.findtext("pose").split()))
    vehicle = ET.parse(vehicle_src)
    base = vehicle.find('./model/link[@name="base_link"]')
    depot = ET.parse(assets / "models/aspire_depot07/model.sdf")
    floor = depot.find('.//collision[@name="concrete_floor"]')
    length, width, _ = map(float, floor.findtext("geometry/box/size").split())
    skids = [c for c in base.findall("collision") if c.get("name").startswith("skid_")]
    rest_z = max(float(c.findtext("geometry/box/size").split()[2]) / 2
                 - float(c.findtext("pose").split()[2]) for c in skids)
    start = [*spawn[:2], 2.2]

    output.parent.mkdir(parents=True, exist_ok=True)
    with TemporaryDirectory(dir=output.parent) as workspace:
        stock = Path(workspace) / "stock"
        stock.mkdir()
        runtime = scene(stock, False, "original", "consistent")
        stock_world = ET.parse(stock / "scene.sdf").find("world")
        camera = ET.parse(stock / "models/annual_camera_0/model.sdf")
        for tag in ("gravity", "magnetic_field", "atmosphere", "spherical_coordinates",
                    'plugin[@filename="gz-sim-contact-system"]'):
            world.append(copy.deepcopy(stock_world.find(tag)))

        # The stock depth mapper expects this co-located pair and mount.
        # PX4 supplies the simulation magnetometer.
        for sensor in base.findall("sensor"):
            if sensor.get("type") in ("rgbd_camera", "magnetometer"):
                base.remove(sensor)
        for sensor in camera.findall("./model/link/sensor"):
            set_text(sensor, "pose", " ".join(map(str, (*DEPTH_TRANSLATION, 0, 0, 0))))
            rate = camera_fps if sensor.get("type") in ("camera", "depth_camera", "rgbd_camera") else 8
            set_text(sensor, "update_rate", f"{rate:g}")
            base.append(sensor)

        stock_base = ET.parse(stock / "models/annual_base_0/model.sdf")
        sensor = stock_base.find('.//sensor[@name="evaluation_contact"]')
        sensor.remove(sensor.find("contact"))
        contact = ET.SubElement(sensor, "contact")
        for collision in base.findall("collision"):
            ET.SubElement(contact, "collision").text = collision.get("name")
        base.append(sensor)
        set_text(drone, "name", "annual_x500_0")
        runtime.update(bounds=[[-length / 2, -width / 2, 0], [length / 2, width / 2, 4.5]],
                       fleet_starts=[start], layout="depot07")
        runtime["vehicles"][0].update(model="annual_x500_0", start=start, spawn=spawn[:3],
                                     initial_position=[*spawn[:2], rest_z])

        staged = Path(workspace) / "prepared"
        staged.mkdir()
        shutil.copytree(assets / "models", staged / "models")
        world_target = staged / "scene.sdf"
        vehicle_target = staged / "models/aspire_r01/model.sdf"
        world_doc.write(world_target, encoding="utf-8", xml_declaration=True)
        vehicle.write(vehicle_target, encoding="utf-8", xml_declaration=True)
        shutil.copyfile(stock / "bridge.yaml", staged / "bridge.yaml")
        (staged / "runtime.json").write_text(json.dumps(runtime, indent=2) + "\n")
        (staged / "bundle.json").write_text(json.dumps({"version": 1, "negative": False}) + "\n")
        (staged / "evaluation-truth.json").write_text(json.dumps(
            {"person": person_pose[:3], "use": "independent evaluation only"}) + "\n")
        audit = {
            "asset": str(assets),
            "source_start_sha256": digest(assets / "start.sh"),
            "asset_manifest_sha256": digest(assets / "SHA256SUMS"),
            "world_sha256": digest(world_src),
            "r01_sha256": digest(vehicle_src),
            "derived_world_sha256": digest(world_target),
            "derived_r01_sha256": digest(vehicle_target),
            "camera_fps": camera_fps,
            "changes": ["runner world and vehicle instance names", "SITL earth and magnetic fields",
                        "isolated contact recorder", f"co-located {camera_fps:g} Hz RGB-D pair and PX4 magnetometer"],
        }
        (staged / "scene-audit.json").write_text(json.dumps(audit, indent=2) + "\n")
        staged.rename(output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--assets", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--camera-fps", type=float, default=8.)
    args = parser.parse_args()
    prepare(args.assets, args.output, args.camera_fps)


if __name__ == "__main__":
    main()
