"""Compose a D-owned world with the flight profile and evaluation contact sensor."""
import argparse
import json
from tempfile import TemporaryDirectory
from pathlib import Path
import xml.etree.ElementTree as ET

from tools.scenarios.room import build_room, DEFAULT_SPEC

PERFORMANCE_PROFILES = ("legacy", "moderate", "consistent", "bounded-fast")


def scene(out, negative, layout="original", performance="legacy", *, spec_path=DEFAULT_SPEC):
    if performance not in PERFORMANCE_PROFILES:
        raise ValueError(f"Unknown flight performance profile: {performance}")
    build_room(out, spec_path, layout, negative)
    base_path = out / "models/annual_base_0/model.sdf"
    tree = ET.parse(base_path)
    link = tree.find('model/link[@name="base_link"]')
    sensor = ET.SubElement(link, "sensor", name="evaluation_contact", type="contact")
    for tag, value in (("always_on", "true"), ("update_rate", "20"), ("topic", "/evaluation/contacts")):
        ET.SubElement(sensor, tag).text = value
    contact = ET.SubElement(sensor, "contact")
    for collision in link.findall("collision"):
        ET.SubElement(contact, "collision").text = collision.get("name")
    tree.write(base_path)
    cfg = json.loads((out / "runtime.json").read_text())
    cfg.update(research_planner="racer_ros2", native_frame_source="mapper_observation",
               racer_planning_scale=.85, racer_flight_limits=[.7, 3.], control_timing_diagnostics=True,
               bootstrap_altitudes=[1.75, 2.65, 2.2], layout=layout, performance_profile=performance)
    if performance != "legacy":
        cfg.update(motion_limits=dict(speed=1., acceleration=1.2, jerk=4., yaw_rate=.9),
                   racer_planning_limits=dict(speed=.8, acceleration=.95, jerk=4., yaw_rate=.75),
                   racer_min_time=performance != "consistent",
                   racer_feasible_time=True)
    if performance == "bounded-fast":
        cfg.update(motion_limits=dict(speed=1.2, acceleration=1.5, jerk=4., yaw_rate=1.),
                   racer_planning_limits=dict(speed=1., acceleration=1.2, jerk=4., yaw_rate=.85),
                   racer_min_time=False, racer_feasible_time=True)
    (out / "runtime.json").write_text(json.dumps(cfg, indent=2))
    return cfg


def main():
    parser = argparse.ArgumentParser(description="Prepare an indoor search scene without starting flight.")
    parser.add_argument("--spec", type=Path, default=DEFAULT_SPEC)
    parser.add_argument("--layout", default="original")
    parser.add_argument("--negative", action="store_true")
    parser.add_argument("--performance", choices=PERFORMANCE_PROFILES, default="legacy")
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if args.output.exists():
        parser.error(f"output already exists: {args.output}")
    with TemporaryDirectory(dir=args.output.parent) as staging:
        ready = Path(staging) / "scene"
        ready.mkdir()
        scene(ready, args.negative, args.layout, args.performance, spec_path=args.spec)
        (ready / "bundle.json").write_text(json.dumps({"version": 1, "negative": args.negative}))
        ready.rename(args.output)
    print(args.output)


if __name__ == "__main__":
    main()
