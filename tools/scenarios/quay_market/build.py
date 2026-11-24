"""Run with Blender in background mode. See README for the two build commands."""

import argparse
import json
import math
from pathlib import Path
import struct
import sys
import xml.etree.ElementTree as ET

import bpy
from mathutils import Vector

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import geometry as geo  # noqa: E402
import market  # noqa: E402
from people import actor  # noqa: E402
from section import make_section  # noqa: E402

REPO = HERE.parents[2]
# XYZ in metres, yaw in degrees. The raised floor is 3.60 m above the auction hall.
PLACEMENTS = [
    ("crouched", -15.1, -7.72, 0, 142),
    ("standing", -14.7, 3.43, -3.6, -75),
    ("crouched", -4.60, -1.77, 0, 38),
    ("standing", 5.70, 5.25, 0, 145),
    ("crouched", 14.88, -6.43, 0, -96),
    ("standing", -13.28, 10.38, -3.6, -122),
    ("standing", -13.22, 10.24, 3.60, -62),
    ("crouched", 14.44, 2.45, 3.60, 145),
]


def save_stl(path, triangles):
    with path.open("wb") as f:
        f.write(bytes(80))
        f.write(struct.pack("<I", len(triangles)))
        for a, b, c in triangles:
            u, v, w = Vector(a), Vector(b), Vector(c)
            n = (v - u).cross(w - u).normalized()
            f.write(struct.pack("<12fH", *n, *u, *v, *w, 0))


# Position, colour and power of intact practical lights. The same fittings
# illuminate the below-grade plant in the simulator and the Blender master.
PRACTICALS = [
    ((-5.1, 2.4, 5.62), (1.0, 0.78, 0.53), 300.0),
    ((5.1, 2.4, 5.62), (1.0, 0.80, 0.59), 300.0),
    ((-5.1, -4.1, 5.62), (1.0, 0.74, 0.49), 310.0),
    ((5.43, -4.1, 5.62), (1.0, 0.77, 0.52), 270.0),
    ((0.14, -8.05, 5.7), (1.0, 0.55, 0.22), 55.0),
    ((-14.3, 1.65, -0.7), (1.0, 0.80, 0.53), 140.0),
    ((-14.2, 8.2, -0.75), (0.75, 0.89, 1.0), 130.0),
    ((-7.1, -11.3, -0.76), (1.0, 0.77, 0.48), 180.0),
    ((2.5, -11.3, -0.76), (0.75, 0.89, 1.0), 165.0),
    ((12.5, -11.3, -0.76), (1.0, 0.76, 0.46), 185.0),
    ((-6.9, 10.6, -0.75), (1.0, 0.76, 0.45), 180.0),
    ((2.6, 10.6, -0.75), (0.78, 0.93, 1.0), 150.0),
    ((12.6, 10.7, -0.75), (1.0, 0.78, 0.50), 145.0),
]


def export_model(root, name, objects, triangles):
    path = root / "models" / name
    path.mkdir(parents=True)
    bpy.ops.object.select_all(action="DESELECT")
    for obj in objects:
        obj.select_set(True)
    bpy.ops.export_scene.gltf(
        filepath=str(path / "appearance.glb"),
        use_selection=True,
        export_format="GLB",
        export_cameras=False,
        export_lights=False,
        export_animations=False,
        export_yup=True,
    )
    save_stl(path / "contact.stl", triangles)
    write_description(path, name)


def write_description(path, name):
    sdf = ET.Element("sdf", version="1.9")
    model = ET.SubElement(sdf, "model", name=name)
    ET.SubElement(model, "static").text = "true"
    link = ET.SubElement(model, "link", name="structure")
    for tag, filename in [("visual", "appearance.glb"), ("collision", "contact.stl")]:
        part = ET.SubElement(link, tag, name=tag)
        if tag == "visual":
            # Harmonic loads glTF vertex axes as authored. Rotate glTF +Y up to SDF +Z up.
            ET.SubElement(part, "pose").text = f"0 0 0 {math.pi / 2} 0 0"
        mesh = ET.SubElement(ET.SubElement(part, "geometry"), "mesh")
        ET.SubElement(mesh, "uri").text = f"model://{name}/{filename}"
    if name == "quay_market":
        for i, (xyz, colour, watts) in enumerate(PRACTICALS):
            light = ET.SubElement(link, "light", name=f"service_lamp_{i}", type="point")
            ET.SubElement(light, "pose").text = " ".join(map(str, (*xyz, 0, 0, 0)))
            ET.SubElement(light, "diffuse").text = " ".join(map(str, (*colour, 1)))
            ET.SubElement(light, "intensity").text = str(watts / 50)
            falloff = ET.SubElement(light, "attenuation")
            for key, value in [("range", 7), ("constant", 1), ("linear", 0.14), ("quadratic", 0.07)]:
                ET.SubElement(falloff, key).text = str(value)
    ET.indent(sdf)
    ET.ElementTree(sdf).write(path / "model.sdf", encoding="utf-8", xml_declaration=True)
    config = ET.Element("model")
    ET.SubElement(config, "name").text = name
    ET.SubElement(config, "version").text = "1.0"
    ET.SubElement(config, "sdf", version="1.9").text = "model.sdf"
    ET.ElementTree(config).write(path / "model.config", encoding="utf-8")


def camera(col, name, position, target, lens, orthographic=False):
    data = bpy.data.cameras.new(name)
    obj = bpy.data.objects.new(name, data)
    col.objects.link(obj)
    obj.location = position
    obj.rotation_euler = (Vector(target) - obj.location).to_track_quat("-Z", "Y").to_euler()
    if orthographic:
        data.type = "ORTHO"
        data.ortho_scale = lens
    else:
        data.lens = lens
    data.clip_end = 200
    return obj


def setup_render():
    scene = bpy.context.scene
    col = geo.collection("12  /  inspection cameras and daylight")
    cameras = [
        camera(col, "01 • Architectural section", (1.2, -58, 8.2), (0, 0.2, 4.0), 39.8, True),
        camera(col, "02 • Auction floor", (-1.30, 6.23, 1.85), (1.5, -6.8, 5.4), 18.5),
        camera(col, "03 • Upper gallery", (7.26, 8.36, 5.13), (0.1, -5.3, 4.9), 20),
    ]
    cameras.append(camera(col, "04 • Brine gallery", (-10.3, 9.85, -2.0), (5.6, 10.65, -1.7), 22))
    for i, (xyz, colour, watts) in enumerate(PRACTICALS):
        light = bpy.data.lights.new(f"Service practical {i + 1:02}", "POINT")
        light.energy, light.color, light.shadow_soft_size = watts, colour, 0.20
        obj = bpy.data.objects.new(light.name, light)
        col.objects.link(obj)
        obj.location = xyz
    sun = bpy.data.lights.new("Light through the broken canopy", "SUN")
    sun.energy = 3.5
    sun.angle = math.radians(3)
    obj = bpy.data.objects.new(sun.name, sun)
    col.objects.link(obj)
    obj.rotation_euler = (math.radians(29), math.radians(-24), math.radians(-19))
    scene.world = bpy.data.worlds.new("Overcast coastal daylight")
    scene.world.use_nodes = True
    bg = scene.world.node_tree.nodes.get("Background")
    bg.inputs[0].default_value = (0.67, 0.76, 0.82, 1)
    bg.inputs[1].default_value = 0.58
    sky = scene.world.node_tree.nodes.new("ShaderNodeTexSky")
    sky.sky_type = "MULTIPLE_SCATTERING"
    sky.sun_disc = False
    sky.sun_elevation = math.radians(37)
    sky.sun_rotation = math.radians(145)
    sky.air_density = 1.1
    sky.aerosol_density = 1.8
    scene.world.node_tree.links.new(sky.outputs[0], bg.inputs[0])
    bg.inputs[1].default_value = 0.25
    scene.render.engine = "CYCLES"
    scene.cycles.samples = 36
    scene.cycles.use_denoising = True
    scene.render.resolution_x, scene.render.resolution_y = 1600, 1024
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.camera = cameras[1]
    for area in bpy.context.screen.areas:
        if area.type == "VIEW_3D":
            area.spaces.active.region_3d.view_perspective = "CAMERA"
            area.spaces.active.shading.color_type = "MATERIAL"
            area.spaces.active.overlay.show_overlays = False
    return cameras


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--output", type=Path, required=True)
    args = ap.parse_args(sys.argv[sys.argv.index("--") + 1 :])
    root = args.output.resolve()
    root.mkdir(parents=True, exist_ok=False)
    bpy.ops.wm.read_factory_settings(use_empty=True)
    geo.materials()
    front = market.architecture()
    market.galleries()
    market.auction()
    market.machinery()
    market.damage()
    market.identity()
    market.service_levels()
    market.loading_gear()
    market.finishes()
    market.scale_details(front)
    market.failed_bay()
    export_model(root, "quay_market", geo.SHELL, geo.COLLISION)
    people_col = geo.collection("11  /  occupants")
    templates = {}
    heights = {}
    for pose in ("standing", "crouched"):
        name = "quay_" + pose
        collection = geo.collection(name)
        objects = actor(REPO, pose == "crouched", collection)
        body = next(obj for obj in objects if obj.name.startswith("Body"))
        body.data.calc_loop_triangles()
        triangles = [
            tuple(body.data.vertices[j].co for j in tri.vertices) for tri in body.data.loop_triangles
        ]
        export_model(root, name, objects, triangles)
        credits = (REPO / "external/rgbd_scene_assets/person_standing/model.config").read_bytes()
        (root / "models" / name / "source-model.config").write_bytes(credits)
        heights[pose] = round(max(v.co.z for ob in objects for v in ob.data.vertices), 3)
        bpy.context.scene.collection.children.unlink(collection)
        templates[pose] = collection
    spec = dict(
        ground_plane_z=-3.77,
        flight_limits=[-2.85, 7.1],
        bounds=[[-18, -13, -3.6], [18, 13, 12.2]],
        starts={"original": [0, -11, 2.2], "visible": [10.7, -6.45, 2.2]},
        environment_model="quay_market",
        boxes=[],
        people=[],
    )
    for i, (pose, x, y, z, yaw) in enumerate(PLACEMENTS, 1):
        instance = bpy.data.objects.new(f"survivor_{i:02}", None)
        people_col.objects.link(instance)
        instance.instance_type = "COLLECTION"
        instance.instance_collection = templates[pose]
        instance.location = (x, y, z)
        instance.rotation_euler.z = math.radians(yaw)
        spec["people"].append(
            dict(model="quay_" + pose, pose=[x, y, z, 0, 0, math.radians(yaw)], height=heights[pose])
        )
    (root / "scene-spec.json").write_text(json.dumps(spec, indent=2) + "\n")
    cameras = setup_render()
    full = bpy.context.scene
    full.name = "Market  /  complete building"
    section = make_section(full, cameras[0])
    credit = bpy.data.texts.new("Sources")
    credit.write(
        "North Quay Fish Exchange. Architecture and props generated in Blender.\n"
        "Occupants derived from person_standing by Marina Kollmitz / MakeHuman, "
        "osrf/gazebo_models 8163eb4b5e7e21985c6591d1c0bfb56468c0093f.\n"
    )
    bpy.ops.object.select_all(action="DESELECT")
    bpy.ops.wm.save_as_mainfile(filepath=str(root / "North Quay 08.blend"), compress=True)
    for i, (cam, filename) in enumerate(
        zip(cameras, ["01-section", "02-auction-floor", "03-gallery", "04-brine-gallery"])
    ):
        scene = section if i == 0 else full
        scene.camera = cam
        scene.render.filepath = str(root / (filename + ".png"))
        bpy.ops.render.render(scene=scene.name, write_still=True)


if __name__ == "__main__":
    main()
