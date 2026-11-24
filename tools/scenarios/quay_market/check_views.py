"""Offline geometric acceptance in Blender, with the saved market .blend open."""

import json
import math
import struct
import xml.etree.ElementTree as ET
from itertools import product
from collections import deque
from pathlib import Path

import bpy
from mathutils import Euler, Vector
from mathutils.bvhtree import BVHTree

from sys import path

path.insert(0, str(Path(__file__).resolve().parents[3] / "src"))
from aspire_camera.mount import DEPTH_HORIZONTAL_FOV, DEPTH_PITCH_LIMIT, DEPTH_TRANSLATION


def surface(objects):
    verts, faces = [], []
    for obj, matrix in objects:
        mesh = obj.data
        offset = len(verts)
        verts.extend(matrix @ v.co for v in mesh.vertices)
        faces.extend(tuple(offset + i for i in p.vertices) for p in mesh.polygons)
    return BVHTree.FromPolygons(verts, faces)


def check_gazebo_axes(directory, anchor):
    """Compare the actual glTF primitive extents in the SDF frame with Blender."""
    raw = (directory / "appearance.glb").read_bytes()
    size = struct.unpack_from("<I", raw, 12)[0]
    gltf = json.loads(raw[20 : 20 + size])
    mesh = next(m for m in gltf["meshes"] if m["name"] == anchor.data.name)
    xml = ET.parse(directory / "model.sdf")
    pose = list(map(float, xml.findtext(".//visual/pose", "0 0 0 0 0 0").split()))
    rotation = Euler(pose[3:]).to_matrix()
    corners = []
    for part in mesh["primitives"]:
        positions = gltf["accessors"][part["attributes"]["POSITION"]]
        for xyz in product(*zip(positions["min"], positions["max"])):
            corners.append(rotation @ Vector(xyz) + Vector(pose[:3]))
    for axis in range(3):
        for extreme in (min, max):
            measured = extreme(p[axis] for p in corners)
            expected = extreme(p[axis] for p in anchor.bound_box)
            if abs(measured - expected) > 0.002:
                raise ValueError(f"{directory.name}: Gazebo visual and authoring axes disagree")


def check():
    root = Path(bpy.data.filepath).parent
    spec = json.loads((root / "scene-spec.json").read_text())
    actors = [bpy.data.objects[f"survivor_{i + 1:02}"] for i in range(len(spec["people"]))]
    building = [(ob, ob.matrix_world) for ob in bpy.context.scene.objects if ob.type == "MESH"]
    visual = surface(building)
    bodies = []
    for instance in actors:
        body = next(o for o in instance.instance_collection.objects if o.name.startswith("Body"))
        bodies.append((body, instance.matrix_world))
    slab = next(ob for ob, _ in building if ob.name.startswith("Raft slab"))
    check_gazebo_axes(root / "models/quay_market", slab)
    for model in {p["model"] for p in spec["people"]}:
        body = next(ob for ob in bpy.data.collections[model].objects if ob.name.startswith("Body"))
        check_gazebo_axes(root / "models" / model, body)
    raw = (root / "models/quay_market/contact.stl").read_bytes()
    corners = []
    for face in struct.iter_unpack("<12fH", raw[84:]):
        corners.extend([face[3:6], face[6:9], face[9:12]])
    occupied = BVHTree.FromPolygons(
        corners, [(i, i + 1, i + 2) for i in range(0, len(corners), 3)], all_triangles=True
    )
    occupants = surface(bodies)
    # A 0.50 m step between samples that are both 0.85 m clear leaves at
    # least 0.60 m clearance everywhere along that edge (distance is Lipschitz).
    step = 0.5
    start = Vector(spec["starts"]["original"])
    queue = deque([(0, 0, 0)])
    visited = {(0, 0, 0)}
    reachable = []
    lo, hi = spec["bounds"]
    while queue:
        idx = queue.popleft()
        point = start + Vector(idx) * step
        if any(not low + 0.6 < v < high - 0.6 for low, v, high in zip(lo, point, hi)):
            continue
        if not spec["flight_limits"][0] <= point.z <= spec["flight_limits"][1]:
            continue
        if any(mesh.find_nearest(point, 0.85)[0] is not None for mesh in (occupied, occupants)):
            continue
        reachable.append(point)
        for axis in range(3):
            for direction in (-1, 1):
                other = list(idx)
                other[axis] += direction
                other = tuple(other)
                if other not in visited:
                    visited.add(other)
                    queue.append(other)
    # Adjacent legs share the exact same witness node. Restrict each search
    # to the intended floor/corridor so a detour around a blocked shaft fails.
    grid = {tuple(round(v, 2) for v in p) for p in reachable}

    def nearest(p):
        q = min(grid, key=lambda n: math.dist(n, p))
        if math.dist(q, p) > 0.88:
            raise ValueError(f"Route gate obstructed at {p}: nearest free sample is {q}")
        return q

    def trace(gates):
        gates = [nearest(p) for p in gates]
        path_nodes = []
        for a, b in zip(gates, gates[1:]):
            vertical = a[:2] == b[:2]
            pad = (0.65, 0.65, 0.25) if vertical else (2.4, 2.4, 1.0)
            limits = [(min(x, y) - d, max(x, y) + d) for x, y, d in zip(a, b, pad)]
            fringe = deque([a])
            parent = {a: None}
            while fringe:
                n = fringe.popleft()
                if n == b:
                    segment = [b]
                    while parent[segment[-1]] is not None:
                        segment.append(parent[segment[-1]])
                    segment.reverse()
                    path_nodes.extend(segment[1:] if path_nodes else segment)
                    break
                for axis in range(3):
                    for d in (-step, step):
                        v = list(n)
                        v[axis] = round(v[axis] + d, 2)
                        dest = tuple(v)
                        if (
                            limits[axis][0] <= v[axis] <= limits[axis][1]
                            and dest in grid
                            and dest not in parent
                        ):
                            parent[dest] = n
                            fringe.append(dest)
            else:
                raise ValueError(f"Route leg blocked between {a} and {b}")
        return dict(gates=gates, grid_path_m=(len(path_nodes) - 1) * step, path=path_nodes)

    dock_lower = (7.0, 10.0, 1.7)
    dock_upper = (7.0, 10.0, 5.2)
    bridge_head = (7.0, 8.5, 5.2)
    office = (0, -6.8, 5.2)
    entry = (0, -11, 2.2)
    routes = {
        "ice_works_to_tally_office": trace(
            [
                entry,
                (-9.3, -6.5, 1.7),
                (-14.85, -5.8, 1.7),
                (-14.85, -3.8, 0.7),
                (-14.85, -1.8, -0.3),
                (-14.85, 0.2, -1.3),
                (-14.85, 2.2, -1.8),
                (-14.5, 10.3, -1.8),
                (*dock_lower[:2], -1.8),
                dock_lower,
                dock_upper,
                bridge_head,
                (5.25, 5.0, 5.2),
                (3.5, 1.55, 5.2),
                (1.75, -1.85, 5.2),
                (0, -4.8, 5.2),
                office,
            ]
        ),
        "chute_to_ticket_booth": trace(
            [
                office,
                (-2.82, -6.45, 5.0),
                (-3.25, -7.4, 4.2),
                (-3.6, -8.25, 3.2),
                (-4.0, -9.2, 2.5),
                (-4.42, -10.05, 1.7),
                entry,
            ]
        ),
        "east_packing_return": trace(
            [
                dock_lower,
                (13.5, 10.0, 1.7),
                (13.7, 6.7, 1.7),
                (13.6, 1.40, 1.7),
                (13.7, -2.6, 1.7),
                (16.3, -3.5, 1.7),
                (16.3, -1.5, 2.7),
                (16.3, 0.5, 3.7),
                (16.3, 2.5, 4.7),
                (16.3, 4.1, 5.2),
                (13.25, 6.8, 5.2),
                (11, 10.0, 5.2),
                dock_upper,
                bridge_head,
            ]
        ),
    }
    office_eye = Vector(nearest((0, -11.5, 5.2)))
    lamp = Vector((0.14, -8.24, 5.9))
    _, _, face, _ = visual.ray_cast(office_eye, (lamp - office_eye).normalized(), (lamp - office_eye).length)
    # BVH face indices follow the concatenated polygon order in surface().
    material = None
    if face is not None:
        for ob, _ in building:
            if face < len(ob.data.polygons):
                material = ob.data.materials[ob.data.polygons[face].material_index].name
                break
            face -= len(ob.data.polygons)
    if material != "beacon":
        raise ValueError(f"Entrance overlook cannot see duty lamp: first hit {material}")
    partial = None
    report = []
    for instance, (body, transform) in zip(actors, bodies):
        foot = instance.location + Vector((0, 0, 0.065))
        support = occupied.ray_cast(foot, Vector((0, 0, -1)), 0.12)[0]
        if support is None or abs(support.z - instance.location.z) > 0.04:
            raise ValueError(f"{instance.name} has no supporting floor")
        body_surface = surface([(body, transform)])
        if any(
            max(body.data.vertices[j].co.z for j in body.data.polygons[index].vertices) > 0.10
            for _, index in visual.overlap(body_surface)
        ):
            raise ValueError(f"{instance.name} intersects the market mesh")
        # Sample across actual torso, head and limb vertices in the posed mesh.
        points = [transform @ v.co for v in body.data.vertices[:][::113] if v.co.z > 0.18]
        center = instance.location

        def fraction(base, yaw, frustum=True):
            direction = Vector((math.cos(yaw), math.sin(yaw), 0))
            lens = base + direction * DEPTH_TRANSLATION[0] + Vector((0, 0, DEPTH_TRANSLATION[2]))
            visible = 0
            for pt in points:
                d = pt - lens
                depth = direction.dot(d)
                if frustum and (
                    depth <= 0
                    or abs(direction.x * d.y - direction.y * d.x) > depth * math.tan(DEPTH_HORIZONTAL_FOV / 2)
                    or abs(d.z) > depth * math.tan(DEPTH_PITCH_LIMIT)
                ):
                    continue
                if visual.ray_cast(lens, d.normalized(), d.length)[0] is None:
                    visible += 1
            return visible / len(points)

        def yaw_to(p):
            return math.atan2(center.y - p.y, center.x - p.x)

        # At the entrance, even turning toward this person must not reveal them.
        entrance = fraction(start, yaw_to(start), False)
        visible_start = fraction(Vector(spec["starts"]["visible"]), 0)
        best = 0
        witness = None
        for point in sorted(reachable, key=lambda p: abs((p - center).length - 3.3)):
            if not 2.0 < (point - center).length < 5.0:
                continue
            score = fraction(point, yaw_to(point))
            if score > best:
                best, witness = score, point
            if score > 0.9:
                break
        if instance.name in ("survivor_03", "survivor_04") and partial is None:
            for i in range(21):
                t = i / 20
                view = Vector(nearest((7 * t, -5.3 + 13.82 * t, 4.7)))
                if not 3 < (view - center).length < 10:
                    continue
                score = fraction(view, yaw_to(view))
                if 0.15 <= score <= 0.65:
                    partial = dict(id=instance.name, view=list(view), visible_fraction=round(score, 4))
                    break
        report.append(
            dict(
                id=instance.name,
                entrance_fraction=round(entrance, 4),
                visible_start_fraction=round(visible_start, 4),
                reachable_fraction=round(best, 4),
                view=list(witness) if witness else None,
            )
        )
    data = dict(
        grid_step_m=step,
        minimum_route_clearance_m=0.60,
        reachable_nodes=len(reachable),
        routes=routes,
        entrance_lamp_view=list(office_eye),
        bridge_partial=partial,
        people=report,
    )
    (root / "offline-views.json").write_text(json.dumps(data, indent=2) + "\n")
    print(f"{len(reachable)} reachable grid nodes, {len(report)} occupants; see offline-views.json")
    if partial is None:
        raise ValueError("No partial observation from the cross-hall bridge")
    if not all(r["entrance_fraction"] == 0 and r["reachable_fraction"] >= 0.65 for r in report):
        raise ValueError("Hidden-start or reachable-view geometry failed")
    if max(r["visible_start_fraction"] for r in report) < 0.65:
        raise ValueError("Visible control has no clear target")


if __name__ == "__main__":
    check()
