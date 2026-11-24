"""Pose the market's MakeHuman mesh in its baked standing frame (metres)."""

import math
import xml.etree.ElementTree as ET

import bpy
import numpy as np
from mathutils import Matrix


# Absolute segment pitches in the sagittal plane, per side. Arm entries give
# upper-arm pitch, forearm pitch and the inward swing of the forearm.
# Body vertices in the DAE are already posed and have no exported skin weights.
# Its armature matrices describe a different rest pose from those vertices.
# Derive the hinges from the actual standing surface, keeping every UV intact.
# torso, neck, L/R (thigh, shin, boot), L/R arm, roll, shirt colour
POSES = {
    "seated_braced": (
        (-8, 11),
        ((-127, -43, -8), (-96, -78, -21)),
        ((-38, -85, 27), (10, -23, 12)),
        0,
        (0.13, 0.21, 0.19),
    ),
    "seated_leg_extended": (
        (4, 24),
        ((-95, -84, -24), (-120, -48, -5)),
        ((-37, -77, 35), (-25, -90, 35)),
        0,
        (0.19, 0.096, 0.055),
    ),
    "side_curled": (
        (13, 35),
        ((-57, 5, 2), (-89, 20, 14)),
        ((-105, -218, -18), (-43, -120, 39)),
        87,
        (0.28, 0.30, 0.22),
    ),
    "kneeling_guard": (
        (24, 57),
        ((-43, 95, 162), (-63, 87, 159)),
        ((-61, -186, 12), (-58, -184, 14)),
        0,
        (0.23, 0.11, 0.066),
    ),
    "cradled_arm": (
        (13, 27),
        ((-19, 14, 0), (-21, -11, 0)),
        ((-15, -91, 63), (-13, -106, 71)),
        0,
        (0.26, 0.235, 0.16),
    ),
    "side_resting": (
        (7, 19),
        ((-22, 7, 3), (-57, 14, 6)),
        ((-102, -213, -12), (-35, -97, 42)),
        87,
        (0.16, 0.21, 0.245),
    ),
    "seated_tucked": (
        (-4, 18),
        ((-135, -40, -4), (-126, -49, -8)),
        ((-51, -103, 24), (-48, -102, 28)),
        0,
        (0.26, 0.16, 0.095),
    ),
    "half_kneeling": (
        (19, 7),
        ((-91, -3, 0), (3, 91, 159)),
        ((-34, -64, 21), (-16, -86, 47)),
        0,
        (0.105, 0.17, 0.20),
    ),
}


def rotation(axis, angle):
    return np.asarray(Matrix.Rotation(math.radians(angle), 3, axis), dtype=float)


def blend(a, b, value, lo, hi):
    t = np.clip((value - lo) / (hi - lo), 0, 1)[:, None]
    t = t * t * (3 - 2 * t)
    return (1 - t) * a + t * b


def hinge(points, pivot, joint, rot):
    return (points - pivot) @ rot.T + joint


def pose_vertices(points, part, pose):
    (lean, nod), legs, arms, roll, _ = POSES[pose]
    pelvis = np.array([0.0, -0.02, 0.99])
    torso = rotation("X", lean)
    spine = hinge(points, pelvis, pelvis, torso)
    x, _, z = points.T
    neck = np.array([0, -0.025, 1.59])
    head = hinge(points, neck, hinge(neck, pelvis, pelvis, torso), rotation("X", nod))
    posed = blend(spine, head, z, 1.575, 1.62)
    for side, leg, arm in zip((1, -1), legs, arms):
        if part in ("jeans01", "Work boots"):
            hip = np.array([side * 0.103, -0.018, 0.99])
            knee = np.array([side * 0.131, -0.042, 0.53])
            ankle = np.array([side * 0.132, -0.031, 0.09])
            thigh, shin, foot = (rotation("X", angle) for angle in leg)
            if roll and side == -1:
                # The upper knee rolls forward onto the floor beside the lower leg.
                thigh = rotation("Z", 40 if pose == "side_curled" else 46) @ thigh
                shin = rotation("Z", 8) @ shin
            knee_at = hinge(knee, hip, hip, thigh)
            ankle_at = hinge(ankle, knee, knee_at, shin)
            lower = blend(
                hinge(points, ankle, ankle_at, foot), hinge(points, knee, knee_at, shin), z, 0.085, 0.16
            )
            lower = blend(lower, hinge(points, hip, hip, thigh), z, 0.45, 0.61)
            lower = blend(lower, spine, z, 0.90, 1.07)
            posed = np.where((side * x > 0)[:, None], lower, posed)
        if part in ("Body", "tshirt02"):
            shoulder = np.array([side * 0.192, -0.024, 1.514])
            elbow = np.array([side * 0.216, -0.017, 1.195])
            upper = rotation("X", arm[0])
            if pose == "kneeling_guard":
                upper = rotation("Z", -side * 27) @ upper
            fore = rotation("Z", -side * arm[2]) @ rotation("X", arm[1])
            shoulder_at = hinge(shoulder, pelvis, pelvis, torso)
            elbow_at = hinge(elbow, shoulder, shoulder_at, upper)
            arm_coords = blend(
                hinge(points, elbow, elbow_at, fore),
                hinge(points, shoulder, shoulder_at, upper),
                z,
                1.14,
                1.245,
            )
            if part == "Body":
                # Exposed arms and head are separate skin islands under the shirt.
                posed = np.where(((side * x > 0.145) & (z < 1.45))[:, None], arm_coords, posed)
            else:
                sleeve = np.interp(z, [1.32, 1.38, 1.49, 1.59], [0.177, 0.155, 0.169, 0.22])
                weight = np.clip((side * x - sleeve + 0.014) / 0.044, 0, 1)
                weight *= np.clip((z - 1.31) / 0.032, 0, 1)
                posed = blend(posed, arm_coords, weight, 0, 1)
    if pose.startswith("seated"):
        # Seat the pelvis while the planted heels stay at their original level.
        # The lowered shoulder carries the resting arm with it.
        settle = np.clip((z - 0.28) / 0.54, 0, 1) if part == "jeans01" else np.ones_like(z)
        if part != "Work boots":
            posed[:, 2] -= 0.056 * settle
        if pose == "seated_braced" and part == "Body":
            wrist = np.array([-0.229, -0.059, 0.925])
            shoulder = np.array([-0.192, -0.024, 1.514])
            elbow = np.array([-0.216, -0.017, 1.195])
            arm, fore, inward = arms[1]
            elbow_at = hinge(elbow, shoulder, hinge(shoulder, pelvis, pelvis, torso), rotation("X", arm))
            wrist_at = hinge(wrist, elbow, elbow_at, rotation("Z", inward) @ rotation("X", fore))
            palm = hinge(points, wrist, wrist_at, rotation("X", -55))
            palm[:, 2] -= 0.056
            hand = (x < -0.15) & (z < 0.97)
            posed[hand] = blend(palm, posed, z, 0.9, 0.97)[hand]
    if roll:
        # Let the lower temple settle into the bent arm rather than suspending
        # the neck horizontally above the floor.
        chest = np.array([0, -0.12, 1.40])
        tilted = hinge(posed, chest, chest, rotation("Y", 19))
        posed = blend(posed, tilted, z, 1.46, 1.62)
    return (posed - pelvis) @ rotation("Y", roll).T


def work_boot_mesh():
    # Rubber deck boots with a toe box, tread band and calf cuff.
    points, faces, bands = [], [], []
    rings = [
        (-0.013, -0.105, 0.072, 0.16),
        (0.009, -0.105, 0.073, 0.162),
        (0.044, -0.102, 0.073, 0.157),
        (0.087, -0.076, 0.073, 0.137),
        (0.133, -0.028, 0.076, 0.104),
        (0.245, -0.021, 0.078, 0.101),
        (0.253, -0.021, 0.08, 0.103),
        (0.27, -0.021, 0.08, 0.103),
    ]
    n = 24
    for side in (1, -1):
        start = len(points)
        for j, (z, y, width, depth) in enumerate(rings):
            for i in range(n):
                angle = math.tau * i / n
                points.append((side * 0.132 + width * math.cos(angle), y + depth * math.sin(angle), z))
            if j:
                for i in range(n):
                    a = start + (j - 1) * n + i
                    b = start + (j - 1) * n + (i + 1) % n
                    faces.append([a, b, b + n, a + n])
                    bands.append(int(j in (1, 7)))
        faces.extend([list(reversed(range(start, start + n))), list(range(len(points) - n, len(points)))])
        bands.extend([1, 0])
    return np.asarray(points), faces, bands


def actor(repo, pose, col):
    source = repo / "external/rgbd_scene_assets/person_standing"
    tree = ET.parse(source / "meshes/standing.dae")
    for element in tree.iter():
        element.tag = element.tag.rsplit("}", 1)[-1]
    textures = dict(
        Body="young_lightskinned_male_diffuse",
        jeans01="jeans_basic_diffuse",
        tshirt02="tshirt02_texture",
        mhair02="male02_diffuse_black",
        Eyebrow001="eyebrow001",
        Teeth_Base="teeth",
        **{"Low-Poly": "green_eye"},
    )
    objects = []
    for g in tree.findall("library_geometries/geometry"):
        part = g.get("name").split(":")[-1]
        mesh = g.find("mesh")
        sources = {}
        for s in mesh.findall("source"):
            stride = int(s.find("technique_common/accessor").get("stride"))
            sources["#" + s.get("id")] = np.fromstring(s.findtext("float_array"), sep=" ").reshape(-1, stride)
        coords = sources[mesh.find("vertices/input").get("source")].copy()
        posed = pose_vertices(coords, part, pose)
        poly = mesh.find("polylist")
        inputs = {el.get("semantic"): el for el in poly.findall("input")}
        stride = max(int(el.get("offset")) for el in inputs.values()) + 1
        corners = np.fromstring(poly.findtext("p"), sep=" ", dtype=int).reshape(-1, stride)
        vi = corners[:, int(inputs["VERTEX"].get("offset"))]
        counts = np.fromstring(poly.findtext("vcount"), sep=" ", dtype=int)
        faces = np.split(vi, np.cumsum(counts)[:-1])
        data = bpy.data.meshes.new(part)
        # Body only contains exposed skin. Deck boots replace the two bare-foot islands.
        keep = (
            np.array([coords[f[0], 2] > 0.3 for f in faces]) if part == "Body" else np.ones(len(faces), bool)
        )
        if part == "jeans01":
            keep = np.array([coords[f, 2].max() > 0.20 for f in faces])
        used, indices = np.unique(
            np.concatenate([f for f, ok in zip(faces, keep) if ok]), return_inverse=True
        )
        faces = np.split(indices, np.cumsum(counts[keep])[:-1])
        data.from_pydata(posed[used].tolist(), [], [f.tolist() for f in faces])
        data.update()
        data.polygons.foreach_set("use_smooth", np.ones(len(faces), dtype=bool))
        uv = inputs["TEXCOORD"]
        uvs = sources[uv.get("source")][corners[:, int(uv.get("offset"))], :2]
        data.uv_layers.new(name="UVMap").data.foreach_set(
            "uv", uvs[np.repeat(keep, counts)].astype(np.float32).ravel()
        )
        mat = bpy.data.materials.new(col.name + " / " + part)
        mat.use_nodes = True
        shader = mat.node_tree.nodes.get("Principled BSDF")
        shader.inputs["Roughness"].default_value = 0.86
        img = bpy.data.images.load(
            str(source / "materials/textures" / (textures[part] + ".png")), check_existing=True
        )
        if max(img.size) > 768:
            w, h = img.size
            img.scale(round(w * 768 / max(w, h)), round(h * 768 / max(w, h)))
        if not img.packed_file:
            img.pack()
        tex = mat.node_tree.nodes.new("ShaderNodeTexImage")
        tex.image = img
        if part == "tshirt02":
            shader.inputs["Base Color"].default_value = (*POSES[pose][-1], 1)
        else:
            mat.node_tree.links.new(tex.outputs["Color"], shader.inputs["Base Color"])
        if part in ("mhair02", "Eyebrow001"):
            mat.node_tree.links.new(tex.outputs["Alpha"], shader.inputs["Alpha"])
        data.materials.append(mat)
        obj = bpy.data.objects.new(part, data)
        col.objects.link(obj)
        objects.append(obj)
    coords, faces, bands = work_boot_mesh()
    data = bpy.data.meshes.new("Work boots")
    data.from_pydata(pose_vertices(coords, "Work boots", pose).tolist(), [], faces)
    for name, rgb in [("Deck rubber", (0.036, 0.049, 0.045)), ("Boot welt", (0.10, 0.085, 0.051))]:
        mat = bpy.data.materials.get(name) or bpy.data.materials.new(name)
        mat.use_nodes = True
        shader = mat.node_tree.nodes.get("Principled BSDF")
        shader.inputs["Base Color"].default_value = (*rgb, 1)
        shader.inputs["Roughness"].default_value = 0.65
        mat.diffuse_color = (*rgb, 1)
        data.materials.append(mat)
    data.polygons.foreach_set("material_index", np.asarray(bands, dtype=np.int32))
    data.polygons.foreach_set("use_smooth", np.ones(len(faces), dtype=bool))
    boots = bpy.data.objects.new("Work boots", data)
    col.objects.link(boots)
    objects.append(boots)
    # The lowest point may be a hip, knee, shoulder or boot. The origin used by
    # the evaluator stays at floor level beneath the middle of the silhouette.
    all_coords = np.array([tuple(v.co) for obj in objects for v in obj.data.vertices])
    origin = (all_coords.min(axis=0) + all_coords.max(axis=0)) * 0.5
    origin[2] = all_coords[:, 2].min() - 0.01
    for obj in objects:
        obj.data.transform(Matrix.Translation(-origin))
    return objects
