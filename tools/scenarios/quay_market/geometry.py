"""Blender mesh primitives shared by the market's architecture and fittings."""

import math

import bpy
import numpy as np
from mathutils import Euler, Vector

# Display colours are specified in sRGB like paint swatches.
SWATCHES = {
    "limewash": "c7bca1",
    "concrete": "a39d90",
    "spall": "8b8170",
    "oxide": "964c32",
    "iron": "435959",
    "coal": "303c3c",
    "steel": "9caeab",
    "tile": "839e92",
    "ceramic": "ced4c3",
    "teal": "276c67",
    "deep": "234e50",
    "ochre": "e0aa55",
    "ink": "263c38",
    "ivory": "efe6ca",
    "umber": "775642",
    "wood": "ad865a",
    "clay": "ae6446",
    "slate": "637982",
    "ice": "c3d8d4",
    "fish": "657e83",
    "belly": "bfccc2",
    "copper": "a76847",
    "water": "586f69",
    "beacon": "ff9b31",
}
WEATHERED = {"limewash", "concrete", "spall", "oxide", "iron", "tile", "wood", "clay"}
MATERIALS = {}
COLLISION = []
SHELL = []


def linear(c):
    return ((c + 0.055) / 1.055) ** 2.4 if c > 0.04045 else c / 12.92


def materials():
    rng = np.random.default_rng(8407)
    size = 256

    # Seamless low-frequency stains, aggregate and pinholes in the same bitmap
    # in Blender and Gazebo. UVs are in metres.
    def noise(grid):
        knots = rng.normal(size=(grid, grid))
        f = np.arange(size) * grid / size
        i = f.astype(int)
        w = f - i
        w = w * w * (3 - 2 * w)
        rows = knots[:, i] * (1 - w) + knots[:, (i + 1) % grid] * w
        return rows[i] * (1 - w[:, None]) + rows[(i + 1) % grid] * w[:, None]

    for name, colour in SWATCHES.items():
        rgb = np.array([int(colour[k : k + 2], 16) / 255 for k in (0, 2, 4)])
        mat = bpy.data.materials.new(name)
        mat.use_nodes = True
        shader = mat.node_tree.nodes.get("Principled BSDF")
        rgba = (*map(linear, rgb), 1)
        mat.diffuse_color = rgba
        shader.inputs["Base Color"].default_value = rgba
        if name == "beacon":
            shader.inputs["Emission Color"].default_value = rgba
            shader.inputs["Emission Strength"].default_value = 4.0
        metallic = name in {"steel", "iron", "copper", "fish"}
        shader.inputs["Metallic"].default_value = 0.52 if metallic else 0.04
        shader.inputs["Roughness"].default_value = 0.2 if name == "water" else 0.43 if metallic else 0.79
        if name in WEATHERED:
            field = (
                0.085 * noise(4) + 0.052 * noise(13) + 0.03 * noise(40) + rng.normal(0, 0.029, (size, size))
            )
            if name in {"oxide", "iron"}:
                field -= 0.28 * (noise(26) > 1.2)
            pixels = np.ones((size, size, 4), dtype=np.float32)
            pixels[:, :, :3] = np.clip(rgb * (1 + field[:, :, None]), 0, 1)
            img = bpy.data.images.new(name + " surface", width=size, height=size)
            img.pixels.foreach_set(pixels.ravel())
            img.pack()
            nodes = mat.node_tree.nodes
            tex = nodes.new("ShaderNodeTexImage")
            tex.image = img
            mat.node_tree.links.new(tex.outputs["Color"], shader.inputs["Base Color"])
        MATERIALS[name] = mat


def collection(name):
    col = bpy.data.collections.new(name)
    bpy.context.scene.collection.children.link(col)
    return col


class Part:
    """One editable assembly. Solid faces also supply the static collision mesh."""

    def __init__(self, name, col):
        self.name, self.col = name, col
        self.vertices, self.faces, self.colours = [], [], []

    def poly(self, points, faces, mat, solid=True):
        points = [tuple(v) for v in points]
        offset = len(self.vertices)
        self.vertices.extend(points)
        for face in faces:
            self.faces.append(tuple(offset + i for i in face))
            self.colours.append(mat)
            if solid:
                for k in range(1, len(face) - 1):
                    COLLISION.append((points[face[0]], points[face[k]], points[face[k + 1]]))

    def box(self, center, size, mat, rot=(0, 0, 0), solid=True):
        x, y, z = (s / 2 for s in size)
        pts = [
            (-x, -y, -z),
            (x, -y, -z),
            (x, y, -z),
            (-x, y, -z),
            (-x, -y, z),
            (x, -y, z),
            (x, y, z),
            (-x, y, z),
        ]
        m = Euler(rot).to_matrix()
        self.poly(
            [Vector(center) + m @ Vector(p) for p in pts],
            [(3, 2, 1, 0), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)],
            mat,
            solid,
        )

    def rod(self, a, b, r, mat, sides=8, solid=True, end_radius=None):
        a, b = Vector(a), Vector(b)
        axis = (b - a).normalized()
        u = axis.cross(Vector((1, 0, 0) if abs(axis.x) < 0.8 else (0, 1, 0))).normalized()
        v = axis.cross(u)
        ring = [
            u * math.cos(2 * math.pi * i / sides) + v * math.sin(2 * math.pi * i / sides)
            for i in range(sides)
        ]
        pts = [a + r * d for d in ring] + [b + (r if end_radius is None else end_radius) * d for d in ring]
        faces = [tuple(reversed(range(sides))), tuple(range(sides, 2 * sides))]
        faces += [(i, (i + 1) % sides, (i + 1) % sides + sides, i + sides) for i in range(sides)]
        self.poly(pts, faces, mat, solid)

    def beam(self, a, b, width, depth, mat, solid=True):
        a, b = Vector(a), Vector(b)
        rot = (b - a).to_track_quat("Z", "Y").to_euler()
        self.box((a + b) / 2, (width, depth, (b - a).length), mat, rot, solid)

    def finish(self):
        mesh = bpy.data.meshes.new(self.name)
        mesh.from_pydata(self.vertices, [], self.faces)
        mesh.update()
        slots = list(dict.fromkeys(self.colours))
        for key in slots:
            mesh.materials.append(MATERIALS[key])
        mesh.polygons.foreach_set("material_index", [slots.index(key) for key in self.colours])
        uv = mesh.uv_layers.new(name="Metre projection")
        coords = []
        for poly in mesh.polygons:
            n = poly.normal
            # Normal-dominant planar projection also works on inclined slabs.
            axes = (1, 2) if abs(n.x) > max(abs(n.y), abs(n.z)) else (0, 2) if abs(n.y) > abs(n.z) else (0, 1)
            for i in poly.vertices:
                p = self.vertices[i]
                coords.extend((p[axes[0]] * 0.65, p[axes[1]] * 0.65))
        uv.data.foreach_set("uv", coords)
        obj = bpy.data.objects.new(self.name, mesh)
        self.col.objects.link(obj)
        SHELL.append(obj)
        return obj


def lettering(col, text, pos, size, material="ivory", rot=(math.pi / 2, 0, 0)):
    curve = bpy.data.curves.new(text, "FONT")
    curve.body, curve.size, curve.align_x, curve.align_y = text, size, "CENTER", "CENTER"
    curve.extrude = 0.002
    curve.materials.append(MATERIALS[material])
    obj = bpy.data.objects.new(text, curve)
    col.objects.link(obj)
    obj.location, obj.rotation_euler = pos, rot
    # Store actual letters in the exchange mesh as well as in the .blend.
    deps = bpy.context.evaluated_depsgraph_get()
    mesh = bpy.data.meshes.new_from_object(obj.evaluated_get(deps))
    letter_mesh = bpy.data.objects.new(text, mesh)
    col.objects.link(letter_mesh)
    letter_mesh.matrix_world = obj.matrix_world.copy()
    bpy.data.objects.remove(obj, do_unlink=True)
    SHELL.append(letter_mesh)
    return letter_mesh
