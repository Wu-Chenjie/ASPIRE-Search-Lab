"""A stepped architectural section in its own Blender scene, from the full-scale building."""

import bmesh
import bpy
from mathutils import Vector


def clipped(source, planes, remove):
    """Split solid faces at the actual section planes before trimming the near side."""
    mesh = bmesh.new()
    mesh.from_mesh(source.data)
    mesh.transform(source.matrix_world)
    for axis, offset in planes:
        origin = Vector((0, 0, 0))
        normal = Vector((0, 0, 0))
        origin[axis], normal[axis] = offset, 1
        bmesh.ops.bisect_plane(
            mesh, geom=[*mesh.verts, *mesh.edges, *mesh.faces], plane_co=origin, plane_no=normal, dist=1e-5
        )
    discard = [face for face in mesh.faces if remove(face.calc_center_median())]
    bmesh.ops.delete(mesh, geom=discard, context="FACES")
    for axis, offset in planes:
        edges = [
            edge
            for edge in mesh.edges
            if edge.is_boundary and all(abs(v.co[axis] - offset) < 0.0002 for v in edge.verts)
        ]
        if edges:
            bmesh.ops.holes_fill(mesh, edges=edges, sides=0)
    data = source.data.copy()
    mesh.to_mesh(data)
    mesh.free()
    return bpy.data.objects.new(source.name + " • A–A", data)


def make_section(full, camera):
    section = full.copy()
    section.use_fake_user = True
    section.name = "A–A  /  building section"
    for col in list(section.collection.children):
        section.collection.children.unlink(col)
    dst = section.collection.objects
    stepped = (
        "Raft slab",
        "North elevation",
        "North gallery and east return",
        "Subgrade L corridor",
        "Plate-ice machine",
        "Ice bins and ticket office awning",
    )
    notch = [(0, 4.9), (0, 9.12), (1, -10.42), (1, 10.1)]
    for ob in full.objects:
        if ob.type != "MESH":
            dst.link(ob)
            continue
        if ob.users_collection[0].name.startswith(("03 ", "03b")):
            continue
        if ob.name.startswith(("Riveted bent at -", "Draped electric conduit", "Buckled roof lattice")):
            continue
        if ob.name.startswith("Service roofs"):
            dst.link(clipped(ob, [(1, 5.3)], lambda p: p.y < 5.3))
        elif ob.name.startswith(stepped):
            dst.link(clipped(ob, notch, lambda p: p.y < -10.42 or (4.9 < p.x < 9.12 and p.y < 10.1)))
        else:
            center = sum((ob.matrix_world @ Vector(c) for c in ob.bound_box), Vector()) / 8
            if 4.9 < center.x < 9.12 and center.y < 7.95 and not ob.name.startswith("Cross-hall bridge"):
                continue
            dst.link(ob)
    # The section drawing uses even daylight so the three cut levels share
    # a legible exposure. Interior cameras keep the physical sky of the hall.
    section.world = full.world.copy()
    bg = section.world.node_tree.nodes.get("Background")
    for wire in list(bg.inputs[0].links):
        section.world.node_tree.links.remove(wire)
    bg.inputs[0].default_value = (0.91, 0.84, 0.71, 1)
    bg.inputs[1].default_value = 0.83
    section.camera = camera
    section.render.resolution_x, section.render.resolution_y = 1720, 900
    return section
