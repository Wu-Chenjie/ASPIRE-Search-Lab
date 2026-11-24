"""Build stationary market occupants from the repository's locked MakeHuman mesh."""

import xml.etree.ElementTree as ET

import bpy
import numpy as np


def actor(repo, crouching, col):
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
        if crouching:
            # Flex the ankle and knee; move the torso down and lean forward at the hip.
            z = coords[:, 2].copy()
            coords[:, 1] += np.interp(z, [0, 0.49, 1.0, 1.9], [0, -0.34, 0.03, -0.24])
            coords[:, 2] = np.interp(z, [-0.1, 0, 0.49, 1.0, 2.1], [-0.1, 0, 0.36, 0.62, 1.72])
        coords[:, 2] += 0.008
        poly = mesh.find("polylist")
        inputs = {el.get("semantic"): el for el in poly.findall("input")}
        stride = max(int(el.get("offset")) for el in inputs.values()) + 1
        corners = np.fromstring(poly.findtext("p"), sep=" ", dtype=int).reshape(-1, stride)
        vi = corners[:, int(inputs["VERTEX"].get("offset"))]
        counts = np.fromstring(poly.findtext("vcount"), sep=" ", dtype=int)
        faces = np.split(vi, np.cumsum(counts)[:-1])
        data = bpy.data.meshes.new(part)
        data.from_pydata(coords.tolist(), [], [f.tolist() for f in faces])
        data.update()
        data.polygons.foreach_set("use_smooth", np.ones(len(faces), dtype=bool))
        uv = inputs["TEXCOORD"]
        uvs = sources[uv.get("source")][corners[:, int(uv.get("offset"))], :2]
        data.uv_layers.new(name="UVMap").data.foreach_set("uv", uvs.astype(np.float32).ravel())
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
        if part == "tshirt02" and crouching:
            shader.inputs["Base Color"].default_value = (0.22, 0.087, 0.040, 1)
        else:
            mat.node_tree.links.new(tex.outputs["Color"], shader.inputs["Base Color"])
        if part in ("mhair02", "Eyebrow001"):
            mat.node_tree.links.new(tex.outputs["Alpha"], shader.inputs["Alpha"])
        data.materials.append(mat)
        obj = bpy.data.objects.new(part, data)
        col.objects.link(obj)
        objects.append(obj)
    return objects
