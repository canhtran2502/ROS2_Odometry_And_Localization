#!/usr/bin/env python3
"""Convert the Gazebo SDF worlds of bumperbot_description into MuJoCo MJCF worlds.

For every model used by a world this script:
  * converts the Collada collision meshes to a set of convex hulls (CoACD), because MuJoCo uses the
    convex hull of every mesh for contacts and cannot load .dae files;
  * converts the visual meshes to decimated binary STL, one mesh per material colour
    (textures are not carried over, the average texture colour is used instead).
All models are emitted as static bodies. Geom groups follow tools/generate_robot.py.

Requires: mujoco, trimesh, pycollada, coacd, fast-simplification, numpy.

Usage:
  python3 tools/sdf_to_mjcf.py                       # convert all worlds
  python3 tools/sdf_to_mjcf.py small_house empty     # convert selected worlds
"""
import argparse
import multiprocessing
import os
import re
import xml.etree.ElementTree as ET

import coacd
import numpy as np
import trimesh
from PIL import Image

PKG_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
DESCRIPTION_DIR = os.path.join(os.path.dirname(PKG_DIR), "bumperbot_description")
WORLD_SRC_DIR = os.path.join(DESCRIPTION_DIR, "worlds")
MODEL_SRC_DIR = os.path.join(DESCRIPTION_DIR, "models")
MESH_OUT_DIR = os.path.join(PKG_DIR, "meshes", "models")
MJCF_OUT_DIR = os.path.join(PKG_DIR, "mjcf")

# Face budget for the visual meshes of one model, shared between its materials
VISUAL_FACE_BUDGET = 6000
# Collision meshes are decimated to this many faces before the convex decomposition
COLLISION_FACE_LIMIT = 1500
MAX_HULLS = 8
# Floor safety net below the world, so the robot cannot fall forever when leaving the mapped area
SAFETY_FLOOR_Z = -0.1

# Hidden by default in the MuJoCo viewer; rays ignore fully transparent geoms, so a group is used
GROUP_COLLISION = 4
GROUP_VISUAL = 2

_model_cache = {}


def parse_pose(text):
    """SDF pose 'x y z roll pitch yaw' -> 4x4 matrix."""
    values = [float(v) for v in text.split()] if text and text.strip() else [0.0] * 6
    values += [0.0] * (6 - len(values))
    matrix = trimesh.transformations.euler_matrix(values[3], values[4], values[5], "sxyz")
    matrix[:3, 3] = values[:3]
    return matrix


def pose_of(element):
    pose = element.find("pose") if element is not None else None
    return parse_pose(pose.text if pose is not None else None)


def pos_quat(matrix):
    quat = trimesh.transformations.quaternion_from_matrix(matrix)  # (w, x, y, z)
    return matrix[:3, 3], quat


def fmt(values):
    return " ".join("%.6g" % v for v in values)


def load_dae(uri, scale):
    """Load a Collada mesh referenced as model://<model>/<path> and return a single scaled mesh list."""
    path = os.path.join(MODEL_SRC_DIR, uri[len("model://"):])
    scene = trimesh.load(path, force="scene")
    # Collada files of this repo are in centimetres; trimesh reports the unit factor in scene.units
    unit = 1.0
    if scene.units:
        unit = float(trimesh.units.unit_conversion(scene.units, "meters"))
    meshes = scene.dump()
    transform = np.diag(list(np.asarray(scale) * unit) + [1.0])
    for mesh in meshes:
        mesh.apply_transform(transform)
    return meshes


def texture_color(model_dir):
    """Average colour of the first texture image shipped with a model, or None."""
    texture_dir = os.path.join(model_dir, "materials", "textures")
    if not os.path.isdir(texture_dir):
        return None
    for name in sorted(os.listdir(texture_dir)):
        if name.lower().endswith((".png", ".jpg", ".jpeg")):
            image = Image.open(os.path.join(texture_dir, name)).convert("RGB").resize((8, 8))
            return tuple(np.asarray(image, dtype=float).reshape(-1, 3).mean(axis=0) / 255.0) + (1.0,)
    return None


def mesh_color(mesh, fallback=None):
    """Representative RGBA of a mesh: texture colour, material colour or vertex colour."""
    try:
        material = getattr(mesh.visual, "material", None)
        image = getattr(material, "image", None) or getattr(material, "baseColorTexture", None)
        if image is not None:
            small = np.asarray(image.convert("RGB").resize((8, 8)), dtype=float) / 255.0
            return tuple(small.reshape(-1, 3).mean(axis=0)) + (1.0,)
        color = tuple(np.asarray(mesh.visual.main_color, dtype=float) / 255.0)
        # Untextured white usually means the texture image could not be resolved by the loader
        if fallback is not None and min(color[:3]) > 0.97:
            return fallback
        return color
    except Exception:
        return fallback or (0.7, 0.7, 0.7, 1.0)


def decimate(mesh, faces):
    if len(mesh.faces) <= faces:
        return mesh
    return mesh.simplify_quadric_decimation(face_count=faces)


def clip_to_bounds(hull, bounds, margin=1e-3):
    """Clamp a hull to the bounding box of the source mesh (CoACD slightly inflates thin parts)."""
    vertices = np.clip(hull.vertices, bounds[0] - margin, bounds[1] + margin)
    try:
        return trimesh.Trimesh(vertices).convex_hull
    except Exception:  # flattened to a plane or a line: drop this hull
        return None


def is_nearly_convex(mesh, ratio=0.85):
    hull = mesh.convex_hull
    return hull.volume > 0.0 and abs(mesh.volume) / hull.volume >= ratio


def decompose(mesh, name):
    """Split a (possibly concave) mesh into convex hulls."""
    if is_nearly_convex(mesh):
        print("  %s: convex, 1 collision hull" % name)
        return [mesh.convex_hull]
    bounds = mesh.bounds
    mesh = decimate(mesh, COLLISION_FACE_LIMIT)
    coacd.set_log_level("error")
    # Reduced search effort: the world is static and only needs to be roughly right for the lidar
    parts = coacd.run_coacd(coacd.Mesh(mesh.vertices, mesh.faces), threshold=0.08,
                            max_convex_hull=MAX_HULLS, resolution=500, mcts_iterations=20,
                            mcts_max_depth=2, seed=0)
    hulls = []
    for vertices, faces in parts:
        hull = clip_to_bounds(trimesh.Trimesh(vertices, faces).convex_hull, bounds)
        if hull is not None and len(hull.faces) >= 4 and hull.volume > 1e-9:
            hulls.append(hull)
    if not hulls:
        hulls = [mesh.convex_hull]
    print("  %s: %d collision hull(s)" % (name, len(hulls)))
    return hulls


def convert_model(model_name):
    """Convert one SDF model; returns {'assets': [(mesh_name, file)], 'geoms': [xml attribute dicts]}."""
    if model_name in _model_cache:
        return _model_cache[model_name]
    short = re.sub(r"^aws_robomaker_", "", model_name)
    out_dir = os.path.join(MESH_OUT_DIR, short)
    os.makedirs(out_dir, exist_ok=True)
    root = ET.parse(os.path.join(MODEL_SRC_DIR, model_name, "model.sdf")).getroot()
    result = {"assets": [], "geoms": []}

    visual_parts = []   # (rgba, trimesh) over all links
    fallback_color = texture_color(os.path.join(MODEL_SRC_DIR, model_name))
    for link in root.iter("link"):
        link_pose = pose_of(link)

        for index, collision in enumerate(link.findall("collision")):
            pose = link_pose @ pose_of(collision)
            geometry = collision.find("geometry")
            if geometry.find("plane") is not None:
                result["geoms"].append({"type": "plane", "size": "50 50 0.1",
                                        "group": GROUP_COLLISION, "kind": "collision"})
                continue
            sphere = geometry.find("sphere")
            if sphere is not None:
                pos, quat = pos_quat(pose)
                result["geoms"].append({"type": "sphere", "size": sphere.findtext("radius").strip(),
                                        "pos": fmt(pos), "quat": fmt(quat),
                                        "group": GROUP_COLLISION, "kind": "collision"})
                continue
            mesh_element = geometry.find("mesh")
            scale = [float(v) for v in (mesh_element.findtext("scale") or "1 1 1").split()]
            meshes = load_dae(mesh_element.findtext("uri"), scale)
            whole = trimesh.util.concatenate(meshes)
            whole.apply_transform(pose)
            for hull_index, hull in enumerate(decompose(whole, "%s/%s" % (short, link.get("name")))):
                name = "%s_%s_c%d_%d" % (short, link.get("name"), index, hull_index)
                hull.export(os.path.join(out_dir, name + ".stl"))
                result["assets"].append((name, "models/%s/%s.stl" % (short, name)))
                result["geoms"].append({"type": "mesh", "mesh": name,
                                        "group": GROUP_COLLISION, "kind": "collision"})

        for visual in link.findall("visual"):
            mesh_element = visual.find("geometry/mesh")
            if mesh_element is None:
                continue
            pose = link_pose @ pose_of(visual)
            scale = [float(v) for v in (mesh_element.findtext("scale") or "1 1 1").split()]
            for mesh in load_dae(mesh_element.findtext("uri"), scale):
                mesh.apply_transform(pose)
                visual_parts.append((mesh_color(mesh, fallback_color), mesh))

    # Merge visual meshes of equal colour and share the face budget in proportion to their size
    by_color = {}
    for rgba, mesh in visual_parts:
        key = tuple(round(c, 2) for c in rgba)
        by_color.setdefault(key, []).append(mesh)
    total_faces = sum(len(m.faces) for _, m in visual_parts) or 1
    for index, (rgba, meshes) in enumerate(sorted(by_color.items())):
        merged = trimesh.util.concatenate(meshes)
        merged = trimesh.Trimesh(merged.vertices, merged.faces)
        budget = max(12, int(VISUAL_FACE_BUDGET * len(merged.faces) / total_faces))
        merged = decimate(merged, budget)
        name = "%s_v%d" % (short, index)
        merged.export(os.path.join(out_dir, name + ".stl"))
        result["assets"].append((name, "models/%s/%s.stl" % (short, name)))
        result["geoms"].append({"type": "mesh", "mesh": name, "group": GROUP_VISUAL,
                                "kind": "visual", "rgba": fmt(rgba)})

    _model_cache[model_name] = result
    return result


def world_instances(world):
    """Yield (instance_name, model_name, pose_matrix) for every model instance of an SDF world."""
    for element in world:
        if element.tag == "model":
            include = element.find("include")
            if include is None:
                continue
            yield element.get("name"), include.findtext("uri"), pose_of(element)
        elif element.tag == "include":
            yield element.findtext("name") or element.findtext("uri"), element.findtext("uri"), pose_of(element)
    # Models defined inline (e.g. the ground plane of empty.world) are handled by inline_models()


def inline_models(world):
    for element in world.findall("model"):
        if element.find("include") is None:
            yield element


def convert_inline_plane(model):
    for collision in model.iter("collision"):
        if collision.find("geometry/plane") is not None:
            return True
    return False


def write_world(world_name):
    print("World:", world_name)
    tree = ET.parse(os.path.join(WORLD_SRC_DIR, world_name + ".world"))
    world = tree.getroot().find("world")

    mujoco = ET.Element("mujoco", {"model": world_name})
    ET.SubElement(mujoco, "compiler", {"angle": "radian", "meshdir": "../meshes"})
    ET.SubElement(mujoco, "option", {"timestep": "0.002", "integrator": "implicitfast",
                                      "gravity": "0 0 -9.81"})
    visual = ET.SubElement(mujoco, "visual")
    ET.SubElement(visual, "headlight", {"ambient": "0.4 0.4 0.4", "diffuse": "0.6 0.6 0.6"})
    ET.SubElement(visual, "global", {"offwidth": "1280", "offheight": "720"})
    ET.SubElement(mujoco, "statistic", {"center": "0 0 1", "extent": "12"})
    asset = ET.SubElement(mujoco, "asset")
    ET.SubElement(asset, "texture", {"name": "skybox", "type": "skybox", "builtin": "gradient",
                                      "rgb1": "0.45 0.6 0.8", "rgb2": "0.05 0.05 0.1",
                                      "width": "512", "height": "3072"})
    ET.SubElement(asset, "texture", {"name": "grid", "type": "2d", "builtin": "checker",
                                      "rgb1": "0.75 0.75 0.75", "rgb2": "0.65 0.65 0.65",
                                      "width": "300", "height": "300"})
    ET.SubElement(asset, "material", {"name": "grid", "texture": "grid", "texrepeat": "50 50",
                                       "reflectance": "0"})
    worldbody = ET.SubElement(mujoco, "worldbody")
    ET.SubElement(worldbody, "light", {"name": "sun", "directional": "true", "pos": "0 0 10",
                                        "dir": "-0.5 0.1 -0.9", "diffuse": "0.7 0.7 0.7"})

    has_plane = any(convert_inline_plane(m) for m in inline_models(world))
    if has_plane:
        ET.SubElement(worldbody, "geom", {"name": "ground_plane", "type": "plane", "size": "50 50 0.1",
                                           "material": "grid", "friction": "1 0.005 0.0001"})
    else:
        ET.SubElement(worldbody, "geom", {"name": "safety_floor", "type": "plane", "size": "50 50 0.1",
                                           "pos": "0 0 %g" % SAFETY_FLOOR_Z, "rgba": "0 0 0 0",
                                           "friction": "1 0.005 0.0001"})

    assets = {}
    for instance, uri, pose in world_instances(world):
        model_name = uri[len("model://"):]
        model = convert_model(model_name)
        pos, quat = pos_quat(pose)
        body = ET.SubElement(worldbody, "body", {"name": re.sub(r"\W", "_", instance),
                                                  "pos": fmt(pos), "quat": fmt(quat)})
        for geom in model["geoms"]:
            attrs = {"type": geom["type"], "group": str(geom["group"])}
            for key in ("mesh", "size", "pos", "quat"):
                if key in geom:
                    attrs[key] = geom[key]
            if geom["kind"] == "collision":
                attrs["rgba"] = "0.5 0.5 0.5 1"
                attrs["friction"] = "1 0.005 0.0001"
            else:
                attrs.update({"contype": "0", "conaffinity": "0", "rgba": geom["rgba"]})
            ET.SubElement(body, "geom", attrs)
        assets.update(dict(model["assets"]))

    for name, file in sorted(assets.items()):
        attrs = {"name": name, "file": file}
        if re.search(r"_v\d+$", name):
            # Visual meshes can be flat (floors, carpets); a volume based inertia would be invalid
            attrs["inertia"] = "shell"
        ET.SubElement(asset, "mesh", attrs)

    ET.SubElement(mujoco, "include", {"file": "robot.xml"})
    ET.indent(mujoco, space="  ")
    out = os.path.join(MJCF_OUT_DIR, world_name + ".xml")
    ET.ElementTree(mujoco).write(out, encoding="unicode")
    print("wrote", out, "(%d meshes)" % len(assets))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("worlds", nargs="*", default=["empty", "small_house", "small_warehouse"])
    parser.add_argument("-j", "--jobs", type=int, default=os.cpu_count(), help="parallel model conversions")
    args = parser.parse_args()
    # Convert the unique models of all requested worlds in parallel, then write the world files
    names = set()
    for world_name in args.worlds:
        world = ET.parse(os.path.join(WORLD_SRC_DIR, world_name + ".world")).getroot().find("world")
        names.update(uri[len("model://"):] for _, uri, _ in world_instances(world))
    with multiprocessing.Pool(args.jobs) as pool:
        for name, result in zip(sorted(names), pool.map(convert_model, sorted(names))):
            _model_cache[name] = result
    for world_name in args.worlds:
        write_world(world_name)


if __name__ == "__main__":
    main()
