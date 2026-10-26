#!/usr/bin/env python3
"""Generate the MuJoCo robot model (mjcf/robot.xml, meshes/robot/*.stl) from the BumperBot xacro.

The URDF is the single source of truth for kinematics and inertia. This script compiles it with
MuJoCo and then adds what URDF cannot express: free base joint, wheel actuators, IMU sensors,
contact parameters and geom groups.

Geom groups used by the simulation node:
  1 - robot visual geoms
  2 - world visual geoms
  3 - robot collision geoms (hidden, ignored by the lidar)
  4 - static world collision geoms (hidden, the only geoms hit by the lidar rays)

Usage: python3 tools/generate_robot.py
"""
import os
import sys
import tempfile
import types
import xml.etree.ElementTree as ET

import mujoco
import trimesh

PKG_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SRC_DIR = os.path.dirname(PKG_DIR)
DESCRIPTION_DIR = os.path.join(SRC_DIR, "bumperbot_description")
XACRO_PATH = os.path.join(DESCRIPTION_DIR, "urdf", "bumperbot.urdf.xacro")
MESH_OUT_DIR = os.path.join(PKG_DIR, "meshes", "robot")
ROBOT_XML = os.path.join(PKG_DIR, "mjcf", "robot.xml")

# MuJoCo refuses meshes with more than 200k faces; stay well below that
MAX_FACES = 8000

# Sensor noise taken from the Gazebo model so both simulators behave alike
IMU_GYRO_NOISE = 2e-4
IMU_ACCEL_NOISE = 1.7e-2

WHEEL_FRICTION = "1.5 0.005 0.0001"
# Stiffer wheel contact than the MuJoCo default (0.02 1): the soft default lets the wheels sink
# ~0.5 mm and shrinks the effective wheel radius (odometry heading scale error). Must stay >= 2 x timestep.
WHEEL_SOLREF = "0.004 1"
# Velocity servo gain [N*m/(rad/s)], torque limit [N*m] and speed limit [rad/s] of the wheel motors
WHEEL_KV = 0.2
WHEEL_TORQUE_LIMIT = 0.3
WHEEL_SPEED_LIMIT = 40.0

WHEEL_JOINTS = ("wheel_left_joint", "wheel_right_joint")


def install_ament_stub():
    """Let xacro resolve $(find pkg) when ROS is not sourced, using the packages of this workspace."""
    try:
        import ament_index_python  # noqa: F401
        return
    except ImportError:
        pass

    def get_package_share_directory(name):
        path = os.path.join(SRC_DIR, name)
        if not os.path.isdir(path):
            raise KeyError(name)
        return path

    pkg = types.ModuleType("ament_index_python")
    packages = types.ModuleType("ament_index_python.packages")
    packages.get_package_share_directory = get_package_share_directory
    pkg.packages = packages
    sys.modules["ament_index_python"] = pkg
    sys.modules["ament_index_python.packages"] = packages


def process_xacro():
    install_ament_stub()
    import xacro
    doc = xacro.process_file(XACRO_PATH, mappings={"simulator": "mujoco", "is_sim": "true"})
    return doc.toxml()


def convert_meshes():
    """Write decimated binary STLs for every robot mesh and return the mesh dir used by the URDF."""
    os.makedirs(MESH_OUT_DIR, exist_ok=True)
    src_dir = os.path.join(DESCRIPTION_DIR, "meshes")
    for name in sorted(os.listdir(src_dir)):
        if not name.lower().endswith(".stl"):
            continue
        mesh = trimesh.load(os.path.join(src_dir, name))
        if len(mesh.faces) > MAX_FACES:
            mesh = mesh.simplify_quadric_decimation(face_count=MAX_FACES)
        out_name = os.path.splitext(name)[0] + ".stl"
        mesh.export(os.path.join(MESH_OUT_DIR, out_name))


def compile_urdf(urdf):
    urdf = urdf.replace("package://bumperbot_description/meshes/", "").replace(".STL", ".stl")
    # fusestatic=false keeps the fixed-joint links (imu_link, laser_link, base_footprint) as bodies
    compiler = ('<mujoco><compiler meshdir="%s" fusestatic="false" discardvisual="false" '
                'balanceinertia="true"/></mujoco>' % MESH_OUT_DIR)
    urdf = urdf.replace('<robot name="bumperbot">', '<robot name="bumperbot">' + compiler, 1)
    with tempfile.TemporaryDirectory() as tmp:
        urdf_path = os.path.join(tmp, "bumperbot.urdf")
        with open(urdf_path, "w") as f:
            f.write(urdf)
        model = mujoco.MjModel.from_xml_path(urdf_path)
        xml_path = os.path.join(tmp, "bumperbot.xml")
        mujoco.mj_saveLastXML(xml_path, model)
        return ET.parse(xml_path).getroot()


def find_body(root, name):
    for body in root.iter("body"):
        if body.get("name") == name:
            return body
    raise KeyError(name)


def postprocess(root):
    # The compiler settings and meshdir belong to the including world file
    for compiler in root.findall("compiler"):
        root.remove(compiler)
    for mesh in root.iter("mesh"):
        mesh.attrib.pop("content_type", None)
        mesh.set("file", "robot/" + mesh.get("file"))

    # Geoms: visuals (contype 0) -> group 1, collision geoms -> group 3
    for body in root.iter("body"):
        for geom in body.findall("geom"):
            if geom.get("contype") == "0":
                geom.set("group", "1")
                geom.attrib.pop("density", None)
            else:
                geom.set("group", "3")
            geom.set("name", "%s_%s" % (body.get("name"),
                                         "visual" if geom.get("group") == "1" else "collision"))

    for name in ("wheel_left_link", "wheel_right_link"):
        geom = find_body(root, name).find("geom[@name='%s_collision']" % name)
        geom.set("friction", WHEEL_FRICTION)
        # priority 1 makes these contact parameters (friction, solref) win over the ground's
        geom.set("priority", "1")
        geom.set("solref", WHEEL_SOLREF)
    for name in ("caster_front_link", "caster_rear_link"):
        # Free-rolling casters: frictionless contact (condim 1), otherwise they drag the robot.
        # MuJoCo uses the geom with the higher priority for contact parameters, and on equal priority
        # the max condim; the ground has condim 3, so the caster needs priority 1 for condim 1 to win.
        geom = find_body(root, name).find("geom[@name='%s_collision']" % name)
        geom.set("condim", "1")
        geom.set("priority", "1")

    # The base is a free-floating body; start 1 mm above the ground to avoid initial penetration
    base = find_body(root, "base_footprint")
    base.set("pos", "0 0 0.001")
    base.insert(0, ET.Element("freejoint", {"name": "base_freejoint"}))

    # Sensor sites follow the URDF frames so TF frames and measurement frames agree
    ET.SubElement(find_body(root, "imu_link"), "site",
                  {"name": "imu_site", "size": "0.002"})
    ET.SubElement(find_body(root, "laser_link"), "site",
                  {"name": "laser_site", "size": "0.002"})

    actuator = ET.SubElement(root, "actuator")
    for joint in WHEEL_JOINTS:
        ET.SubElement(actuator, "velocity", {
            "name": joint + "_motor", "joint": joint, "kv": str(WHEEL_KV),
            "ctrllimited": "true", "ctrlrange": "%g %g" % (-WHEEL_SPEED_LIMIT, WHEEL_SPEED_LIMIT),
            "forcelimited": "true", "forcerange": "%g %g" % (-WHEEL_TORQUE_LIMIT, WHEEL_TORQUE_LIMIT)})

    # Noise attributes are not applied by the engine; the ROS node adds them from sensor_noise
    sensor = ET.SubElement(root, "sensor")
    ET.SubElement(sensor, "gyro", {"name": "imu_gyro", "site": "imu_site",
                                    "noise": str(IMU_GYRO_NOISE)})
    ET.SubElement(sensor, "accelerometer", {"name": "imu_accel", "site": "imu_site",
                                             "noise": str(IMU_ACCEL_NOISE)})
    ET.SubElement(sensor, "framequat", {"name": "imu_quat", "objtype": "site",
                                         "objname": "imu_site"})
    return root


def main():
    urdf = process_xacro()
    convert_meshes()
    root = postprocess(compile_urdf(urdf))
    root.attrib.pop("model", None)
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(ROBOT_XML, encoding="unicode")
    print("wrote", ROBOT_XML)


if __name__ == "__main__":
    main()
