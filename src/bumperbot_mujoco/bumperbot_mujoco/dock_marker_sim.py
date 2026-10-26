#!/usr/bin/env python3
"""SIMULATION ONLY: emulated dock marker detector for the MuJoCo simulation.

It emulates a fiducial-style detector that looks at the charger. It reads the simulator ground
truth (/mujoco/ground_truth), applies range and field-of-view gating plus seeded Gaussian noise
and publishes the charger pose in base_footprint on /dock/detected_pose. On the real robot a
marker or laser dock detector publishes the same message and this node is not started.
"""
import math
import random
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

# Below this distance the dock bearing is undefined and the dock counts as visible
MIN_BEARING_RANGE = 0.05


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


def normalize_angle(angle):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def _as_pose(value):
    if isinstance(value, (tuple, list)):
        return Pose2D(float(value[0]), float(value[1]), float(value[2]))
    return Pose2D(float(value.x), float(value.y), float(value.yaw))


def emulate_detection(gt_pose, dock_pose, max_range, fov, facing="rear"):
    """Return the charger pose expressed in base_footprint, or None when it is not visible.

    gt_pose and dock_pose are (x, y, yaw) tuples or objects with x, y, yaw, both in the world
    frame. The dock is visible when its distance is <= max_range and its bearing lies within
    +-fov/2 of the detector axis (rear: opposite to the robot heading, front: along it).
    """
    gt = _as_pose(gt_pose)
    dock = _as_pose(dock_pose)
    dx = dock.x - gt.x
    dy = dock.y - gt.y
    c = math.cos(gt.yaw)
    s = math.sin(gt.yaw)
    rel = Pose2D(c * dx + s * dy, -s * dx + c * dy, normalize_angle(dock.yaw - gt.yaw))
    distance = math.hypot(rel.x, rel.y)
    if distance > max_range:
        return None
    if distance >= MIN_BEARING_RANGE:
        bearing = math.atan2(rel.y, rel.x)
        if facing == "rear":
            bearing = normalize_angle(bearing - math.pi)
        if abs(bearing) > fov / 2.0:
            return None
    return rel


def add_noise(pose, rng, position_noise, yaw_noise):
    """Return a copy of the pose with Gaussian noise (std in m and rad) from the given RNG."""
    return Pose2D(
        pose.x + rng.gauss(0.0, position_noise),
        pose.y + rng.gauss(0.0, position_noise),
        normalize_angle(pose.yaw + rng.gauss(0.0, yaw_noise)))


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class DockMarkerSim(Node):
    def __init__(self):
        super().__init__("dock_marker_sim")
        self.declare_parameter("dock_x", -0.5)
        self.declare_parameter("dock_y", 0.0)
        self.declare_parameter("dock_yaw", 0.0)
        self.declare_parameter("max_range", 2.0)
        self.declare_parameter("fov", 2.1)
        self.declare_parameter("facing", "rear")
        self.declare_parameter("position_noise", 0.005)
        self.declare_parameter("yaw_noise", 0.01)
        self.declare_parameter("seed", 7)
        self.declare_parameter("rate", 10.0)
        self.declare_parameter("ground_truth_topic", "/mujoco/ground_truth")
        self.declare_parameter("detection_topic", "/dock/detected_pose")
        self.declare_parameter("detection_frame", "base_footprint")

        self.dock_pose = Pose2D(
            self.get_parameter("dock_x").value,
            self.get_parameter("dock_y").value,
            self.get_parameter("dock_yaw").value)
        self.max_range = self.get_parameter("max_range").value
        self.fov = self.get_parameter("fov").value
        self.facing = self.get_parameter("facing").value
        if self.facing not in ("rear", "front"):
            raise ValueError("facing must be 'rear' or 'front', got '%s'" % self.facing)
        self.position_noise = self.get_parameter("position_noise").value
        self.yaw_noise = self.get_parameter("yaw_noise").value
        self.rng = random.Random(self.get_parameter("seed").value)
        self.frame = self.get_parameter("detection_frame").value

        self.latest = None
        self.pub = self.create_publisher(
            PoseStamped, self.get_parameter("detection_topic").value, 10)
        self.create_subscription(
            Odometry, self.get_parameter("ground_truth_topic").value, self.on_ground_truth, 10)
        self.create_timer(1.0 / self.get_parameter("rate").value, self.on_timer)
        self.get_logger().info("Dock marker emulator started (simulation only)")

    def on_ground_truth(self, msg):
        pose = msg.pose.pose
        self.latest = (msg.header.stamp,
                       (pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation)))

    def on_timer(self):
        if self.latest is None:
            return
        stamp, gt = self.latest
        rel = emulate_detection(gt, self.dock_pose, self.max_range, self.fov, self.facing)
        if rel is None:
            return
        rel = add_noise(rel, self.rng, self.position_noise, self.yaw_noise)
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame
        msg.pose.position.x = rel.x
        msg.pose.position.y = rel.y
        msg.pose.orientation.z = math.sin(rel.yaw / 2.0)
        msg.pose.orientation.w = math.cos(rel.yaw / 2.0)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DockMarkerSim()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        # A signal shuts the context down while spinning: this is a normal exit
        pass
    try:
        node.destroy_node()
    except RCLError:
        pass
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except RCLError:
        pass


if __name__ == "__main__":
    main()
