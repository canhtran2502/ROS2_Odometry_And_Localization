#!/usr/bin/env python3
"""ROS 2 node that runs the MuJoCo simulation of BumperBot.

It replaces the Gazebo + ros_gz_bridge pair and provides the same interface to the rest of the stack:
  publishes   /clock, /scan, /imu/out, /mujoco/joint_states
  subscribes  /mujoco/joint_commands   (wheel velocity commands from topic_based_ros2_control)
"""
import math
import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState, LaserScan

from bumperbot_mujoco.sim_core import BumperbotSim, WHEEL_JOINTS

# Lidar and IMU parameters mirror the Gazebo sensor definitions
LASER_SAMPLES = 360
LASER_ANGLE_MIN = 0.0
LASER_ANGLE_MAX = 6.28
LASER_RANGE_MIN = 0.12
LASER_RANGE_MAX = 12.0
LASER_NOISE_STD = 0.01
LASER_RATE = 5.0
TICK_RATE = 100.0


def to_time_msg(seconds):
    sec = int(math.floor(seconds))
    return Time(sec=sec, nanosec=int((seconds - sec) * 1e9))


class MujocoSimNode(Node):
    def __init__(self):
        super().__init__("mujoco_sim")
        self.declare_parameter("world", "small_house")
        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("z", 0.001)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("viewer", True)

        world = self.get_parameter("world").value
        if not world.endswith(".xml"):
            world = os.path.join(get_package_share_directory("bumperbot_mujoco"), "mjcf", world + ".xml")
        self.sim = BumperbotSim(
            world,
            x=self.get_parameter("x").value,
            y=self.get_parameter("y").value,
            z=self.get_parameter("z").value,
            yaw=self.get_parameter("yaw").value)

        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self.imu_pub = self.create_publisher(Imu, "/imu/out", 10)
        self.joint_state_pub = self.create_publisher(JointState, "/mujoco/joint_states", 10)
        self.create_subscription(JointState, "/mujoco/joint_commands", self.command_callback, 10)

        self.viewer = None
        if self.get_parameter("viewer").value:
            self.start_viewer()

        self.tick_period = 1.0 / TICK_RATE
        self.scan_every = int(round(TICK_RATE / LASER_RATE))
        self.tick = 0
        self.create_timer(self.tick_period, self.on_timer)
        self.get_logger().info("MuJoCo simulation started with world %s" % world)

    def start_viewer(self):
        try:
            import mujoco.viewer
            self.viewer = mujoco.viewer.launch_passive(self.sim.model, self.sim.data)
        except Exception as error:  # no display available, keep running headless
            self.get_logger().warn("Viewer disabled: %s" % error)

    def command_callback(self, msg):
        velocities = dict(zip(msg.name, msg.velocity))
        if all(joint in velocities for joint in WHEEL_JOINTS):
            self.sim.set_wheel_velocities(velocities[WHEEL_JOINTS[0]], velocities[WHEEL_JOINTS[1]])

    def on_timer(self):
        self.sim.step(self.tick_period)
        stamp = to_time_msg(self.sim.time)
        self.clock_pub.publish(Clock(clock=stamp))
        self.publish_joint_states(stamp)
        self.publish_imu(stamp)
        if self.tick % self.scan_every == 0:
            self.publish_scan(stamp)
        self.tick += 1
        if self.viewer is not None:
            if self.viewer.is_running():
                self.viewer.sync()
            else:
                self.viewer = None

    def publish_joint_states(self, stamp):
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = list(WHEEL_JOINTS)
        msg.position = self.sim.wheel_positions()
        msg.velocity = self.sim.wheel_velocities()
        self.joint_state_pub.publish(msg)

    def publish_imu(self, stamp):
        orientation, gyro, accel = self.sim.imu()
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = "imu_link"
        msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z = orientation
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gyro
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = accel
        self.imu_pub.publish(msg)

    def publish_scan(self, stamp):
        increment = (LASER_ANGLE_MAX - LASER_ANGLE_MIN) / (LASER_SAMPLES - 1)
        msg = LaserScan()
        msg.header.stamp = stamp
        msg.header.frame_id = "laser_link"
        msg.angle_min = LASER_ANGLE_MIN
        msg.angle_max = LASER_ANGLE_MAX
        msg.angle_increment = increment
        msg.scan_time = 1.0 / LASER_RATE
        msg.range_min = LASER_RANGE_MIN
        msg.range_max = LASER_RANGE_MAX
        msg.ranges = self.sim.scan(LASER_ANGLE_MIN, increment, LASER_SAMPLES, LASER_RANGE_MIN,
                                   LASER_RANGE_MAX, LASER_NOISE_STD).astype(float).tolist()
        self.scan_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MujocoSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
