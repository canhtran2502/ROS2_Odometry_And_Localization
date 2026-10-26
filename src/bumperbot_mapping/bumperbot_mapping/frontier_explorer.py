#!/usr/bin/env python3
"""
Frontier exploration node (thin rclpy wrapper around ``frontier_exploration.ExplorerCore``).

Reads /map (slam_toolbox), the robot pose from TF and /scan, drives the robot to the best
reachable frontier and publishes Twist on ``explore_vel`` (a low-priority twist_mux input).
When no reachable frontier is left, it stops, saves the map and reports on /explore/status.

Python-only on purpose: the logic lives in a pure Python module shared with the headless
tests; a C++ port would be a separate feature that takes that module as its reference.
"""
import math
import os

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.srv import SaveMap as Nav2SaveMap
from slam_toolbox.srv import SaveMap as SlamSaveMap
from tf2_ros import Buffer, TransformListener, TransformException

from bumperbot_mapping import frontier_exploration as fe

# Parameters that are forwarded unchanged to ExplorerCore (name -> default).
CORE_PARAMS = fe.DEFAULT_PARAMS


class FrontierExplorer(Node):

    def __init__(self):
        super().__init__("frontier_explorer")

        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("cmd_vel_topic", "explore_vel")
        self.declare_parameter("control_rate", 10.0)
        for name, default in CORE_PARAMS.items():
            self.declare_parameter(name, default)
        self.declare_parameter("save_map_on_complete", True)
        self.declare_parameter("map_save_path", "~/bumperbot_maps/explored_map")
        self.declare_parameter("save_map_timeout", 5.0)
        self.declare_parameter("shutdown_on_complete", False)

        self.global_frame = self.get_parameter("global_frame").value
        self.robot_frame = self.get_parameter("robot_frame").value
        self.save_on_complete = self.get_parameter("save_map_on_complete").value
        self.save_timeout = float(self.get_parameter("save_map_timeout").value)
        self.shutdown_on_complete = self.get_parameter("shutdown_on_complete").value
        self.map_save_path = os.path.expanduser(self.get_parameter("map_save_path").value)

        core_params = {name: self.get_parameter(name).value for name in CORE_PARAMS}
        self.core = fe.ExplorerCore(core_params)

        self.scan = None
        self.laser_offset_known = False
        self.map_received = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        status_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, "map", self.map_callback, map_qos)
        self.create_subscription(LaserScan, "scan", self.scan_callback, qos_profile_sensor_data)

        self.vel_pub = self.create_publisher(
            Twist, self.get_parameter("cmd_vel_topic").value, 10)
        self.status_pub = self.create_publisher(String, "/explore/status", status_qos)
        self.path_pub = self.create_publisher(Path, "/explore/path", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/explore/frontiers", 10)

        self.nav2_saver = self.create_client(Nav2SaveMap, "/map_saver/save_map")
        self.slam_saver = self.create_client(SlamSaveMap, "/slam_toolbox/save_map")

        # Completion / saving sequence: None -> ZEROING -> SAVING -> DONE
        self.finish_stage = None
        self.finish_t = 0.0
        self.save_future = None
        self.save_deadline = 0.0
        self.using_fallback = False

        self.status = None
        self.publish_status("WAITING_FOR_MAP")

        period = 1.0 / float(self.get_parameter("control_rate").value)
        self.timer = self.create_timer(period, self.control_loop)
        self.get_logger().info("Frontier explorer started, waiting for /map")

    # -- helpers ----------------------------------------------------------
    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def publish_status(self, status):
        if status == self.status:
            return
        self.status = status
        self.status_pub.publish(String(data=status))

    def publish_zero(self):
        self.vel_pub.publish(Twist())

    # -- callbacks --------------------------------------------------------
    def map_callback(self, msg: OccupancyGrid):
        if msg.info.width == 0 or msg.info.height == 0:
            return
        grid = fe.grid_from_msg_data(msg.data, msg.info.width, msg.info.height)
        info = fe.GridInfo(msg.info.resolution, msg.info.origin.position.x,
                           msg.info.origin.position.y, msg.info.width, msg.info.height)
        self.core.update_map(grid, info, self.now_sec())
        self.map_received = True

    def scan_callback(self, msg: LaserScan):
        ranges = np.asarray(msg.ranges, dtype=float)
        self.scan = fe.ScanData(ranges, msg.angle_min, msg.angle_increment)
        if not self.laser_offset_known:
            self.lookup_laser_offset(msg.header.frame_id)

    def lookup_laser_offset(self, laser_frame):
        """Yaw of the laser in the base frame. The lidar is mounted backwards (yaw ~ pi)."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.robot_frame, laser_frame, Time())
        except TransformException:
            return
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.core.laser_yaw_offset = yaw
        self.laser_offset_known = True
        self.get_logger().info(f"Laser yaw offset in {self.robot_frame}: {yaw:.3f} rad")

    def get_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame, self.robot_frame, Time())
        except TransformException as ex:
            self.get_logger().warn(f"TF {self.global_frame} -> {self.robot_frame} not "
                                   f"available: {ex}", throttle_duration_sec=5.0)
            return None
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (tf.transform.translation.x, tf.transform.translation.y, yaw)

    # -- control loop -----------------------------------------------------
    def control_loop(self):
        t = self.now_sec()
        if self.finish_stage is not None:
            self.finish_loop(t)
            return

        pose = self.get_pose()
        if pose is None or not self.laser_offset_known:
            if not self.laser_offset_known and self.scan is not None:
                self.lookup_laser_offset("laser_link")
            self.publish_zero()
            return

        cmd = self.core.step(pose, self.scan, t)
        self.publish_visuals(cmd)

        if cmd.state == fe.COMPLETE:
            self.publish_zero()
            self.start_finish(t, cmd.timed_out)
            return
        if cmd.state in (fe.INITIAL_SPIN, fe.EXPLORING):
            self.publish_status("EXPLORING")
        twist = Twist()
        twist.linear.x = float(cmd.v)
        twist.angular.z = float(cmd.w)
        self.vel_pub.publish(twist)

    # -- completion -------------------------------------------------------
    def start_finish(self, t, timed_out):
        reason = "time limit reached" if timed_out else "no reachable frontier left"
        self.get_logger().info(f"Exploration complete ({reason})")
        self.publish_status("COMPLETE")
        self.finish_stage = "ZEROING"
        self.finish_t = t

    def finish_loop(self, t):
        if self.finish_stage == "ZEROING":
            self.publish_zero()
            if t - self.finish_t >= 1.0:
                if self.save_on_complete:
                    self.begin_save(t)
                else:
                    self.finish_stage = "DONE"
        elif self.finish_stage == "SAVING":
            self.poll_save(t)
        elif self.finish_stage == "DONE":
            self.timer.cancel()
            if self.shutdown_on_complete:
                self.get_logger().info("shutdown_on_complete is set, shutting down")
                rclpy.shutdown()

    def begin_save(self, t):
        directory = os.path.dirname(self.map_save_path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        self.finish_stage = "SAVING"
        self.save_deadline = t + self.save_timeout
        self.save_future = None
        self.using_fallback = False
        self.get_logger().info(f"Saving map to {self.map_save_path}")

    def poll_save(self, t):
        if self.save_future is None:
            client = self.slam_saver if self.using_fallback else self.nav2_saver
            if client.service_is_ready():
                if self.using_fallback:
                    req = SlamSaveMap.Request()
                    req.name = String(data=self.map_save_path)
                else:
                    req = Nav2SaveMap.Request()
                    req.map_topic = "map"
                    req.map_url = self.map_save_path
                    req.image_format = "pgm"
                    req.map_mode = "trinary"
                    req.free_thresh = 0.196
                    req.occupied_thresh = 0.65
                self.save_future = client.call_async(req)
                self.save_deadline = t + self.save_timeout
                return
            if t > self.save_deadline:
                self.save_failed_or_fallback(t, "service not available")
            return
        if self.save_future.done():
            try:
                res = self.save_future.result()
            except Exception as ex:  # noqa: BLE001
                self.save_failed_or_fallback(t, f"service call failed: {ex}")
                return
            ok = bool(res.result) if not self.using_fallback else res.result == 0
            if ok:
                self.get_logger().info(f"Map saved to {self.map_save_path}")
                self.publish_status("MAP_SAVED")
                self.finish_stage = "DONE"
            else:
                self.save_failed_or_fallback(t, "service reported failure")
        elif t > self.save_deadline:
            self.save_failed_or_fallback(t, "service call timed out")

    def save_failed_or_fallback(self, t, reason):
        if not self.using_fallback:
            self.get_logger().warn(f"nav2 map_saver failed ({reason}), "
                                   "falling back to slam_toolbox/save_map")
            self.using_fallback = True
            self.save_future = None
            self.save_deadline = t + self.save_timeout
        else:
            self.get_logger().error(f"Map saving failed ({reason})")
            self.publish_status("MAP_SAVE_FAILED")
            self.finish_stage = "DONE"

    # -- visualization ----------------------------------------------------
    def publish_visuals(self, cmd):
        stamp = self.get_clock().now().to_msg()
        path = Path()
        path.header.frame_id = self.global_frame
        path.header.stamp = stamp
        for x, y in cmd.path:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path_pub.publish(path)

        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        info = self.core.info
        if info is not None:
            for i, fr in enumerate(cmd.frontiers):
                m = Marker()
                m.header.frame_id = self.global_frame
                m.header.stamp = stamp
                m.ns = "frontiers"
                m.id = i
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                wx, wy = info.cell_to_world(*fr.centroid_cell)
                m.pose.position = Point(x=wx, y=wy, z=0.05)
                m.pose.orientation.w = 1.0
                m.scale.x = m.scale.y = m.scale.z = 0.15
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.8, 0.0, 0.9
                markers.markers.append(m)
            if cmd.goal is not None:
                g = Marker()
                g.header.frame_id = self.global_frame
                g.header.stamp = stamp
                g.ns = "goal"
                g.id = 0
                g.type = Marker.SPHERE
                g.action = Marker.ADD
                g.pose.position = Point(x=cmd.goal[0], y=cmd.goal[1], z=0.1)
                g.pose.orientation.w = 1.0
                g.scale.x = g.scale.y = g.scale.z = 0.25
                g.color.r, g.color.g, g.color.b, g.color.a = 0.0, 1.0, 0.0, 0.9
                markers.markers.append(g)
        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
