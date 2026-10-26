"""AC6: launch_testing test of the frontier_explorer node with synthetic inputs (no simulator)."""
import math
import os
import tempfile
import threading
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav2_msgs.srv import SaveMap
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster

SAVE_DIR = tempfile.mkdtemp(prefix="explorer_test_")
SAVE_PATH = os.path.join(SAVE_DIR, "maps", "explored_map")
COMPLETION_TIMEOUT = 2.0
SIZE = 100


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    explorer = launch_ros.actions.Node(
        package="bumperbot_mapping",
        executable="frontier_explorer.py",
        name="frontier_explorer",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "completion_timeout": COMPLETION_TIMEOUT,
            "map_save_path": SAVE_PATH,
            # A static TF can never complete the 360 degree spin of the start sequence
            "initial_spin": False,
        }],
    )
    return launch.LaunchDescription([explorer, launch_testing.actions.ReadyToTest()]), {}


def make_map(open_top):
    """100x100 map, 0.05 m. Free room x,y in [20, 80). Top wall missing -> frontier row."""
    msg = OccupancyGrid()
    msg.header.frame_id = "map"
    msg.info.resolution = 0.05
    msg.info.width = SIZE
    msg.info.height = SIZE
    msg.info.origin.orientation.w = 1.0
    g = [-1] * (SIZE * SIZE)

    def setc(x, y, v):
        g[y * SIZE + x] = v

    for y in range(20, 80):
        for x in range(20, 80):
            setc(x, y, 0)
    for y in range(18, 82 if not open_top else 80):
        for x in range(18, 82):
            if not (20 <= x < 80 and 20 <= y < 80):
                setc(x, y, 100)
    msg.data = g
    return msg


class TestExplorerNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("explorer_test_harness")
        cls.lock = threading.Lock()
        cls.status = []
        cls.twists = []
        cls.paths = []
        cls.save_requests = []

        latched = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        cls.map_pub = cls.node.create_publisher(OccupancyGrid, "map", latched)
        cls.scan_pub = cls.node.create_publisher(LaserScan, "scan", qos_profile_sensor_data)
        cls.node.create_subscription(String, "/explore/status", cls.on_status, latched)
        cls.node.create_subscription(Twist, "explore_vel", cls.on_twist, 10)
        cls.node.create_subscription(Path, "/explore/path", cls.on_path, 10)
        cls.node.create_service(SaveMap, "/map_saver/save_map", cls.on_save)

        # Static TF: map -> base_footprint (2, 2, yaw 0), base_footprint -> laser_link (yaw pi)
        cls.tf_broadcaster = StaticTransformBroadcaster(cls.node)
        t1 = TransformStamped()
        t1.header.stamp = cls.node.get_clock().now().to_msg()
        t1.header.frame_id = "map"
        t1.child_frame_id = "base_footprint"
        t1.transform.translation.x = 2.0
        t1.transform.translation.y = 2.0
        t1.transform.rotation.w = 1.0
        t2 = TransformStamped()
        t2.header.stamp = t1.header.stamp
        t2.header.frame_id = "base_footprint"
        t2.child_frame_id = "laser_link"
        t2.transform.rotation.z = math.sin(math.pi / 2)
        t2.transform.rotation.w = math.cos(math.pi / 2)
        cls.tf_broadcaster.sendTransform([t1, t2])

        cls.node.create_timer(0.1, cls.publish_scan)
        cls.executor = MultiThreadedExecutor(num_threads=2)
        cls.executor.add_node(cls.node)
        cls.spin_thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.spin_thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.node.destroy_node()
        rclpy.shutdown()

    # -- harness callbacks (class-level because of the shared state) ------
    @classmethod
    def on_status(cls, msg):
        with cls.lock:
            cls.status.append(msg.data)

    @classmethod
    def on_twist(cls, msg):
        with cls.lock:
            cls.twists.append((time.time(), msg.linear.x, msg.angular.z))

    @classmethod
    def on_path(cls, msg):
        with cls.lock:
            cls.paths.append(len(msg.poses))

    @classmethod
    def on_save(cls, request, response):
        with cls.lock:
            cls.save_requests.append(request.map_url)
        response.result = True
        return response

    @classmethod
    def publish_scan(cls):
        msg = LaserScan()
        msg.header.stamp = cls.node.get_clock().now().to_msg()
        msg.header.frame_id = "laser_link"
        msg.angle_min = 0.0
        msg.angle_max = 6.28
        msg.angle_increment = 6.28 / 359
        msg.range_min = 0.12
        msg.range_max = 12.0
        msg.ranges = [5.0] * 360
        cls.scan_pub.publish(msg)

    def wait_for(self, predicate, timeout, what):
        end = time.time() + timeout
        while time.time() < end:
            with self.lock:
                if predicate():
                    return
            time.sleep(0.1)
        with self.lock:
            self.fail(f"timeout ({timeout}s) waiting for {what}; status={self.status}, "
                      f"n_twists={len(self.twists)}, nonzero="
                      f"{sum(1 for _, v, w in self.twists if v or w)}, paths={self.paths[-3:]}, "
                      f"saves={self.save_requests}")

    def test_1_exploring_then_complete_and_saved(self):
        # Phase 1: map with a frontier
        self.map_pub.publish(make_map(open_top=True))
        self.wait_for(lambda: "EXPLORING" in self.status, 15.0, "status EXPLORING")
        self.wait_for(lambda: any(v or w for _, v, w in self.twists), 15.0,
                      "non-zero Twist on explore_vel")
        self.wait_for(lambda: any(n > 0 for n in self.paths), 15.0,
                      "non-empty /explore/path")

        # Phase 2: fully known closed map, no frontier left
        self.map_pub.publish(make_map(open_top=False))
        self.wait_for(lambda: "COMPLETE" in self.status, COMPLETION_TIMEOUT + 10.0,
                      "status COMPLETE")

        def zero_tail():
            tail = self.twists[-3:]
            return len(tail) == 3 and all(v == 0.0 and w == 0.0 for _, v, w in tail)
        self.wait_for(zero_tail, 5.0, "zero Twist on explore_vel after COMPLETE")
        self.wait_for(lambda: len(self.save_requests) >= 1, 10.0, "save_map request")
        self.wait_for(lambda: "MAP_SAVED" in self.status, 10.0, "status MAP_SAVED")
        with self.lock:
            self.assertEqual(self.save_requests[0], SAVE_PATH)
            self.assertEqual(self.status[-1], "MAP_SAVED")
            self.assertLess(self.status.index("EXPLORING"), self.status.index("COMPLETE"))
            self.assertLess(self.status.index("COMPLETE"), self.status.index("MAP_SAVED"))
        self.assertTrue(os.path.isdir(os.path.dirname(SAVE_PATH)),
                        "node must create the map directory")


@launch_testing.post_shutdown_test()
class TestProcessExit(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
