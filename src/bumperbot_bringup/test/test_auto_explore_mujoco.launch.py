"""AC8: full-stack headless MuJoCo smoke test of auto_explore.launch.py."""
import math
import os
import tempfile
import threading
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

DEADLINE = 120.0


def make_explorer_config():
    """Copy of the default explorer YAML with save_map_on_complete disabled."""
    src = os.path.join(get_package_share_directory("bumperbot_mapping"),
                       "config", "frontier_explorer.yaml")
    with open(src, encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    cfg["frontier_explorer"]["ros__parameters"]["save_map_on_complete"] = False
    fd, path = tempfile.mkstemp(prefix="explorer_smoke_", suffix=".yaml")
    with os.fdopen(fd, "w") as f:
        yaml.safe_dump(cfg, f)
    return path


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    import mujoco  # noqa: F401  (a missing mujoco is a test failure, not a skip)
    auto_explore = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("bumperbot_bringup"),
                     "launch", "auto_explore.launch.py"),
        launch_arguments={
            "simulator": "mujoco",
            "world_name": "small_house",
            "viewer": "false",
            "use_rviz": "false",
            "explorer_config": make_explorer_config(),
        }.items())
    return launch.LaunchDescription([auto_explore, launch_testing.actions.ReadyToTest()]), {}


class TestAutoExploreMujoco(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("auto_explore_smoke_test")
        latched = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        cls.lock = threading.Lock()
        cls.map_received = False
        cls.statuses = []
        cls.node.create_subscription(OccupancyGrid, "/map", cls.on_map, latched)
        cls.node.create_subscription(String, "/explore/status", cls.on_status, latched)
        cls.tf_buffer = Buffer()
        cls.tf_listener = TransformListener(cls.tf_buffer, cls.node)
        cls.executor = MultiThreadedExecutor(num_threads=2)
        cls.executor.add_node(cls.node)
        cls.thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def on_map(cls, msg):
        with cls.lock:
            cls.map_received = True

    @classmethod
    def on_status(cls, msg):
        with cls.lock:
            cls.statuses.append(msg.data)

    def pose(self):
        try:
            t = self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
        except TransformException:
            return None
        return t.transform.translation.x, t.transform.translation.y

    def test_stack_explores(self):
        start = time.time()
        first = None
        moved = 0.0
        while time.time() - start < DEADLINE:
            p = self.pose()
            if p is not None:
                if first is None:
                    first = p
                moved = max(moved, math.hypot(p[0] - first[0], p[1] - first[1]))
            with self.lock:
                done = self.map_received and "EXPLORING" in self.statuses and moved >= 0.3
            if done:
                print(f"\nAC8 OK after {time.time() - start:.1f}s: moved {moved:.2f} m")
                return
            time.sleep(0.5)
        with self.lock:
            self.fail(f"within {DEADLINE}s: map_received={self.map_received}, "
                      f"statuses={self.statuses}, first_tf={first}, moved={moved:.3f} m")
