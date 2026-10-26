"""AC9: end-to-end SLAM map quality on headless MuJoCo (opt-in: BUMPERBOT_SLAM_E2E=1).

Run on a dedicated domain, for example:
  BUMPERBOT_SLAM_E2E=1 ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1 timeout 2100 \
      launch_test src/bumperbot_bringup/test/test_slam_map_quality_mujoco.launch.py
"""
import os
import shutil
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
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

ENABLED = os.environ.get("BUMPERBOT_SLAM_E2E") == "1"
MAX_EXPLORATION_TIME = 1500.0
WAIT_TIMEOUT = 1800.0
OUT_DIR = tempfile.mkdtemp(prefix="slam_quality_")
MAP_BASE = os.path.join(OUT_DIR, "explored")


def make_explorer_config():
    src = os.path.join(get_package_share_directory("bumperbot_mapping"),
                       "config", "frontier_explorer.yaml")
    with open(src, encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    cfg["frontier_explorer"]["ros__parameters"]["max_exploration_time"] = MAX_EXPLORATION_TIME
    path = os.path.join(OUT_DIR, "explorer_override.yaml")
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(cfg, f)
    return path


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    if not ENABLED:
        return launch.LaunchDescription([launch_testing.actions.ReadyToTest()]), {}
    domain = os.environ.get("ROS_DOMAIN_ID", "")
    if domain in ("", "0"):
        raise RuntimeError("AC9 needs a dedicated ROS_DOMAIN_ID (not unset or 0)")
    import mujoco  # noqa: F401  (a missing mujoco is a failure)
    auto_explore = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("bumperbot_bringup"),
                     "launch", "auto_explore.launch.py"),
        launch_arguments={
            "simulator": "mujoco",
            "world_name": "small_house",
            "viewer": "false",
            "use_rviz": "false",
            "map_save_path": MAP_BASE,
            "explorer_config": make_explorer_config(),
        }.items())
    return launch.LaunchDescription([auto_explore, launch_testing.actions.ReadyToTest()]), {}


@unittest.skipUnless(ENABLED, "set BUMPERBOT_SLAM_E2E=1 to run the SLAM map quality test")
class TestSlamMapQuality(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("slam_quality_test")
        latched = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        cls.lock = threading.Lock()
        cls.statuses = []   # (wall s, sim s, status)
        cls.sim_time = 0.0
        cls.t0 = time.time()
        cls.node.create_subscription(String, "/explore/status", cls.on_status, latched)
        cls.node.create_subscription(Clock, "/clock", cls.on_clock, 10)
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
    def on_clock(cls, msg):
        cls.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    @classmethod
    def on_status(cls, msg):
        with cls.lock:
            cls.statuses.append((time.time() - cls.t0, cls.sim_time, msg.data))

    def test_1_single_publishers(self):
        time.sleep(max(0.0, 20.0 - (time.time() - self.t0)))
        clock = self.node.count_publishers("/clock")
        scan = self.node.count_publishers("/scan")
        print(f"\nAC9 publishers: /clock={clock} /scan={scan} domain={os.environ.get('ROS_DOMAIN_ID')}")
        self.assertEqual(clock, 1, "stale stack? /clock publisher count")
        self.assertEqual(scan, 1, "stale stack? /scan publisher count")

    def test_2_map_quality(self):
        from bumperbot_mapping import map_compare as mc
        start = time.time()
        while time.time() - start < WAIT_TIMEOUT:
            with self.lock:
                names = [s[2] for s in self.statuses]
            if "MAP_SAVED" in names or "MAP_SAVE_FAILED" in names:
                break
            time.sleep(1.0)
        with self.lock:
            history = list(self.statuses)
        print("\nAC9 status history (wall s, sim s, status):")
        for h in history:
            print("  %8.1f %8.1f %s" % h)
        names = [h[2] for h in history]
        self.assertIn("MAP_SAVED", names, f"no MAP_SAVED within {WAIT_TIMEOUT}s: {names}")
        t_expl = next((h for h in history if h[2] == "EXPLORING"), None)
        t_done = next((h for h in history if h[2] == "COMPLETE"), None)
        if t_expl and t_done:
            sim_dur = t_done[1] - t_expl[1]
            reason = "time limit" if sim_dur >= MAX_EXPLORATION_TIME - 5.0 else "natural COMPLETE"
            print(f"AC9 exploration: sim {sim_dur:.0f} s, wall {t_done[0] - t_expl[0]:.0f} s, ended by {reason}")

        artifacts = os.environ.get("BUMPERBOT_TEST_ARTIFACTS")
        if artifacts:
            os.makedirs(artifacts, exist_ok=True)
            for ext in (".pgm", ".yaml"):
                shutil.copy(MAP_BASE + ext, os.path.join(artifacts, "slam_quality_map" + ext))
            # keep the yaml image line valid after the copy
            with open(os.path.join(artifacts, "slam_quality_map.yaml")) as f:
                meta = yaml.safe_load(f)
            meta["image"] = "slam_quality_map.pgm"
            with open(os.path.join(artifacts, "slam_quality_map.yaml"), "w") as f:
                yaml.safe_dump(meta, f)
        print("AC9 saved map:", MAP_BASE + ".yaml")

        ref = os.path.join(get_package_share_directory("bumperbot_mapping"),
                           "maps", "small_house", "map.yaml")
        quality = mc.compare_maps(MAP_BASE + ".yaml", ref)
        print("AC9 MapQuality:", quality)
        for t in quality.tiles:
            print("  tile (%.1f, %.1f) n=%d rot=%.2f shift=%.2f inl=%.2f" % (
                t.center[0], t.center[1], t.n_points, t.rotation_deg, t.shift_m, t.inlier_fraction))
        ok, reasons = quality.passes()
        self.assertTrue(ok, "map quality failed: %s" % reasons)
