"""
AC18: the real twist_mux with the repository's topic and lock configuration.

The output is remapped like in joystick_teleop.launch.py. dock_vel must lose against the
safety_stop lock and the keyboard, and win against Nav2 (nav_vel).

  ROS_DOMAIN_ID=90 ROS_LOCALHOST_ONLY=1 launch_test test/test_dock_twist_mux.launch.py
"""
import os
import subprocess
import threading
import time
import unittest

os.environ.setdefault("ROS_DOMAIN_ID", "90")
os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")

import launch  # noqa: E402
import launch.actions  # noqa: E402
import launch_testing  # noqa: E402
import launch_testing.actions  # noqa: E402
import pytest  # noqa: E402
import rclpy  # noqa: E402
import yaml  # noqa: E402
from ament_index_python.packages import get_package_share_directory  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.abspath(os.path.join(HERE, "..", ".."))
ROOT = os.path.abspath(os.path.join(SRC, ".."))
CONFIG = os.path.join(SRC, "bumperbot_controller", "config")
OUTPUT = "bumperbot_controller/cmd_vel_unstamped"


@pytest.mark.launch_test
def generate_test_description():
    mux = launch.actions.IncludeLaunchDescription(
        os.path.join(get_package_share_directory("twist_mux"), "launch", "twist_mux_launch.py"),
        launch_arguments={
            "cmd_vel_out": OUTPUT,
            "config_topics": os.path.join(CONFIG, "twist_mux_topics.yaml"),
            "config_locks": os.path.join(CONFIG, "twist_mux_locks.yaml"),
            "config_joy": os.path.join(CONFIG, "twist_mux_joy.yaml"),
            "use_sim_time": "false",
        }.items())
    return launch.LaunchDescription([mux, launch_testing.actions.ReadyToTest()]), {}


def twist(x):
    msg = Twist()
    msg.linear.x = x
    return msg


class TestDockTwistMux(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("dock_twist_mux_test")
        cls.pubs = {name: cls.node.create_publisher(Twist, name, 10)
                    for name in ("dock_vel", "nav_vel", "key_vel")}
        cls.lock_pub = cls.node.create_publisher(Bool, "safety_stop", 10)
        cls.lock = threading.Lock()
        cls.out = []
        cls.node.create_subscription(Twist, "/" + OUTPUT, cls.on_out, 100)
        cls.active = {}
        cls.lock_value = None
        cls.executor = MultiThreadedExecutor(num_threads=3)
        cls.executor.add_node(cls.node)
        cls.thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.thread.start()
        cls.stop = threading.Event()
        cls.feeder = threading.Thread(target=cls.feed, daemon=True)
        cls.feeder.start()
        end = time.monotonic() + 30.0
        while time.monotonic() < end and not all(
                cls.node.count_subscribers(name) >= 1
                for name in ("dock_vel", "nav_vel", "key_vel", "safety_stop")):
            time.sleep(0.1)

    @classmethod
    def tearDownClass(cls):
        cls.stop.set()
        cls.feeder.join(timeout=2)
        cls.executor.shutdown()
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def feed(cls):
        """Publish the active inputs at 20 Hz (twist_mux inputs time out after 0.5 s)."""
        while not cls.stop.is_set():
            with cls.lock:
                active = dict(cls.active)
                lock_value = cls.lock_value
            for name, value in active.items():
                cls.pubs[name].publish(twist(value))
            if lock_value is not None:
                msg = Bool()
                msg.data = lock_value
                cls.lock_pub.publish(msg)
            time.sleep(0.05)

    @classmethod
    def on_out(cls, msg):
        with cls.lock:
            cls.out.append((time.monotonic(), msg.linear.x))

    def setUp(self):
        self.configure({}, False)

    def configure(self, inputs, locked):
        with self.lock:
            self.__class__.active = dict(inputs)
            self.__class__.lock_value = locked
        time.sleep(0.8)     # inputs and the lock settle, old output expires

    def collect(self, seconds):
        with self.lock:
            start = len(self.out)
        time.sleep(seconds)
        with self.lock:
            return [v for _, v in self.out[start:]]

    def test_inputs_are_visible_to_the_mux(self):
        for name in ("dock_vel", "nav_vel", "key_vel", "safety_stop"):
            self.assertGreaterEqual(self.node.count_subscribers(name), 1, name)

    def test_safety_stop_lock_blocks_dock_vel(self):
        """AC18a: dock_vel 0.1 m/s with the lock set: nothing moves for 2 s."""
        # control: the same input passes while the lock is released, so silence means blocked
        self.configure({"dock_vel": 0.1}, False)
        free = self.collect(1.0)
        self.assertTrue(free and abs(free[-1] - 0.1) < 1e-9, "dock_vel does not pass: {}".format(
            free[:5]))
        self.configure({"dock_vel": 0.1}, True)
        values = self.collect(2.0)
        self.assertEqual([v for v in values if v != 0.0], [], "motion through the lock")

    def test_dock_vel_passes_without_lock(self):
        """AC18b: with the lock released dock_vel reaches the output."""
        self.configure({"dock_vel": 0.1}, False)
        values = self.collect(1.5)
        self.assertGreaterEqual(len(values), 10)
        self.assertTrue(all(abs(v - 0.1) < 1e-9 for v in values[3:]), values[:10])

    def test_dock_vel_wins_against_nav_vel(self):
        """AC18c: nav_vel and dock_vel together: the output equals dock_vel."""
        self.configure({"dock_vel": 0.1, "nav_vel": 0.3}, False)
        values = self.collect(1.5)
        self.assertGreaterEqual(len(values), 10)
        self.assertTrue(all(abs(v - 0.1) < 1e-9 for v in values), sorted(set(values)))

    def test_nav_vel_alone_still_passes(self):
        self.configure({"nav_vel": 0.3}, False)
        values = self.collect(1.0)
        self.assertTrue(values and all(abs(v - 0.3) < 1e-9 for v in values[2:]))

    def test_keyboard_overrides_dock_vel(self):
        """AC18d: key_vel wins against dock_vel."""
        self.configure({"dock_vel": 0.1, "key_vel": -0.2}, False)
        values = self.collect(1.5)
        self.assertGreaterEqual(len(values), 10)
        self.assertTrue(all(abs(v + 0.2) < 1e-9 for v in values), sorted(set(values)))

    def test_lock_also_blocks_the_keyboard_and_dock_resumes_after_release(self):
        self.configure({"dock_vel": 0.1}, True)
        self.assertEqual([v for v in self.collect(1.0) if v != 0.0], [])
        with self.lock:
            self.__class__.lock_value = False
        time.sleep(0.5)
        values = self.collect(1.0)
        self.assertTrue(values and all(abs(v - 0.1) < 1e-9 for v in values[2:]), values[:10])


# ------------------------------------------------------------------ static checks of the configs
def load_topics():
    with open(os.path.join(CONFIG, "twist_mux_topics.yaml")) as f:
        return yaml.safe_load(f)["twist_mux"]["ros__parameters"]["topics"]


class TestTwistMuxConfig(unittest.TestCase):

    def test_docking_priority_is_between_navigation_and_keyboard(self):
        topics = load_topics()
        by_topic = {v["topic"]: v for v in topics.values()}
        self.assertIn("dock_vel", by_topic)
        dock = by_topic["dock_vel"]["priority"]
        self.assertGreater(dock, by_topic["nav_vel"]["priority"])
        self.assertLess(dock, by_topic["key_vel"]["priority"])
        self.assertLess(dock, by_topic["joy_vel"]["priority"])
        self.assertGreater(dock, by_topic["explore_vel"]["priority"])
        self.assertEqual(by_topic["dock_vel"]["priority"], 30)
        self.assertLess(dock, 255)

    def test_existing_inputs_keep_their_priorities(self):
        by_topic = {v["topic"]: v["priority"] for v in load_topics().values()}
        self.assertEqual((by_topic["joy_vel"], by_topic["key_vel"], by_topic["nav_vel"],
                          by_topic["explore_vel"]), (99, 90, 20, 10))

    def test_safety_stop_lock_is_still_255(self):
        with open(os.path.join(CONFIG, "twist_mux_locks.yaml")) as f:
            locks = yaml.safe_load(f)["twist_mux"]["ros__parameters"]["locks"]
        self.assertEqual(locks["safety_stop"]["priority"], 255)
        self.assertEqual(locks["safety_stop"]["topic"], "safety_stop")

    def test_locks_file_unchanged_vs_main(self):
        res = subprocess.run(
            ["git", "diff", "--quiet", "main", "--",
             "src/bumperbot_controller/config/twist_mux_locks.yaml"], cwd=ROOT)
        self.assertEqual(res.returncode, 0, "twist_mux_locks.yaml differs from main")
