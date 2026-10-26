"""
AC13: the real twist_mux (repository topic and lock configuration) with the safety_supervisor.

The twist_mux output is remapped to safety/cmd_vel_in as in simulated_robot.launch.py. Both
supervisor executables run against MockSafetyWorld (scan, TF, odometry). Observed on
bumperbot_controller/cmd_vel_unstamped.

  ROS_DOMAIN_ID=92 ROS_LOCALHOST_ONLY=1 launch_test test/test_safety_twist_mux.launch.py
"""
import os
import signal
import subprocess
import sys
import threading
import time
import unittest

os.environ.setdefault("ROS_DOMAIN_ID", "92")
os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.abspath(os.path.join(HERE, "..", ".."))
ROOT = os.path.abspath(os.path.join(SRC, ".."))
sys.path.insert(0, os.path.join(SRC, "bumperbot_utils", "test"))
CONFIG = os.path.join(SRC, "bumperbot_controller", "config")

import launch  # noqa: E402
import launch.actions  # noqa: E402
import launch_testing  # noqa: E402
import launch_testing.actions  # noqa: E402
import pytest  # noqa: E402
import rclpy  # noqa: E402
import yaml  # noqa: E402
from ament_index_python.packages import get_package_share_directory  # noqa: E402
from bumperbot_utils import safety_logic as sl  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from mock_safety_world import MockSafetyWorld  # noqa: E402
from safety_harness import SupervisorProcess, Workdir  # noqa: E402

CFG = sl.Config()


@pytest.mark.launch_test
@launch_testing.parametrize("impl", ["cpp", "py"])
def generate_test_description(impl):
    mux = launch.actions.IncludeLaunchDescription(
        os.path.join(get_package_share_directory("twist_mux"), "launch", "twist_mux_launch.py"),
        launch_arguments={
            "cmd_vel_out": "safety/cmd_vel_in",
            "config_topics": os.path.join(CONFIG, "twist_mux_topics.yaml"),
            "config_locks": os.path.join(CONFIG, "twist_mux_locks.yaml"),
            "config_joy": os.path.join(CONFIG, "twist_mux_joy.yaml"),
            "use_sim_time": "false",
        }.items())
    return launch.LaunchDescription([mux, launch_testing.actions.ReadyToTest()]), {"impl": impl}


def twist(x):
    msg = Twist()
    msg.linear.x = x
    return msg


class TestSafetyTwistMux(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.world = MockSafetyWorld("mock_safety_world_mux")
        cls.workdir = Workdir()
        cls.pubs = {name: cls.world.node.create_publisher(Twist, name, 10)
                    for name in ("key_vel", "dock_vel", "nav_vel")}
        cls.guard = threading.Lock()
        cls.active = {}
        cls.stop = threading.Event()
        cls.feeder = threading.Thread(target=cls.feed, daemon=True)
        cls.feeder.start()
        end = time.monotonic() + 30.0
        while time.monotonic() < end and not all(
                cls.world.node.count_subscribers(name) >= 1
                for name in ("key_vel", "dock_vel", "nav_vel", "safety_stop")):
            time.sleep(0.1)

    @classmethod
    def tearDownClass(cls):
        cls.stop.set()
        cls.feeder.join(timeout=2)
        cls.world.shutdown()
        rclpy.shutdown()
        cls.workdir.cleanup()

    @classmethod
    def feed(cls):
        """Publish the active inputs at 20 Hz (twist_mux inputs time out after 0.5 s)."""
        while not cls.stop.is_set():
            with cls.guard:
                active = dict(cls.active)
            for name, value in active.items():
                cls.pubs[name].publish(twist(value))
            time.sleep(0.05)

    def feeds(self, **inputs):
        with self.guard:
            self.__class__.active = dict(inputs)

    def start(self, impl):
        w = self.world
        w.reset()
        w.set_laser_tf(True)
        w.set_odom("follow_output")
        self.feeds()
        params = self.workdir.write_params({})
        self.proc = SupervisorProcess(impl, params, self.workdir)
        self.assertIsNotNone(w.wait_status(lambda m: m.state == "ACTIVE", 20.0),
                             "not ACTIVE\n" + self.proc.output())
        time.sleep(0.5)

    def finish(self):
        self.feeds()
        if self.proc.alive():
            self.proc.stop()
        time.sleep(0.3)

    def window(self, seconds):
        mark = self.world.mark()
        time.sleep(seconds)
        return [v for _, v in self.world.outs_since(mark)]

    def settle(self, value, timeout=3.0, tol=1e-3):
        ok = self.world.wait_for(
            lambda: self.world.last_out() is not None
            and abs(self.world.last_out() - value) < tol, timeout)
        self.assertTrue(ok, "output did not settle at {}: last {}".format(
            value, self.world.last_out()))

    # ------------------------------------------------------------------ (a)
    def test_key_vel_is_limited_to_the_speed_limit(self, impl):
        """AC13 (a): key_vel 0.7 with a front obstacle at 0.20 m gives speed_limit(0.20)."""
        w = self.world
        self.start(impl)
        try:
            self.feeds(key_vel=0.7)
            self.settle(0.7)
            w.set_obstacles([(0.10 + 0.20, 0.0)])
            expected = sl.speed_limit(0.20, CFG.stop_margin, CFG)
            self.settle(expected)
            values = self.window(1.0)
            self.assertTrue(values and all(abs(v - expected) < 1e-3 for v in values),
                            "expected {} got {}".format(expected, sorted(set(values))[:5]))
            self.assertFalse(w.last_lock())
        finally:
            self.finish()

    # ------------------------------------------------------------------ (b)
    def test_dock_vel_with_rear_and_front_obstacles(self, impl):
        """AC13 (b): dock_vel -0.1: stopped by a rear obstacle at 0.03 m, front one passes."""
        w = self.world
        self.start(impl)
        try:
            self.feeds(dock_vel=-0.1)
            self.settle(-0.1)
            w.set_obstacles([(-0.10 - 0.03, 0.0)])
            self.settle(0.0)
            values = self.window(1.5)
            self.assertTrue(values and all(v == 0.0 for v in values), sorted(set(values)))
            w.set_obstacles([(0.10 + 0.03, 0.0)])
            self.settle(-0.1)
            values = self.window(1.0)
            self.assertTrue(values and all(abs(v + 0.1) < 1e-6 for v in values),
                            sorted(set(values)))
        finally:
            self.finish()

    # ------------------------------------------------------------------ (c)
    def test_estop_blocks_every_source(self, impl):
        """AC13 (c): after trigger_estop key_vel, dock_vel and nav_vel all give 0 for 2 s."""
        w = self.world
        self.start(impl)
        try:
            for name in ("key_vel", "dock_vel", "nav_vel"):
                self.feeds(**{name: 0.2})
                self.settle(0.2)
            self.assertEqual(w.call_trigger("/safety_supervisor/trigger_estop"),
                             (True, "estop latched"))
            time.sleep(0.5)
            for name in ("key_vel", "dock_vel", "nav_vel"):
                self.feeds(**{name: 0.2})
                values = self.window(2.0)
                self.assertTrue(all(v == 0.0 for v in values), (name, sorted(set(values))))
            self.feeds(key_vel=0.2, dock_vel=0.2, nav_vel=0.2)
            values = self.window(2.0)
            self.assertTrue(all(v == 0.0 for v in values), sorted(set(values)))
            self.assertEqual(w.last_status().state, "ESTOP")
        finally:
            self.finish()

    # ------------------------------------------------------------------ (d)
    def test_no_motion_while_the_brake_lock_is_true(self, impl):
        """AC13 (d): no non-zero output while /safety_stop is true during a brake pulse."""
        w = self.world
        self.start(impl)
        try:
            self.feeds(key_vel=0.3)
            self.settle(0.3)
            self.assertTrue(w.wait_for(lambda: abs(w.odom_v - 0.3) < 0.05, 3.0))
            mark = w.mark()
            w.set_obstacles([(0.10 + 0.03, 0.0)])
            self.assertTrue(w.wait_for(lambda: w.last_lock() is True, 1.0), "no brake lock")
            self.assertTrue(w.wait_for(lambda: w.last_lock() is False, 4.0), "lock not released")
            locks = w.locks_since(mark)
            t_on = next(t for t, v in locks if v)
            t_off = next(t for t, v in locks if not v and t > t_on)
            outs = [(t, v) for t, v in w.outs_since(mark) if t_on + 0.1 < t < t_off]
            self.assertGreater(len(outs), 5)
            self.assertTrue(all(v == 0.0 for _, v in outs), sorted({v for _, v in outs}))
        finally:
            self.finish()

    # ------------------------------------------------------------------ (e)
    def test_sigterm_leaves_the_lock_engaged(self, impl):
        """AC13 (e): SIGTERM to the supervisor: no motion for 3 s while key_vel publishes."""
        w = self.world
        self.start(impl)
        try:
            self.feeds(key_vel=0.3)
            self.settle(0.3)
            self.proc.signal(signal.SIGTERM)
            self.assertEqual(self.proc.wait(5.0), 0)
            time.sleep(0.5)
            self.assertIs(w.last_lock(), True)
            values = self.window(3.0)
            self.assertTrue(all(v == 0.0 for v in values), sorted(set(values)))
        finally:
            self.finish()

    # ------------------------------------------------------------------ (f)
    def test_single_publisher_of_the_lock(self, impl):
        """AC13 (f): /safety_stop has exactly one publisher, the supervisor."""
        self.start(impl)
        try:
            info = self.world.node.get_publishers_info_by_topic("/safety_stop")
            self.assertEqual([p.node_name for p in info], ["safety_supervisor"])
        finally:
            self.finish()

    def test_locks_file_is_unchanged_vs_main(self, impl):
        res = subprocess.run(["git", "diff", "--quiet", "main", "--",
                              "src/bumperbot_controller/config/twist_mux_locks.yaml"], cwd=ROOT)
        self.assertEqual(res.returncode, 0, "twist_mux_locks.yaml differs from main")
        with open(os.path.join(CONFIG, "twist_mux_locks.yaml")) as f:
            locks = yaml.safe_load(f)["twist_mux"]["ros__parameters"]["locks"]
        self.assertEqual(locks["safety_stop"]["priority"], 255)
