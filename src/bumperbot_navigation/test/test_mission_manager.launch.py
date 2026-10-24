"""
AC5-AC11 (+ AC13 parameter check): mission_manager (C++ and Python) against MockNav2.

Each test method runs for both implementations (impl = cpp | py) with the same expectations.
No simulator and no Nav2 are needed.
"""
import math
import os
import sys
import threading
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from action_msgs.msg import GoalStatus
from bumperbot_msgs.action import ExecuteMission
from bumperbot_msgs.msg import MissionStatus
from rcl_interfaces.srv import GetParameters, ListParameters
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from mock_nav2 import MockNav2  # noqa: E402

STATIONS_FILE = os.path.join(HERE, "data", "stations_test.yaml")
STATION_POSES = {  # x, y, yaw, dwell (mirrors stations_test.yaml, checked in test_ordered_visit)
    "dock": (0.0, 0.0, 0.0, 0.0),
    "a": (1.0, 0.5, 0.5, 1.0),
    "b": (2.0, -1.0, 1.57, 0.2),
    "c": (-1.5, 2.0, 3.14, 0.2),
    "u1": (3.0, 3.0, 0.0, 0.2),
    "u2": (-3.0, -3.0, -1.0, 0.2),
}
NODE_PARAMS = {
    "stations_file": STATIONS_FILE,
    "retry_delay": 0.1,
    "nav_server_timeout": 5.0,
    "nav_goal_timeout": 10.0,
    "use_sim_time": False,
}
EXPECTED_PARAMS = {
    "stations_file": STATIONS_FILE, "nav_action_name": "navigate_to_pose",
    "global_frame": "map", "robot_frame": "base_footprint", "goal_tolerance": 0.2,
    "max_retries": 2, "retry_delay": 0.1, "max_consecutive_failures": 2,
    "dock_max_retries": 3, "return_to_dock": True, "nav_server_timeout": 5.0,
    "nav_goal_timeout": 10.0, "tf_timeout": 1.0, "status_rate": 2.0, "behavior_tree": "",
    "use_sim_time": False,
}


@pytest.mark.launch_test
@launch_testing.parametrize("impl", ["cpp", "py"])
def generate_test_description(impl):
    executable = "mission_manager" if impl == "cpp" else "mission_manager.py"
    node = launch_ros.actions.Node(
        package="bumperbot_navigation", executable=executable, name="mission_manager",
        output="screen", parameters=[NODE_PARAMS])
    return launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()]), {
        "mission_manager": node}


def wait_for(future, timeout):
    end = time.monotonic() + timeout
    while not future.done():
        if time.monotonic() > end:
            return False
        time.sleep(0.01)
    return True


class TestMissionManager(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.mock = MockNav2(STATIONS_FILE)
        cls.node = Node("mission_manager_test_client")
        cls.lock = threading.Lock()
        cls.statuses = []
        cls.feedback = []
        cls.first_status = None
        qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        cls.node.create_subscription(MissionStatus, "/mission/status", cls.on_status, qos)
        cls.client = ActionClient(cls.node, ExecuteMission, "execute_mission")
        cls.tf_buffer = Buffer()
        cls.tf_listener = TransformListener(cls.tf_buffer, cls.node)
        cls.executor = MultiThreadedExecutor(num_threads=6)
        cls.executor.add_node(cls.mock)
        cls.executor.add_node(cls.node)
        cls.thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.thread.start()
        assert cls.client.wait_for_server(timeout_sec=30.0), "execute_mission not available"
        end = time.monotonic() + 15.0
        while cls.first_status is None and time.monotonic() < end:
            time.sleep(0.05)

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.mock.destroy_node()
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def on_status(cls, msg):
        with cls.lock:
            if cls.first_status is None:
                cls.first_status = msg
            cls.statuses.append(msg)

    def settle_status(self, quiet=0.5, timeout=10.0):
        """Wait until no /mission/status message arrived for ``quiet`` seconds."""
        end = time.monotonic() + timeout
        last = -1
        stable_since = time.monotonic()
        while time.monotonic() < end:
            with self.lock:
                count = len(self.statuses)
            if count != last:
                last = count
                stable_since = time.monotonic()
            elif time.monotonic() - stable_since >= quiet:
                return
            time.sleep(0.05)

    def setUp(self):
        # late status messages of the previous test must not leak into this one
        self.settle_status()
        self.mock.reset()
        with self.lock:
            self.statuses.clear()
            self.feedback.clear()

    # ----------------------------------------------------------------- helpers
    def send(self, stations, timeout=20.0):
        goal = ExecuteMission.Goal()
        goal.stations = list(stations)
        fut = self.client.send_goal_async(goal, feedback_callback=self.on_feedback)
        self.assertTrue(wait_for(fut, timeout), "no goal response")
        return fut.result()

    def on_feedback(self, msg):
        with self.lock:
            self.feedback.append(msg.feedback)

    def result_of(self, handle, timeout=60.0):
        fut = handle.get_result_async()
        self.assertTrue(wait_for(fut, timeout), "mission did not finish in time")
        return fut.result()

    def run_mission(self, stations, behaviors=None):
        self.mock.reset(behaviors)
        with self.lock:
            self.statuses.clear()
            self.feedback.clear()
        handle = self.send(stations)
        self.assertTrue(handle.accepted, "mission goal rejected")
        res = self.result_of(handle)
        return res.status, res.result

    def goal_names(self):
        return [g["name"] for g in self.mock.snapshot()[0]]

    # ----------------------------------------------------------------- AC5
    def test_ordered_visit(self):
        status, result = self.run_mission(["a", "b", "c"])
        goals, _, reached = self.mock.snapshot()
        self.assertEqual([g["name"] for g in goals], ["a", "b", "c", "dock"])
        for g in goals:
            x, y, yaw, _ = STATION_POSES[g["name"]]
            self.assertEqual(g["frame_id"], "map")
            self.assertAlmostEqual(g["x"], x, delta=1e-6)
            self.assertAlmostEqual(g["y"], y, delta=1e-6)
            self.assertAlmostEqual(g["qx"], 0.0, delta=1e-6)
            self.assertAlmostEqual(g["qy"], 0.0, delta=1e-6)
            self.assertAlmostEqual(g["qz"], math.sin(yaw / 2.0), delta=1e-6)
            self.assertAlmostEqual(g["qw"], math.cos(yaw / 2.0), delta=1e-6)
        self.assertEqual([r["name"] for r in reached], ["a", "b", "c", "dock"])
        for r in reached:
            x, y, _, _ = STATION_POSES[r["name"]]
            self.assertLessEqual(math.hypot(r["x"] - x, r["y"] - y), 0.2)
        self.assertEqual(status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(result.outcome, ExecuteMission.Result.OUTCOME_COMPLETED)
        self.assertTrue(result.docked)
        self.assertEqual(list(result.visited), ["a", "b", "c"])
        self.assertEqual(list(result.skipped), [])
        # the TF seen by the test (same source the node uses) is at the dock at the end
        tf = self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
        self.assertLessEqual(math.hypot(tf.transform.translation.x,
                                        tf.transform.translation.y), 0.2)

    # ----------------------------------------------------------------- AC6
    def test_dwell(self):
        status, result = self.run_mission(["a", "b"])
        goals, _, reached = self.mock.snapshot()
        self.assertEqual(status, GoalStatus.STATUS_SUCCEEDED)
        t_a = [r["t"] for r in reached if r["name"] == "a"][0]
        t_b = [g["t"] for g in goals if g["name"] == "b"][0]
        gap = t_b - t_a
        self.assertGreaterEqual(gap, 0.95, f"next goal only {gap:.2f}s after station a")
        self.assertLessEqual(gap, 1.0 + 0.1 + 1.0, f"next goal {gap:.2f}s after station a")
        with self.lock:
            dwelling = [s for s in self.statuses
                        if s.state == "DWELLING" and s.current_station == "a"]
        self.assertTrue(dwelling, "no DWELLING status for station a")

    # ----------------------------------------------------------------- AC7
    def test_retry_and_skip(self):
        status, result = self.run_mission(
            ["b", "u1", "c"], {"b": {"abort_first": 2}, "u1": {"always_abort": True}})
        names = self.goal_names()
        self.assertEqual(names, ["b", "b", "b", "u1", "u1", "u1", "c", "dock"])
        self.assertIn("b", list(result.visited))
        self.assertIn("c", list(result.visited))
        self.assertEqual(list(result.skipped), ["u1"])
        self.assertEqual(result.outcome, ExecuteMission.Result.OUTCOME_COMPLETED_WITH_SKIPS)
        self.assertTrue(result.docked)
        self.assertEqual(status, GoalStatus.STATUS_SUCCEEDED)

    # ----------------------------------------------------------------- AC8
    def test_tolerance_check(self):
        status, result = self.run_mission(["b"], {"b": {"offsets": [0.35]}})
        self.assertEqual(self.goal_names(), ["b", "b", "dock"])
        self.assertEqual(list(result.visited), ["b"])
        self.assertEqual(status, GoalStatus.STATUS_SUCCEEDED)
        status, result = self.run_mission(["b"], {"b": {"offsets": [0.15]}})
        self.assertEqual(self.goal_names(), ["b", "dock"])
        self.assertEqual(list(result.visited), ["b"])

    # ----------------------------------------------------------------- AC9
    def test_abort_returns_to_dock(self):
        status, result = self.run_mission(
            ["u1", "u2", "c"], {"u1": {"always_abort": True}, "u2": {"always_abort": True}})
        names = self.goal_names()
        self.assertNotIn("c", names)
        self.assertEqual(names[-1], "dock")
        self.assertEqual(names, ["u1"] * 3 + ["u2"] * 3 + ["dock"])
        self.assertEqual(status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(result.outcome, ExecuteMission.Result.OUTCOME_ABORTED)
        self.assertTrue(result.docked)

    def test_cancel_returns_to_dock(self):
        self.mock.reset({"b": {"duration": 30.0}})
        handle = self.send(["a", "b", "c"])
        self.assertTrue(handle.accepted)
        end = time.monotonic() + 30.0
        while "b" not in self.goal_names() and time.monotonic() < end:
            time.sleep(0.05)
        self.assertIn("b", self.goal_names(), "navigation to b never started")
        time.sleep(0.3)
        cancel_future = handle.cancel_goal_async()
        self.assertTrue(wait_for(cancel_future, 10.0), "no answer to the cancel request")
        res = self.result_of(handle, 60.0)
        goals, cancels, _ = self.mock.snapshot()
        self.assertEqual([c["name"] for c in cancels], ["b"], "Nav2 goal for b was not canceled")
        names = [g["name"] for g in goals]
        self.assertEqual(names, ["a", "b", "dock"])
        self.assertEqual(res.status, GoalStatus.STATUS_CANCELED)
        self.assertEqual(res.result.outcome, ExecuteMission.Result.OUTCOME_CANCELED)
        self.assertTrue(res.result.docked)

    def test_dock_failure(self):
        status, result = self.run_mission(["a"], {"dock": {"always_abort": True}})
        names = self.goal_names()
        self.assertEqual(names, ["a"] + ["dock"] * 4)
        self.assertEqual(status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(result.outcome, ExecuteMission.Result.OUTCOME_DOCK_FAILED)
        self.assertFalse(result.docked)

    # ----------------------------------------------------------------- AC10
    def test_goal_rejection(self):
        self.mock.reset()
        self.assertFalse(self.send([]).accepted, "empty mission accepted")
        self.assertFalse(self.send(["a", "nowhere"]).accepted, "unknown station accepted")
        self.assertEqual(self.goal_names(), [])
        first = self.send(["a", "b", "c"])
        self.assertTrue(first.accepted)
        second = self.send(["b"])
        self.assertFalse(second.accepted, "second mission accepted while one is active")
        res = self.result_of(first)
        self.assertEqual(res.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(res.result.outcome, ExecuteMission.Result.OUTCOME_COMPLETED)
        self.assertEqual(self.goal_names(), ["a", "b", "c", "dock"])
        # after the mission a new one is accepted again
        again = self.send(["b"])
        self.assertTrue(again.accepted)
        self.assertEqual(self.result_of(again).status, GoalStatus.STATUS_SUCCEEDED)

    # ----------------------------------------------------------------- AC11
    def test_status_topic(self):
        # transient_local: the subscriber created in setUpClass (after node start) got IDLE
        self.assertIsNotNone(self.first_status, "no /mission/status message received")
        self.assertEqual(self.first_status.state, "IDLE")
        status, result = self.run_mission(["a", "b", "c"])
        end = time.monotonic() + 5.0
        while time.monotonic() < end:
            with self.lock:
                if self.statuses and self.statuses[-1].state == "COMPLETED":
                    break
            time.sleep(0.05)
        with self.lock:
            statuses = list(self.statuses)
            feedback = list(self.feedback)
        self.assertTrue(statuses)
        # A periodic (2 Hz) status of the previous, finished mission can still arrive after
        # run_mission() cleared the list: the new mission starts at the first NAVIGATING status.
        first = next((i for i, s in enumerate(statuses) if s.state == "NAVIGATING"), None)
        self.assertIsNotNone(first, "no NAVIGATING status published")
        statuses = statuses[first:]
        seq = []
        for s in statuses:
            if s.current_station and (not seq or seq[-1] != s.current_station):
                seq.append(s.current_station)
        self.assertEqual(seq, ["a", "b", "c", "dock"])
        progress = [s.progress for s in statuses]
        for earlier, later in zip(progress, progress[1:]):
            self.assertLessEqual(earlier, later + 1e-6, progress)
        self.assertAlmostEqual(statuses[-1].progress, 1.0, delta=1e-6)
        self.assertEqual(statuses[-1].state, "COMPLETED")
        for station in ("a", "b", "c"):
            self.assertTrue([f for f in feedback if f.current_station == station],
                            f"no feedback for station {station}")

    # ----------------------------------------------------------------- AC13 (runtime)
    def test_parameter_values(self):
        list_client = self.node.create_client(ListParameters, "/mission_manager/list_parameters")
        get_client = self.node.create_client(GetParameters, "/mission_manager/get_parameters")
        self.assertTrue(list_client.wait_for_service(10.0))
        self.assertTrue(get_client.wait_for_service(10.0))
        req = ListParameters.Request()
        fut = list_client.call_async(req)
        self.assertTrue(wait_for(fut, 10.0))
        names = [n for n in fut.result().result.names]
        self.assertEqual(set(EXPECTED_PARAMS) - set(names), set(), "missing parameters")
        req = GetParameters.Request()
        req.names = sorted(EXPECTED_PARAMS)
        fut = get_client.call_async(req)
        self.assertTrue(wait_for(fut, 10.0))
        values = {}
        for name, v in zip(req.names, fut.result().values):
            values[name] = {1: v.bool_value, 2: v.integer_value, 3: v.double_value,
                            4: v.string_value}[v.type]
        for name, expected in EXPECTED_PARAMS.items():
            if isinstance(expected, float):
                self.assertAlmostEqual(values[name], expected, places=9, msg=name)
            else:
                self.assertEqual(values[name], expected, name)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
