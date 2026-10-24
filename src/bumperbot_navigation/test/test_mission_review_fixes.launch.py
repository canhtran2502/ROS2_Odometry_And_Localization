"""
Iteration 3 review fixes: mission_manager (C++ and Python) against MockNav2.

Separate file from test_mission_manager.launch.py (whose CMake timeout is already almost used up).
Transient/continuous goal rejection, cancel right after acceptance, cancel/result races.
"""
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
NODE_PARAMS = {  # same as test_mission_manager.launch.py
    "stations_file": STATIONS_FILE,
    "retry_delay": 0.1,
    "nav_server_timeout": 5.0,
    "nav_goal_timeout": 10.0,
    "use_sim_time": False,
}

# REVIEW_IMPL=cpp|py runs one implementation only (used by the pytest wrappers that fit the
# per-test timeout of the colcon registration); unset runs both.
IMPLS = [os.environ["REVIEW_IMPL"]] if os.environ.get("REVIEW_IMPL") else ["cpp", "py"]


@pytest.mark.launch_test
@launch_testing.parametrize("impl", IMPLS)
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


class TestMissionReviewFixes(unittest.TestCase):

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
        # status is published at 2 Hz: a quiet time above its 0.5 s period can be reached
        self.settle_status(quiet=0.9)
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

    # ----------------------------------------------------------------- review fixes (iter 3)
    def test_transient_rejection_does_not_consume_retries(self):
        # Nav2 rejects goals for 2 s (< nav_server_timeout 5 s), then accepts. With max_retries 2
        # the station would be skipped if each rejection counted as a failed attempt.
        status, result = self.run_mission(["b"], {"b": {"reject_for": 2.0}})
        with self.mock.lock:
            rejected = [r for r in self.mock.rejected if r["name"] == "b"]
        self.assertGreaterEqual(len(rejected), 2, "mock did not reject the goal repeatedly")
        self.assertEqual(self.goal_names(), ["b", "dock"])
        self.assertEqual(status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(result.outcome, ExecuteMission.Result.OUTCOME_COMPLETED)
        self.assertEqual(list(result.visited), ["b"])
        self.assertEqual(list(result.skipped), [])
        self.assertTrue(result.docked)

    def test_rejection_window_leaves_full_retry_budget(self):
        # after the rejection window the station still gets all 1 + max_retries real attempts
        status, result = self.run_mission(
            ["b"], {"b": {"reject_for": 2.0, "abort_first": 2}})
        self.assertEqual(self.goal_names(), ["b", "b", "b", "dock"])
        self.assertEqual(list(result.visited), ["b"])
        self.assertEqual(list(result.skipped), [])
        self.assertEqual(result.outcome, ExecuteMission.Result.OUTCOME_COMPLETED)

    def test_continuous_rejection_counts_as_failed_attempt(self):
        # nav_server_timeout is 5 s: every attempt of u1 is rejected for the whole timeout, so it
        # takes 3 attempts * 5 s, then the station is skipped and the mission goes on.
        status, result = self.run_mission(
            ["u1", "c"], {"u1": {"reject_for": 1e9}})
        with self.mock.lock:
            rejected = [r["t"] for r in self.mock.rejected if r["name"] == "u1"]
        goals = self.mock.snapshot()[0]
        self.assertEqual([g["name"] for g in goals], ["c", "dock"], "u1 must never be accepted")
        self.assertGreaterEqual(len(rejected), 6)
        span = max(rejected) - min(rejected)
        self.assertGreaterEqual(span, 3 * 5.0 - 1.5,
                                f"u1 rejections span only {span:.1f}s, not 3 failed attempts")
        self.assertEqual(list(result.skipped), ["u1"])
        self.assertEqual(list(result.visited), ["c"])
        self.assertEqual(result.outcome, ExecuteMission.Result.OUTCOME_COMPLETED_WITH_SKIPS)
        self.assertEqual(status, GoalStatus.STATUS_SUCCEEDED)
        self.assertTrue(result.docked)

    def test_cancel_right_after_goal_acceptance_is_honored(self):
        self.mock.reset({"a": {"duration": 30.0}, "b": {"duration": 30.0}})
        handle = self.send(["a", "b", "c"])
        self.assertTrue(handle.accepted)
        cancel_future = handle.cancel_goal_async()  # immediately, no wait for the Nav2 goal
        self.assertTrue(wait_for(cancel_future, 10.0), "no answer to the cancel request")
        res = self.result_of(handle, 60.0)
        names = self.goal_names()
        self.assertEqual(res.status, GoalStatus.STATUS_CANCELED)
        self.assertEqual(res.result.outcome, ExecuteMission.Result.OUTCOME_CANCELED)
        self.assertTrue(res.result.docked)
        self.assertEqual(names[-1], "dock")
        self.assertNotIn("b", names)
        self.assertNotIn("c", names)
        self.assertEqual(list(res.result.visited), [])
        # the node is idle again and accepts the next mission
        self.mock.reset()
        again = self.send(["b"])
        self.assertTrue(again.accepted, "mission after the cancel was rejected")
        self.assertEqual(self.result_of(again).status, GoalStatus.STATUS_SUCCEEDED)

    def _cancel_race(self, behavior, cancel_delays):
        """Cancel b at several moments around the moment Nav2 finishes the goal."""
        for delay in cancel_delays:
            with self.subTest(delay=delay, behavior=behavior):
                self.mock.reset({"b": dict(behavior)})
                handle = self.send(["b", "c"])
                self.assertTrue(handle.accepted)
                end = time.monotonic() + 30.0
                while "b" not in self.goal_names() and time.monotonic() < end:
                    time.sleep(0.01)
                self.assertIn("b", self.goal_names())
                t_b = [g["t"] for g in self.mock.snapshot()[0] if g["name"] == "b"][0]
                wait = t_b + delay - time.monotonic()
                if wait > 0:
                    time.sleep(wait)
                wait_for(handle.cancel_goal_async(), 10.0)
                res = self.result_of(handle, 60.0)
                self.assertEqual(res.status, GoalStatus.STATUS_CANCELED)
                self.assertEqual(res.result.outcome, ExecuteMission.Result.OUTCOME_CANCELED)
                self.assertTrue(res.result.docked)
                self.assertEqual(self.goal_names()[-1], "dock")
                self.assertNotIn("c", self.goal_names())
                # a following mission is accepted and completes: the node did not crash/hang
                follow = self.send(["b"])
                self.assertTrue(follow.accepted, "mission after the race was rejected")
                self.mock.reset()
                follow_res = self.result_of(follow, 60.0)
                self.assertEqual(follow_res.status, GoalStatus.STATUS_SUCCEEDED)
                self.assertEqual(follow_res.result.outcome,
                                 ExecuteMission.Result.OUTCOME_COMPLETED)

    def test_cancel_racing_with_nav2_success(self):
        # Nav2 finishes SUCCEEDED although a cancel request arrived (cancel accepted, result wins)
        self._cancel_race({"duration": 0.6, "succeed_on_cancel": True},
                          [0.0, 0.55, 0.6, 0.65])

    def test_cancel_racing_with_nav2_rejecting_the_cancel(self):
        # Nav2 already ended the goal: the cancel request to Nav2 is rejected
        self._cancel_race({"duration": 0.6, "cancel_reject": True},
                          [0.1, 0.55, 0.6, 0.7])


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
