"""
AC15 smoke test and AC16 opt-in E2E of mission_sim.launch.py on headless MuJoCo.

AC15 (default): within 120 s /navigate_to_pose and /execute_mission are available and
/mission/status reports IDLE.
AC16 (BUMPERBOT_MISSION_E2E=1): mission [pickup, dropoff] completes and docks.

  ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1 timeout 240 launch_test \
      src/bumperbot_bringup/test/test_mission_sim_mujoco.launch.py
"""
import math
import os
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
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from bumperbot_msgs.action import ExecuteMission
from bumperbot_msgs.msg import MissionStatus
from launch.actions import IncludeLaunchDescription
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_ros import Buffer, TransformException, TransformListener

DEADLINE = 120.0
E2E = os.environ.get("BUMPERBOT_MISSION_E2E") == "1"
E2E_TIMEOUT = 600.0
TOLERANCE = 0.2


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    import mujoco  # noqa: F401  (a missing mujoco is a test failure, not a skip)
    mission = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("bumperbot_bringup"),
                     "launch", "mission_sim.launch.py"),
        launch_arguments={
            "simulator": "mujoco",
            "world_name": "small_house",
            "viewer": "false",
            "use_rviz": "false",
        }.items())
    return launch.LaunchDescription([mission, launch_testing.actions.ReadyToTest()]), {}


class TestMissionSimMujoco(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("mission_sim_smoke_test")
        latched = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=500,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        cls.lock = threading.Lock()
        cls.statuses = []
        cls.node.create_subscription(MissionStatus, "/mission/status", cls.on_status, latched)
        cls.nav_client = ActionClient(cls.node, NavigateToPose, "navigate_to_pose")
        cls.mission_client = ActionClient(cls.node, ExecuteMission, "execute_mission")
        cls.bt_state = cls.node.create_client(GetState, "/bt_navigator/get_state")
        cls.tf_buffer = Buffer()
        cls.tf_listener = TransformListener(cls.tf_buffer, cls.node)
        cls.executor = MultiThreadedExecutor(num_threads=3)
        cls.executor.add_node(cls.node)
        cls.thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def on_status(cls, msg):
        with cls.lock:
            cls.statuses.append(msg)

    def pose(self):
        try:
            t = self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
        except TransformException:
            return None
        return t.transform.translation.x, t.transform.translation.y

    def bt_navigator_active(self):
        if not self.bt_state.service_is_ready():
            return False
        future = self.bt_state.call_async(GetState.Request())
        end = time.time() + 5.0
        while not future.done() and time.time() < end:
            time.sleep(0.02)
        return future.done() and future.result().current_state.id == State.PRIMARY_STATE_ACTIVE

    def test_stack_up(self):
        start = time.time()
        nav_ok = mission_ok = idle = False
        while time.time() - start < DEADLINE:
            nav_ok = self.nav_client.server_is_ready() and self.bt_navigator_active()
            mission_ok = mission_ok or self.mission_client.server_is_ready()
            with self.lock:
                idle = idle or any(s.state == "IDLE" for s in self.statuses)
            if nav_ok and mission_ok and idle:
                print(f"\nAC15 OK after {time.time() - start:.1f}s")
                return
            time.sleep(0.5)
        self.fail(f"within {DEADLINE}s: navigate_to_pose={nav_ok}, execute_mission={mission_ok}, "
                  f"IDLE status={idle}")

    @unittest.skipUnless(E2E, "set BUMPERBOT_MISSION_E2E=1 to run the full mission")
    def test_full_mission(self):
        with open(os.path.join(get_package_share_directory("bumperbot_navigation"),
                               "config", "stations.yaml")) as f:
            stations = yaml.safe_load(f)["stations"]
        self.assertTrue(self.mission_client.wait_for_server(timeout_sec=DEADLINE))
        self.assertTrue(self.nav_client.wait_for_server(timeout_sec=DEADLINE))
        end = time.time() + DEADLINE
        while not self.bt_navigator_active() and time.time() < end:
            time.sleep(0.5)
        self.assertTrue(self.bt_navigator_active(), "bt_navigator did not become active")
        # AMCL needs a TF map->odom before the first goal
        end = time.time() + DEADLINE
        while self.pose() is None and time.time() < end:
            time.sleep(0.5)
        self.assertIsNotNone(self.pose(), "no TF map->base_footprint")

        goal = ExecuteMission.Goal()
        goal.stations = ["pickup", "dropoff"]
        send = self.mission_client.send_goal_async(goal)
        while not send.done():
            time.sleep(0.05)
        handle = send.result()
        self.assertTrue(handle.accepted)
        result_future = handle.get_result_async()

        reached_distances = {}
        order = []
        start = time.time()
        while not result_future.done() and time.time() - start < E2E_TIMEOUT:
            with self.lock:
                last = self.statuses[-1] if self.statuses else None
            if last is not None and last.current_station:
                if not order or order[-1] != last.current_station:
                    order.append(last.current_station)
                if last.state == "DWELLING" and last.current_station not in reached_distances:
                    p = self.pose()
                    s = stations[last.current_station]
                    reached_distances[last.current_station] = (
                        None if p is None else math.hypot(p[0] - s["x"], p[1] - s["y"]))
            time.sleep(0.2)
        self.assertTrue(result_future.done(), f"mission not finished in {E2E_TIMEOUT}s, "
                        f"order={order}")
        res = result_future.result()
        self.assertEqual(res.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(res.result.outcome, ExecuteMission.Result.OUTCOME_COMPLETED)
        self.assertTrue(res.result.docked)
        self.assertEqual(order, ["pickup", "dropoff", "dock"])
        for name in ("pickup", "dropoff"):
            self.assertIn(name, reached_distances)
            self.assertIsNotNone(reached_distances[name])
            self.assertLessEqual(reached_distances[name], TOLERANCE, name)
        p = self.pose()
        self.assertIsNotNone(p)
        self.assertLessEqual(math.hypot(p[0] - stations["dock"]["x"],
                                        p[1] - stations["dock"]["y"]), TOLERANCE)
