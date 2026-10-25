"""
AC24 smoke test and AC25 opt-in E2E of dock_sim.launch.py on headless MuJoCo.

AC24 (default): within 150 s /dock_robot is available, /dock/status is UNDOCKED with
dock_pose == (-0.5, 0, 0), /battery_state arrives, battery_monitor/state is MONITORING and
/mujoco/ground_truth is within 0.02 m of (0, 0).
AC25 (BUMPERBOT_DOCK_E2E=1): mission [pickup, dropoff], a low battery during the mission, automatic
dock, charge, undock.

  ROS_DOMAIN_ID=91 ROS_LOCALHOST_ONLY=1 timeout 300 launch_test \
      src/bumperbot_bringup/test/test_dock_sim_mujoco.launch.py
  BUMPERBOT_DOCK_E2E=1 ROS_DOMAIN_ID=91 ROS_LOCALHOST_ONLY=1 timeout 1100 launch_test \
      src/bumperbot_bringup/test/test_dock_sim_mujoco.launch.py
"""
import math
import os
import subprocess
import tempfile
import threading
import time
import unittest

os.environ.setdefault("ROS_DOMAIN_ID", "91")
os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")

import launch  # noqa: E402
import launch_testing  # noqa: E402
import launch_testing.actions  # noqa: E402
import launch_testing.markers  # noqa: E402
import pytest  # noqa: E402
import rclpy  # noqa: E402
from action_msgs.msg import GoalStatus, GoalStatusArray  # noqa: E402
from ament_index_python.packages import get_package_share_directory  # noqa: E402
from bumperbot_msgs.action import DockRobot, ExecuteMission  # noqa: E402
from bumperbot_msgs.msg import DockStatus, MissionStatus  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from launch.actions import IncludeLaunchDescription  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (  # noqa: E402
    QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy)
from sensor_msgs.msg import BatteryState  # noqa: E402
from std_msgs.msg import String  # noqa: E402

DEADLINE = 150.0
E2E = os.environ.get("BUMPERBOT_DOCK_E2E") == "1"
E2E_LIMIT = 900.0
CHARGER = (-0.5, 0.0, 0.0)
STAGING = (0.0, 0.0, 0.0)
CHARGE_RATE = 0.05
LOW_OVERRIDE = 0.15
TMP = tempfile.mkdtemp(prefix="dock_sim_test_")
BATTERY_SIM_CONFIG = os.path.join(TMP, "battery_sim_test.yaml")
with open(BATTERY_SIM_CONFIG, "w") as _f:
    _f.write("battery_sim:\n  ros__parameters:\n    charge_rate: {}\n".format(CHARGE_RATE))


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    import mujoco  # noqa: F401  (a missing mujoco is a test failure, not a skip)
    dock = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("bumperbot_bringup"),
                     "launch", "dock_sim.launch.py"),
        launch_arguments={
            "simulator": "mujoco",
            "world_name": "small_house",
            "viewer": "false",
            "use_rviz": "false",
            "battery_initial": "1.0",
            "battery_sim_config": BATTERY_SIM_CONFIG,
        }.items())
    return launch.LaunchDescription([dock, launch_testing.actions.ReadyToTest()]), {}


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class DockSimBase(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("dock_sim_test")
        latched = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=500,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        cls.lock = threading.Lock()
        cls.dock_status = []        # (t, DockStatus)
        cls.battery = []            # (t, percentage, status)
        cls.monitor_states = []     # (t, state)
        cls.mission_states = []     # (t, state)
        cls.truth = None            # (x, y, yaw)
        cls.mux_out = None          # (t, linear.x, angular.z)
        cls.dock_goal_status = []   # statuses of /dock_robot goals
        cls.node.create_subscription(DockStatus, "/dock/status", cls.on_dock_status, latched)
        cls.node.create_subscription(BatteryState, "/battery_state", cls.on_battery, 10)
        cls.node.create_subscription(String, "/battery_monitor/state", cls.on_monitor, latched)
        cls.node.create_subscription(MissionStatus, "/mission/status", cls.on_mission, latched)
        cls.node.create_subscription(Odometry, "/mujoco/ground_truth", cls.on_truth, 10)
        cls.node.create_subscription(Twist, "/bumperbot_controller/cmd_vel_unstamped",
                                     cls.on_mux, 10)
        cls.node.create_subscription(GoalStatusArray, "/dock_robot/_action/status",
                                     cls.on_goal_status, latched)
        cls.dock_client = ActionClient(cls.node, DockRobot, "dock_robot")
        cls.mission_client = ActionClient(cls.node, ExecuteMission, "execute_mission")
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
    def on_dock_status(cls, msg):
        with cls.lock:
            cls.dock_status.append((time.monotonic(), msg))

    @classmethod
    def on_battery(cls, msg):
        with cls.lock:
            cls.battery.append((time.monotonic(), msg.percentage, msg.power_supply_status))

    @classmethod
    def on_monitor(cls, msg):
        with cls.lock:
            cls.monitor_states.append((time.monotonic(), msg.data))

    @classmethod
    def on_mission(cls, msg):
        with cls.lock:
            cls.mission_states.append((time.monotonic(), msg.state))

    @classmethod
    def on_truth(cls, msg):
        p = msg.pose.pose
        with cls.lock:
            cls.truth = (p.position.x, p.position.y, yaw_of(p.orientation))

    @classmethod
    def on_mux(cls, msg):
        with cls.lock:
            cls.mux_out = (time.monotonic(), msg.linear.x, msg.angular.z)

    @classmethod
    def on_goal_status(cls, msg):
        with cls.lock:
            cls.dock_goal_status = [s.status for s in msg.status_list]

    def last_dock_status(self):
        with self.lock:
            return self.dock_status[-1][1] if self.dock_status else None

    def monitor_state(self):
        with self.lock:
            return self.monitor_states[-1][1] if self.monitor_states else None

    def ground_truth(self):
        with self.lock:
            return self.truth


class TestDockSim01StackUp(DockSimBase):

    def test_stack_up(self):
        """AC24: the headless stack publishes everything within 150 s."""
        start = time.time()
        found = {}
        while time.time() - start < DEADLINE:
            found["dock_robot"] = found.get("dock_robot") or self.dock_client.server_is_ready()
            status = self.last_dock_status()
            if status is not None and status.state == "UNDOCKED":
                found["status"] = (status.dock_pose.x, status.dock_pose.y, status.dock_pose.theta)
            with self.lock:
                found["battery"] = found.get("battery") or bool(self.battery)
            found["monitoring"] = found.get("monitoring") or self.monitor_state() == "MONITORING"
            truth = self.ground_truth()
            if truth is not None:
                found["truth"] = math.hypot(truth[0], truth[1])
            ok = (found.get("dock_robot") and found.get("battery") and found.get("monitoring")
                  and "status" in found and found.get("truth", 1.0) <= 0.02)
            if ok:
                print("\nAC24 OK after {:.1f}s".format(time.time() - start))
                break
            time.sleep(0.5)
        else:
            self.fail("within {}s: {}".format(DEADLINE, found))
        dock_pose = found["status"]
        self.assertAlmostEqual(dock_pose[0], CHARGER[0], delta=1e-6)
        self.assertAlmostEqual(dock_pose[1], CHARGER[1], delta=1e-6)
        self.assertAlmostEqual(dock_pose[2], CHARGER[2], delta=1e-6)
        status = self.last_dock_status()
        self.assertEqual(status.frame_id, "map")
        self.assertFalse(status.docked)
        self.assertAlmostEqual(status.staging_pose.x, 0.0, delta=1e-6)
        with self.lock:
            percentages = [p for _, p, _ in self.battery]
        self.assertTrue(all(0.0 <= p <= 1.0 for p in percentages))
        self.assertGreater(percentages[-1], 0.9, "the battery should start at battery_initial")

    def test_ground_truth_topic_matches_the_simulator_frame(self):
        """AC22 (launch part): /mujoco/ground_truth is published at the spawn pose."""
        end = time.time() + 30.0
        while self.ground_truth() is None and time.time() < end:
            time.sleep(0.2)
        truth = self.ground_truth()
        self.assertIsNotNone(truth, "no /mujoco/ground_truth message")
        self.assertLessEqual(math.hypot(truth[0], truth[1]), 0.02)
        self.assertLessEqual(abs(truth[2]), 0.05)
        info = self.node.get_publishers_info_by_topic("/mujoco/ground_truth")
        self.assertTrue(any(i.node_name == "mujoco_sim" for i in info),
                        [i.node_name for i in info])


class TestDockSim02E2E(DockSimBase):

    def truth_error(self, pose):
        x, y, yaw = self.ground_truth()
        c, s = math.cos(pose[2]), math.sin(pose[2])
        dx, dy = x - pose[0], y - pose[1]
        rel_x, rel_y = c * dx + s * dy, -s * dx + c * dy
        return math.hypot(rel_x, rel_y), abs(math.atan2(math.sin(yaw - pose[2]),
                                                        math.cos(yaw - pose[2])))

    def wait_for(self, predicate, timeout, what):
        end = time.time() + timeout
        while time.time() < end:
            if predicate():
                return
            time.sleep(0.2)
        self.fail("timeout waiting for {} (monitor {}, dock {}, mission {})".format(
            what, self.monitor_state(),
            self.last_dock_status().state if self.last_dock_status() else None,
            self.mission_states[-1] if self.mission_states else None))

    @unittest.skipUnless(E2E, "set BUMPERBOT_DOCK_E2E=1 to run the low-battery dock sequence")
    def test_low_battery_dock_charge_undock(self):
        """AC25: low battery during a mission -> cancel -> dock -> charge -> undock."""
        t_start = time.time()
        self.assertTrue(self.mission_client.wait_for_server(timeout_sec=DEADLINE))
        self.assertTrue(self.dock_client.wait_for_server(timeout_sec=DEADLINE))
        self.wait_for(lambda: self.monitor_state() == "MONITORING", DEADLINE, "MONITORING")
        self.wait_for(lambda: self.ground_truth() is not None, 30.0, "ground truth")
        # AMCL needs a moment before the first Nav2 goal; the mission waits for Nav2 itself
        time.sleep(10.0)

        goal = ExecuteMission.Goal()
        goal.stations = ["pickup", "dropoff"]
        send = self.mission_client.send_goal_async(goal)
        while not send.done():
            time.sleep(0.05)
        handle = send.result()
        self.assertTrue(handle.accepted)
        result_future = handle.get_result_async()

        def mission_state():
            with self.lock:
                return self.mission_states[-1][1] if self.mission_states else None

        self.wait_for(lambda: mission_state() == "NAVIGATING", 300.0, "mission NAVIGATING")
        for _ in range(4):  # the ros2 daemon may not have discovered the node yet
            res = subprocess.run(
                ["ros2", "param", "set", "/battery_sim", "percentage_override", str(LOW_OVERRIDE)],
                capture_output=True, text=True, timeout=60)
            if "Node not found" not in res.stdout + res.stderr:
                break
            time.sleep(2.0)
        self.assertEqual(res.returncode, 0, res.stdout + res.stderr)

        self.wait_for(result_future.done, 400.0, "the mission result")
        mission = result_future.result()
        self.assertEqual(mission.status, GoalStatus.STATUS_CANCELED)
        self.assertEqual(mission.result.outcome, ExecuteMission.Result.OUTCOME_CANCELED)
        self.assertTrue(mission.result.docked, "the mission did not return to the staging pose")

        # a DOCK goal follows automatically
        def docked():
            status = self.last_dock_status()
            return status is not None and status.docked and status.state in (
                "DOCKED", "CHARGING")

        self.wait_for(docked, 300.0, "DOCKED")
        time.sleep(1.5)
        pos, yaw = self.truth_error(CHARGER)
        self.assertLessEqual(pos, 0.05, "docked position error {:.3f} m".format(pos))
        self.assertLessEqual(yaw, 0.1, "docked yaw error {:.3f} rad".format(yaw))
        with self.lock:
            succeeded = [s for s in self.dock_goal_status if s == GoalStatus.STATUS_SUCCEEDED]
        self.assertGreaterEqual(len(succeeded), 1, "no DockRobot goal succeeded")

        self.wait_for(lambda: self.last_dock_status().state == "CHARGING"
                      or self.last_dock_status().charging, 60.0, "CHARGING")
        # the charge reaches full_threshold and the monitor undocks by itself
        self.wait_for(lambda: self.monitor_state() == "UNDOCKING", 120.0, "UNDOCKING")
        with self.lock:
            percentages = [p for _, p, _ in self.battery]
        self.assertGreaterEqual(max(percentages[-10:]), 0.95 - 1e-3)
        self.wait_for(lambda: self.monitor_state() == "MONITORING", 120.0, "MONITORING")
        status = self.last_dock_status()
        self.assertFalse(status.docked)
        time.sleep(2.0)
        pos, _ = self.truth_error(STAGING)
        self.assertLessEqual(pos, 0.1, "undocked {:.3f} m from the staging pose".format(pos))
        with self.lock:
            self.assertGreaterEqual(
                len([s for s in self.dock_goal_status if s == GoalStatus.STATUS_SUCCEEDED]), 2,
                "expected a successful DOCK and a successful UNDOCK")
            mux = self.mux_out
        self.assertIsNotNone(mux)
        self.assertEqual((mux[1], mux[2]), (0.0, 0.0), "the last twist_mux output is not zero")
        self.assertEqual(self.monitor_state(), "MONITORING")
        self.assertLessEqual(time.time() - t_start, E2E_LIMIT)
