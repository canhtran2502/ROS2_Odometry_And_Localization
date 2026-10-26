"""
AC18: smoke test and opt-in E2E of the safety supervisor in the headless MuJoCo stack.

Smoke (default): within 120 s /safety/status is ACTIVE with locked == false, /safety_stop has
exactly one publisher (the supervisor), the supervisor is the only publisher of
bumperbot_controller/cmd_vel_unstamped, exactly one safety_supervisor node (C++) and no
safety_stop node run.
E2E (BUMPERBOT_SAFETY_E2E=1, within 300 s): find a heading with a wall 1-3 m ahead, drive into it
with key_vel 0.3 (the robot stops in front of the wall without contact), back off with -0.2,
then trigger_estop blocks key_vel 0.2 for 3 s and reset lets the robot move again.

  ROS_DOMAIN_ID=93 ROS_LOCALHOST_ONLY=1 timeout 400 launch_test \
      src/bumperbot_bringup/test/test_safety_sim_mujoco.launch.py
  BUMPERBOT_SAFETY_E2E=1 ROS_DOMAIN_ID=93 ROS_LOCALHOST_ONLY=1 timeout 700 launch_test \
      src/bumperbot_bringup/test/test_safety_sim_mujoco.launch.py
"""
import math
import os
import subprocess
import threading
import time
import unittest

os.environ.setdefault("ROS_DOMAIN_ID", "93")
os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")

import launch  # noqa: E402
import launch_testing  # noqa: E402
import launch_testing.actions  # noqa: E402
import launch_testing.markers  # noqa: E402
import pytest  # noqa: E402
import rclpy  # noqa: E402
from ament_index_python.packages import get_package_share_directory  # noqa: E402
from bumperbot_msgs.msg import SafetyStatus  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from launch.actions import IncludeLaunchDescription  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (  # noqa: E402
    qos_profile_sensor_data, QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
    QoSReliabilityPolicy)
from sensor_msgs.msg import LaserScan  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402
from tf2_ros import Buffer, TransformListener  # noqa: E402

SMOKE_DEADLINE = 120.0
E2E = os.environ.get("BUMPERBOT_SAFETY_E2E") == "1"
E2E_LIMIT = 300.0
X_FRONT = 0.10
HALF_WIDTH = 0.09
LATERAL_MARGIN = 0.03


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    import mujoco  # noqa: F401  (a missing mujoco is a test failure, not a skip)
    sim = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("bumperbot_bringup"),
                     "launch", "simulated_robot.launch.py"),
        launch_arguments={
            "simulator": "mujoco",
            "world_name": "small_house",
            "viewer": "false",
            "use_rviz": "false",
        }.items())
    return launch.LaunchDescription([sim, launch_testing.actions.ReadyToTest()]), {}


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SafetySimBase(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("safety_sim_test")
        latched = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        cls.lock = threading.Lock()
        cls.status = None
        cls.scan = None
        cls.truth = None
        cls.truth_time = 0.0
        cls.key = None
        cls.node.create_subscription(SafetyStatus, "/safety/status", cls.on_status, latched)
        cls.node.create_subscription(LaserScan, "/scan", cls.on_scan, qos_profile_sensor_data)
        cls.node.create_subscription(Odometry, "/mujoco/ground_truth", cls.on_truth, 10)
        cls.key_pub = cls.node.create_publisher(Twist, "/key_vel", 10)
        cls.tf_buffer = Buffer()
        cls.tf_listener = TransformListener(cls.tf_buffer, cls.node)
        cls.executor = MultiThreadedExecutor(num_threads=3)
        cls.executor.add_node(cls.node)
        cls.thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.thread.start()
        cls.stop_feed = threading.Event()
        cls.feeder = threading.Thread(target=cls.feed, daemon=True)
        cls.feeder.start()

    @classmethod
    def tearDownClass(cls):
        cls.stop_feed.set()
        cls.feeder.join(timeout=2)
        cls.executor.shutdown()
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def feed(cls):
        """key_vel at 20 Hz while a command is set (twist_mux inputs time out)."""
        while not cls.stop_feed.is_set():
            with cls.lock:
                key = cls.key
            if key is not None:
                msg = Twist()
                msg.linear.x, msg.angular.z = key
                cls.key_pub.publish(msg)
            time.sleep(0.05)

    @classmethod
    def on_status(cls, msg):
        with cls.lock:
            cls.status = msg

    @classmethod
    def on_scan(cls, msg):
        with cls.lock:
            cls.scan = msg

    @classmethod
    def on_truth(cls, msg):
        p = msg.pose.pose
        with cls.lock:
            cls.truth = (p.position.x, p.position.y, yaw_of(p.orientation))
            cls.truth_time = time.monotonic()

    def set_key(self, linear, angular=0.0):
        with self.lock:
            self.__class__.key = (linear, angular)

    def clear_key(self):
        with self.lock:
            self.__class__.key = None

    def get_status(self):
        with self.lock:
            return self.status

    def get_truth(self):
        with self.lock:
            return self.truth


class TestSafetySim01Smoke(SafetySimBase):

    def test_stack_up_with_the_supervisor(self):
        """AC18 smoke: ACTIVE and unlocked within 120 s, single lock publisher, C++ node."""
        start = time.time()
        while time.time() - start < SMOKE_DEADLINE:
            status = self.get_status()
            if status is not None and status.state == "ACTIVE" and not status.locked:
                print("\nAC18 smoke OK after {:.1f}s".format(time.time() - start))
                break
            time.sleep(0.5)
        else:
            self.fail("not ACTIVE within {}s: {}".format(SMOKE_DEADLINE, self.get_status()))
        status = self.get_status()
        self.assertEqual(status.enforcement, "filter")
        locks = self.node.get_publishers_info_by_topic("/safety_stop")
        self.assertEqual([p.node_name for p in locks], ["safety_supervisor"])
        out = self.node.get_publishers_info_by_topic("/bumperbot_controller/cmd_vel_unstamped")
        self.assertEqual([p.node_name for p in out], ["safety_supervisor"])
        names = self.node.get_node_names()
        self.assertEqual(names.count("safety_supervisor"), 1, names)
        self.assertEqual(names.count("safety_stop"), 0, names)
        ps = subprocess.run(["ps", "-eo", "args"], capture_output=True, text=True).stdout
        cpp = [ln for ln in ps.splitlines() if "/lib/bumperbot_utils/safety_supervisor" in ln
               and ".py" not in ln]
        self.assertEqual(len(cpp), 1, cpp)
        self.assertFalse([ln for ln in ps.splitlines() if "safety_supervisor.py" in ln])


@unittest.skipUnless(E2E, "set BUMPERBOT_SAFETY_E2E=1 to run the closed-loop E2E")
class TestSafetySim02E2E(SafetySimBase):

    def laser_yaw(self):
        tf = self.tf_buffer.lookup_transform("base_footprint", "laser_link", rclpy.time.Time())
        t = tf.transform.translation
        return t.x, t.y, yaw_of(tf.transform.rotation)

    def front_clearance(self):
        """Smallest distance from the front edge to a scan point inside the front corridor."""
        with self.lock:
            scan = self.scan
        if scan is None:
            return None
        x0, y0, yaw = self.laser_yaw()
        best = math.inf
        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r) or r < scan.range_min or r > scan.range_max:
                continue
            a = scan.angle_min + i * scan.angle_increment + yaw
            px, py = x0 + r * math.cos(a), y0 + r * math.sin(a)
            if px >= X_FRONT and abs(py) <= HALF_WIDTH + LATERAL_MARGIN:
                best = min(best, px - X_FRONT)
        return best

    def call(self, name):
        client = self.node.create_client(Trigger, name)
        self.assertTrue(client.wait_for_service(timeout_sec=10.0), name)
        future = client.call_async(Trigger.Request())
        end = time.time() + 10.0
        while not future.done() and time.time() < end:
            time.sleep(0.02)
        self.assertTrue(future.done(), name)
        return future.result()

    def distance_moved(self, since):
        now = self.get_truth()
        return math.hypot(now[0] - since[0], now[1] - since[1])

    def test_wall_stop_back_off_and_estop(self):
        start = time.time()
        # the supervisor is up
        while time.time() - start < 120.0:
            st = self.get_status()
            ready = (st is not None and st.state == "ACTIVE" and self.get_truth() is not None
                     and self.scan is not None
                     and self.tf_buffer.can_transform("base_footprint", "laser_link",
                                                      rclpy.time.Time()))
            if ready:
                break
            time.sleep(0.5)
        else:
            self.fail("stack not ready")
        # rotate until a wall is 1-3 m ahead
        self.set_key(0.0, 0.5)
        found = False
        while time.time() - start < 90.0:
            clearance = self.front_clearance()
            if clearance is not None and 1.0 <= clearance <= 3.0:
                found = True
                break
            time.sleep(0.05)
        self.clear_key()
        self.assertTrue(found, "no heading with a wall 1-3 m ahead")
        time.sleep(1.5)
        # drive into the wall
        self.set_key(0.3)
        stopped_at = None
        while time.time() - start < E2E_LIMIT - 150.0:
            time.sleep(0.2)
            st = self.get_status()
            if st.front_zone in ("STOP", "CONTACT") or st.state == "ESTOP":
                stopped_at = time.time()
                break
        self.assertIsNotNone(stopped_at, "never reached the wall zone: {}".format(
            self.get_status()))
        time.sleep(2.5)
        st = self.get_status()
        self.assertNotEqual(st.state, "ESTOP", "latched on approach: {}".format(st))
        self.assertIn(st.front_zone, ("STOP", "WARNING"), st)
        self.assertEqual(st.max_forward_speed, 0.0, st)
        clearance = self.front_clearance()
        print("\nfront clearance at rest: {:.4f} m".format(clearance))
        self.assertGreater(clearance, 0.03)
        before = self.get_truth()
        time.sleep(1.5)
        self.assertLess(self.distance_moved(before), 0.01, "robot still moving into the wall")
        # back off
        self.set_key(-0.2)
        before = self.get_truth()
        time.sleep(2.5)
        self.assertGreaterEqual(self.distance_moved(before), 0.10, "did not back away")
        self.clear_key()
        time.sleep(1.0)
        # trigger_estop: key_vel 0.2 gives no motion, reset lets it move again
        res = self.call("/safety_supervisor/trigger_estop")
        self.assertTrue(res.success, res.message)
        time.sleep(0.5)
        self.assertEqual(self.get_status().state, "ESTOP")
        self.set_key(0.2)
        before = self.get_truth()
        time.sleep(3.0)
        self.assertLess(self.distance_moved(before), 0.01, "moved while the e-stop was latched")
        res = self.call("/safety_supervisor/reset")
        self.assertTrue(res.success, res.message)
        before = self.get_truth()
        time.sleep(2.0)
        self.assertGreaterEqual(self.distance_moved(before), 0.03, "did not move after the reset")
        self.clear_key()
        self.assertLessEqual(time.time() - start, E2E_LIMIT + 90.0)
