"""
AC16 and AC17: battery_monitor and battery_sim as processes against mock servers.

battery_monitor runs next to a mock DockRobot server, a mock execute_mission cancel service, a
mission status publisher and a mock /battery_state. battery_sim runs next to mock /dock/status and
odometry publishers. No simulator is needed. Each scenario starts its own node process (own
process group), so no state leaks between the tests.

  ROS_DOMAIN_ID=90 ROS_LOCALHOST_ONLY=1 launch_test test/test_battery_nodes.launch.py
"""
import os
import signal
import subprocess
import tempfile
import threading
import time
import unittest

os.environ.setdefault("ROS_DOMAIN_ID", "90")
os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")

import launch  # noqa: E402
import launch_testing  # noqa: E402
import launch_testing.actions  # noqa: E402
import launch_testing.markers  # noqa: E402
import pytest  # noqa: E402
import rclpy  # noqa: E402
from action_msgs.srv import CancelGoal  # noqa: E402
from ament_index_python.packages import get_package_prefix  # noqa: E402
from bumperbot_msgs.action import DockRobot  # noqa: E402
from bumperbot_msgs.msg import DockStatus, MissionStatus  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.action import ActionServer, CancelResponse, GoalResponse  # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from sensor_msgs.msg import BatteryState  # noqa: E402
from std_msgs.msg import String  # noqa: E402

CHARGING = BatteryState.POWER_SUPPLY_STATUS_CHARGING
DISCHARGING = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
FULL = BatteryState.POWER_SUPPLY_STATUS_FULL
LATCHED = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
EXIT_CODES = []

MONITOR_PARAMS = {
    "update_rate": 10.0, "battery_timeout": 1.0, "dock_retry_delay": 0.5,
    "dock_request_timeout": 3.0, "mission_cancel_timeout": 60.0, "max_dock_attempts": 3,
    "low_threshold": 0.20, "full_threshold": 0.95}


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return launch.LaunchDescription([launch_testing.actions.ReadyToTest()]), {}


def wait_until(predicate, timeout, period=0.02):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(period)
    return bool(predicate())


class NodeProcess:
    """A battery_monitor.py / battery_sim.py process in its own process group."""

    def __init__(self, script, params, tmp):
        exe = os.path.join(get_package_prefix("bumperbot_navigation"), "lib",
                           "bumperbot_navigation", script)
        args = [exe, "--ros-args"]
        for key, value in params.items():
            text = ("true" if value else "false") if isinstance(value, bool) else str(value)
            args += ["-p", "{}:={}".format(key, text)]
        self.log_path = os.path.join(tmp, script + ".log")
        self.log_file = open(self.log_path, "w")
        self.proc = subprocess.Popen(args, stdout=self.log_file, stderr=subprocess.STDOUT,
                                     start_new_session=True)

    def log(self):
        if not self.log_file.closed:
            self.log_file.flush()
        with open(self.log_path) as f:
            return f.read()

    def stop(self, sig=signal.SIGTERM, timeout=10.0):
        code = self.proc.poll()
        if code is None:
            os.kill(self.proc.pid, sig)
            try:
                code = self.proc.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                os.killpg(self.proc.pid, signal.SIGKILL)
                self.proc.wait(timeout=5)
                code = "killed"
        self.log_file.close()
        return code


class MockWorld(Node):
    """Mock DockRobot server, mission cancel service and topic publishers/recorders."""

    def __init__(self):
        super().__init__("battery_mock_world")
        self.group = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.dock_mode = "hold"        # hold: run until released; reject: reject every goal
        self.dock_outcome = DockRobot.Result.OUTCOME_DOCKED
        self.release = threading.Event()
        self.dock_goals = []           # (t, command)
        self.dock_cancels = []         # t
        self.cancel_requests = []      # dicts: t, uuid, sec, nanosec
        self.monitor_states = []       # (t, state)
        self.battery_samples = []      # (t, percentage, status, present)
        self.battery = None            # (percentage, status) published at 10 Hz when set
        self.mission_state = None
        self.dock_status = None
        self.odom_twist = None
        self.closing = False

        self.server = ActionServer(
            self, DockRobot, "dock_robot", execute_callback=self.execute,
            goal_callback=self.on_goal, cancel_callback=self.on_cancel,
            callback_group=self.group)
        self.create_service(CancelGoal, "execute_mission/_action/cancel_goal", self.on_cancel_all,
                            callback_group=self.group)
        self.battery_pub = self.create_publisher(BatteryState, "battery_state", 10)
        self.mission_pub = self.create_publisher(MissionStatus, "mission/status", LATCHED)
        self.dock_status_pub = self.create_publisher(DockStatus, "dock/status", LATCHED)
        self.odom_pub = self.create_publisher(Odometry, "bumperbot_controller/odom", 10)
        self.create_subscription(String, "battery_monitor/state", self.on_state, LATCHED,
                                 callback_group=self.group)
        self.create_subscription(BatteryState, "battery_state", self.on_battery_sample, 10,
                                 callback_group=self.group)
        self.create_timer(0.1, self.publish_inputs, callback_group=self.group)

    # ----- DockRobot server
    def on_goal(self, request):
        with self.lock:
            self.dock_goals.append((time.monotonic(), request.command))
            mode = self.dock_mode
        return GoalResponse.REJECT if mode == "reject" else GoalResponse.ACCEPT

    def on_cancel(self, _handle):
        with self.lock:
            self.dock_cancels.append(time.monotonic())
        return CancelResponse.ACCEPT

    def execute(self, goal_handle):
        try:
            while not self.release.wait(0.05):
                if self.closing:
                    return DockRobot.Result()
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return DockRobot.Result(outcome=DockRobot.Result.OUTCOME_CANCELED)
            with self.lock:
                outcome = self.dock_outcome
            self.release.clear()
            result = DockRobot.Result(outcome=outcome, docked=outcome == 0)
            goal_handle.succeed()
            return result
        except Exception:  # noqa: BLE001 - the goal may already be terminal at teardown
            return DockRobot.Result()

    def finish_goal(self, outcome):
        with self.lock:
            self.dock_outcome = outcome
        self.release.set()

    def goals_of(self, command):
        with self.lock:
            return [t for t, c in self.dock_goals if c == command]

    # ----- execute_mission cancel service
    def on_cancel_all(self, request, response):
        info = request.goal_info
        with self.lock:
            self.cancel_requests.append({
                "t": time.monotonic(), "uuid": bytes(info.goal_id.uuid),
                "sec": info.stamp.sec, "nanosec": info.stamp.nanosec})
        response.return_code = CancelGoal.Response.ERROR_NONE
        return response

    # ----- recorders and publishers
    def on_state(self, msg):
        with self.lock:
            self.monitor_states.append((time.monotonic(), msg.data))

    def on_battery_sample(self, msg):
        with self.lock:
            self.battery_samples.append(
                (time.monotonic(), msg.percentage, msg.power_supply_status, msg.present))

    def states(self):
        with self.lock:
            raw = [s for _, s in self.monitor_states]
        out = []
        for state in raw:
            if not out or out[-1] != state:
                out.append(state)
        return out

    def publish_inputs(self):
        with self.lock:
            battery = self.battery
            twist = self.odom_twist
        if battery is not None and battery[0] is not None:
            msg = BatteryState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.percentage = float(battery[0])
            msg.power_supply_status = int(battery[1])
            msg.present = True
            self.battery_pub.publish(msg)
        if twist is not None:
            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.twist.twist.linear.x = twist[0]
            odom.twist.twist.angular.z = twist[1]
            self.odom_pub.publish(odom)

    def set_battery(self, percentage, status=DISCHARGING):
        with self.lock:
            self.battery = (percentage, status)

    def stop_battery(self):
        with self.lock:
            self.battery = None

    def set_mission(self, state):
        msg = MissionStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.state = state
        self.mission_state = state
        self.mission_pub.publish(msg)

    def set_dock_status(self, docked):
        msg = DockStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.state = "DOCKED" if docked else "UNDOCKED"
        msg.docked = docked
        self.dock_status_pub.publish(msg)

    def reset(self):
        self.release.clear()
        with self.lock:
            self.dock_mode = "hold"
            self.dock_outcome = DockRobot.Result.OUTCOME_DOCKED
            self.dock_goals = []
            self.dock_cancels = []
            self.cancel_requests = []
            self.monitor_states = []
            self.battery_samples = []
            self.battery = None
            self.odom_twist = None

    def close(self):
        self.closing = True
        time.sleep(0.2)
        self.server.destroy()


class BatteryTestCase(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.world = MockWorld()
        cls.executor = MultiThreadedExecutor(num_threads=8)
        cls.executor.add_node(cls.world)
        cls.thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.thread.start()
        cls.tmp = tempfile.mkdtemp(prefix="battery_test_")
        cls.procs = []

    @classmethod
    def tearDownClass(cls):
        for proc in cls.procs:
            code = proc.stop()
            EXIT_CODES.append((os.path.basename(proc.log_path), code))
        cls.world.close()
        cls.executor.shutdown()
        cls.thread.join(timeout=5)
        cls.world.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        self.world.reset()
        self.world.set_mission("IDLE")
        self.monitor = None
        self.sim = None

    def tearDown(self):
        for proc in (self.monitor, self.sim):
            if proc is not None:
                code = proc.stop()
                EXIT_CODES.append((os.path.basename(proc.log_path), code))
        time.sleep(0.3)

    def start_monitor(self, **overrides):
        params = dict(MONITOR_PARAMS)
        params.update(overrides)
        self.monitor = NodeProcess("battery_monitor.py", params, self.tmp)
        # the monitor publishes its (transient_local) state at once
        self.assertTrue(wait_until(lambda: self.world.states() != [], 20.0),
                        "battery_monitor did not start\n" + self.monitor.log())

    def wait_state(self, state, timeout=5.0):
        self.assertTrue(wait_until(lambda: (self.world.states() or [None])[-1] == state, timeout),
                        "state {} not reached: {}\n{}".format(
                            state, self.world.states(), self.monitor.log()))


# ====================================================================== AC16
class TestBatteryMonitor(BatteryTestCase):

    def test_low_battery_sends_exactly_one_dock_goal(self):
        """AC16a: 0.5 -> 0.15 with no mission: one DOCK goal within 2 s, not more."""
        w = self.world
        self.start_monitor()
        w.set_battery(0.5)
        self.wait_state("MONITORING")
        time.sleep(1.0)
        self.assertEqual(w.dock_goals, [], "a goal was sent at 50 %")
        t0 = time.monotonic()
        w.set_battery(0.15)
        self.assertTrue(wait_until(lambda: len(w.goals_of(DockRobot.Goal.COMMAND_DOCK)) >= 1, 2.0),
                        "no DOCK goal within 2 s\n" + self.monitor.log())
        self.assertLessEqual(w.goals_of(DockRobot.Goal.COMMAND_DOCK)[0] - t0, 2.0)
        time.sleep(2.0)
        self.assertEqual(len(w.dock_goals), 1, w.dock_goals)
        self.assertEqual(w.dock_goals[0][1], DockRobot.Goal.COMMAND_DOCK)
        self.assertEqual(w.cancel_requests, [], "a mission cancel was sent without a mission")
        self.assertEqual(w.states()[-1], "DOCKING")

    def test_active_mission_is_canceled_before_docking(self):
        """AC16b: cancel-all (zero goal id) first, DOCK only after the mission status ends."""
        w = self.world
        self.start_monitor()
        w.set_battery(0.5)
        w.set_mission("NAVIGATING")
        self.wait_state("MONITORING")
        w.set_battery(0.15)
        self.assertTrue(wait_until(lambda: len(w.cancel_requests) >= 1, 3.0),
                        "no CancelGoal request\n" + self.monitor.log())
        request = w.cancel_requests[0]
        self.assertEqual(request["uuid"], bytes(16), "the goal id is not zero")
        self.assertEqual((request["sec"], request["nanosec"]), (0, 0))
        self.wait_state("CANCELING_MISSION")
        time.sleep(1.5)
        self.assertEqual(w.dock_goals, [], "DOCK sent before the mission ended")
        t_end = time.monotonic()
        w.set_mission("CANCELED")
        self.assertTrue(wait_until(lambda: len(w.dock_goals) >= 1, 3.0),
                        "no DOCK after the mission ended\n" + self.monitor.log())
        self.assertGreater(w.dock_goals[0][0], t_end)
        self.assertGreater(w.dock_goals[0][0], request["t"])
        time.sleep(1.0)
        self.assertEqual(len(w.dock_goals), 1)

    def test_charge_then_undock_sequence(self):
        """AC16c: DOCKED, charging to 0.96, exactly one UNDOCK, state sequence."""
        w = self.world
        self.start_monitor()
        w.set_battery(0.5)
        self.wait_state("MONITORING")
        w.set_battery(0.15)
        self.assertTrue(wait_until(lambda: len(w.dock_goals) >= 1, 3.0))
        self.wait_state("DOCKING")
        w.set_dock_status(True)
        w.finish_goal(DockRobot.Result.OUTCOME_DOCKED)
        self.wait_state("CHARGING")
        w.set_battery(0.5, CHARGING)
        time.sleep(1.0)
        self.assertEqual(len(w.dock_goals), 1)
        w.set_battery(0.96, CHARGING)
        self.assertTrue(wait_until(lambda: len(w.goals_of(DockRobot.Goal.COMMAND_UNDOCK)) >= 1,
                                   3.0), "no UNDOCK goal\n" + self.monitor.log())
        self.wait_state("UNDOCKING")
        w.set_dock_status(False)
        w.set_battery(0.96, DISCHARGING)
        w.finish_goal(DockRobot.Result.OUTCOME_UNDOCKED)
        self.wait_state("MONITORING")
        time.sleep(2.0)
        self.assertEqual(len(w.goals_of(DockRobot.Goal.COMMAND_UNDOCK)), 1)
        self.assertEqual(len(w.goals_of(DockRobot.Goal.COMMAND_DOCK)), 1)
        states = [s for s in w.states() if s != "WAITING_FOR_BATTERY"]
        self.assertEqual(states, ["MONITORING", "DOCKING", "CHARGING", "UNDOCKING", "MONITORING"])

    def test_rejected_dock_retries_then_error(self):
        """AC16d: rejected goals are retried every dock_retry_delay, ERROR after the timeout."""
        w = self.world
        w.dock_mode = "reject"
        self.start_monitor()
        w.set_battery(0.5)
        self.wait_state("MONITORING")
        w.set_battery(0.1)
        self.assertTrue(wait_until(lambda: len(w.dock_goals) >= 1, 3.0))
        t_first = w.dock_goals[0][0]
        self.wait_state("ERROR", 3.0 + 3.0)
        t_error = next(t for t, s in w.monitor_states if s == "ERROR")
        self.assertGreaterEqual(t_error - t_first, 2.5)
        goals = [t for t, _ in w.dock_goals]
        self.assertGreaterEqual(len(goals), 4, goals)
        gaps = [b - a for a, b in zip(goals, goals[1:])]
        for gap in gaps:
            self.assertGreaterEqual(gap, 0.4, gaps)
            self.assertLessEqual(gap, 1.5, gaps)
        count = len(w.dock_goals)
        time.sleep(1.5)
        self.assertEqual(len(w.dock_goals), count, "goals after ERROR")
        self.assertEqual(w.states()[-1], "ERROR")

    def test_silent_battery_never_triggers(self):
        """AC16e: a silent battery gives WAITING_FOR_BATTERY and no goal; data resumes it."""
        w = self.world
        self.start_monitor()
        w.set_battery(0.5)
        self.wait_state("MONITORING")
        w.stop_battery()
        self.wait_state("WAITING_FOR_BATTERY", 1.0 + 2.0)
        time.sleep(2.0)
        self.assertEqual(w.dock_goals, [], "a goal without battery data")
        self.assertEqual(w.states()[-1], "WAITING_FOR_BATTERY")
        # control: the same monitor reacts again when low data arrives
        w.set_battery(0.1)
        self.assertTrue(wait_until(lambda: len(w.dock_goals) >= 1, 3.0))

    def test_sigint_cancels_the_active_dock_goal(self):
        """AC16f: SIGINT while DOCK is active cancels the goal and exits with code 0."""
        w = self.world
        self.start_monitor()
        w.set_battery(0.1)
        self.assertTrue(wait_until(lambda: len(w.dock_goals) >= 1, 5.0), self.monitor.log())
        time.sleep(1.0)
        self.assertEqual(w.dock_cancels, [])
        code = self.monitor.stop(signal.SIGINT, 10.0)
        EXIT_CODES.append(("battery_monitor.py SIGINT", code))
        self.assertEqual(code, 0)
        self.assertTrue(wait_until(lambda: len(w.dock_cancels) >= 1, 3.0),
                        "the DockRobot goal was not canceled")
        self.monitor = None


# ====================================================================== AC17
class TestBatterySim(BatteryTestCase):

    def start_sim(self, **overrides):
        params = {"initial_percentage": 0.5, "charge_rate": 0.05, "publish_rate": 2.0}
        params.update(overrides)
        self.sim = NodeProcess("battery_sim.py", params, self.tmp)
        self.assertTrue(wait_until(lambda: len(self.world.battery_samples) >= 2, 20.0),
                        "battery_sim did not publish\n" + self.sim.log())

    def sample_slope(self, seconds):
        """Return (percentage change per second, rate in Hz) over a time window."""
        w = self.world
        with w.lock:
            start = len(w.battery_samples)
        t0 = time.monotonic()
        time.sleep(seconds)
        with w.lock:
            samples = w.battery_samples[start:]
        self.assertGreaterEqual(len(samples), 3, "too few samples")
        span = samples[-1][0] - samples[0][0]
        slope = (samples[-1][1] - samples[0][1]) / span
        return slope, (len(samples) - 1) / span, samples, time.monotonic() - t0

    def test_publishes_battery_state(self):
        """AC17: 2 Hz +-30 %, present, 0 <= percentage <= 1, DISCHARGING without a dock."""
        w = self.world
        self.start_sim()
        slope, rate, samples, _ = self.sample_slope(6.0)
        self.assertGreaterEqual(rate, 2.0 * 0.7, rate)
        self.assertLessEqual(rate, 2.0 * 1.3, rate)
        for _, percentage, status, present in samples:
            self.assertTrue(present)
            self.assertTrue(0.0 <= percentage <= 1.0)
            self.assertEqual(status, DISCHARGING)
        self.assertLess(slope, 0.0, "the idle battery should drain")
        self.assertAlmostEqual(slope, -0.0005, delta=0.0003)
        self.assertTrue(w.battery_samples)

    def test_charges_while_docked(self):
        """AC17: CHARGING and a rising percentage after /dock/status docked == true."""
        w = self.world
        self.start_sim()
        w.set_dock_status(True)
        self.assertTrue(wait_until(lambda: w.battery_samples[-1][2] == CHARGING, 3.0),
                        "status did not become CHARGING")
        slope, _, samples, _ = self.sample_slope(4.0)
        self.assertGreater(slope, 0.03, slope)
        self.assertAlmostEqual(slope, 0.05, delta=0.015)
        w.set_dock_status(False)
        self.assertTrue(wait_until(lambda: w.battery_samples[-1][2] == DISCHARGING, 3.0))

    def test_moving_drains_faster(self):
        """AC17: a 0.2 m/s odometry twist drains faster than idle."""
        w = self.world
        self.start_sim(initial_percentage=0.9)
        idle, _, _, _ = self.sample_slope(5.0)
        with w.lock:
            w.odom_twist = (0.2, 0.0)
        time.sleep(0.7)
        moving, _, _, _ = self.sample_slope(5.0)
        self.assertLess(moving, idle - 0.0004, (idle, moving))
        self.assertAlmostEqual(moving, -(0.0005 + 0.004 * 0.2), delta=0.0003)

    def test_percentage_override_via_ros2_param(self):
        """AC17: ros2 param set /battery_sim percentage_override 0.1 -> 0.1 +- 0.01 within 1 s."""
        w = self.world
        self.start_sim(initial_percentage=0.8)
        self.assertTrue(wait_until(lambda: w.battery_samples[-1][1] > 0.7, 3.0))
        t0 = time.monotonic()
        res = subprocess.run(
            ["ros2", "param", "set", "/battery_sim", "percentage_override", "0.1"],
            capture_output=True, text=True, timeout=30)
        self.assertEqual(res.returncode, 0, res.stdout + res.stderr)
        self.assertIn("successful", res.stdout.lower())
        self.assertTrue(wait_until(lambda: abs(w.battery_samples[-1][1] - 0.1) <= 0.01, 1.0),
                        "last percentage {}".format(w.battery_samples[-1][1]))
        self.assertLessEqual(time.monotonic() - t0, 30.0)
        # an out-of-range value is rejected and the charge stays
        res = subprocess.run(
            ["ros2", "param", "set", "/battery_sim", "percentage_override", "5.0"],
            capture_output=True, text=True, timeout=30)
        self.assertNotIn("successful", res.stdout.lower().replace("set parameter failed", ""))
        time.sleep(1.2)
        self.assertLess(w.battery_samples[-1][1], 0.2)


@launch_testing.post_shutdown_test()
class TestNodesExitCleanly(unittest.TestCase):

    def test_exit_codes(self):
        self.assertTrue(EXIT_CODES, "no node was ever stopped")
        bad = [(name, code) for name, code in EXIT_CODES if code != 0]
        self.assertFalse(bad, bad)
