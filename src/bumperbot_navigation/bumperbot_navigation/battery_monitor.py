#!/usr/bin/env python3
"""
Battery monitor: starts an automatic dock sequence when the battery is low.

It watches /battery_state. Below low_threshold it cancels an active mission (mission_manager then
drives back to the dock station), sends DockRobot DOCK, waits in CHARGING until full_threshold
and then sends DockRobot UNDOCK. The state machine lives in battery_logic.py
(BatteryMonitorLogic); this node only does the ROS I/O. Simulator independent.
"""
import math
import signal
import sys
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from bumperbot_msgs.action import DockRobot
from bumperbot_msgs.msg import DockStatus, MissionStatus
from rclpy._rclpy_pybind11 import RCLError
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from bumperbot_navigation.battery_logic import (
    ACTION_ABANDON_DOCK, ACTION_CANCEL_MISSION, ACTION_SEND_DOCK, ACTION_SEND_UNDOCK,
    RESULT_ALREADY_DOCKED, RESULT_DOCKED, RESULT_FAILED, RESULT_REJECTED, RESULT_UNDOCKED,
    BatteryMonitorLogic, FallbackClock, MonitorParams)

# Seconds after a DOCKED result during which a stale "not docked" status is ignored
DOCKED_GRACE = 1.0
# Seconds without a /mission/status publisher after which a latched active mission is stale
MISSION_STALE_TIMEOUT = 5.0
MISSION_ACTIVE_STATES = (
    MissionStatus.STATE_NAVIGATING, MissionStatus.STATE_DWELLING,
    MissionStatus.STATE_RETURNING_TO_DOCK)


class BatteryMonitor(Node):
    def __init__(self):
        super().__init__("battery_monitor")

        self.declare_parameter("low_threshold", 0.20)
        self.declare_parameter("full_threshold", 0.95)
        self.declare_parameter("rearm_hysteresis", 0.05)
        self.declare_parameter("battery_timeout", 5.0)
        self.declare_parameter("mission_cancel_timeout", 180.0)
        self.declare_parameter("dock_retry_delay", 5.0)
        self.declare_parameter("dock_request_timeout", 120.0)
        self.declare_parameter("max_dock_attempts", 3)
        self.declare_parameter("undock_when_full", True)
        self.declare_parameter("update_rate", 2.0)
        self.declare_parameter("clock_stall_timeout", 5.0)

        get = self.get_parameter
        self.params = MonitorParams(
            low_threshold=float(get("low_threshold").value),
            full_threshold=float(get("full_threshold").value),
            rearm_hysteresis=float(get("rearm_hysteresis").value),
            battery_timeout=float(get("battery_timeout").value),
            mission_cancel_timeout=float(get("mission_cancel_timeout").value),
            dock_retry_delay=float(get("dock_retry_delay").value),
            dock_request_timeout=float(get("dock_request_timeout").value),
            max_dock_attempts=int(get("max_dock_attempts").value),
            undock_when_full=bool(get("undock_when_full").value))
        self.update_rate = float(get("update_rate").value)
        self.clock_stall_timeout = float(get("clock_stall_timeout").value)
        self.validate()

        self.group = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.logic = BatteryMonitorLogic(self.params)
        self.fallback_clock = FallbackClock(self.clock_stall_timeout)
        self.dock_handle = None
        # Identifies the pending dock request: callbacks of an abandoned request are ignored
        self.request_id = 0
        self.mission_status_seen = False
        self.mission_stale = False
        self.mission_gone_since = None
        self.docked_result_time = -math.inf
        self.published_state = None

        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(String, "battery_monitor/state", state_qos)

        self.create_subscription(
            BatteryState, "battery_state", self.battery_callback, 10, callback_group=self.group)
        self.create_subscription(
            DockStatus, "dock/status", self.dock_status_callback, state_qos,
            callback_group=self.group)
        self.create_subscription(
            MissionStatus, "mission/status", self.mission_callback, state_qos,
            callback_group=self.group)

        self.dock_client = ActionClient(
            self, DockRobot, "dock_robot", callback_group=self.group)
        self.cancel_client = self.create_client(
            CancelGoal, "execute_mission/_action/cancel_goal", callback_group=self.group)
        self.create_service(
            Trigger, "~/reset", self.reset_callback, callback_group=self.group)

        self.publish_state()
        # Wall timer: the logic time has its own wall fallback for a stalled node clock
        self.timer = self.create_timer(
            1.0 / self.update_rate, self.update, callback_group=self.group,
            clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info("Battery monitor ready (low {:.2f}, full {:.2f})".format(
            self.params.low_threshold, self.params.full_threshold))

    def validate(self):
        par = self.params
        positive = {
            "low_threshold": par.low_threshold, "full_threshold": par.full_threshold,
            "battery_timeout": par.battery_timeout,
            "mission_cancel_timeout": par.mission_cancel_timeout,
            "dock_retry_delay": par.dock_retry_delay,
            "dock_request_timeout": par.dock_request_timeout,
            "update_rate": self.update_rate, "clock_stall_timeout": self.clock_stall_timeout}
        for name, value in positive.items():
            if not (math.isfinite(value) and value > 0.0):
                raise ValueError("parameter '{}' must be a positive number".format(name))
        if not (math.isfinite(par.rearm_hysteresis) and par.rearm_hysteresis >= 0.0):
            raise ValueError("parameter 'rearm_hysteresis' must not be negative")
        if par.max_dock_attempts < 1:
            raise ValueError("parameter 'max_dock_attempts' must be at least 1")
        if par.low_threshold >= par.full_threshold:
            raise ValueError("low_threshold must be below full_threshold")

    # ----- time -----
    def logic_now(self):
        return self.fallback_clock.now(
            self.get_clock().now().nanoseconds * 1e-9, time.monotonic())

    # ----- subscriptions -----
    def battery_callback(self, msg):
        with self.lock:
            self.logic.on_battery(
                self.logic_now(), float(msg.percentage),
                msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING)

    def dock_status_callback(self, msg):
        with self.lock:
            if not msg.docked and time.monotonic() - self.docked_result_time < DOCKED_GRACE:
                return
            self.logic.on_dock_status(bool(msg.docked))

    def mission_callback(self, msg):
        with self.lock:
            self.mission_stale = False
            self.mission_status_seen = True
            self.logic.on_mission_status(msg.state in MISSION_ACTIVE_STATES)

    def check_stale_mission(self):
        """Treat a latched active mission as gone when mission_manager no longer publishes."""
        has_publisher = self.count_publishers("mission/status") > 0
        with self.lock:
            if has_publisher or not self.mission_status_seen or self.mission_stale:
                self.mission_gone_since = None
                return
            now = time.monotonic()
            if self.mission_gone_since is None:
                self.mission_gone_since = now
            elif now - self.mission_gone_since >= MISSION_STALE_TIMEOUT:
                self.mission_stale = True
                self.logic.on_mission_status(False)
                stale = True
            else:
                return
        if stale:
            self.get_logger().warn(
                "mission_manager does not publish /mission/status any more: "
                "the last mission status is treated as stale (no active mission)")

    # ----- periodic update -----
    def update(self):
        self.check_stale_mission()
        server_ready = self.dock_client.server_is_ready()
        with self.lock:
            actions = self.logic.update(self.logic_now(), server_ready)
        for action in actions:
            if action == ACTION_CANCEL_MISSION:
                self.cancel_missions()
            elif action == ACTION_ABANDON_DOCK:
                self.abandon_dock_request()
            elif action == ACTION_SEND_DOCK:
                self.send_dock_goal(DockRobot.Goal.COMMAND_DOCK)
            elif action == ACTION_SEND_UNDOCK:
                self.send_dock_goal(DockRobot.Goal.COMMAND_UNDOCK)
        self.publish_state()

    def publish_state(self):
        with self.lock:
            state = self.logic.state
            message = self.logic.message
        if state != self.published_state:
            self.get_logger().info("State: {}{}".format(
                state, " ({})".format(message) if message else ""))
            self.published_state = state
        msg = String()
        msg.data = state
        try:
            self.state_pub.publish(msg)
        except Exception:  # the context may already be gone during shutdown
            pass

    # ----- requests -----
    def cancel_missions(self):
        """Cancel every ExecuteMission goal: CancelGoal with a zero goal info means cancel all."""
        if not self.cancel_client.service_is_ready():
            self.get_logger().warn("The execute_mission cancel service is not available")
            return
        self.get_logger().info("Low battery: canceling the active mission")
        self.cancel_client.call_async(CancelGoal.Request()).add_done_callback(
            self.cancel_response_callback)

    def cancel_response_callback(self, future):
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn("The mission cancel request failed: {}".format(exc))
            return
        if response is not None and response.return_code != CancelGoal.Response.ERROR_NONE:
            # The logic repeats the cancel while the mission stays active
            self.get_logger().warn("The mission cancel was refused (code {})".format(
                response.return_code))

    def abandon_dock_request(self):
        """Forget the pending dock request: late callbacks are ignored, a late goal canceled."""
        with self.lock:
            self.request_id += 1
            handle = self.dock_handle
            self.dock_handle = None
        self.get_logger().warn("The DockRobot request got no answer in time: counted as failed")
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warn("Could not cancel the DockRobot goal: {}".format(exc))

    def send_dock_goal(self, command):
        name = "DOCK" if command == DockRobot.Goal.COMMAND_DOCK else "UNDOCK"
        if not self.dock_client.server_is_ready():
            self.get_logger().warn("The dock_robot action server is not available")
            with self.lock:
                request_id = self.request_id
            self.report_result(RESULT_REJECTED, request_id)
            return
        self.get_logger().info("Sending DockRobot {}".format(name))
        goal = DockRobot.Goal()
        goal.command = command
        with self.lock:
            request_id = self.request_id
        self.dock_client.send_goal_async(goal).add_done_callback(
            lambda future: self.goal_response_callback(future, request_id))

    def goal_response_callback(self, future, request_id):
        try:
            handle = future.result()
        except Exception as exc:
            self.get_logger().warn("DockRobot goal request failed: {}".format(exc))
            self.report_result(RESULT_REJECTED, request_id)
            return
        with self.lock:
            current = request_id == self.request_id
            if current and handle is not None and handle.accepted:
                self.dock_handle = handle
                self.logic.on_dock_accepted()
        if not current:
            # The request was abandoned meanwhile: a goal accepted late must not keep running
            if handle is not None and handle.accepted:
                try:
                    handle.cancel_goal_async()
                except Exception as exc:
                    self.get_logger().warn("Could not cancel the DockRobot goal: {}".format(exc))
            return
        if handle is None or not handle.accepted:
            self.get_logger().warn("DockRobot goal rejected")
            self.report_result(RESULT_REJECTED, request_id)
            return
        handle.get_result_async().add_done_callback(
            lambda result_future: self.result_callback(result_future, request_id))

    def result_callback(self, future, request_id):
        with self.lock:
            if request_id != self.request_id:
                return
            self.dock_handle = None
        try:
            wrapped = future.result()
        except Exception as exc:
            self.get_logger().warn("DockRobot result failed: {}".format(exc))
            self.report_result(RESULT_FAILED, request_id)
            return
        result = wrapped.result
        succeeded = wrapped.status == GoalStatus.STATUS_SUCCEEDED
        if succeeded and result.outcome == DockRobot.Result.OUTCOME_DOCKED:
            outcome = RESULT_DOCKED
        elif succeeded and result.outcome == DockRobot.Result.OUTCOME_ALREADY_DOCKED:
            outcome = RESULT_ALREADY_DOCKED
        elif succeeded and result.outcome == DockRobot.Result.OUTCOME_UNDOCKED:
            outcome = RESULT_UNDOCKED
        else:
            outcome = RESULT_FAILED
        self.get_logger().info("DockRobot finished: {} ({})".format(outcome, result.message))
        self.report_result(outcome, request_id)

    def report_result(self, outcome, request_id):
        with self.lock:
            if request_id != self.request_id:
                return
            if outcome in (RESULT_DOCKED, RESULT_ALREADY_DOCKED):
                self.docked_result_time = time.monotonic()
            self.logic.on_dock_result(self.logic_now(), outcome)
        self.publish_state()

    def reset_callback(self, _request, response):
        with self.lock:
            self.logic.reset()
        self.publish_state()
        response.success = True
        response.message = "reset"
        return response

    # ----- shutdown -----
    def shutdown_cancel(self):
        """Cancel the own active DockRobot goal (best effort, bounded)."""
        with self.lock:
            handle = self.dock_handle
            self.dock_handle = None
        if handle is None:
            return
        try:
            done = threading.Event()
            handle.cancel_goal_async().add_done_callback(lambda _f: done.set())
            done.wait(3.0)
        except Exception as exc:
            self.get_logger().warn("Could not cancel the DockRobot goal: {}".format(exc))


def spin_executor(executor):
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        # A shutdown can invalidate the context while spinning
        pass


def main(args=None):
    stop_requested = threading.Event()

    def on_signal(_signum, _frame):
        stop_requested.set()

    # Own signal handling: the context must stay valid while the active dock goal is canceled
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    try:
        node = BatteryMonitor()
    except Exception as exc:  # noqa: BLE001 - any constructor failure is fatal
        rclpy.logging.get_logger("battery_monitor").fatal(str(exc))
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(1)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=spin_executor, args=(executor,), name="monitor_spin")
    spin_thread.start()
    while not stop_requested.is_set() and rclpy.ok() and spin_thread.is_alive():
        time.sleep(0.05)
    node.shutdown_cancel()
    executor.shutdown(timeout_sec=1.0)
    spin_thread.join(2.0)
    try:
        node.destroy_node()
    except RCLError:
        pass
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except RCLError:
        pass


if __name__ == "__main__":
    main()
