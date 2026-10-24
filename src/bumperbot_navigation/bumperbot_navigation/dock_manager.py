#!/usr/bin/env python3
"""
AGV dock manager: DockRobot action server (DOCK / UNDOCK).

It drives the robot to the staging pose with Nav2 and then docks it with a closed-loop controller
on odometry.

The C++ twin is src/dock_manager.cpp (same parameters, interfaces and behavior). The pure logic
lives in dock_logic.py (DockController, DetectionFilter, PhaseTimer, DockTracker); this node only
does the ROS I/O.
"""
import math
import os
import signal
import sys
import threading
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from bumperbot_msgs.action import DockRobot
from bumperbot_msgs.msg import DockStatus, MissionStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy._rclpy_pybind11 import RCLError
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

from bumperbot_navigation.dock_logic import (
    COMMAND_DOCK, COMMAND_UNDOCK,
    DETECTION_NONE, DETECTION_OPTIONAL, DETECTION_REQUIRED,
    OUTCOME_ALREADY_DOCKED, OUTCOME_DOCKED, OUTCOME_ERROR, OUTCOME_UNDOCKED,
    PHASE_MISALIGNED, PHASE_OK, PHASE_SAFETY_TIMEOUT, PHASE_STALL, PHASE_TIMEOUT,
    STATE_CHARGING, STATE_UNDOCKED,
    STEP_ALIGN, STEP_FINISHED, STEP_INSERT, STEP_NAVIGATE_STAGING, STEP_SETTLE_VERIFY,
    STEP_UNDOCK, STEP_WAIT_DETECTION,
    ControlParams, DetectionFilter, DockController, DockTracker, PhaseTimer, Pose2D,
    TrackerParams, compose, compute_dock_pose, inverse, normalize_angle, project_pose,
    quaternion_yaw, relative)
from bumperbot_navigation.mission_logic import load_stations, yaw_to_quaternion

# Number of accepted detections collected at the pre-dock pose before the alignment starts
MIN_DETECTIONS = 5
# Pause between two Nav2 attempts (s)
NAV_RETRY_PAUSE = 0.5
# Active mission states of /mission/status (same strings as the MissionStatus.msg constants)
MISSION_ACTIVE_STATES = (
    MissionStatus.STATE_NAVIGATING, MissionStatus.STATE_DWELLING,
    MissionStatus.STATE_RETURNING_TO_DOCK)


class DockManager(Node):
    def __init__(self):
        super().__init__("dock_manager")

        self.declare_parameter("stations_file", "")
        self.declare_parameter("dock_station", "")
        self.declare_parameter("nav_action_name", "navigate_to_pose")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("cmd_vel_topic", "dock_vel")
        self.declare_parameter("detection_topic", "dock/detected_pose")
        self.declare_parameter("safety_stop_topic", "safety_stop")
        self.declare_parameter("battery_topic", "battery_state")
        self.declare_parameter("mission_status_topic", "mission/status")
        self.declare_parameter("dock_backwards", True)
        self.declare_parameter("staging_distance", 0.5)
        self.declare_parameter("staging_tolerance", 0.25)
        self.declare_parameter("staging_yaw_tolerance", 0.6)
        self.declare_parameter("nav_max_retries", 2)
        self.declare_parameter("nav_server_timeout", 10.0)
        self.declare_parameter("nav_goal_timeout", 180.0)
        self.declare_parameter("detection_mode", "optional")
        self.declare_parameter("detection_timeout", 3.0)
        self.declare_parameter("detection_max_age", 0.5)
        self.declare_parameter("detection_max_jump", 0.3)
        self.declare_parameter("detection_filter_alpha", 0.3)
        self.declare_parameter("detection_offset_x", 0.0)
        self.declare_parameter("detection_offset_y", 0.0)
        self.declare_parameter("detection_offset_yaw", 0.0)
        self.declare_parameter("align_position_tolerance", 0.02)
        self.declare_parameter("align_yaw_tolerance", 0.03)
        self.declare_parameter("dock_position_tolerance", 0.03)
        self.declare_parameter("dock_yaw_tolerance", 0.05)
        self.declare_parameter("max_linear_speed", 0.15)
        self.declare_parameter("max_angular_speed", 0.6)
        self.declare_parameter("approach_speed", 0.05)
        self.declare_parameter("slow_down_distance", 0.15)
        self.declare_parameter("k_lateral", 2.0)
        self.declare_parameter("k_heading", 2.0)
        self.declare_parameter("max_insert_lateral_error", 0.08)
        self.declare_parameter("max_insert_yaw_error", 0.3)
        self.declare_parameter("max_retries", 2)
        self.declare_parameter("align_timeout", 30.0)
        self.declare_parameter("docking_timeout", 40.0)
        self.declare_parameter("undocking_timeout", 30.0)
        self.declare_parameter("stall_timeout", 5.0)
        self.declare_parameter("safety_stop_timeout", 10.0)
        self.declare_parameter("settle_time", 0.5)
        self.declare_parameter("undock_detect_distance", 0.10)
        self.declare_parameter("tf_timeout", 1.0)
        self.declare_parameter("tf_stale_timeout", 0.5)
        self.declare_parameter("control_rate", 20.0)
        self.declare_parameter("status_rate", 2.0)
        self.declare_parameter("clock_stall_timeout", 5.0)

        self.load_configuration()

        self.group = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.shutdown_event = threading.Event()
        self.worker = None

        # Goal state (guarded by self.lock)
        self.active = False
        self.goal_handle = None
        self.tracker = DockTracker(self.tracker_params)
        self.controller = DockController(self.control_params)
        self.detection_filter = DetectionFilter(self.detection_max_jump, self.detection_alpha)
        self.dock_estimate = None
        self.accepting = False
        self.detection_count = 0
        self.nav_handle = None
        self.live_distance = -1.0
        self.live_yaw_error = -1.0
        self.goal_clock = None
        self.last_robot_odom = None
        self.next_tick_time = 0.0
        self.tf_fail_since = None

        # Idle state (guarded by self.lock)
        self.docked = False
        self.idle_state = STATE_UNDOCKED
        self.idle_message = ""
        self.last_attempt = 1
        self.last_position_error = -1.0
        self.last_yaw_error = -1.0
        self.last_detection_used = False
        self.docked_robot_odom = None
        self.saved_dock_estimate = None

        # Inputs
        self.safety_locked = False
        self.battery_percentage = -1.0
        self.battery_charging = False
        self.mission_active = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(DockStatus, "/dock/status", status_qos)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.create_subscription(
            PoseStamped, self.detection_topic, self.detection_callback, 10,
            callback_group=self.group)
        self.create_subscription(
            Bool, self.safety_stop_topic, self.safety_callback, 10, callback_group=self.group)
        self.create_subscription(
            BatteryState, self.battery_topic, self.battery_callback, 10,
            callback_group=self.group)
        self.create_subscription(
            MissionStatus, self.mission_status_topic, self.mission_callback, status_qos,
            callback_group=self.group)

        self.nav_client = ActionClient(
            self, NavigateToPose, self.nav_action_name, callback_group=self.group)

        self.action_server = ActionServer(
            self, DockRobot, "dock_robot",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.group)

        self.publish_status()
        # Wall timer: the status keeps flowing when the node clock (sim time) stalls
        self.status_timer = self.create_timer(
            1.0 / self.status_rate, self.status_timer_callback,
            callback_group=self.group, clock=Clock(clock_type=ClockType.STEADY_TIME))

        self.get_logger().info(
            "Dock manager ready (action: /dock_robot, staging ({:.2f}, {:.2f}, {:.2f}), "
            "charger ({:.2f}, {:.2f}, {:.2f}))".format(
                self.staging.x, self.staging.y, self.staging.yaw,
                self.dock_pose.x, self.dock_pose.y, self.dock_pose.yaw))

    # ----- configuration -----
    def positive(self, name):
        value = float(self.get_parameter(name).value)
        if not (math.isfinite(value) and value > 0.0):
            raise ValueError("parameter '{}' must be a positive number, got {}".format(
                name, value))
        return value

    def load_configuration(self):
        get = self.get_parameter
        stations_file = get("stations_file").value
        if stations_file == "":
            stations_file = os.path.join(
                get_package_share_directory("bumperbot_navigation"), "config", "stations.yaml")
        stations = load_stations(stations_file)

        self.global_frame = get("global_frame").value
        self.odom_frame = get("odom_frame").value
        self.robot_frame = get("robot_frame").value
        frames = (self.global_frame, self.odom_frame, self.robot_frame)
        if "" in frames or len(set(frames)) != 3:
            raise ValueError("global_frame, odom_frame and robot_frame must be 3 distinct "
                             "non-empty names, got {}".format(list(frames)))
        if stations.frame_id != self.global_frame:
            raise ValueError("stations frame_id '{}' differs from global_frame '{}'".format(
                stations.frame_id, self.global_frame))
        dock_station = get("dock_station").value or stations.dock_station
        station = stations.find(dock_station)
        if station is None:
            raise ValueError("dock station '{}' is not defined".format(dock_station))
        self.stations_file = stations_file

        mode = get("detection_mode").value
        if mode not in (DETECTION_NONE, DETECTION_OPTIONAL, DETECTION_REQUIRED):
            raise ValueError("detection_mode '{}' is invalid (none, optional or required)".format(
                mode))
        self.detection_mode = mode

        self.nav_action_name = get("nav_action_name").value
        self.cmd_vel_topic = get("cmd_vel_topic").value
        self.detection_topic = get("detection_topic").value
        self.safety_stop_topic = get("safety_stop_topic").value
        self.battery_topic = get("battery_topic").value
        self.mission_status_topic = get("mission_status_topic").value

        staging_distance = self.positive("staging_distance")
        self.staging_tolerance = self.positive("staging_tolerance")
        self.staging_yaw_tolerance = self.positive("staging_yaw_tolerance")
        self.nav_server_timeout = self.positive("nav_server_timeout")
        self.nav_goal_timeout = self.positive("nav_goal_timeout")
        self.detection_timeout = self.positive("detection_timeout")
        self.detection_max_age = self.positive("detection_max_age")
        self.detection_max_jump = self.positive("detection_max_jump")
        self.detection_alpha = float(get("detection_filter_alpha").value)
        if not (math.isfinite(self.detection_alpha) and 0.0 < self.detection_alpha <= 1.0):
            raise ValueError("detection_filter_alpha must be in (0, 1], got {}".format(
                self.detection_alpha))
        offsets = [float(get(name).value) for name in (
            "detection_offset_x", "detection_offset_y", "detection_offset_yaw")]
        if not all(math.isfinite(v) for v in offsets):
            raise ValueError("detection offsets must be finite")
        self.detection_offset = Pose2D(offsets[0], offsets[1], offsets[2])
        self.align_timeout = self.positive("align_timeout")
        self.docking_timeout = self.positive("docking_timeout")
        self.undocking_timeout = self.positive("undocking_timeout")
        self.stall_timeout = self.positive("stall_timeout")
        self.safety_stop_timeout = self.positive("safety_stop_timeout")
        self.settle_time = self.positive("settle_time")
        self.undock_detect_distance = self.positive("undock_detect_distance")
        self.tf_timeout = self.positive("tf_timeout")
        self.tf_stale_timeout = self.positive("tf_stale_timeout")
        self.control_rate = self.positive("control_rate")
        self.status_rate = self.positive("status_rate")
        self.clock_stall_timeout = self.positive("clock_stall_timeout")
        for name in ("max_retries", "nav_max_retries"):
            if int(get(name).value) < 0:
                raise ValueError("parameter '{}' must not be negative".format(name))

        self.control_params = ControlParams(
            staging_distance=staging_distance,
            dock_backwards=bool(get("dock_backwards").value),
            max_linear_speed=self.positive("max_linear_speed"),
            max_angular_speed=self.positive("max_angular_speed"),
            approach_speed=self.positive("approach_speed"),
            slow_down_distance=self.positive("slow_down_distance"),
            k_lateral=self.positive("k_lateral"),
            k_heading=self.positive("k_heading"),
            align_position_tolerance=self.positive("align_position_tolerance"),
            align_yaw_tolerance=self.positive("align_yaw_tolerance"),
            max_insert_lateral_error=self.positive("max_insert_lateral_error"),
            max_insert_yaw_error=self.positive("max_insert_yaw_error"))
        self.tracker_params = TrackerParams(
            max_retries=int(get("max_retries").value),
            nav_max_retries=int(get("nav_max_retries").value),
            detection_mode=mode,
            dock_position_tolerance=self.positive("dock_position_tolerance"),
            dock_yaw_tolerance=self.positive("dock_yaw_tolerance"))

        self.station = station
        self.staging = Pose2D(station.x, station.y, station.yaw)
        self.dock_pose = compute_dock_pose(
            self.staging, staging_distance, self.control_params.dock_backwards)

    # ----- time and TF helpers -----
    def node_now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def lookup(self, parent, child):
        """Return (Pose2D, stamp in s) of the latest transform; raises when unavailable."""
        transform = self.tf_buffer.lookup_transform(parent, child, Time())
        t = transform.transform.translation
        q = transform.transform.rotation
        stamp = Time.from_msg(transform.header.stamp).nanoseconds * 1e-9
        return Pose2D(t.x, t.y, quaternion_yaw(q.x, q.y, q.z, q.w)), stamp

    def can_transform(self, parent, child):
        try:
            return self.tf_buffer.can_transform(parent, child, Time(), Duration(seconds=0.0))
        except Exception:
            return False

    def near_staging(self):
        """Return True when the robot is within the staging tolerances (False if unknown)."""
        try:
            pose, _ = self.lookup(self.global_frame, self.robot_frame)
        except Exception:
            return False
        return (math.hypot(pose.x - self.staging.x, pose.y - self.staging.y)
                <= self.staging_tolerance
                and abs(normalize_angle(pose.yaw - self.staging.yaw))
                <= self.staging_yaw_tolerance)

    # ----- subscriptions -----
    def safety_callback(self, msg):
        self.safety_locked = bool(msg.data)

    def battery_callback(self, msg):
        percentage = float(msg.percentage)
        self.battery_percentage = percentage if math.isfinite(percentage) else -1.0
        self.battery_charging = (
            msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING)

    def mission_callback(self, msg):
        self.mission_active = msg.state in MISSION_ACTIVE_STATES

    def detection_callback(self, msg):
        """Validate a dock detection, bring it to odom and feed the filter."""
        if self.detection_mode == DETECTION_NONE:
            return
        with self.lock:
            if not self.accepting:
                return
        try:
            detection = self.detection_to_odom(msg)
        except Exception as exc:
            self.get_logger().debug("Detection ignored: {}".format(exc))
            return
        with self.lock:
            if not self.accepting or self.dock_estimate is None:
                return
            accepted, estimate = self.detection_filter.update(self.dock_estimate, detection)
            if accepted:
                self.dock_estimate = estimate
                self.detection_count += 1

    def detection_to_odom(self, msg):
        p = msg.pose.position
        q = msg.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        if not all(math.isfinite(v) for v in values):
            raise ValueError("non-finite pose")
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if norm < 1e-6:
            raise ValueError("zero quaternion")
        if msg.header.frame_id == "":
            raise ValueError("empty frame id")
        stamp = Time.from_msg(msg.header.stamp)
        if stamp.nanoseconds != 0:
            age = self.node_now() - stamp.nanoseconds * 1e-9
            if age > self.detection_max_age:
                raise ValueError("detection too old ({:.2f} s)".format(age))
        try:
            transform = self.tf_buffer.lookup_transform(
                self.odom_frame, msg.header.frame_id, stamp)
        except Exception:
            # The stamp may be outside the TF history: fall back to the latest transform
            transform = self.tf_buffer.lookup_transform(
                self.odom_frame, msg.header.frame_id, Time())
        t = transform.transform.translation
        r = transform.transform.rotation
        frame_tf = (t.x, t.y, t.z, r.x, r.y, r.z, r.w)
        marker = project_pose(frame_tf, (p.x, p.y, p.z, q.x / norm, q.y / norm, q.z / norm,
                                         q.w / norm))
        return compose(marker, self.detection_offset)

    # ----- status -----
    def build_status(self):
        msg = DockStatus()
        msg.stamp = self.get_clock().now().to_msg()
        with self.lock:
            if self.active:
                msg.state = self.tracker.state
                msg.attempt = self.tracker.attempt
                msg.position_error = float(self.tracker.position_error)
                msg.yaw_error = float(self.tracker.yaw_error)
                msg.detection_used = self.tracker.detection_used or self.detection_count > 0
                msg.message = self.tracker.message
            else:
                msg.state = self.idle_state
                if self.docked and self.battery_charging:
                    msg.state = STATE_CHARGING
                msg.attempt = self.last_attempt
                msg.position_error = float(self.last_position_error)
                msg.yaw_error = float(self.last_yaw_error)
                msg.detection_used = self.last_detection_used
                msg.message = self.idle_message
            msg.docked = bool(self.docked)
            msg.charging = bool(self.docked and self.battery_charging)
        msg.battery_percentage = float(self.battery_percentage)
        msg.frame_id = self.global_frame
        msg.staging_pose.x = self.staging.x
        msg.staging_pose.y = self.staging.y
        msg.staging_pose.theta = self.staging.yaw
        msg.dock_pose.x = self.dock_pose.x
        msg.dock_pose.y = self.dock_pose.y
        msg.dock_pose.theta = self.dock_pose.yaw
        return msg

    def publish_status(self):
        try:
            self.status_pub.publish(self.build_status())
        except Exception:  # the context may already be gone during shutdown
            pass

    def publish_feedback(self):
        handle = self.goal_handle
        if handle is None:
            return
        status = self.build_status()
        feedback = DockRobot.Feedback()
        feedback.state = status.state
        feedback.attempt = status.attempt
        feedback.distance_to_dock = float(self.live_distance)
        feedback.yaw_error = float(self.live_yaw_error)
        feedback.detection_used = status.detection_used
        try:
            handle.publish_feedback(feedback)
        except Exception:  # the goal may already be terminal
            pass

    def publish_all(self):
        self.publish_status()
        self.publish_feedback()

    def status_timer_callback(self):
        self.update_docked_watch()
        self.publish_status()
        if self.active:
            self.publish_feedback()

    def update_docked_watch(self):
        """Clear the docked flag when the robot was driven away from the charger."""
        with self.lock:
            if self.active or not self.docked or self.docked_robot_odom is None:
                return
            reference = self.docked_robot_odom
        try:
            pose, _ = self.lookup(self.odom_frame, self.robot_frame)
        except Exception:
            return
        if math.hypot(pose.x - reference.x, pose.y - reference.y) > self.undock_detect_distance:
            with self.lock:
                if not self.active and self.docked:
                    self.docked = False
                    self.idle_state = STATE_UNDOCKED
                    self.idle_message = "the robot left the dock"
                    self.get_logger().info("The robot left the dock")

    # ----- action server callbacks -----
    def goal_callback(self, goal_request):
        command = int(goal_request.command)
        if command not in (COMMAND_DOCK, COMMAND_UNDOCK):
            return self.reject("unknown command {}".format(command))
        if self.shutdown_event.is_set():
            return self.reject("the node is shutting down")
        self.update_docked_watch()
        with self.lock:
            if self.active:
                return self.reject("another dock goal is active")
            docked = self.docked
            if command == COMMAND_UNDOCK and not docked:
                return self.reject("UNDOCK while the robot is not docked")
            if command == COMMAND_DOCK and self.mission_active:
                return self.reject("DOCK while a mission is active")
            if not self.can_transform(self.global_frame, self.robot_frame):
                return self.reject("TF {} -> {} is not available".format(
                    self.global_frame, self.robot_frame))
            if not self.can_transform(self.odom_frame, self.robot_frame):
                return self.reject("TF {} -> {} is not available".format(
                    self.odom_frame, self.robot_frame))
            if (command == COMMAND_DOCK and not docked and not self.near_staging()
                    and not self.nav_client.server_is_ready()):
                return self.reject("the Nav2 action server is not ready")
            self.active = True
        return GoalResponse.ACCEPT

    def reject(self, reason):
        self.get_logger().warn("Dock goal rejected: {}".format(reason))
        return GoalResponse.REJECT

    def cancel_callback(self, goal_handle):
        # The worker polls is_cancel_requested, which also covers a cancel right after acceptance
        self.get_logger().info("Dock cancel requested")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        holder = {}
        try:
            worker = threading.Thread(
                target=self.worker_main, args=(goal_handle, holder), name="dock_worker")
            self.worker = worker
            worker.start()
        except Exception as exc:
            self.get_logger().error("Could not start the dock worker: {}".format(exc))
            with self.lock:
                self.active = False
            goal_handle.abort()
            return DockRobot.Result(outcome=OUTCOME_ERROR, message="internal error")
        while worker.is_alive():
            worker.join(0.1)
        return holder.get("result", DockRobot.Result(outcome=OUTCOME_ERROR, message="no result"))

    # ----- worker -----
    def worker_main(self, goal_handle, holder):
        try:
            self.run_goal(goal_handle)
        except BaseException as exc:  # noqa: BLE001 - whatever happens, the cleanup must run
            try:
                self.get_logger().error("Dock goal error: {}".format(exc))
                with self.lock:
                    self.tracker.abort("internal error: {}".format(exc))
            except Exception:
                pass
        finally:
            self.finish_goal(goal_handle, holder)

    def finish_goal(self, goal_handle, holder):
        """Stop the robot, build the result, reset the node state and end the goal once."""
        result = DockRobot.Result()
        result.outcome = OUTCOME_ERROR
        result.message = "internal error"
        result.position_error = -1.0
        result.yaw_error = -1.0
        try:
            for _ in range(3):
                self.publish_cmd(0.0, 0.0)
                time.sleep(0.01)
            self.cancel_nav_goal(2.0)
            with self.lock:
                tracker = self.tracker
                if not tracker.finished:
                    tracker.abort("internal error: goal ended unexpectedly")
                result.outcome = int(tracker.outcome)
                result.docked = bool(tracker.docked)
                result.attempts = int(tracker.attempt)
                result.position_error = float(tracker.position_error)
                result.yaw_error = float(tracker.yaw_error)
                result.message = tracker.message
                self.docked = bool(tracker.docked)
                self.idle_state = tracker.state
                self.idle_message = tracker.message
                self.last_attempt = tracker.attempt
                self.last_position_error = tracker.position_error
                self.last_yaw_error = tracker.yaw_error
                self.last_detection_used = tracker.detection_used or self.detection_count > 0
                if tracker.docked:
                    if tracker.outcome == OUTCOME_DOCKED:
                        self.saved_dock_estimate = self.dock_estimate
                        self.docked_robot_odom = self.last_robot_odom
                else:
                    self.docked_robot_odom = None
                self.accepting = False
        except Exception as exc:
            self.get_logger().error("Could not build the dock result: {}".format(exc))
        with self.lock:
            # From here on the node accepts the next goal; only locals are used below
            self.goal_handle = None
            self.nav_handle = None
            self.active = False
        holder["result"] = result
        self.publish_status()
        try:
            # A cancel in progress wins: the goal can only end CANCELED (or ABORTED) from there
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            elif result.outcome in (OUTCOME_DOCKED, OUTCOME_UNDOCKED, OUTCOME_ALREADY_DOCKED):
                goal_handle.succeed()
            else:
                goal_handle.abort()
        except Exception as exc:
            self.get_logger().error("Could not report the dock result: {}".format(exc))
        self.get_logger().info("Dock goal finished: outcome {} ({})".format(
            result.outcome, result.message))

    def run_goal(self, goal_handle):
        command = int(goal_handle.request.command)
        self.get_logger().info("Dock goal started: {}".format(
            "DOCK" if command == COMMAND_DOCK else "UNDOCK"))
        self.update_docked_watch()
        near = command == COMMAND_DOCK and self.near_staging()
        with self.lock:
            self.goal_handle = goal_handle
            self.dock_estimate = None
            self.accepting = False
            self.detection_count = 0
            self.nav_handle = None
            self.live_distance = -1.0
            self.live_yaw_error = -1.0
            self.tf_fail_since = None
            self.last_robot_odom = None
            self.controller.reset()
            self.goal_clock = PhaseTimer(1e12, self.clock_stall_timeout)
            self.goal_clock.start(self.node_now(), time.monotonic())
            self.next_tick_time = time.monotonic()
            self.tracker.start(command, self.docked, near)
        self.publish_all()

        last_signature = None
        while True:
            self.check_interrupts()
            with self.lock:
                step = self.tracker.next_step()
                signature = (self.tracker.state, self.tracker.attempt)
            if signature != last_signature:
                last_signature = signature
                self.publish_all()
            if step.kind == STEP_FINISHED:
                break
            if step.kind == STEP_NAVIGATE_STAGING:
                self.run_navigate()
            elif step.kind == STEP_WAIT_DETECTION:
                self.run_wait_detection()
            elif step.kind == STEP_ALIGN:
                self.run_align()
            elif step.kind == STEP_INSERT:
                self.run_insert()
            elif step.kind == STEP_SETTLE_VERIFY:
                self.run_verify()
            elif step.kind == STEP_UNDOCK:
                self.run_undock()
        self.publish_all()

    # ----- interrupts and ticks -----
    def check_interrupts(self):
        """Apply shutdown / cancel / clock stall to the tracker. Returns True when it ended."""
        with self.lock:
            tracker = self.tracker
            if tracker.finished:
                return True
            if self.shutdown_event.is_set():
                tracker.abort("shutdown")
            elif self.goal_clock.expired(self.node_now(), time.monotonic()) \
                    and self.goal_clock.stalled():
                tracker.abort("the node clock stalled")
            elif self.goal_handle is not None and self.goal_handle.is_cancel_requested:
                tracker.request_cancel()
            ended = tracker.finished
        if ended:
            self.publish_cmd(0.0, 0.0)
            self.publish_all()
        return ended

    def tick(self):
        """Sleep until the next control period. Returns False when the goal ended."""
        period = 1.0 / self.control_rate
        now = time.monotonic()
        self.next_tick_time += period
        if self.next_tick_time < now - period:
            self.next_tick_time = now
        delay = self.next_tick_time - now
        if delay > 0.0:
            time.sleep(delay)
        return not self.check_interrupts()

    def wait_interruptible(self, seconds):
        """Wall wait that returns False as soon as the goal ended."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            time.sleep(0.02)
            if self.check_interrupts():
                return False
        return True

    def odom_pose(self):
        """Robot pose in odom. Returns None (robot stopped) when the TF is stale or lost."""
        try:
            pose, stamp = self.lookup(self.odom_frame, self.robot_frame)
            age = max(0.0, self.node_now() - stamp)
            self.tf_fail_since = None
        except Exception:
            pose = None
            age = 0.0
            if self.tf_fail_since is None:
                self.tf_fail_since = time.monotonic()
            elif time.monotonic() - self.tf_fail_since > self.tf_timeout:
                age = self.tf_timeout + 1.0
        if pose is None or age > self.tf_stale_timeout:
            self.publish_cmd(0.0, 0.0)
            if age > self.tf_timeout:
                self.get_logger().error("The {} -> {} TF is lost".format(
                    self.odom_frame, self.robot_frame))
                with self.lock:
                    self.tracker.abort("the odom TF is lost")
                self.publish_all()
            return None
        self.last_robot_odom = pose
        return pose

    # ----- command output -----
    def publish_cmd(self, v, w):
        msg = Twist()
        par = self.control_params
        msg.linear.x = max(-par.max_linear_speed, min(par.max_linear_speed, float(v)))
        msg.angular.z = max(-par.max_angular_speed, min(par.max_angular_speed, float(w)))
        try:
            self.cmd_pub.publish(msg)
        except Exception:  # the context may already be gone during shutdown
            pass

    # ----- dock estimate -----
    def begin_approach(self, use_saved):
        """Set the charger pose estimate in odom (from the map or the saved docking)."""
        with self.lock:
            if self.dock_estimate is not None:
                return True
            if use_saved and self.saved_dock_estimate is not None:
                self.dock_estimate = self.saved_dock_estimate
                return True
        try:
            map_to_odom, _ = self.lookup(self.global_frame, self.odom_frame)
        except Exception:
            self.get_logger().error("The {} -> {} TF is not available".format(
                self.global_frame, self.odom_frame))
            with self.lock:
                self.tracker.abort("the map to odom TF is not available")
            return False
        # Snapshot of map -> odom: the charger pose in odom stays fixed for the final approach
        estimate = compose(inverse(map_to_odom), self.dock_pose)
        with self.lock:
            self.dock_estimate = estimate
            self.accepting = self.detection_mode != DETECTION_NONE and not use_saved
        return True

    def robot_in_dock_frame(self, odom):
        with self.lock:
            estimate = self.dock_estimate
        return relative(estimate, odom)

    def update_live(self, p):
        position, yaw = DockController.pose_error(p)
        self.live_distance = position
        self.live_yaw_error = yaw

    # ----- Nav2 -----
    def make_goal(self):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.global_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.staging.x
        goal.pose.pose.position.y = self.staging.y
        z, w = yaw_to_quaternion(self.staging.yaw)
        goal.pose.pose.orientation.z = z
        goal.pose.pose.orientation.w = w
        return goal

    def cancel_nav_goal(self, wait_seconds):
        """Best-effort cancel of the active Nav2 goal; never raises."""
        with self.lock:
            handle = self.nav_handle
            self.nav_handle = None
        if handle is None:
            return
        try:
            cancel_done = threading.Event()
            handle.cancel_goal_async().add_done_callback(lambda _f: cancel_done.set())
            cancel_done.wait(wait_seconds)
        except Exception as exc:  # e.g. the goal already ended or the context is gone
            try:
                self.get_logger().warn("Could not cancel the Nav2 goal: {}".format(exc))
            except Exception:
                pass

    def run_navigate(self):
        result = self.navigate()
        if result is None:
            return
        with self.lock:
            self.tracker.report_nav(result)

    def navigate(self):
        """One Nav2 attempt. Returns True/False, or None when the goal ended meanwhile."""
        with self.lock:
            attempt = self.tracker.attempt
        if attempt > 1 and not self.wait_interruptible(NAV_RETRY_PAUSE):
            return None
        self.get_logger().info("Navigating to the staging pose (attempt {})".format(attempt))
        rejected_since = None
        while True:
            deadline = time.monotonic() + self.nav_server_timeout
            while not self.nav_client.server_is_ready():
                if time.monotonic() >= deadline:
                    self.get_logger().warn("Nav2 action server is not available")
                    return False
                if not self.wait_interruptible(0.1):
                    return None

            accepted = threading.Event()
            holder = {}

            def on_goal_response(future, accepted=accepted, holder=holder):
                holder["handle"] = future.result()
                accepted.set()

            self.nav_client.send_goal_async(self.make_goal()).add_done_callback(on_goal_response)
            answer_deadline = time.monotonic() + self.nav_server_timeout
            while not accepted.wait(0.05):
                if self.check_interrupts():
                    return None
                if time.monotonic() >= answer_deadline:
                    self.get_logger().warn("Nav2 did not answer the goal request")
                    return False
            handle = holder["handle"]
            if handle is not None and handle.accepted:
                break
            # A rejection means the Nav2 lifecycle is probably not active yet: re-send without
            # counting an attempt until nav_server_timeout of continuous rejection has passed
            if rejected_since is None:
                rejected_since = time.monotonic()
            if time.monotonic() - rejected_since >= self.nav_server_timeout:
                self.get_logger().warn("Nav2 rejected the staging goal")
                return False
            if not self.wait_interruptible(0.5):
                return None

        with self.lock:
            self.nav_handle = handle
        done = threading.Event()
        result_holder = {}

        def on_result(future):
            result_holder["result"] = future.result()
            done.set()

        handle.get_result_async().add_done_callback(on_result)

        goal_deadline = time.monotonic() + self.nav_goal_timeout
        server_down_since = None
        while not done.wait(0.05):
            if self.check_interrupts():
                self.cancel_nav_goal(2.0)
                return None
            if time.monotonic() >= goal_deadline:
                self.get_logger().warn("Navigation to the staging pose timed out")
                self.cancel_nav_goal(5.0)
                return False
            if self.nav_client.server_is_ready():
                server_down_since = None
            elif server_down_since is None:
                server_down_since = time.monotonic()
            elif time.monotonic() - server_down_since >= self.nav_server_timeout:
                self.get_logger().error("Nav2 action server disappeared during the navigation")
                with self.lock:
                    self.nav_handle = None
                    self.tracker.abort("the Nav2 action server disappeared")
                return None
        with self.lock:
            self.nav_handle = None
        return result_holder["result"].status == GoalStatus.STATUS_SUCCEEDED

    # ----- phases -----
    def run_wait_detection(self):
        if not self.begin_approach(False):
            return
        timer = PhaseTimer(self.detection_timeout, self.clock_stall_timeout)
        timer.start(self.node_now(), time.monotonic())
        self.get_logger().info("Waiting for a dock detection")
        while True:
            if not self.tick():
                return
            self.publish_cmd(0.0, 0.0)
            if self.odom_pose() is None and self.check_interrupts():
                return
            with self.lock:
                count = self.detection_count
            if count >= MIN_DETECTIONS:
                break
            if timer.expired(self.node_now(), time.monotonic()):
                break
        with self.lock:
            self.tracker.report_detection(count >= 1)

    def run_align(self):
        if not self.begin_approach(False):
            return
        self.controller.reset()
        result = self.control_phase(self.align_timeout, self.controller.align_command)
        if result is not None:
            with self.lock:
                self.tracker.report_phase(result)

    def run_insert(self):
        result = self.control_phase(self.docking_timeout, self.controller.insert_command)
        if result is not None:
            with self.lock:
                self.tracker.report_phase(result)

    def run_undock(self):
        if not self.begin_approach(True):
            return
        result = self.control_phase(self.undocking_timeout, self.controller.undock_command)
        if result is not None:
            with self.lock:
                self.tracker.report_phase(result)

    def control_phase(self, timeout, compute):
        """
        Run a closed-loop control phase at control_rate.

        Returns a PHASE_* result, or None when the goal ended (cancel, abort) meanwhile. The robot
        is stopped before every return.
        """
        timer = PhaseTimer(timeout, self.clock_stall_timeout)
        timer.start(self.node_now(), time.monotonic())
        locked_seconds = 0.0
        last_wall = time.monotonic()
        stall_ref = None
        stall_since = 0.0
        while True:
            if not self.tick():
                return None
            wall = time.monotonic()
            dt = wall - last_wall
            last_wall = wall
            odom = self.odom_pose()
            if odom is None:
                stall_ref = None
                if self.check_interrupts():
                    return None
                continue
            p = self.robot_in_dock_frame(odom)
            self.update_live(p)
            cmd = compute(p)
            if cmd.done:
                self.publish_cmd(0.0, 0.0)
                return PHASE_OK
            if cmd.failed:
                self.publish_cmd(0.0, 0.0)
                return PHASE_MISALIGNED

            locked = self.safety_locked
            if locked:
                locked_seconds += dt
                self.publish_cmd(0.0, 0.0)
                stall_ref = None
                if locked_seconds >= self.safety_stop_timeout:
                    self.get_logger().warn("safety_stop held too long, attempt failed")
                    return PHASE_SAFETY_TIMEOUT
            else:
                self.publish_cmd(cmd.v, cmd.w)
                if cmd.v == 0.0 and cmd.w == 0.0:
                    stall_ref = None
                elif stall_ref is None or (
                        math.hypot(odom.x - stall_ref.x, odom.y - stall_ref.y) >= 0.01
                        or abs(normalize_angle(odom.yaw - stall_ref.yaw)) >= 0.02):
                    stall_ref = odom
                    stall_since = wall
                elif wall - stall_since >= self.stall_timeout:
                    self.publish_cmd(0.0, 0.0)
                    self.get_logger().warn("The robot does not move, attempt failed")
                    return PHASE_STALL

            if timer.expired(self.node_now(), wall):
                self.publish_cmd(0.0, 0.0)
                if timer.stalled():
                    with self.lock:
                        self.tracker.abort("the node clock stalled")
                    return None
                return PHASE_TIMEOUT

    def run_verify(self):
        timer = PhaseTimer(self.settle_time, self.clock_stall_timeout)
        timer.start(self.node_now(), time.monotonic())
        while True:
            if not self.tick():
                return
            self.publish_cmd(0.0, 0.0)
            odom = self.odom_pose()
            if odom is None:
                if self.check_interrupts():
                    return
                continue
            if timer.expired(self.node_now(), time.monotonic()):
                if timer.stalled():
                    with self.lock:
                        self.tracker.abort("the node clock stalled")
                    return
                break
        p = self.robot_in_dock_frame(odom)
        position, yaw = DockController.pose_error(p)
        self.update_live(p)
        with self.lock:
            self.tracker.report_verify(position, yaw)

    # ----- shutdown -----
    def shutdown_goal(self):
        """Ask the worker to stop the robot and end the goal; wait for it (bounded)."""
        self.shutdown_event.set()
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            with self.lock:
                if not self.active:
                    break
            time.sleep(0.05)
        worker = self.worker
        if worker is not None and worker.is_alive():
            worker.join(max(0.0, deadline - time.monotonic()))
        # Let the executor deliver the final result to the client
        time.sleep(0.3)
        self.publish_cmd(0.0, 0.0)


def main(args=None):
    stop_requested = threading.Event()

    def on_signal(_signum, _frame):
        stop_requested.set()

    # Own signal handling: the context must stay valid while the worker stops the robot and
    # cancels the active Nav2 goal, so rclpy's default shutdown-on-signal is disabled.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    try:
        node = DockManager()
    except Exception as exc:  # noqa: BLE001 - any constructor failure is fatal
        rclpy.logging.get_logger("dock_manager").fatal(str(exc))
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(1)
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    spin_thread = threading.Thread(target=spin_executor, args=(executor,), name="dock_spin")
    spin_thread.start()
    while not stop_requested.is_set() and rclpy.ok() and spin_thread.is_alive():
        time.sleep(0.05)
    # Stop the robot and end the goal while the executor still processes callbacks
    node.shutdown_goal()
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


def spin_executor(executor):
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        # A shutdown can invalidate the context while spinning
        pass


if __name__ == "__main__":
    main()
