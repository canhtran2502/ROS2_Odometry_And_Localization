#!/usr/bin/env python3
"""
AGV mission manager: ExecuteMission action server driving the Nav2 NavigateToPose action.

The C++ twin is src/mission_manager.cpp (same parameters, interfaces and behavior). The state
machine itself lives in mission_logic.py (MissionTracker); this node only does the ROS I/O.
"""
import math
import os
import threading
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from bumperbot_msgs.action import ExecuteMission
from bumperbot_msgs.msg import MissionStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from bumperbot_navigation.mission_logic import (
    STEP_DWELL, STEP_FINISHED,
    MissionTracker, TrackerParams, load_stations, planar_distance, validate_mission,
    yaw_to_quaternion,
    OUTCOME_ABORTED, OUTCOME_DOCK_FAILED)


class MissionManager(Node):
    def __init__(self):
        super().__init__("mission_manager")

        self.declare_parameter("stations_file", "")
        self.declare_parameter("nav_action_name", "navigate_to_pose")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("goal_tolerance", 0.2)
        self.declare_parameter("max_retries", 2)
        self.declare_parameter("retry_delay", 1.0)
        self.declare_parameter("max_consecutive_failures", 2)
        self.declare_parameter("dock_max_retries", 3)
        self.declare_parameter("return_to_dock", True)
        self.declare_parameter("nav_server_timeout", 10.0)
        self.declare_parameter("nav_goal_timeout", 300.0)
        self.declare_parameter("tf_timeout", 1.0)
        self.declare_parameter("status_rate", 2.0)
        self.declare_parameter("behavior_tree", "")

        stations_file = self.get_parameter("stations_file").value
        if stations_file == "":
            stations_file = os.path.join(
                get_package_share_directory("bumperbot_navigation"), "config", "stations.yaml")
        self.stations = load_stations(stations_file)
        self.get_logger().info("Loaded {} stations from {}".format(
            len(self.stations.stations), stations_file))

        self.global_frame = self.get_parameter("global_frame").value
        self.robot_frame = self.get_parameter("robot_frame").value
        self.retry_delay = float(self.get_parameter("retry_delay").value)
        self.nav_server_timeout = float(self.get_parameter("nav_server_timeout").value)
        self.nav_goal_timeout = float(self.get_parameter("nav_goal_timeout").value)
        self.tf_timeout = float(self.get_parameter("tf_timeout").value)
        self.behavior_tree = self.get_parameter("behavior_tree").value
        self.tracker_params = TrackerParams(
            max_retries=int(self.get_parameter("max_retries").value),
            max_consecutive_failures=int(self.get_parameter("max_consecutive_failures").value),
            dock_max_retries=int(self.get_parameter("dock_max_retries").value),
            return_to_dock=bool(self.get_parameter("return_to_dock").value),
            goal_tolerance=float(self.get_parameter("goal_tolerance").value))

        self.group = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.active = False
        self.tracker = None
        self.mission_names = []
        self.distance_remaining = -1.0
        self.goal_handle = None
        self.cancel_event = threading.Event()
        self.cancel_handled = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(MissionStatus, "/mission/status", status_qos)

        self.nav_client = ActionClient(
            self, NavigateToPose, self.get_parameter("nav_action_name").value,
            callback_group=self.group)

        self.action_server = ActionServer(
            self, ExecuteMission, "execute_mission",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.group)

        self.publish_status()
        rate = float(self.get_parameter("status_rate").value)
        if rate > 0.0:
            self.status_timer = self.create_timer(
                1.0 / rate, self.publish_status, callback_group=self.group)

        self.get_logger().info("Mission manager ready (action: /execute_mission)")

    # ----- action server callbacks -----
    def goal_callback(self, goal_request):
        names = list(goal_request.stations)
        reason = validate_mission(names, self.stations)
        if reason != "":
            self.get_logger().warn("Mission rejected: {}".format(reason))
            return GoalResponse.REJECT
        with self.lock:
            if self.active:
                self.get_logger().warn("Mission rejected: another mission is active")
                return GoalResponse.REJECT
            self.active = True
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Mission cancel requested")
        self.cancel_event.set()
        return CancelResponse.ACCEPT

    # ----- status -----
    def build_status(self):
        msg = MissionStatus()
        msg.stamp = self.get_clock().now().to_msg()
        with self.lock:
            tracker = self.tracker
            if tracker is None:
                msg.state = MissionStatus.STATE_IDLE
                msg.current_index = -1
                return msg
            msg.state = tracker.state
            msg.current_station = tracker.current_station
            msg.current_index = tracker.current_index
            msg.total_stations = len(self.mission_names)
            msg.attempt = tracker.attempt
            msg.progress = float(tracker.progress)
            msg.stations = list(self.mission_names)
            msg.visited = tracker.visited
            msg.skipped = tracker.skipped
            msg.message = tracker.message
        return msg

    def publish_status(self):
        self.status_pub.publish(self.build_status())

    def publish_feedback(self):
        status = self.build_status()
        handle = self.goal_handle
        if handle is None:
            return
        feedback = ExecuteMission.Feedback()
        feedback.state = status.state
        feedback.current_station = status.current_station
        feedback.current_index = status.current_index
        feedback.total_stations = status.total_stations
        feedback.attempt = status.attempt
        feedback.progress = status.progress
        feedback.distance_remaining = float(self.distance_remaining)
        try:
            handle.publish_feedback(feedback)
        except Exception:  # the goal may already be terminal
            pass

    def publish_all(self):
        self.publish_status()
        self.publish_feedback()

    # ----- helpers -----
    def handle_cancel_if_pending(self):
        """Forward a pending cancel request to the tracker. Returns True when it was new."""
        if self.cancel_event.is_set() and not self.cancel_handled:
            self.cancel_handled = True
            with self.lock:
                self.tracker.request_cancel()
            return True
        return False

    def wait_clock(self, seconds, interruptible):
        """Wait on the node clock. Returns True when interrupted by a cancel request."""
        end = self.get_clock().now() + Duration(seconds=seconds)
        while rclpy.ok() and self.get_clock().now() < end:
            time.sleep(0.02)
            self.handle_cancel_if_pending()
            if interruptible and self.cancel_handled:
                return True
        return False

    def make_goal(self, station):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.stations.frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = station.x
        goal.pose.pose.position.y = station.y
        z, w = yaw_to_quaternion(station.yaw)
        goal.pose.pose.orientation.z = z
        goal.pose.pose.orientation.w = w
        goal.behavior_tree = self.behavior_tree
        return goal

    def distance_to_station(self, station):
        """Planar distance from the robot (TF) to the station; NaN when the TF is unknown."""
        deadline = time.monotonic() + self.tf_timeout
        while True:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.global_frame, self.robot_frame, Time())
                return planar_distance(
                    transform.transform.translation.x, transform.transform.translation.y,
                    station.x, station.y)
            except Exception:
                if time.monotonic() >= deadline:
                    return math.nan
                time.sleep(0.05)

    def nav_feedback_callback(self, feedback_msg):
        self.distance_remaining = float(feedback_msg.feedback.distance_remaining)
        self.publish_feedback()

    def navigate(self, step):
        """Run one NavigateToPose attempt. Returns (succeeded, interrupted_by_cancel)."""
        station = self.stations.find(step.station)
        self.distance_remaining = -1.0

        deadline = time.monotonic() + self.nav_server_timeout
        while not self.nav_client.server_is_ready():
            if time.monotonic() >= deadline:
                self.get_logger().warn("Nav2 action server is not available")
                return False, False
            time.sleep(0.1)
            if self.handle_cancel_if_pending() and not step.docking:
                return False, True

        accepted = threading.Event()
        holder = {}

        def on_goal_response(future):
            holder["handle"] = future.result()
            accepted.set()

        send_future = self.nav_client.send_goal_async(
            self.make_goal(station), feedback_callback=self.nav_feedback_callback)
        send_future.add_done_callback(on_goal_response)
        if not accepted.wait(self.nav_server_timeout):
            self.get_logger().warn("Nav2 did not answer the goal request")
            return False, False
        nav_handle = holder["handle"]
        if nav_handle is None or not nav_handle.accepted:
            self.get_logger().warn("Nav2 rejected the goal for '{}'".format(step.station))
            return False, False

        done = threading.Event()
        result_holder = {}

        def on_result(future):
            result_holder["result"] = future.result()
            done.set()

        nav_handle.get_result_async().add_done_callback(on_result)

        goal_deadline = time.monotonic() + self.nav_goal_timeout
        interrupted = False
        timed_out = False
        while not done.wait(0.05):
            if not rclpy.ok():
                return False, False
            if self.cancel_event.is_set() and not self.cancel_handled:
                self.handle_cancel_if_pending()
                if not step.docking:
                    interrupted = True
                    break
            if time.monotonic() >= goal_deadline:
                self.get_logger().warn("Navigation to '{}' timed out".format(step.station))
                timed_out = True
                break

        if interrupted or timed_out:
            cancel_done = threading.Event()
            nav_handle.cancel_goal_async().add_done_callback(lambda _f: cancel_done.set())
            cancel_done.wait(5.0)
            done.wait(5.0)
            return False, interrupted

        status = result_holder["result"].status
        return status == GoalStatus.STATUS_SUCCEEDED, False

    # ----- mission execution -----
    def execute_callback(self, goal_handle):
        names = list(goal_handle.request.stations)
        self.get_logger().info("Mission started: [{}]".format(", ".join(names)))
        self.goal_handle = goal_handle
        self.cancel_handled = False
        # A cancel that arrived before this point still counts
        self.cancel_event.clear()
        if goal_handle.is_cancel_requested:
            self.cancel_event.set()
        with self.lock:
            self.mission_names = names
            self.tracker = MissionTracker(self.stations, names, self.tracker_params)
        self.publish_all()

        try:
            self.run_mission()
        except Exception as exc:  # unexpected error: dock and report it
            self.get_logger().error("Mission error: {}".format(exc))
            with self.lock:
                self.tracker.abort("internal error: {}".format(exc))
            self.run_mission()

        tracker = self.tracker
        result = ExecuteMission.Result()
        result.outcome = int(tracker.outcome)
        result.docked = bool(tracker.docked)
        result.visited = tracker.visited
        result.skipped = tracker.skipped
        result.message = tracker.message
        self.publish_all()

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        elif tracker.outcome in (OUTCOME_ABORTED, OUTCOME_DOCK_FAILED):
            goal_handle.abort()
        else:
            goal_handle.succeed()
        self.get_logger().info("Mission finished: {} ({})".format(tracker.state, tracker.message))
        self.goal_handle = None
        with self.lock:
            self.active = False
        return result

    def run_mission(self):
        tracker = self.tracker
        while rclpy.ok():
            if self.handle_cancel_if_pending():
                self.publish_all()
            with self.lock:
                step = tracker.next_step()
            self.publish_all()
            if step.kind == STEP_FINISHED:
                break

            if step.kind == STEP_DWELL:
                self.get_logger().info(
                    "Dwelling {:.1f} s at '{}'".format(step.dwell, step.station))
                if not self.wait_clock(step.dwell, True):
                    with self.lock:
                        tracker.report_dwell_done()
                continue

            # NAVIGATE: wait between attempts, then navigate and check the tolerance
            if step.attempt > 1 and self.wait_clock(self.retry_delay, not step.docking):
                continue
            self.get_logger().info("Navigating to '{}' (attempt {}{})".format(
                step.station, step.attempt, ", docking" if step.docking else ""))
            succeeded, interrupted = self.navigate(step)
            if interrupted:
                continue
            error = math.nan
            if succeeded:
                error = self.distance_to_station(self.stations.find(step.station))
                if not math.isnan(error) and error > self.tracker_params.goal_tolerance:
                    self.get_logger().warn("'{}' reported reached but the robot is {:.2f} m "
                                           "away".format(step.station, error))
            with self.lock:
                tracker.report_navigation(succeeded, error)


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
