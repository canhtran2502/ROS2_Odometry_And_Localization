"""
MockDockWorld: a kinematic stand-in for Nav2, the simulator, twist_mux and the dock detector.

It is one rclpy node that runs next to a dock_manager under test:

  * a diff-drive robot integrates the dock_vel commands at 50 Hz (a command expires 0.5 s after
    the last message, like a twist_mux input). Two poses are kept: the true pose T and the odometry
    pose O. They move together; an unobserved slip changes T only.
  * TF odom -> base_footprint is O, TF map -> odom is the localization error E (so the pose that
    the node sees in map is E * O).
  * a navigate_to_pose server with scripted outcomes. On success it moves the robot so that
    E * T equals the goal plus staging_error.
  * optional dock detections computed from T (plus outliers, NaN and malformed messages).
  * /safety_stop lock (the commands are ignored while it is set), stall mode (the commands are
    ignored), TF switches, /mission/status and /battery_state publishers.
  * a recorder for every dock_vel command, Nav2 goal and cancel, and every /dock/status message.

The frame of the charger is fixed: true world == map == odom frame at the start.
"""
import math
import threading
import time

from bumperbot_msgs.action import DockRobot
from bumperbot_msgs.msg import DockStatus, MissionStatus
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster

CHARGER = (-0.5, 0.0, 0.0)
MUX_TIMEOUT = 0.5
STEP_PERIOD = 0.02


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], wrap(-a[2]))


def relative(frame, pose):
    return compose(inverse(frame), pose)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class GoalRun:
    """Book-keeping of one DockRobot goal sent by the test."""

    def __init__(self, command):
        self.command = command
        self.t_sent = time.monotonic()
        self.t_decision = None
        self.accepted = None
        self.handle = None
        self.feedback = []      # (t, Feedback)
        self.result = None      # GetResult response (status + result)
        self.t_result = None
        self.t_cancel = None
        self.cancel_code = None     # return_code of the cancel response (ERROR_NONE = accepted)
        self.cancel_state = None    # /dock/status state that triggers a cancel (cancel_on_state)
        self.error = None

    @property
    def done(self):
        return self.result is not None or self.accepted is False


class MockDockWorld(Node):
    def __init__(self, use_sim_time=False, charger=CHARGER, nav_action="navigate_to_pose"):
        overrides = [Parameter("use_sim_time", Parameter.Type.BOOL, True)] if use_sim_time else []
        super().__init__("mock_dock_world", parameter_overrides=overrides)
        self.charger = tuple(charger)
        self.lock = threading.RLock()
        self.timer_group = MutuallyExclusiveCallbackGroup()
        self.action_group = ReentrantCallbackGroup()
        self.wall = Clock(clock_type=ClockType.STEADY_TIME)

        # configuration (see reset())
        self.pose_true = (0.0, 0.0, 0.0)
        self.pose_odom = (0.0, 0.0, 0.0)
        self.localization_error = (0.0, 0.0, 0.0)
        self.staging_error = (0.0, 0.0, 0.0)
        self.nav_script = {}
        self.detections = True
        self.inject_active = False
        self.inject_kinds = ["outlier", "nan", "zero_quat", "empty_frame", "unknown_frame"]
        self.slip_first_insert = False
        self.slip_applied = False
        self.slip_amount = 0.06
        self.stall = False
        self.tf_enabled = True
        self.odom_tf_enabled = True
        self.safety_lock = False
        self.mission_state = None
        self.battery = (0.5, BatteryState.POWER_SUPPLY_STATUS_DISCHARGING)
        self.battery_enabled = False

        # state and records
        self.cmd = (0.0, 0.0)
        self.cmd_time = 0.0
        self.last_step = time.monotonic()
        self.state = None
        self.cmds = []          # dicts: t, v, w, state
        self.nav_goals = []     # dicts: t, frame_id, x, y, yaw
        self.nav_cancels = []   # dicts: t
        self.nav_results = []   # dicts: t, result
        self.nav_count = 0
        self.nav_accept_delay = 0.0     # seconds the Nav2 goal response is held back
        self.nav_cancel_requests = []   # dicts: t (every cancel request the Nav2 server received)
        self.status_log = []    # dicts, one per /dock/status message
        self.cancel_triggers = []   # GoalRuns that are canceled when their cancel_state shows up
        self.inject_index = 0
        self.nav_epoch = 0
        self.closing = False

        self.broadcaster = TransformBroadcaster(self)
        latched = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Twist, "dock_vel", self.on_cmd, 100,
                                 callback_group=self.timer_group)
        self.create_subscription(DockStatus, "/dock/status", self.on_status, latched,
                                 callback_group=self.timer_group)
        self.safety_pub = self.create_publisher(Bool, "/safety_stop", 10)
        self.detection_pub = self.create_publisher(PoseStamped, "/dock/detected_pose", 10)
        self.mission_pub = None
        self.make_mission_publisher()
        self.battery_pub = self.create_publisher(BatteryState, "/battery_state", 10)
        for period, callback in ((STEP_PERIOD, self.step), (0.1, self.publish_detection),
                                 (0.02, self.publish_safety), (0.5, self.publish_battery),
                                 (0.05, self.publish_map_odom)):
            self.create_timer(period, callback, callback_group=self.timer_group, clock=self.wall)

        self.nav_action = nav_action
        self.nav_server = None
        self.start_nav_server()
        self.client = ActionClient(self, DockRobot, "dock_robot", callback_group=self.action_group)

    # ------------------------------------------------------------------ configuration
    def reset(self, pose=(0.0, 0.0, 0.0), localization_error=(0.0, 0.0, 0.0),
              staging_error=(0.0, 0.0, 0.0), nav_script=None, detections=True,
              slip_first_insert=False, stall=False, tf_enabled=True, safety_lock=False,
              detection_range=None):
        """Restore the defaults and place the robot (O = T = pose). No goal may be active."""
        with self.lock:
            self.pose_true = tuple(pose)
            self.pose_odom = tuple(pose)
            self.localization_error = tuple(localization_error)
            self.staging_error = tuple(staging_error)
            self.nav_script = dict(nav_script or {})
            self.detections = detections
            self.inject_active = False
            self.slip_first_insert = slip_first_insert
            self.slip_applied = False
            self.stall = stall
            self.tf_enabled = tf_enabled
            self.odom_tf_enabled = True
            self.safety_lock = safety_lock
            self.cmd = (0.0, 0.0)
            self.cmd_time = 0.0
            self.cmds = []
            self.nav_goals = []
            self.nav_cancels = []
            self.nav_results = []
            self.nav_count = 0
            self.nav_accept_delay = 0.0
            self.nav_cancel_requests = []
            self.cancel_triggers = []
            self.status_log = []
        self.make_mission_publisher()

    def teleport(self, pose):
        """Move the robot (O and T) without any command, e.g. to push it away from the dock."""
        with self.lock:
            self.pose_true = tuple(pose)
            self.pose_odom = tuple(pose)

    def start_nav_server(self):
        if self.nav_server is None:
            self.nav_server = ActionServer(
                self, NavigateToPose, self.nav_action,
                execute_callback=self.nav_execute, goal_callback=self.nav_on_goal,
                cancel_callback=self.nav_on_cancel, callback_group=self.action_group)

    def stop_nav_server(self):
        """Make the Nav2 action server disappear (a dead Nav2)."""
        with self.lock:
            self.nav_epoch += 1
        if self.nav_server is not None:
            self.nav_server.destroy()
            self.nav_server = None

    def close(self):
        self.closing = True
        with self.lock:
            self.nav_epoch += 1
        time.sleep(0.2)
        if self.nav_server is not None:
            self.nav_server.destroy()
            self.nav_server = None

    def make_mission_publisher(self):
        """Create the latched /mission/status publisher unless it exists."""
        if self.mission_pub is None:
            self.mission_pub = self.create_publisher(
                MissionStatus, "/mission/status",
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def drop_mission_publisher(self):
        """
        Remove the /mission/status publisher (mission_manager died).

        The last sample stays latched in the subscribers but nobody publishes any more.
        """
        if self.mission_pub is not None:
            self.destroy_publisher(self.mission_pub)
            self.mission_pub = None

    def set_mission(self, state):
        self.make_mission_publisher()
        msg = MissionStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.state = state
        self.mission_state = state
        self.mission_pub.publish(msg)

    def set_battery(self, percentage, status):
        with self.lock:
            self.battery = (percentage, status)
            self.battery_enabled = True

    # ------------------------------------------------------------------ truth and queries
    def truth(self):
        with self.lock:
            return self.pose_true

    def odom(self):
        with self.lock:
            return self.pose_odom

    def charger_error(self):
        """Return (position error in m, absolute yaw error in rad) of T versus the charger."""
        with self.lock:
            rel = relative(self.charger, self.pose_true)
        return math.hypot(rel[0], rel[1]), abs(wrap(rel[2]))

    def pose_error(self, pose):
        """Return (position error, absolute yaw error) of the true pose versus a map pose."""
        with self.lock:
            rel = relative(pose, self.pose_true)
        return math.hypot(rel[0], rel[1]), abs(wrap(rel[2]))

    def states(self, since=0.0):
        """Return the states of the /dock/status messages with consecutive duplicates removed."""
        with self.lock:
            log = [s for s in self.status_log if s["t"] >= since]
        out = []
        for entry in log:
            if not out or out[-1] != entry["state"]:
                out.append(entry["state"])
        return out

    def last_status(self):
        with self.lock:
            return dict(self.status_log[-1]) if self.status_log else None

    def commands_since(self, t):
        with self.lock:
            return [c for c in self.cmds if c["t"] >= t]

    def publishers_of(self, topic):
        return [info.node_name for info in self.get_publishers_info_by_topic(topic)]

    # ------------------------------------------------------------------ subscriptions
    def on_cmd(self, msg):
        now = time.monotonic()
        with self.lock:
            self.cmd = (msg.linear.x, msg.angular.z)
            self.cmd_time = now
            self.cmds.append({"t": now, "v": msg.linear.x, "w": msg.angular.z,
                              "state": self.state})

    def on_status(self, msg):
        entry = {
            "t": time.monotonic(), "state": msg.state, "docked": msg.docked,
            "charging": msg.charging, "attempt": msg.attempt,
            "detection_used": msg.detection_used, "position_error": msg.position_error,
            "yaw_error": msg.yaw_error, "battery_percentage": msg.battery_percentage,
            "frame_id": msg.frame_id, "message": msg.message,
            "staging": (msg.staging_pose.x, msg.staging_pose.y, msg.staging_pose.theta),
            "dock": (msg.dock_pose.x, msg.dock_pose.y, msg.dock_pose.theta)}
        with self.lock:
            self.state = msg.state
            self.status_log.append(entry)
            runs = list(self.cancel_triggers)
        for run in runs:
            if (run.cancel_state == msg.state and run.t_cancel is None
                    and run.handle is not None and run.result is None):
                self.cancel(run)

    # ------------------------------------------------------------------ periodic work
    def step(self):
        now = time.monotonic()
        with self.lock:
            dt = min(now - self.last_step, 0.1)
            self.last_step = now
            v, w = self.cmd
            if now - self.cmd_time > MUX_TIMEOUT:
                v, w = 0.0, 0.0
            if self.safety_lock or self.stall:
                v, w = 0.0, 0.0
            if v != 0.0 or w != 0.0:
                self.pose_odom = self.advance(self.pose_odom, v, w, dt)
                self.pose_true = self.advance(self.pose_true, v, w, dt)
            self.maybe_slip()
            odom = self.pose_odom
            publish_odom = self.tf_enabled and self.odom_tf_enabled
        if publish_odom:
            self.send_tf("odom", "base_footprint", odom)

    @staticmethod
    def advance(pose, v, w, dt):
        mid = pose[2] + 0.5 * w * dt
        return (pose[0] + v * math.cos(mid) * dt, pose[1] + v * math.sin(mid) * dt,
                wrap(pose[2] + w * dt))

    def maybe_slip(self):
        """Apply the unobserved lateral slip once, near the end of the first insertion."""
        if not self.slip_first_insert or self.slip_applied or self.state != "DOCKING":
            return
        rel = relative(self.charger, self.pose_true)
        if rel[0] < 0.06:
            self.slip_applied = True
            x, y, yaw = self.pose_true
            self.pose_true = (x, y + self.slip_amount, yaw)

    def publish_map_odom(self):
        with self.lock:
            enabled = self.tf_enabled
            error = self.localization_error
        if enabled:
            self.send_tf("map", "odom", error)

    def send_tf(self, parent, child, pose):
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = parent
        msg.child_frame_id = child
        msg.transform.translation.x = pose[0]
        msg.transform.translation.y = pose[1]
        msg.transform.rotation.z = math.sin(pose[2] / 2.0)
        msg.transform.rotation.w = math.cos(pose[2] / 2.0)
        self.broadcaster.sendTransform(msg)

    def publish_safety(self):
        msg = Bool()
        with self.lock:
            msg.data = bool(self.safety_lock)
        self.safety_pub.publish(msg)

    def publish_battery(self):
        with self.lock:
            enabled = self.battery_enabled
            percentage, status = self.battery
        if not enabled:
            return
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.percentage = float(percentage)
        msg.power_supply_status = int(status)
        msg.present = True
        self.battery_pub.publish(msg)

    def make_detection(self, rel, frame="base_footprint"):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame
        msg.pose.position.x = rel[0]
        msg.pose.position.y = rel[1]
        msg.pose.orientation.z = math.sin(rel[2] / 2.0)
        msg.pose.orientation.w = math.cos(rel[2] / 2.0)
        return msg

    def publish_detection(self):
        with self.lock:
            enabled = self.detections
            inject = self.inject_active
            rel = relative(self.pose_true, self.charger)
            kind = None
            if inject:
                kind = self.inject_kinds[self.inject_index % len(self.inject_kinds)]
                self.inject_index += 1
        if enabled:
            self.detection_pub.publish(self.make_detection(rel))
        if kind is not None:
            self.detection_pub.publish(self.make_bad_detection(kind, rel))

    def make_bad_detection(self, kind, rel):
        if kind == "outlier":
            return self.make_detection((rel[0] + 0.5, rel[1] + 0.5, rel[2]))
        if kind == "nan":
            return self.make_detection((math.nan, rel[1], rel[2]))
        if kind == "zero_quat":
            msg = self.make_detection(rel)
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 0.0
            return msg
        if kind == "empty_frame":
            return self.make_detection(rel, frame="")
        return self.make_detection(rel, frame="no_such_frame")

    # ------------------------------------------------------------------ Nav2 mock
    def nav_on_goal(self, request):
        pose = request.pose
        with self.lock:
            self.nav_count += 1
            self.nav_goals.append({
                "t": time.monotonic(), "frame_id": pose.header.frame_id, "x": pose.pose.position.x,
                "y": pose.pose.position.y, "yaw": yaw_of(pose.pose.orientation)})
            delay = self.nav_accept_delay
        # A slow Nav2: the goal response is held back, then the goal is accepted
        end = time.monotonic() + delay
        while time.monotonic() < end and not self.closing:
            time.sleep(0.01)
        return GoalResponse.ACCEPT

    def nav_on_cancel(self, _handle):
        with self.lock:
            self.nav_cancel_requests.append({"t": time.monotonic()})
        return CancelResponse.ACCEPT

    def nav_execute(self, goal_handle):
        try:
            return self.nav_run(goal_handle)
        except Exception:  # e.g. the server was destroyed while the goal was running
            return NavigateToPose.Result()

    def nav_run(self, goal_handle):
        with self.lock:
            epoch = self.nav_epoch
            script = dict(self.nav_script)
            index = self.nav_count
        end = time.monotonic() + float(script.get("duration", 0.3))
        while time.monotonic() < end:
            if self.closing or epoch != self.nav_epoch:
                return NavigateToPose.Result()
            if goal_handle.is_cancel_requested:
                with self.lock:
                    self.nav_cancels.append({"t": time.monotonic()})
                goal_handle.canceled()
                return NavigateToPose.Result()
            feedback = NavigateToPose.Feedback()
            feedback.distance_remaining = max(0.0, end - time.monotonic())
            goal_handle.publish_feedback(feedback)
            time.sleep(0.05)
        if goal_handle.is_cancel_requested:
            with self.lock:
                self.nav_cancels.append({"t": time.monotonic()})
            goal_handle.canceled()
            return NavigateToPose.Result()
        if script.get("always_abort") or index <= int(script.get("abort_first", 0)):
            with self.lock:
                self.nav_results.append({"t": time.monotonic(), "result": "aborted"})
            goal_handle.abort()
            return NavigateToPose.Result()
        pose = goal_handle.request.pose
        goal = (pose.pose.position.x, pose.pose.position.y, yaw_of(pose.pose.orientation))
        with self.lock:
            perceived = (goal[0] + self.staging_error[0], goal[1] + self.staging_error[1],
                         wrap(goal[2] + self.staging_error[2]))
            # the localization error E maps the true pose to the pose the node sees in map
            target = compose(inverse(self.localization_error), perceived)
            self.pose_true = target
            self.pose_odom = target
        time.sleep(0.15)
        with self.lock:
            self.nav_results.append({"t": time.monotonic(), "result": "succeeded"})
        goal_handle.succeed()
        return NavigateToPose.Result()

    # ------------------------------------------------------------------ DockRobot client
    def send_goal(self, command):
        """Send a DockRobot goal without blocking; returns the GoalRun that fills itself."""
        run = GoalRun(command)
        goal = DockRobot.Goal()
        goal.command = command

        def on_feedback(msg):
            run.feedback.append((time.monotonic(), msg.feedback))

        def on_response(future):
            try:
                handle = future.result()
            except Exception as exc:  # noqa: BLE001
                run.error = exc
                run.accepted = False
                run.t_decision = time.monotonic()
                return
            run.t_decision = time.monotonic()
            run.handle = handle
            run.accepted = bool(handle.accepted)
            if run.accepted:
                handle.get_result_async().add_done_callback(on_result)

        def on_result(future):
            run.t_result = time.monotonic()
            run.result = future.result()

        self.client.send_goal_async(goal, feedback_callback=on_feedback).add_done_callback(
            on_response)
        return run

    def cancel(self, run):
        run.t_cancel = time.monotonic()
        future = run.handle.cancel_goal_async()

        def on_cancel_response(done):
            try:
                run.cancel_code = done.result().return_code
            except Exception:  # noqa: BLE001
                run.cancel_code = -1

        future.add_done_callback(on_cancel_response)
        return future

    def cancel_on_state(self, run, state):
        """Cancel the goal as soon as a /dock/status message with this state arrives."""
        run.cancel_state = state
        with self.lock:
            self.cancel_triggers.append(run)

    def wait_decision(self, run, timeout=5.0):
        end = time.monotonic() + timeout
        while run.t_decision is None and time.monotonic() < end:
            time.sleep(0.005)
        return run.t_decision is not None

    def wait_result(self, run, timeout=60.0):
        end = time.monotonic() + timeout
        while run.result is None and run.accepted is not False and time.monotonic() < end:
            time.sleep(0.02)
        return run.result is not None


def spin_in_thread(nodes, num_threads=8):
    """Spin the nodes on a MultiThreadedExecutor in a daemon thread; returns (executor, thread)."""
    executor = MultiThreadedExecutor(num_threads=num_threads)
    for node in nodes:
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    return executor, thread
