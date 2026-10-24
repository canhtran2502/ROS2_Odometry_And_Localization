"""
MockNav2: a navigate_to_pose action server with scripted outcomes plus a TF broadcaster.

It stands in for the whole Nav2 + simulator stack in the mission_manager launch tests. Per
station the behavior is a dict (set by each test through ``behaviors``):

    {"abort_first": n,      # abort the first n goals for this station, then succeed
     "always_abort": bool,  # abort every goal for this station
     "offset": meters,      # robot ends this far from the station (along +x) when it succeeds
     "offsets": [m, ...],   # per-attempt offsets (overrides "offset" for the listed attempts)
     "duration": seconds,   # time before the goal finishes (default 0.2); cancel is honored
     "reject_for": seconds, # reject goals for this station until this long after its first goal
     "succeed_on_cancel": bool,  # on a cancel request finish SUCCEEDED (cancel/result race)
     "cancel_reject": bool}      # reject cancel requests for this station (goal already done)

Before returning SUCCEEDED the mock moves its robot pose to station + offset and keeps broadcasting
map -> base_footprint at 20 Hz; it waits 0.1 s after the move so the TF reaches the node.
"""
import math
import threading
import time

import yaml
from geometry_msgs.msg import TransformStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class MockNav2(Node):
    def __init__(self, stations_file, action_name="navigate_to_pose"):
        super().__init__("mock_nav2")
        with open(stations_file) as f:
            data = yaml.safe_load(f)
        self.stations = {k: v for k, v in data["stations"].items()}
        self.lock = threading.Lock()
        self.behaviors = {}
        self.goals = []        # dicts: name, frame_id, x, y, qx, qy, qz, qw, t
        self.cancels = []      # dicts: name, t
        self.reached = []      # dicts: name, x, y, t (robot pose when SUCCEEDED was returned)
        self.rejected = []     # dicts: name, t (goals rejected by the goal callback)
        self.first_goal_t = {}
        self.cancel_requests = {}
        self.counts = {}
        self.robot = (0.0, 0.0)
        self.group = ReentrantCallbackGroup()
        self.broadcaster = TransformBroadcaster(self)
        self.create_timer(0.05, self.publish_tf, callback_group=self.group)
        self.action_name = action_name
        self.closing = False
        self.server = None
        self.start_server()

    def start_server(self):
        """(Re)create the navigate_to_pose action server."""
        if self.server is None:
            self.server = ActionServer(
                self, NavigateToPose, self.action_name,
                execute_callback=self.execute,
                goal_callback=self.on_goal,
                cancel_callback=self.on_cancel,
                callback_group=self.group)

    def stop_server(self):
        """Make the action server disappear (simulates a dead Nav2)."""
        if self.server is not None:
            self.server.destroy()
            self.server = None

    def close(self):
        """Test teardown: end running goal threads and drop the server."""
        self.closing = True
        time.sleep(0.2)
        self.stop_server()

    def reset(self, behaviors=None):
        with self.lock:
            self.behaviors = dict(behaviors or {})
            self.goals = []
            self.cancels = []
            self.reached = []
            self.rejected = []
            self.first_goal_t = {}
            self.cancel_requests = {}
            self.counts = {}
            self.robot = (0.0, 0.0)

    def on_goal(self, request):
        p = request.pose.pose.position
        name = self.resolve(p.x, p.y)
        with self.lock:
            beh = self.behaviors.get(name, {})
            window = float(beh.get("reject_for", 0.0))
            now = time.monotonic()
            first = self.first_goal_t.setdefault(name, now)
            if window > 0.0 and now - first < window:
                self.rejected.append({"name": name, "t": now})
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def on_cancel(self, handle):
        p = handle.request.pose.pose.position
        name = self.resolve(p.x, p.y)
        with self.lock:
            self.cancel_requests[name] = self.cancel_requests.get(name, 0) + 1
            if self.behaviors.get(name, {}).get("cancel_reject"):
                return CancelResponse.REJECT
        return CancelResponse.ACCEPT

    def snapshot(self):
        with self.lock:
            return list(self.goals), list(self.cancels), list(self.reached)

    def publish_tf(self):
        with self.lock:
            x, y = self.robot
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.child_frame_id = "base_footprint"
        msg.transform.translation.x = x
        msg.transform.translation.y = y
        msg.transform.rotation.w = 1.0
        self.broadcaster.sendTransform(msg)

    def resolve(self, x, y):
        best, best_d = None, 1e9
        for name, s in self.stations.items():
            d = math.hypot(x - s["x"], y - s["y"])
            if d < best_d:
                best, best_d = name, d
        return best

    def execute(self, goal_handle):
        try:
            return self.run_goal(goal_handle)
        except Exception:  # e.g. the server was destroyed while the goal was running
            return NavigateToPose.Result()

    def run_goal(self, goal_handle):
        pose = goal_handle.request.pose
        p, q = pose.pose.position, pose.pose.orientation
        name = self.resolve(p.x, p.y)
        with self.lock:
            self.goals.append({
                "name": name, "frame_id": pose.header.frame_id, "x": p.x, "y": p.y,
                "qx": q.x, "qy": q.y, "qz": q.z, "qw": q.w, "t": time.monotonic()})
            n = self.counts[name] = self.counts.get(name, 0) + 1
            beh = dict(self.behaviors.get(name, {}))
        end = time.monotonic() + float(beh.get("duration", 0.2))
        while time.monotonic() < end:
            if self.closing:
                return NavigateToPose.Result()  # test teardown: drop the goal thread
            if goal_handle.is_cancel_requested:
                if beh.get("succeed_on_cancel"):
                    break
                with self.lock:
                    self.cancels.append({"name": name, "t": time.monotonic()})
                goal_handle.canceled()
                return NavigateToPose.Result()
            fb = NavigateToPose.Feedback()
            fb.distance_remaining = max(0.0, end - time.monotonic())
            goal_handle.publish_feedback(fb)
            time.sleep(0.05)
        if goal_handle.is_cancel_requested and not beh.get("succeed_on_cancel"):
            with self.lock:
                self.cancels.append({"name": name, "t": time.monotonic()})
            goal_handle.canceled()
            return NavigateToPose.Result()
        if beh.get("always_abort") or n <= int(beh.get("abort_first", 0)):
            goal_handle.abort()
            return NavigateToPose.Result()
        offsets = beh.get("offsets")
        if offsets is not None and n - 1 < len(offsets):
            offset = float(offsets[n - 1])
        else:
            offset = float(beh.get("offset", 0.0))
        s = self.stations[name]
        with self.lock:
            self.robot = (s["x"] + offset, s["y"])
        time.sleep(0.1)
        with self.lock:
            self.reached.append({"name": name, "x": s["x"] + offset, "y": s["y"],
                                 "t": time.monotonic()})
        goal_handle.succeed()
        return NavigateToPose.Result()
