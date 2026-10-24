"""MockNav2: a navigate_to_pose action server with scripted outcomes plus a TF broadcaster.

It stands in for the whole Nav2 + simulator stack in the mission_manager launch tests. Per
station the behavior is a dict (set by each test through ``behaviors``):

    {"abort_first": n,      # abort the first n goals for this station, then succeed
     "always_abort": bool,  # abort every goal for this station
     "offset": meters,      # robot ends this far from the station (along +x) when it succeeds
     "offsets": [m, ...],   # per-attempt offsets (overrides "offset" for the listed attempts)
     "duration": seconds}   # time before the goal finishes (default 0.2); cancel is honored

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
        self.counts = {}
        self.robot = (0.0, 0.0)
        self.group = ReentrantCallbackGroup()
        self.broadcaster = TransformBroadcaster(self)
        self.create_timer(0.05, self.publish_tf, callback_group=self.group)
        self.server = ActionServer(
            self, NavigateToPose, action_name,
            execute_callback=self.execute,
            goal_callback=lambda _g: GoalResponse.ACCEPT,
            cancel_callback=lambda _h: CancelResponse.ACCEPT,
            callback_group=self.group)

    def reset(self, behaviors=None):
        with self.lock:
            self.behaviors = dict(behaviors or {})
            self.goals = []
            self.cancels = []
            self.reached = []
            self.counts = {}
            self.robot = (0.0, 0.0)

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
            if goal_handle.is_cancel_requested:
                with self.lock:
                    self.cancels.append({"name": name, "t": time.monotonic()})
                goal_handle.canceled()
                return NavigateToPose.Result()
            fb = NavigateToPose.Feedback()
            fb.distance_remaining = max(0.0, end - time.monotonic())
            goal_handle.publish_feedback(fb)
            time.sleep(0.05)
        if goal_handle.is_cancel_requested:
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
