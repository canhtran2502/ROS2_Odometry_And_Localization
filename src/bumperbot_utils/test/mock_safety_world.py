"""
MockSafetyWorld: replaces everything around the safety_supervisor in node tests.

One rclpy node in the test process (own executor thread). It publishes synthetic scans built from
obstacle points given in base_footprint (the URDF laser yaw 3.14 and offset are applied), the
static TF base_footprint -> laser_link (switchable), an optional TF map -> base_footprint, odom
(scripted or following the supervisor output with a first-order lag), cmd_vel_in, the e-stop
input, /dock/status (transient_local, 2 Hz) and an optional /clock. It records every lock, output
and status message with the wall time.
"""
import math
import threading
import time

import rclpy
from bumperbot_msgs.msg import DockStatus, SafetyStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, qos_profile_sensor_data,
                       ReliabilityPolicy)
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

# URDF laser_joint (base_link) plus base_joint z: base_footprint -> laser_link
LASER_X = -0.0050526
LASER_Y = -0.0023221
LASER_Z = 0.1538
LASER_YAW = 3.14
BEAMS = 360
ANGLE_INC = 2.0 * math.pi / BEAMS
RANGE_MIN = 0.05
RANGE_MAX = 12.0
SCAN_PERIOD = 0.2          # 5 Hz like the MuJoCo laser
OUT_TOPIC = "/bumperbot_controller/cmd_vel_unstamped"

SCAN_MODES = ("normal", "stop", "nan", "empty", "zero_increment", "bad_frame", "empty_frame")
ODOM_MODES = ("follow_output", "scripted", "stop")


def stamp_of(node):
    return node.get_clock().now().to_msg()


class MockSafetyWorld:
    """Test double of the robot, the LiDAR, TF, odometry and the dock manager."""

    def __init__(self, name="mock_safety_world"):
        self.node = Node(name)
        self.lock = threading.RLock()
        self._build_publishers()
        self._build_subscribers()
        self._reset_state()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self._stop = threading.Event()
        self.thread = threading.Thread(target=self._spin, daemon=True)
        self.thread.start()
        self.node.create_timer(SCAN_PERIOD, self._scan_tick)
        self.node.create_timer(0.05, self._fast_tick)
        self.node.create_timer(0.5, self._dock_tick)

    # ------------------------------------------------------------------ construction
    def _build_publishers(self):
        n = self.node
        self.scan_pub = n.create_publisher(LaserScan, "scan", qos_profile_sensor_data)
        self.odom_pub = n.create_publisher(Odometry, "bumperbot_controller/odom", 10)
        self.cmd_pub = n.create_publisher(Twist, "safety/cmd_vel_in", 10)
        self.estop_pub = n.create_publisher(Bool, "safety/estop", 10)
        self.tf_pub = n.create_publisher(TFMessage, "/tf", 100)
        self.clock_pub = n.create_publisher(Clock, "/clock", 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.dock_qos = latched
        self.dock_pub = None
        self.static_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.static_pub = None
        self.obs_pub = None

    def _build_subscribers(self):
        n = self.node
        self.locks, self.outs, self.statuses, self.outs_w = [], [], [], []
        n.create_subscription(Bool, "safety_stop", self._on_lock, 100)
        n.create_subscription(Twist, "bumperbot_controller/cmd_vel_unstamped", self._on_out, 100)
        latched = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        n.create_subscription(SafetyStatus, "safety/status", self._on_status, latched)

    def _reset_state(self):
        with self.lock:
            self._obstacles = []
            self._version = 0
            self.scan_log = []              # (t_wall, obstacle version) of every published scan
            self.scan_mode = "normal"
            self.odom_mode = "stop"
            self._odom_v = 0.0
            self._odom_script = 0.0
            self._last_out = (0.0, 0.0)     # (t_wall, linear.x)
            self._cmd = None
            self.last_cmd_time = 0.0
            self._observed = None
            self._estop = None
            self._dock_state = None
            self._dock_pose = (0.0, 0.0)
            self._map_tf = None
            self._clock_mode = "off"
            self._clock_t = 0.0
            self.locks.clear()
            self.outs.clear()
            self.outs_w.clear()
            self.statuses.clear()
        self.set_laser_tf(False)
        if self.obs_pub is not None:
            self.node.destroy_publisher(self.obs_pub)
            self.obs_pub = None
        if self.dock_pub is not None:
            # drop the latched /dock/status of the previous scenario
            self.node.destroy_publisher(self.dock_pub)
            self.dock_pub = None

    def reset(self):
        """Back to the default: nothing is published and nothing is recorded."""
        self._reset_state()

    def _spin(self):
        while not self._stop.is_set():
            try:
                self.executor.spin_once(timeout_sec=0.02)
            except Exception:  # noqa: BLE001 - keep the mock alive across a shutdown race
                time.sleep(0.01)

    def shutdown(self):
        self._stop.set()
        self.thread.join(timeout=3.0)
        self.executor.shutdown()
        self.node.destroy_node()

    # ------------------------------------------------------------------ recording
    def _on_lock(self, msg):
        with self.lock:
            self.locks.append((time.monotonic(), msg.data))

    def _on_out(self, msg):
        with self.lock:
            now = time.monotonic()
            self.outs.append((now, msg.linear.x))
            self.outs_w.append((now, msg.angular.z))
            self._last_out = (now, msg.linear.x)

    def _on_status(self, msg):
        with self.lock:
            self.statuses.append((time.monotonic(), msg))

    def mark(self):
        """Return the current record positions, for the *_since() helpers."""
        with self.lock:
            return len(self.locks), len(self.outs), len(self.statuses)

    def locks_since(self, mark):
        with self.lock:
            return list(self.locks[mark[0]:])

    def outs_since(self, mark):
        with self.lock:
            return list(self.outs[mark[1]:])

    def statuses_since(self, mark):
        with self.lock:
            return list(self.statuses[mark[2]:])

    def last_status(self):
        with self.lock:
            return self.statuses[-1][1] if self.statuses else None

    def last_lock(self):
        with self.lock:
            return self.locks[-1][1] if self.locks else None

    def last_out(self):
        with self.lock:
            return self.outs[-1][1] if self.outs else None

    def wait_status(self, predicate, timeout, since=None):
        """Return (t_wall, msg) of the first status after `since` that satisfies the predicate."""
        with self.lock:
            # default: the current (latest) status and everything newer
            start = max(0, len(self.statuses) - 1) if since is None else since[2]
        end = time.monotonic() + timeout
        seen = start
        while time.monotonic() < end:
            with self.lock:
                new = self.statuses[seen:]
                seen = len(self.statuses)
            for t, msg in new:
                if predicate(msg):
                    return t, msg
            time.sleep(0.01)
        return None

    def wait_for(self, predicate, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if predicate():
                return True
            time.sleep(0.01)
        return False

    # ------------------------------------------------------------------ scenario controls
    def set_obstacles(self, points):
        """Points in base_footprint. Returns the version, see first_scan_with()."""
        with self.lock:
            self._obstacles = [tuple(p) for p in points]
            self._version += 1
            return self._version

    def first_scan_with(self, version, timeout=2.0):
        """Wall time of the first scan published with the given obstacle version."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            with self.lock:
                for t, v in self.scan_log:
                    if v >= version:
                        return t
            time.sleep(0.005)
        return None

    def set_cmd(self, v, w=0.0):
        with self.lock:
            self._cmd = None if v is None else (float(v), float(w))

    @property
    def odom_v(self):
        with self.lock:
            return self._odom_v

    def set_observed(self, v):
        """lock_only: publish an observed command on cmd_vel_out_topic (None: stop)."""
        with self.lock:
            self._observed = None if v is None else float(v)
        if v is not None and self.obs_pub is None:
            self.obs_pub = self.node.create_publisher(
                Twist, "bumperbot_controller/cmd_vel_unstamped", 10)

    def publish_dock_now(self):
        self._dock_tick()

    def set_estop(self, value):
        with self.lock:
            self._estop = value

    def set_odom(self, mode, v=0.0):
        assert mode in ODOM_MODES
        with self.lock:
            self.odom_mode = mode
            self._odom_script = float(v)
            if mode == "scripted":
                self._odom_v = float(v)

    def set_scan_mode(self, mode):
        assert mode in SCAN_MODES
        with self.lock:
            self.scan_mode = mode

    def set_dock(self, state, dock_xy=(0.0, 0.0)):
        with self.lock:
            self._dock_state = state
            self._dock_pose = dock_xy

    def set_map_tf(self, robot_xy):
        """TF map -> base_footprint with the robot at robot_xy (None: stop publishing)."""
        with self.lock:
            self._map_tf = robot_xy
        if robot_xy is not None:
            self._publish_map_tf(robot_xy)

    def _publish_map_tf(self, map_tf):
        msg = TFMessage()
        tf = TransformStamped()
        tf.header.stamp = stamp_of(self.node)
        tf.header.frame_id = "map"
        tf.child_frame_id = "base_footprint"
        tf.transform.translation.x = float(map_tf[0])
        tf.transform.translation.y = float(map_tf[1])
        tf.transform.rotation.w = 1.0
        msg.transforms.append(tf)
        self.tf_pub.publish(msg)

    def set_clock(self, mode):
        assert mode in ("off", "running", "stopped")
        with self.lock:
            if mode == "running" and self._clock_mode == "off":
                self._clock_t = 1000.0
            self._clock_mode = mode

    def set_laser_tf(self, enabled):
        """Switch the latched static TF base_footprint -> laser_link on or off."""
        if enabled and self.static_pub is None:
            self.static_pub = self.node.create_publisher(TFMessage, "/tf_static", self.static_qos)
            msg = TFMessage()
            tf = TransformStamped()
            tf.header.stamp = stamp_of(self.node)
            tf.header.frame_id = "base_footprint"
            tf.child_frame_id = "laser_link"
            tf.transform.translation.x = LASER_X
            tf.transform.translation.y = LASER_Y
            tf.transform.translation.z = LASER_Z
            tf.transform.rotation.z = math.sin(LASER_YAW / 2.0)
            tf.transform.rotation.w = math.cos(LASER_YAW / 2.0)
            msg.transforms.append(tf)
            self.static_pub.publish(msg)
        elif not enabled and self.static_pub is not None:
            self.node.destroy_publisher(self.static_pub)
            self.static_pub = None

    # ------------------------------------------------------------------ publishing
    def build_ranges(self, obstacles):
        """Ranges (laser frame) whose points land at the given base_footprint positions."""
        ranges = [float("inf")] * BEAMS
        for ox, oy in obstacles:
            dx, dy = ox - LASER_X, oy - LASER_Y
            angle = math.atan2(dy, dx) - LASER_YAW
            index = int(round((angle % (2.0 * math.pi)) / ANGLE_INC)) % BEAMS
            beam = index * ANGLE_INC + LASER_YAW
            if abs(math.cos(beam)) > 0.5:
                r = dx / math.cos(beam)        # match x exactly (front and rear obstacles)
            else:
                r = math.hypot(dx, dy)
            ranges[index] = min(ranges[index], r)
        return ranges

    def _scan_tick(self):
        with self.lock:
            mode, obstacles, version = self.scan_mode, list(self._obstacles), self._version
        if mode == "stop":
            return
        msg = LaserScan()
        msg.header.stamp = stamp_of(self.node)
        msg.header.frame_id = "laser_link"
        msg.angle_min = 0.0
        msg.angle_max = ANGLE_INC * (BEAMS - 1)
        msg.angle_increment = ANGLE_INC
        msg.scan_time = SCAN_PERIOD
        msg.range_min = RANGE_MIN
        msg.range_max = RANGE_MAX
        ranges = self.build_ranges(obstacles)
        if mode == "nan":
            for i in range(int(0.8 * BEAMS)):
                ranges[i] = float("nan")
        elif mode == "empty":
            ranges = []
        elif mode == "zero_increment":
            msg.angle_increment = 0.0
        elif mode == "bad_frame":
            msg.header.frame_id = "other_link"
        elif mode == "empty_frame":
            msg.header.frame_id = ""
        msg.ranges = ranges
        self.scan_pub.publish(msg)
        with self.lock:
            self.scan_log.append((time.monotonic(), version))
            del self.scan_log[:-200]

    def _fast_tick(self):
        """20 Hz: odom, cmd_vel_in, e-stop, map TF and clock."""
        dt = 0.05
        with self.lock:
            odom_mode, script, cmd = self.odom_mode, self._odom_script, self._cmd
            estop, map_tf, clock_mode = self._estop, self._map_tf, self._clock_mode
            if odom_mode == "follow_output":
                t_out, v_out = self._last_out
                target = v_out if time.monotonic() - t_out < 0.3 else 0.0
                self._odom_v += (target - self._odom_v) * dt / (0.1 + dt)
            elif odom_mode == "scripted":
                self._odom_v = script
            v = self._odom_v
            if clock_mode == "running":
                self._clock_t += dt
            clock_t = self._clock_t
        if odom_mode != "stop":
            odom = Odometry()
            odom.header.stamp = stamp_of(self.node)
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_footprint"
            odom.twist.twist.linear.x = float(v)
            self.odom_pub.publish(odom)
        if cmd is not None:
            twist = Twist()
            twist.linear.x, twist.angular.z = cmd
            self.cmd_pub.publish(twist)
            with self.lock:
                self.last_cmd_time = time.monotonic()
        with self.lock:
            observed = self._observed
        if observed is not None and self.obs_pub is not None:
            twist = Twist()
            twist.linear.x = observed
            self.obs_pub.publish(twist)
        if estop is not None:
            self.estop_pub.publish(Bool(data=bool(estop)))
        if map_tf is not None:
            self._publish_map_tf(map_tf)
        if clock_mode == "running":
            clock = Clock()
            clock.clock.sec = int(clock_t)
            clock.clock.nanosec = int((clock_t - int(clock_t)) * 1e9)
            self.clock_pub.publish(clock)

    def _dock_tick(self):
        with self.lock:
            state, pose = self._dock_state, self._dock_pose
        if state is None:
            return
        if self.dock_pub is None:
            self.dock_pub = self.node.create_publisher(DockStatus, "dock/status", self.dock_qos)
        msg = DockStatus()
        msg.stamp = stamp_of(self.node)
        msg.state = state
        msg.frame_id = "map"
        msg.dock_pose.x = float(pose[0])
        msg.dock_pose.y = float(pose[1])
        self.dock_pub.publish(msg)

    # ------------------------------------------------------------------ services and graph
    def call_trigger(self, service, timeout=10.0):
        """Call a std_srvs/Trigger service; returns (success, message) or None on timeout."""
        from std_srvs.srv import Trigger
        client = self.node.create_client(Trigger, service)
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                return None
            future = client.call_async(Trigger.Request())
            end = time.monotonic() + timeout
            while not future.done() and time.monotonic() < end:
                time.sleep(0.005)
            if not future.done():
                return None
            res = future.result()
            return res.success, res.message
        finally:
            self.node.destroy_client(client)

    def get_parameters(self, node_name, timeout=10.0):
        """Return name -> value of all parameters of a node (retried: discovery may be slow)."""
        from rcl_interfaces.srv import GetParameters, ListParameters
        from rclpy.parameter import parameter_value_to_python
        end = time.monotonic() + timeout
        lst = self.node.create_client(ListParameters, node_name + "/list_parameters")
        get = self.node.create_client(GetParameters, node_name + "/get_parameters")
        try:
            while time.monotonic() < end:
                if not (lst.wait_for_service(timeout_sec=0.5) and
                        get.wait_for_service(timeout_sec=0.5)):
                    continue
                fut = lst.call_async(ListParameters.Request())
                while not fut.done() and time.monotonic() < end:
                    time.sleep(0.005)
                if not fut.done():
                    continue
                names = list(fut.result().result.names)
                req = GetParameters.Request()
                req.names = names
                fut2 = get.call_async(req)
                while not fut2.done() and time.monotonic() < end:
                    time.sleep(0.005)
                if not fut2.done():
                    continue
                values = [parameter_value_to_python(v) for v in fut2.result().values]
                return dict(zip(names, values))
            return None
        finally:
            self.node.destroy_client(lst)
            self.node.destroy_client(get)


def init_ros():
    if not rclpy.ok():
        rclpy.init()
