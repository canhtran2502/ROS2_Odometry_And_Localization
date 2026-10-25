#!/usr/bin/env python3
"""
AGV safety supervisor: direction-aware speed limiting, latched e-stop and sensor watchdog.

The C++ twin is src/safety_supervisor.cpp (same parameters, interfaces and behavior). All
decisions live in safety_logic.py (SafetyCore); this node only does the ROS I/O.

Fail-safe rules:
  * the protection loop runs on a steady (wall) timer, so it survives a stalled time source;
  * missing or stale inputs lock the robot (INITIALIZING / FAULT) with a stated reason;
  * the lock topic is true on shutdown and on a startup error (bad configuration): the lock
    stays engaged until a new supervisor (or an operator) publishes false.
"""
import math
import signal
import sys
import time

import rclpy
from bumperbot_msgs.msg import DockStatus, SafetyStatus
from bumperbot_utils.safety_logic import (
    Config, DockGate, ENFORCEMENT_FILTER, Evaluation, Inputs, Outputs, SafetyCore, STATE_FAULT,
    STATE_SHUTDOWN, ZONE_UNKNOWN, blind_zone_warning, evaluate_points, scan_to_points,
    validate_config)
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy._rclpy_pybind11 import RCLError
from rclpy.clock import Clock, ClockType
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

_NAN = float("nan")
_FALLBACK_LOCK_TOPIC = "safety_stop"


class StartupError(Exception):
    """Raised when the node cannot start (the lock has been published and FATAL logged)."""


def _quaternion_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SafetySupervisor(Node):
    def __init__(self):
        super().__init__("safety_supervisor")
        self._started = False
        try:
            self._setup()
        except StartupError:
            raise
        except Exception as exc:  # noqa: BLE001 - any setup failure is fatal and fail-safe
            self._fail_safe("startup failed: " + str(exc))
        self._started = True

    # -----------------------------------------------------------------------------------------
    # Setup
    # -----------------------------------------------------------------------------------------

    def _declare(self, name, default, kind):
        """Declare a parameter and check its type (a mismatching override is a startup error)."""
        try:
            self.declare_parameter(name, default)
            value = self.get_parameter(name).value
        except Exception as exc:  # noqa: BLE001 - rclpy raises several exception types
            raise ValueError("parameter '%s': %s" % (name, exc))
        if kind == "float":
            ok = isinstance(value, float)
        elif kind == "int":
            ok = isinstance(value, int) and not isinstance(value, bool)
        elif kind == "bool":
            ok = isinstance(value, bool)
        elif kind == "str":
            ok = isinstance(value, str)
        else:
            ok = (isinstance(value, (list, tuple)) and
                  all(isinstance(item, float) for item in value))
        if not ok:
            raise ValueError("parameter '%s' has the wrong type (expected %s)" % (name, kind))
        return value

    def _load_config(self):
        """Declare all parameters and return the Config (raises ValueError on a bad one)."""
        cfg = Config()
        for name in ("enforcement", "robot_frame", "scan_topic", "odom_topic",
                     "cmd_vel_in_topic", "cmd_vel_out_topic", "lock_topic", "status_topic",
                     "estop_topic", "dock_status_topic"):
            setattr(cfg, name, self._declare(name, getattr(cfg, name), "str"))
        for name in ("use_odom", "latch_faults", "latch_on_contact", "dock_approach_enabled",
                     "require_reset_after_restart"):
            setattr(cfg, name, self._declare(name, getattr(cfg, name), "bool"))
        cfg.footprint = list(self._declare("footprint", list(cfg.footprint), "float_array"))
        for name in ("lateral_margin", "stop_margin", "stop_release_hysteresis",
                     "slowdown_distance", "contact_margin", "max_linear_speed",
                     "max_angular_speed", "response_time", "max_decel", "stopped_speed",
                     "brake_hold_time", "scan_timeout", "odom_timeout", "cmd_timeout",
                     "recovery_time", "max_invalid_beam_ratio", "rate", "status_rate",
                     "dock_max_speed", "dock_stop_margin", "dock_approach_radius",
                     "dock_approach_max_time", "dock_status_timeout"):
            setattr(cfg, name, self._declare(name, getattr(cfg, name), "float"))
        cfg.contact_debounce_scans = self._declare(
            "contact_debounce_scans", cfg.contact_debounce_scans, "int")
        cfg.debug_inject_tick_exceptions = self._declare(
            "debug_inject_tick_exceptions", cfg.debug_inject_tick_exceptions, "int")
        return cfg

    def _fail_safe(self, reason):
        """Publish the lock (true) on a startup error, log FATAL and abort the startup."""
        topic = _FALLBACK_LOCK_TOPIC
        try:
            value = self.get_parameter("lock_topic").value
            if isinstance(value, str) and value:
                topic = value
        except Exception:  # noqa: BLE001 - parameter missing or undeclared
            pass
        try:
            lock_pub = self.create_publisher(Bool, topic, 10)
            time.sleep(0.2)
            for _ in range(3):
                lock_pub.publish(Bool(data=True))
                time.sleep(0.1)
        except Exception as exc:  # noqa: BLE001 - nothing more can be done
            self.get_logger().error("could not publish the lock: %s" % exc)
        self.get_logger().fatal("safety_supervisor cannot start: %s" % reason)
        raise StartupError(reason)

    def _setup(self):
        try:
            cfg = self._load_config()
        except ValueError as exc:
            self._fail_safe(str(exc))
        error = validate_config(cfg)
        if error:
            self._fail_safe("invalid configuration: " + error)
        self.cfg = cfg
        self.filter_mode = cfg.enforcement == ENFORCEMENT_FILTER
        self.core = SafetyCore(cfg)
        self.dock_gate = DockGate(cfg)

        # Inputs, written by the callbacks only
        self._scan_time = None
        self._scan_frame = None
        self._scan_invalid = ""
        self._scan_ok = False
        self._scan_pending = None
        self._evaluation = Evaluation()
        self._evaluation_seq = 0
        self._blind_checked = False
        self._tf = None
        self._odom_time = None
        self._odom_v = _NAN
        self._cmd_time = None
        self._cmd_v = _NAN
        self._estop_input = False
        self._dock_state = ""
        self._dock_time = None
        self._dock_frame = ""
        self._dock_xy = (_NAN, _NAN)
        self._pending_exception = ""
        self._tick_count = 0
        self._last_outputs = Outputs()
        self._last_inputs = Inputs()
        self._last_status_key = None
        self._last_status_time = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.lock_pub = self.create_publisher(Bool, cfg.lock_topic, 10)
        status_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(SafetyStatus, cfg.status_topic, status_qos)
        self.out_pub = None
        if self.filter_mode:
            self.out_pub = self.create_publisher(Twist, cfg.cmd_vel_out_topic, 10)

        self.create_subscription(LaserScan, cfg.scan_topic, self._on_scan, qos_profile_sensor_data)
        if cfg.use_odom:
            self.create_subscription(Odometry, cfg.odom_topic, self._on_odom, 10)
        if self.filter_mode:
            self.create_subscription(Twist, cfg.cmd_vel_in_topic, self._on_cmd, 10)
        else:
            # lock_only: the command is only observed (direction of travel)
            self.create_subscription(Twist, cfg.cmd_vel_out_topic, self._on_observed_cmd, 10)
        # Two subscriptions: volatile (any publisher) and transient_local (replays the last value
        # of a latched publisher to a restarted supervisor)
        self.create_subscription(Bool, cfg.estop_topic, self._on_estop, 10)
        estop_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, cfg.estop_topic, self._on_estop, estop_qos)
        dock_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(DockStatus, cfg.dock_status_topic, self._on_dock, dock_qos)

        self.create_service(Trigger, "~/reset", self._on_reset)
        self.create_service(Trigger, "~/trigger_estop", self._on_trigger_estop)

        # Steady timer: keeps running when the ROS time source stalls
        self.tick_timer = self.create_timer(
            1.0 / cfg.rate, self._tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(
            "safety_supervisor started (Python), enforcement=%s, scan=%s, lock=%s"
            % (cfg.enforcement, cfg.scan_topic, cfg.lock_topic))

    # -----------------------------------------------------------------------------------------
    # Callbacks (store only)
    # -----------------------------------------------------------------------------------------

    def _note_exception(self, where, exc):
        self._pending_exception = "%s: %s" % (where, exc)
        self.get_logger().error("exception in %s: %s" % (where, exc), throttle_duration_sec=2.0)

    def _on_scan(self, msg):
        try:
            self._handle_scan(msg)
        except Exception as exc:  # noqa: BLE001 - never let a callback end the loop
            self._note_exception("scan callback", exc)

    def _handle_scan(self, msg):
        self._scan_time = time.monotonic()
        self._scan_ok = False
        self._scan_pending = None
        frame = msg.header.frame_id
        if not frame:
            self._scan_invalid = "invalid scan: empty frame_id"
            return
        if self._scan_frame is None:
            self._scan_frame = frame
        elif frame != self._scan_frame:
            self._scan_invalid = "scan frame changed"
            return
        if len(msg.ranges) == 0:
            self._scan_invalid = "invalid scan: empty ranges"
            return
        if not (math.isfinite(msg.angle_increment) and msg.angle_increment > 0.0):
            self._scan_invalid = "invalid scan: angle_increment <= 0"
            return
        self._scan_invalid = ""
        self._scan_pending = msg
        self._try_evaluate()

    def _lookup_laser_tf(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.cfg.robot_frame, self._scan_frame, Time())
        except TransformException:
            return None
        t = tf.transform.translation
        return (t.x, t.y, _quaternion_yaw(tf.transform.rotation))

    def _try_evaluate(self):
        """Evaluate the pending scan once the laser TF is known (looked up with timeout 0)."""
        if self._scan_pending is None:
            return
        if self._tf is None:
            if self._scan_frame is None:
                return
            self._tf = self._lookup_laser_tf()
            if self._tf is None:
                return
        msg = self._scan_pending
        self._scan_pending = None
        points, ratio = scan_to_points(
            list(msg.ranges), msg.angle_min, msg.angle_increment, msg.range_min,
            msg.range_max, self._tf[0], self._tf[1], self._tf[2])
        if ratio > self.cfg.max_invalid_beam_ratio:
            self._scan_invalid = "invalid scan: nan ratio %.2f" % ratio
            return
        self._evaluation = evaluate_points(points, self.cfg)
        self._evaluation_seq += 1
        self._scan_ok = True
        if not self._blind_checked:
            self._blind_checked = True
            warning = blind_zone_warning(self.cfg, self._tf[0], self._tf[1], msg.range_min)
            if warning:
                self.get_logger().warn(warning)

    def _on_odom(self, msg):
        try:
            self._odom_time = time.monotonic()
            self._odom_v = msg.twist.twist.linear.x
        except Exception as exc:  # noqa: BLE001
            self._note_exception("odom callback", exc)

    def _on_cmd(self, msg):
        try:
            self._cmd_time = time.monotonic()
            self._cmd_v = msg.linear.x
            v, w = self.core.limit_command(msg.linear.x, msg.angular.z, self._last_outputs)
            out = Twist()
            out.linear.x = v
            out.angular.z = w
            self.out_pub.publish(out)
        except Exception as exc:  # noqa: BLE001
            self._note_exception("cmd callback", exc)

    def _on_observed_cmd(self, msg):
        try:
            self._cmd_time = time.monotonic()
            self._cmd_v = msg.linear.x
        except Exception as exc:  # noqa: BLE001
            self._note_exception("cmd callback", exc)

    def _on_estop(self, msg):
        self._estop_input = bool(msg.data)

    def _on_dock(self, msg):
        try:
            self._dock_state = msg.state
            self._dock_time = time.monotonic()
            self._dock_frame = msg.frame_id
            self._dock_xy = (msg.dock_pose.x, msg.dock_pose.y)
        except Exception as exc:  # noqa: BLE001
            self._note_exception("dock callback", exc)

    def _on_reset(self, request, response):
        try:
            ok, message = self.core.request_reset()
            if ok and message == "reset":
                self.get_logger().info("e-stop reset")
                self._tick()
        except Exception as exc:  # noqa: BLE001
            ok, message = False, "internal error: %s" % exc
        response.success = ok
        response.message = message
        return response

    def _on_trigger_estop(self, request, response):
        try:
            self.core.trigger_estop("e-stop requested by service")
            self.get_logger().warn("e-stop latched by service")
            self._tick()
            response.success = True
            response.message = "estop latched"
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = "internal error: %s" % exc
        return response

    # -----------------------------------------------------------------------------------------
    # Tick
    # -----------------------------------------------------------------------------------------

    def _dock_distance(self):
        """Distance robot_frame to the dock pose through TF (NaN when unavailable)."""
        if not self._dock_frame:
            return _NAN
        try:
            tf = self.tf_buffer.lookup_transform(
                self._dock_frame, self.cfg.robot_frame, Time())
        except TransformException:
            return _NAN
        stamp = tf.header.stamp
        if stamp.sec != 0 or stamp.nanosec != 0:
            age = self.get_clock().now().nanoseconds * 1e-9 - (stamp.sec + stamp.nanosec * 1e-9)
            if age > self.cfg.dock_status_timeout:
                return _NAN
        t = tf.transform.translation
        return math.hypot(t.x - self._dock_xy[0], t.y - self._dock_xy[1])

    def _build_inputs(self, now):
        cfg = self.cfg
        self._try_evaluate()
        inp = Inputs()
        if self._scan_time is not None:
            inp.scan_age = now - self._scan_time
        inp.scan_invalid_reason = self._scan_invalid
        inp.scan_ok = self._scan_ok
        inp.tf_ok = self._tf is not None
        inp.tf_description = "%s <- %s" % (
            cfg.robot_frame, self._scan_frame if self._scan_frame else "<scan frame>")
        inp.evaluation = self._evaluation
        inp.scan_seq = self._evaluation_seq
        if cfg.use_odom and self._odom_time is not None:
            inp.odom_age = now - self._odom_time
            inp.v_odom = self._odom_v
        if self._cmd_time is not None:
            inp.cmd_age = now - self._cmd_time
            inp.v_cmd = self._cmd_v
        inp.estop_input = self._estop_input
        dock_age = now - self._dock_time if self._dock_time is not None else -1.0
        distance = _NAN
        if (cfg.dock_approach_enabled and self._dock_state == "DOCKING" and
                0.0 <= dock_age < cfg.dock_status_timeout):
            distance = self._dock_distance()
        inp.dock_active, inp.dock_reason = self.dock_gate.update(
            now, cfg.dock_approach_enabled, self._dock_state, dock_age, distance)
        return inp

    def _tick(self):
        now = time.monotonic()
        try:
            self._tick_count += 1
            if self._tick_count <= self.cfg.debug_inject_tick_exceptions:
                raise RuntimeError("injected tick exception %d" % self._tick_count)
            inp = self._build_inputs(now)
            if self._pending_exception:
                inp.tick_exception = self._pending_exception
        except Exception as exc:  # noqa: BLE001 - the loop must survive any error
            self.get_logger().error("exception in tick: %s" % exc, throttle_duration_sec=2.0)
            inp = Inputs(tick_exception=str(exc))
        self._pending_exception = ""
        try:
            outputs = self.core.update(now, inp)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error("exception in core: %s" % exc, throttle_duration_sec=2.0)
            outputs = Outputs(state=STATE_FAULT, locked=True, reason="internal error: %s" % exc)
        self._last_outputs = outputs
        self._last_inputs = inp
        try:
            self._publish(outputs, inp, now)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error("exception while publishing: %s" % exc,
                                    throttle_duration_sec=2.0)

    def _publish(self, out, inp, now):
        cfg = self.cfg
        self.lock_pub.publish(Bool(data=bool(out.locked)))
        if self.filter_mode:
            cmd_stale = inp.cmd_age < 0.0 or inp.cmd_age > cfg.cmd_timeout
            if out.locked or cmd_stale:
                self.out_pub.publish(Twist())
        key = (out.state, out.active_zone, out.front_zone, out.rear_zone, out.locked,
               out.braking, out.estop_latched, out.dock_profile_active, out.reason)
        period = 1.0 / cfg.status_rate
        if key != self._last_status_key or now - self._last_status_time >= period - 0.5 / cfg.rate:
            self._last_status_key = key
            self._last_status_time = now
            self.status_pub.publish(self._build_status(out, inp))

    def _build_status(self, out, inp):
        ev = inp.evaluation
        known = out.front_zone != ZONE_UNKNOWN
        msg = SafetyStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.state = out.state
        msg.enforcement = self.cfg.enforcement
        msg.active_zone = out.active_zone
        msg.front_zone = out.front_zone
        msg.rear_zone = out.rear_zone
        msg.reason = out.reason
        msg.locked = bool(out.locked)
        msg.braking = bool(out.braking)
        msg.estop_latched = bool(out.estop_latched)
        msg.dock_profile_active = bool(out.dock_profile_active)
        msg.nearest_obstacle_distance = float(ev.nearest_distance) if known else -1.0
        msg.nearest_obstacle_bearing = float(ev.nearest_bearing) if known else 0.0
        msg.front_clearance = float(ev.front_clearance) if known else -1.0
        msg.rear_clearance = float(ev.rear_clearance) if known else -1.0
        msg.max_forward_speed = float(out.max_forward)
        msg.max_reverse_speed = float(out.max_reverse)
        msg.scan_age = float(inp.scan_age)
        msg.odom_age = float(inp.odom_age)
        msg.cmd_age = float(inp.cmd_age)
        return msg

    # -----------------------------------------------------------------------------------------
    # Shutdown
    # -----------------------------------------------------------------------------------------

    def finalize(self):
        """Lock the robot: SHUTDOWN status, lock true and a zero command, 3 times."""
        if not self._started:
            return
        self.core.shutdown()
        inp = self._last_inputs
        out = self.core.update(time.monotonic(), Inputs())
        for _ in range(3):
            try:
                self.lock_pub.publish(Bool(data=True))
                if self.filter_mode:
                    self.out_pub.publish(Twist())
                status = self._build_status(out, inp)
                status.state = STATE_SHUTDOWN
                status.locked = True
                status.max_forward_speed = 0.0
                status.max_reverse_speed = 0.0
                self.status_pub.publish(status)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error("shutdown publish failed: %s" % exc)
            time.sleep(0.05)
        self.get_logger().info("safety_supervisor stopped, lock engaged")


def main(args=None):
    stop = {"requested": False}

    def on_signal(_signum, _frame):
        stop["requested"] = True

    # Own signal handling: the context must stay valid while the final lock is published
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    status = 0
    node = None
    executor = None
    try:
        node = SafetySupervisor()
    except StartupError:
        status = 1
    except Exception as exc:  # noqa: BLE001 - any constructor failure is fatal
        rclpy.logging.get_logger("safety_supervisor").fatal(str(exc))
        status = 1
    if node is not None:
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        while not stop["requested"] and rclpy.ok():
            try:
                executor.spin_once(timeout_sec=0.05)
            except RCLError:
                if not rclpy.ok():
                    break
            except Exception as exc:  # noqa: BLE001 - the loop must survive callback errors
                node.get_logger().error("spin error: %s" % exc, throttle_duration_sec=2.0)
        try:
            node.finalize()
        except Exception as exc:  # noqa: BLE001
            print("finalize failed: %s" % exc, file=sys.stderr)
        try:
            executor.shutdown(timeout_sec=0.5)
        except Exception:  # noqa: BLE001
            pass
        try:
            node.destroy_node()
        except RCLError:
            pass
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except RCLError:
        pass
    return status


if __name__ == "__main__":
    sys.exit(main())
