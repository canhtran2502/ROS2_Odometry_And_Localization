"""
Pure safety logic of the AGV safety supervisor (no rclpy, no clock, no threads).

The C++ twin is src/safety_logic.cpp with include/bumperbot_utils/safety_logic.hpp (same API,
same formulas, same strings). The node (safety_supervisor.py / .cpp) only does the ROS I/O and
drives SafetyCore with a monotonic time stamp.

Conventions:
  * Polygons are lists of (x, y) pairs in the robot frame (the footprint parameter is a flat list).
  * x_rear of footprint_extents() is the signed minimum x (a negative number for a normal robot).
  * Clearances are distances in metres from the footprint bounding box edge, inf means none.
"""
import math
from dataclasses import dataclass, field

STATE_INITIALIZING = "INITIALIZING"
STATE_ACTIVE = "ACTIVE"
STATE_DOCK_APPROACH = "DOCK_APPROACH"
STATE_FAULT = "FAULT"
STATE_ESTOP = "ESTOP"
STATE_SHUTDOWN = "SHUTDOWN"

ZONE_CLEAR = "CLEAR"
ZONE_WARNING = "WARNING"
ZONE_STOP = "STOP"
ZONE_CONTACT = "CONTACT"
ZONE_UNKNOWN = "UNKNOWN"

ENFORCEMENT_FILTER = "filter"
ENFORCEMENT_LOCK_ONLY = "lock_only"

DOCK_STATE_DOCKING = "DOCKING"

_BOUNDARY_EPS = 1e-12
_SELF_HIT_EPS = 1e-9
_AREA_EPS = 1e-12
_NAN = float("nan")
_INF = float("inf")

_ZONE_RANK = {ZONE_CLEAR: 0, ZONE_WARNING: 1, ZONE_STOP: 2, ZONE_CONTACT: 3}


def _default_footprint():
    return [0.10, 0.125, 0.10, -0.125, -0.10, -0.125, -0.10, 0.125]


@dataclass
class Config:
    """All parameters of the supervisor (defaults equal config/safety_supervisor.yaml)."""

    enforcement: str = ENFORCEMENT_FILTER
    robot_frame: str = "base_footprint"
    scan_topic: str = "scan"
    odom_topic: str = "bumperbot_controller/odom"
    cmd_vel_in_topic: str = "safety/cmd_vel_in"
    cmd_vel_out_topic: str = "bumperbot_controller/cmd_vel_unstamped"
    lock_topic: str = "safety_stop"
    status_topic: str = "safety/status"
    estop_topic: str = "safety/estop"
    dock_status_topic: str = "dock/status"
    use_odom: bool = True
    footprint: list = field(default_factory=_default_footprint)
    lateral_margin: float = 0.03
    stop_margin: float = 0.05
    stop_release_hysteresis: float = 0.02
    slowdown_distance: float = 0.35
    contact_margin: float = 0.015
    max_linear_speed: float = 0.7
    max_angular_speed: float = 5.0
    response_time: float = 0.3
    max_decel: float = 0.5
    stopped_speed: float = 0.02
    brake_hold_time: float = 0.5
    scan_timeout: float = 0.6
    odom_timeout: float = 0.5
    cmd_timeout: float = 0.5
    recovery_time: float = 0.5
    max_invalid_beam_ratio: float = 0.5
    latch_faults: bool = False
    latch_on_contact: bool = True
    rate: float = 20.0
    status_rate: float = 5.0
    dock_approach_enabled: bool = True
    dock_max_speed: float = 0.06
    dock_stop_margin: float = 0.01
    dock_approach_radius: float = 0.8
    dock_approach_max_time: float = 30.0
    dock_status_timeout: float = 2.0
    debug_inject_tick_exceptions: int = 0


# ---------------------------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------------------------

def footprint_polygon(flat):
    """Convert the flat x,y footprint list into a list of (x, y) pairs."""
    return [(float(flat[i]), float(flat[i + 1])) for i in range(0, len(flat) - 1, 2)]


def _orient(ax, ay, bx, by, cx, cy):
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)


def _on_segment(ax, ay, bx, by, px, py):
    return min(ax, bx) <= px <= max(ax, bx) and min(ay, by) <= py <= max(ay, by)


def _segments_intersect(p1, p2, p3, p4):
    o1 = _orient(p1[0], p1[1], p2[0], p2[1], p3[0], p3[1])
    o2 = _orient(p1[0], p1[1], p2[0], p2[1], p4[0], p4[1])
    o3 = _orient(p3[0], p3[1], p4[0], p4[1], p1[0], p1[1])
    o4 = _orient(p3[0], p3[1], p4[0], p4[1], p2[0], p2[1])
    if ((o1 > 0 > o2) or (o1 < 0 < o2)) and ((o3 > 0 > o4) or (o3 < 0 < o4)):
        return True
    if o1 == 0 and _on_segment(p1[0], p1[1], p2[0], p2[1], p3[0], p3[1]):
        return True
    if o2 == 0 and _on_segment(p1[0], p1[1], p2[0], p2[1], p4[0], p4[1]):
        return True
    if o3 == 0 and _on_segment(p3[0], p3[1], p4[0], p4[1], p1[0], p1[1]):
        return True
    if o4 == 0 and _on_segment(p3[0], p3[1], p4[0], p4[1], p2[0], p2[1]):
        return True
    return False


def polygon_is_simple(poly):
    """Check that the polygon has >= 3 vertices, no zero-length edge and no edge crossings."""
    n = len(poly)
    if n < 3:
        return False
    for i in range(n):
        if poly[i] == poly[(i + 1) % n]:
            return False
    for i in range(n):
        for j in range(i + 1, n):
            a1, a2 = poly[i], poly[(i + 1) % n]
            b1, b2 = poly[j], poly[(j + 1) % n]
            if j == i + 1 or (i == 0 and j == n - 1):
                # Adjacent edges: they may only share their common vertex
                if j == i + 1:
                    shared, other_a, other_b = a2, a1, b2
                else:
                    shared, other_a, other_b = a1, a2, b1
                cross = _orient(other_a[0], other_a[1], shared[0], shared[1],
                                other_b[0], other_b[1])
                dot = ((other_a[0] - shared[0]) * (other_b[0] - shared[0]) +
                       (other_a[1] - shared[1]) * (other_b[1] - shared[1]))
                if cross == 0 and dot > 0:
                    return False
            elif _segments_intersect(a1, a2, b1, b2):
                return False
    return True


def _segment_distance(px, py, a, b):
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    length_sq = dx * dx + dy * dy
    if length_sq <= 0.0:
        return math.hypot(px - a[0], py - a[1])
    t = ((px - a[0]) * dx + (py - a[1]) * dy) / length_sq
    t = max(0.0, min(1.0, t))
    return math.hypot(px - (a[0] + t * dx), py - (a[1] + t * dy))


def _boundary_distance(px, py, poly):
    best = _INF
    n = len(poly)
    for i in range(n):
        best = min(best, _segment_distance(px, py, poly[i], poly[(i + 1) % n]))
    return best


def _ray_inside(px, py, poly):
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > py) != (y2 > py):
            x_cross = x1 + (py - y1) * (x2 - x1) / (y2 - y1)
            if px < x_cross:
                inside = not inside
    return inside


def point_in_polygon(px, py, poly):
    """Check that the point is inside the polygon or on its boundary."""
    if _boundary_distance(px, py, poly) <= _BOUNDARY_EPS:
        return True
    return _ray_inside(px, py, poly)


def distance_to_polygon(px, py, poly):
    """Distance from a point to the polygon (0 inside or on the boundary)."""
    if point_in_polygon(px, py, poly):
        return 0.0
    return _boundary_distance(px, py, poly)


def footprint_extents(poly):
    """Return (x_front, x_rear, half_width) from the bounding box (x_rear is the signed min x)."""
    xs = [p[0] for p in poly]
    ys = [p[1] for p in poly]
    return max(xs), min(xs), max(abs(y) for y in ys)


def _polygon_area(poly):
    total = 0.0
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        total += x1 * y2 - x2 * y1
    return 0.5 * total


def scan_to_points(ranges, angle_min, angle_increment, range_min, range_max,
                   tf_x, tf_y, tf_yaw):
    """
    Convert a LaserScan into points of the robot frame.

    NaN beams count as invalid, inf and out-of-range beams are skipped.
    Returns (points, invalid_ratio) with invalid_ratio = NaN beams / total beams.
    """
    points = []
    invalid = 0
    total = len(ranges)
    for i in range(total):
        r = ranges[i]
        if math.isnan(r):
            invalid += 1
            continue
        if math.isinf(r) or r < range_min or r > range_max:
            continue
        angle = angle_min + i * angle_increment + tf_yaw
        points.append((tf_x + r * math.cos(angle), tf_y + r * math.sin(angle)))
    ratio = float(invalid) / float(total) if total > 0 else 0.0
    return points, ratio


@dataclass
class Evaluation:
    front_clearance: float = _INF
    rear_clearance: float = _INF
    nearest_distance: float = _INF
    nearest_bearing: float = 0.0
    contact_front: bool = False
    contact_rear: bool = False


def evaluate_points(points, cfg):
    """Corridor clearances, nearest obstacle and contact flags for points in the robot frame."""
    poly = footprint_polygon(cfg.footprint)
    x_front, x_rear, half_width = footprint_extents(poly)
    band = half_width + cfg.lateral_margin
    ev = Evaluation()
    for x, y in points:
        if not (math.isfinite(x) and math.isfinite(y)):
            continue
        # Points strictly inside the footprint are self hits
        if _boundary_distance(x, y, poly) > _SELF_HIT_EPS and _ray_inside(x, y, poly):
            continue
        d = distance_to_polygon(x, y, poly)
        if d < ev.nearest_distance:
            ev.nearest_distance = d
            ev.nearest_bearing = math.atan2(y, x)
        if abs(y) <= band:
            if x >= x_front:
                ev.front_clearance = min(ev.front_clearance, x - x_front)
            if x <= x_rear:
                ev.rear_clearance = min(ev.rear_clearance, x_rear - x)
    ev.contact_front = ev.front_clearance <= cfg.contact_margin
    ev.contact_rear = ev.rear_clearance <= cfg.contact_margin
    return ev


# ---------------------------------------------------------------------------------------------
# Limits, zones, direction
# ---------------------------------------------------------------------------------------------

def speed_limit(d, margin, cfg):
    """Return the allowed speed magnitude (m/s) for a clearance d (non-decreasing in d)."""
    if math.isnan(d) or d <= margin:
        return 0.0
    if math.isinf(d):
        return cfg.max_linear_speed
    s = d - margin
    ramp = cfg.max_linear_speed * s / cfg.slowdown_distance
    t = cfg.response_time
    brake = cfg.max_decel * (-t + math.sqrt(t * t + 2.0 * s / cfg.max_decel))
    return min(cfg.max_linear_speed, ramp, brake)


def zone_for(d, prev_zone, margin, cfg, allow_contact=True):
    """CONTACT / STOP / WARNING / CLEAR for a clearance, with hysteresis when leaving STOP."""
    if math.isnan(d):
        return ZONE_STOP
    if allow_contact and d <= cfg.contact_margin:
        return ZONE_CONTACT
    if d <= margin:
        return ZONE_STOP
    if prev_zone in (ZONE_STOP, ZONE_CONTACT) and d <= margin + cfg.stop_release_hysteresis:
        return ZONE_STOP
    if d < margin + cfg.slowdown_distance:
        return ZONE_WARNING
    return ZONE_CLEAR


def travel_direction(v_cmd, v_odom, stopped_speed):
    """+1 forward, -1 reverse, 0 none: odometry first, then the command."""
    if math.isfinite(v_odom) and abs(v_odom) > stopped_speed:
        return 1 if v_odom > 0.0 else -1
    if math.isfinite(v_cmd) and abs(v_cmd) > stopped_speed:
        return 1 if v_cmd > 0.0 else -1
    return 0


# ---------------------------------------------------------------------------------------------
# Config validation
# ---------------------------------------------------------------------------------------------

def _validate_footprint(flat):
    if len(flat) % 2 != 0:
        return "footprint must have an even number of values"
    if len(flat) < 6:
        return "footprint needs at least 3 points"
    for value in flat:
        if not math.isfinite(value):
            return "footprint values must be finite"
    poly = footprint_polygon(flat)
    if not polygon_is_simple(poly):
        return "footprint must be a simple polygon"
    if abs(_polygon_area(poly)) <= _AREA_EPS:
        return "footprint area must be > 0"
    if not point_in_polygon(0.0, 0.0, poly):
        return "footprint must contain the origin"
    return ""


def validate_config(cfg):
    """Return "" when the configuration is valid, otherwise a description of the first error."""
    if cfg.enforcement not in (ENFORCEMENT_FILTER, ENFORCEMENT_LOCK_ONLY):
        return "enforcement must be 'filter' or 'lock_only'"
    text_params = (
        ("robot_frame", cfg.robot_frame), ("scan_topic", cfg.scan_topic),
        ("odom_topic", cfg.odom_topic), ("cmd_vel_in_topic", cfg.cmd_vel_in_topic),
        ("cmd_vel_out_topic", cfg.cmd_vel_out_topic), ("lock_topic", cfg.lock_topic),
        ("status_topic", cfg.status_topic), ("estop_topic", cfg.estop_topic),
        ("dock_status_topic", cfg.dock_status_topic))
    for name, value in text_params:
        if value == "":
            return name + " must not be empty"
    if cfg.cmd_vel_in_topic == cfg.cmd_vel_out_topic:
        return "cmd_vel_in_topic and cmd_vel_out_topic must differ"
    error = _validate_footprint(cfg.footprint)
    if error:
        return error
    numbers = (
        ("lateral_margin", cfg.lateral_margin), ("stop_margin", cfg.stop_margin),
        ("stop_release_hysteresis", cfg.stop_release_hysteresis),
        ("slowdown_distance", cfg.slowdown_distance), ("contact_margin", cfg.contact_margin),
        ("max_linear_speed", cfg.max_linear_speed), ("max_angular_speed", cfg.max_angular_speed),
        ("response_time", cfg.response_time), ("max_decel", cfg.max_decel),
        ("stopped_speed", cfg.stopped_speed), ("brake_hold_time", cfg.brake_hold_time),
        ("scan_timeout", cfg.scan_timeout), ("odom_timeout", cfg.odom_timeout),
        ("cmd_timeout", cfg.cmd_timeout), ("recovery_time", cfg.recovery_time),
        ("max_invalid_beam_ratio", cfg.max_invalid_beam_ratio), ("rate", cfg.rate),
        ("status_rate", cfg.status_rate), ("dock_max_speed", cfg.dock_max_speed),
        ("dock_stop_margin", cfg.dock_stop_margin),
        ("dock_approach_radius", cfg.dock_approach_radius),
        ("dock_approach_max_time", cfg.dock_approach_max_time),
        ("dock_status_timeout", cfg.dock_status_timeout))
    for name, value in numbers:
        if not math.isfinite(value):
            return name + " must be finite"
    if cfg.lateral_margin < 0.0:
        return "lateral_margin must be >= 0"
    if cfg.contact_margin < 0.0:
        return "contact_margin must be >= 0"
    if cfg.stop_margin <= cfg.contact_margin:
        return "stop_margin must be > contact_margin"
    if cfg.stop_release_hysteresis < 0.0:
        return "stop_release_hysteresis must be >= 0"
    if cfg.slowdown_distance <= 0.0:
        return "slowdown_distance must be > 0"
    if cfg.max_linear_speed <= 0.0:
        return "max_linear_speed must be > 0"
    if cfg.max_angular_speed <= 0.0:
        return "max_angular_speed must be > 0"
    if cfg.response_time < 0.0:
        return "response_time must be >= 0"
    if cfg.max_decel <= 0.0:
        return "max_decel must be > 0"
    if cfg.stopped_speed < 0.0:
        return "stopped_speed must be >= 0"
    if cfg.brake_hold_time < 0.0:
        return "brake_hold_time must be >= 0"
    if cfg.scan_timeout <= 0.0:
        return "scan_timeout must be > 0"
    if cfg.odom_timeout <= 0.0:
        return "odom_timeout must be > 0"
    if cfg.cmd_timeout <= 0.0:
        return "cmd_timeout must be > 0"
    if cfg.recovery_time < 0.0:
        return "recovery_time must be >= 0"
    if cfg.max_invalid_beam_ratio < 0.0 or cfg.max_invalid_beam_ratio > 1.0:
        return "max_invalid_beam_ratio must be within [0, 1]"
    if cfg.rate <= 0.0:
        return "rate must be > 0"
    if cfg.status_rate <= 0.0:
        return "status_rate must be > 0"
    if cfg.dock_max_speed <= 0.0 or cfg.dock_max_speed > cfg.max_linear_speed:
        return "dock_max_speed must be within (0, max_linear_speed]"
    if cfg.dock_stop_margin < 0.0 or cfg.dock_stop_margin > cfg.stop_margin:
        return "dock_stop_margin must be within [0, stop_margin]"
    if cfg.dock_approach_radius <= 0.0:
        return "dock_approach_radius must be > 0"
    if cfg.dock_approach_max_time <= 0.0:
        return "dock_approach_max_time must be > 0"
    if cfg.dock_status_timeout <= 0.0:
        return "dock_status_timeout must be > 0"
    if cfg.debug_inject_tick_exceptions < 0:
        return "debug_inject_tick_exceptions must be >= 0"
    return ""


# ---------------------------------------------------------------------------------------------
# Dock approach gate
# ---------------------------------------------------------------------------------------------

class DockGate:
    """
    Decides whether the bounded rear dock-approach profile may be active.

    An episode starts on the transition into DOCKING, expires after dock_approach_max_time and
    re-arms only after the (last known) dock state leaves DOCKING.
    """

    def __init__(self, cfg):
        self.cfg = cfg
        self.reset()

    def reset(self):
        self._in_episode = False
        self._episode_start = 0.0
        self._expired = False

    def update(self, now, enabled, status_state, status_age, distance):
        """
        Update the episode and return (active, reason).

        status_state: last known dock state ("" or None: no status yet).
        status_age: seconds since it was received (negative or NaN: never).
        distance: robot distance to the dock pose in m (None or NaN: TF unavailable).
        """
        state = status_state if status_state else ""
        if state == DOCK_STATE_DOCKING:
            if not self._in_episode:
                self._in_episode = True
                self._episode_start = now
                self._expired = False
            elif now - self._episode_start >= self.cfg.dock_approach_max_time:
                self._expired = True
        else:
            self._in_episode = False
            self._expired = False
        if not enabled:
            return False, "dock approach disabled"
        if state == "":
            return False, "no dock status"
        age_ok = (status_age is not None and math.isfinite(status_age) and
                  0.0 <= status_age < self.cfg.dock_status_timeout)
        if not age_ok:
            return False, "dock status stale"
        if state != DOCK_STATE_DOCKING:
            return False, "dock state " + state
        if self._expired:
            return False, "dock approach time limit"
        if distance is None or not math.isfinite(distance):
            return False, "dock TF unavailable"
        if distance > self.cfg.dock_approach_radius:
            return False, "dock out of radius"
        return True, "dock approach"


# ---------------------------------------------------------------------------------------------
# State machine
# ---------------------------------------------------------------------------------------------

@dataclass
class Inputs:
    """One snapshot of everything the node knows at a tick (defaults are fail-safe)."""

    scan_ok: bool = False             # a valid scan was evaluated (Evaluation is meaningful)
    scan_age: float = -1.0            # s since the last scan arrived, negative: never
    scan_invalid_reason: str = ""     # non-empty: the last scan was invalid
    tf_ok: bool = False               # robot_frame <- scan frame is known
    tf_description: str = ""          # e.g. "base_footprint <- laser_link"
    evaluation: Evaluation = field(default_factory=Evaluation)
    odom_age: float = -1.0            # s, negative: never
    v_odom: float = _NAN              # m/s, NaN: unknown
    cmd_age: float = -1.0             # s, negative: never
    v_cmd: float = _NAN               # m/s command (or observed command), NaN: unknown
    estop_input: bool = False
    dock_active: bool = False         # DockGate result
    dock_reason: str = ""
    tick_exception: str = ""          # non-empty: the previous tick body raised


@dataclass
class Outputs:
    state: str = STATE_INITIALIZING
    locked: bool = True
    braking: bool = False
    estop_latched: bool = False
    front_zone: str = ZONE_UNKNOWN
    rear_zone: str = ZONE_UNKNOWN
    active_zone: str = ZONE_UNKNOWN
    max_forward: float = 0.0
    max_reverse: float = 0.0
    reason: str = ""
    dock_profile_active: bool = False


def _finite_or_nan(value):
    return value if value is not None and math.isfinite(value) else _NAN


class SafetyCore:
    """Safety state machine: zones, brake pulse, e-stop latch, watchdog and dock profile."""

    def __init__(self, cfg):
        self.cfg = cfg
        self._shutdown = False
        self._estop_latched = False
        self._latch_reason = ""
        self._brake_active = False
        self._brake_start = 0.0
        self._fault_active = False
        self._fault_reason = ""
        self._clear_since = None
        self._prev_front = ZONE_UNKNOWN
        self._prev_rear = ZONE_UNKNOWN
        self._last_problem = "waiting for scan"
        self._last_estop_input = False
        self._last_contact_front = False
        self._last_contact_rear = False
        self._last_dock_active = False

    def shutdown(self):
        self._shutdown = True
        self._brake_active = False

    def trigger_estop(self, reason):
        if not self._estop_latched:
            self._estop_latched = True
            self._latch_reason = reason
        self._brake_active = False

    def request_reset(self):
        """Return (ok, message) following the reset rules."""
        if not self._estop_latched:
            return True, "not latched"
        if self._last_estop_input:
            return False, "estop input active"
        if self._last_problem:
            return False, "fault: " + self._last_problem
        if self._last_contact_front:
            return False, "obstacle in contact zone (front)"
        if self._last_contact_rear and not self._last_dock_active:
            return False, "obstacle in contact zone (rear)"
        self._estop_latched = False
        self._latch_reason = ""
        self._brake_active = False
        self._fault_active = False
        self._fault_reason = ""
        self._clear_since = None
        return True, "reset"

    def limit_command(self, v, w, outputs):
        """Apply the allowed speeds to a command, returns (v, w)."""
        if outputs.locked:
            return 0.0, 0.0
        if v is None or w is None or not (math.isfinite(v) and math.isfinite(w)):
            return 0.0, 0.0
        v = max(-outputs.max_reverse, min(outputs.max_forward, v))
        w = max(-self.cfg.max_angular_speed, min(self.cfg.max_angular_speed, w))
        return v, w

    def _latch(self, reason):
        if not self._estop_latched:
            self._estop_latched = True
            self._latch_reason = reason
        self._brake_active = False

    def update(self, now, inp):
        cfg = self.cfg
        if self._shutdown:
            self._brake_active = False
            return Outputs(state=STATE_SHUTDOWN, locked=True, estop_latched=self._estop_latched,
                           reason="shutdown")

        # Input problems: not ready (INITIALIZING) and faults
        init_reason = ""
        if inp.scan_age < 0.0:
            init_reason = "waiting for scan"
        elif not inp.tf_ok:
            init_reason = "waiting for TF"
            if inp.tf_description:
                init_reason += " " + inp.tf_description
        elif cfg.use_odom and inp.odom_age < 0.0:
            init_reason = "waiting for odom"
        fault_reason = ""
        if inp.tick_exception:
            fault_reason = "internal error: " + inp.tick_exception
        elif not init_reason:
            if inp.scan_invalid_reason:
                fault_reason = inp.scan_invalid_reason
            elif inp.scan_age > cfg.scan_timeout:
                fault_reason = "scan timeout"
            elif cfg.use_odom and inp.odom_age > cfg.odom_timeout:
                fault_reason = "odom timeout"
            elif not inp.scan_ok:
                fault_reason = "no valid scan"
        self._last_problem = fault_reason if fault_reason else init_reason
        self._last_estop_input = inp.estop_input

        if fault_reason:
            if not self._fault_active:
                self._fault_active = True
                self._brake_active = False
                if cfg.latch_faults:
                    self._latch("fault: " + fault_reason)
            self._fault_reason = fault_reason
            self._clear_since = None
        elif self._fault_active:
            if init_reason:
                self._clear_since = None
            else:
                if self._clear_since is None:
                    self._clear_since = now
                if now - self._clear_since >= cfg.recovery_time:
                    self._fault_active = False
                    self._fault_reason = ""
                    self._clear_since = None

        # Dock profile (filter mode only)
        dock_active = inp.dock_active
        dock_reason = inp.dock_reason
        if cfg.enforcement != ENFORCEMENT_FILTER:
            dock_active = False
            dock_reason = "dock profile unavailable in lock_only"

        # Zones and limits per direction
        usable = (inp.scan_ok and inp.tf_ok and not inp.scan_invalid_reason and
                  0.0 <= inp.scan_age <= cfg.scan_timeout)
        ev = inp.evaluation
        prev_front = self._prev_front
        prev_rear = self._prev_rear
        if dock_active != self._last_dock_active:
            # The rear margin changes with the profile: do not carry the old margin's STOP
            # hysteresis over, evaluate the rear zone afresh against the new margin
            prev_rear = ZONE_UNKNOWN
        if usable:
            front_zone = zone_for(ev.front_clearance, prev_front, cfg.stop_margin, cfg)
            if front_zone in (ZONE_STOP, ZONE_CONTACT):
                fwd_limit = 0.0
            else:
                fwd_limit = speed_limit(ev.front_clearance, cfg.stop_margin, cfg)
            if dock_active:
                rear_zone = zone_for(ev.rear_clearance, prev_rear, cfg.dock_stop_margin, cfg,
                                     False)
                rev_limit = 0.0 if rear_zone == ZONE_STOP else cfg.dock_max_speed
            else:
                rear_zone = zone_for(ev.rear_clearance, prev_rear, cfg.stop_margin, cfg)
                if rear_zone in (ZONE_STOP, ZONE_CONTACT):
                    rev_limit = 0.0
                else:
                    rev_limit = speed_limit(ev.rear_clearance, cfg.stop_margin, cfg)
        else:
            front_zone = ZONE_UNKNOWN
            rear_zone = ZONE_UNKNOWN
            fwd_limit = 0.0
            rev_limit = 0.0
        self._prev_front = front_zone
        self._prev_rear = rear_zone
        self._last_contact_front = usable and ev.contact_front
        self._last_contact_rear = usable and ev.contact_rear
        self._last_dock_active = dock_active

        # Motion: odometry decides when it exists, otherwise the command
        v_odom = _finite_or_nan(inp.v_odom) if cfg.use_odom else _NAN
        cmd_fresh = 0.0 <= inp.cmd_age <= cfg.cmd_timeout
        v_cmd = _finite_or_nan(inp.v_cmd) if cmd_fresh else _NAN
        direction = travel_direction(v_cmd, v_odom, cfg.stopped_speed)
        odom_present = math.isfinite(v_odom)
        moving = abs(v_odom) > cfg.stopped_speed if odom_present else direction != 0
        zone_dir = ZONE_UNKNOWN
        prev_dir = ZONE_UNKNOWN
        if direction > 0:
            zone_dir, prev_dir = front_zone, prev_front
        elif direction < 0:
            zone_dir, prev_dir = rear_zone, prev_rear

        if inp.estop_input:
            self._latch("estop input")

        healthy = usable and not init_reason and not self._fault_active
        if self._brake_active:
            stopped = (not odom_present) or abs(v_odom) < cfg.stopped_speed
            if now - self._brake_start >= cfg.brake_hold_time and stopped:
                self._brake_active = False
        if healthy and moving and zone_dir in (ZONE_STOP, ZONE_CONTACT):
            if zone_dir == ZONE_CONTACT and cfg.latch_on_contact:
                self._latch("contact zone (" + ("front" if direction > 0 else "rear") + ")")
            elif not self._brake_active and (
                    odom_present or prev_dir not in (ZONE_STOP, ZONE_CONTACT)):
                self._brake_active = True
                self._brake_start = now

        # State, priority: ESTOP > INITIALIZING > FAULT > DOCK_APPROACH / ACTIVE
        if self._estop_latched:
            state = STATE_ESTOP
        elif init_reason and not inp.tick_exception:
            state = STATE_INITIALIZING
        elif self._fault_active:
            state = STATE_FAULT
        elif dock_active:
            state = STATE_DOCK_APPROACH
        else:
            state = STATE_ACTIVE
        running = state in (STATE_ACTIVE, STATE_DOCK_APPROACH)
        if not running:
            self._brake_active = False

        out = Outputs()
        out.state = state
        out.braking = self._brake_active
        out.locked = (not running) or self._brake_active
        out.estop_latched = self._estop_latched
        out.front_zone = front_zone
        out.rear_zone = rear_zone
        out.dock_profile_active = running and dock_active
        out.max_forward = fwd_limit if running else 0.0
        out.max_reverse = rev_limit if running else 0.0
        if direction > 0:
            out.active_zone = front_zone
        elif direction < 0:
            out.active_zone = rear_zone
        elif front_zone == ZONE_UNKNOWN or rear_zone == ZONE_UNKNOWN:
            out.active_zone = ZONE_UNKNOWN
        else:
            out.active_zone = (front_zone if _ZONE_RANK[front_zone] >= _ZONE_RANK[rear_zone]
                               else rear_zone)

        if state == STATE_ESTOP:
            out.reason = self._latch_reason
        elif state == STATE_INITIALIZING:
            out.reason = init_reason
        elif state == STATE_FAULT:
            out.reason = self._fault_reason
        else:
            parts = []
            if front_zone not in (ZONE_CLEAR, ZONE_UNKNOWN):
                parts.append("front zone " + front_zone)
            if rear_zone not in (ZONE_CLEAR, ZONE_UNKNOWN):
                if dock_active:
                    parts.append("rear zone " + rear_zone + " (dock approach)")
                elif dock_reason:
                    parts.append(dock_reason)
                else:
                    parts.append("rear zone " + rear_zone)
            if dock_active and not parts:
                parts.append("dock approach")
            if self._brake_active:
                parts.append("brake pulse")
            out.reason = "; ".join(parts)
        return out
