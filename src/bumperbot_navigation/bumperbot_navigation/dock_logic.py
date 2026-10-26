"""
Pure docking logic (no ROS dependency).

It holds the geometry, the closed-loop dock controller, the detection filter, the phase timer and
the DockTracker state machine.

The C++ twin lives in include/bumperbot_navigation/dock_logic.hpp and src/dock_logic.cpp.
Both implementations expose the same API and are checked against test/data/dock_cases.yaml.

Conventions
  * Poses are planar (x, y, yaw) with yaw in rad.
  * The controller works on the robot pose p expressed in the dock frame D: the origin is the
    charger pose (the base_footprint pose when docked) and the x axis is the docked heading.
  * s = +1 for backwards docking (the robot backs into the charger), s = -1 for forwards docking.
    The pre-dock pose is P = (s * L, 0, 0) with L = staging_distance, which is the staging pose.
"""
import math
from dataclasses import dataclass
from typing import Tuple

# Commands (same values as the DockRobot.action constants)
COMMAND_DOCK = 0
COMMAND_UNDOCK = 1

# Outcomes (same values as the DockRobot.action constants)
OUTCOME_DOCKED = 0
OUTCOME_UNDOCKED = 1
OUTCOME_ALREADY_DOCKED = 2
OUTCOME_CANCELED = 3
OUTCOME_STAGING_FAILED = 4
OUTCOME_DETECTION_FAILED = 5
OUTCOME_DOCKING_FAILED = 6
OUTCOME_UNDOCKING_FAILED = 7
OUTCOME_ERROR = 8

# States (same strings as the DockStatus.msg constants)
STATE_UNDOCKED = "UNDOCKED"
STATE_NAVIGATING_TO_STAGING = "NAVIGATING_TO_STAGING"
STATE_WAITING_FOR_DETECTION = "WAITING_FOR_DETECTION"
STATE_ALIGNING = "ALIGNING"
STATE_DOCKING = "DOCKING"
STATE_DOCKED = "DOCKED"
STATE_CHARGING = "CHARGING"
STATE_UNDOCKING = "UNDOCKING"
STATE_FAILED = "FAILED"

# Detection modes
DETECTION_NONE = "none"
DETECTION_OPTIONAL = "optional"
DETECTION_REQUIRED = "required"

# Step kinds
STEP_NAVIGATE_STAGING = "NAVIGATE_STAGING"
STEP_WAIT_DETECTION = "WAIT_DETECTION"
STEP_ALIGN = "ALIGN"
STEP_INSERT = "INSERT"
STEP_SETTLE_VERIFY = "SETTLE_VERIFY"
STEP_UNDOCK = "UNDOCK"
STEP_FINISHED = "FINISHED"

# Phase results reported by the node
PHASE_OK = "OK"
PHASE_TIMEOUT = "TIMEOUT"
PHASE_STALL = "STALL"
PHASE_SAFETY_TIMEOUT = "SAFETY_TIMEOUT"
PHASE_MISALIGNED = "MISALIGNED"

# Aligner tuning (fixed, not parameters): hysteresis of the rotate/drive switch and minimum speeds
_ROTATE_ENTER = 0.5
_ROTATE_EXIT = 0.1
_POSITION_GAIN = 1.0
_MIN_LINEAR = 0.02
_MIN_ANGULAR = 0.05
_MAX_LINE_HEADING = 0.3


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


def normalize_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def compose(a: Pose2D, b: Pose2D) -> Pose2D:
    """Return a * b: the pose b (expressed in frame a) expressed in the parent frame of a."""
    c = math.cos(a.yaw)
    s = math.sin(a.yaw)
    return Pose2D(
        a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y, normalize_angle(a.yaw + b.yaw))


def inverse(a: Pose2D) -> Pose2D:
    c = math.cos(a.yaw)
    s = math.sin(a.yaw)
    return Pose2D(-c * a.x - s * a.y, s * a.x - c * a.y, normalize_angle(-a.yaw))


def relative(frame: Pose2D, pose: Pose2D) -> Pose2D:
    """Return the pose expressed in the given frame."""
    return compose(inverse(frame), pose)


def quaternion_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def _rotate(q: Tuple[float, float, float, float], v: Tuple[float, float, float]):
    """Rotate the vector v by the unit quaternion q = (x, y, z, w)."""
    qx, qy, qz, qw = q
    vx, vy, vz = v
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (vx + qw * tx + (qy * tz - qz * ty),
            vy + qw * ty + (qz * tx - qx * tz),
            vz + qw * tz + (qx * ty - qy * tx))


def _quaternion_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def project_pose(frame_tf, pose) -> Pose2D:
    """
    Express a 3D pose in the parent frame of frame_tf and project it to the plane.

    frame_tf and pose are sequences (x, y, z, qx, qy, qz, qw); frame_tf is the transform from
    the pose's frame to the target frame. Returns the planar (x, y, yaw) result.
    """
    q_frame = tuple(frame_tf[3:7])
    q_pose = tuple(pose[3:7])
    rotated = _rotate(q_frame, tuple(pose[0:3]))
    q = _quaternion_multiply(q_frame, q_pose)
    return Pose2D(frame_tf[0] + rotated[0], frame_tf[1] + rotated[1], quaternion_yaw(*q))


def compute_dock_pose(staging: Pose2D, staging_distance: float, dock_backwards: bool) -> Pose2D:
    """Return the charger pose: staging_distance behind (backwards) or ahead of staging."""
    sign = -1.0 if dock_backwards else 1.0
    return Pose2D(staging.x + sign * staging_distance * math.cos(staging.yaw),
                  staging.y + sign * staging_distance * math.sin(staging.yaw),
                  staging.yaw)


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def _sign(value: float) -> float:
    if value > 0.0:
        return 1.0
    if value < 0.0:
        return -1.0
    return 0.0


@dataclass
class ControlParams:
    staging_distance: float = 0.5
    dock_backwards: bool = True
    max_linear_speed: float = 0.15
    max_angular_speed: float = 0.6
    approach_speed: float = 0.05
    slow_down_distance: float = 0.15
    k_lateral: float = 2.0
    k_heading: float = 2.0
    align_position_tolerance: float = 0.02
    align_yaw_tolerance: float = 0.03
    max_insert_lateral_error: float = 0.08
    max_insert_yaw_error: float = 0.3


@dataclass
class Command:
    v: float = 0.0
    w: float = 0.0
    done: bool = False
    failed: bool = False


class Aligner:
    """Rotate towards the pre-dock pose, drive to it, rotate to the docked heading."""

    _ROTATE_TO_TARGET = 0
    _DRIVE = 1
    _ROTATE_FINAL = 2

    def __init__(self, params: ControlParams):
        self._params = params
        self._stage = self._ROTATE_TO_TARGET

    @property
    def side(self) -> float:
        return 1.0 if self._params.dock_backwards else -1.0

    def reset(self) -> None:
        self._stage = self._ROTATE_TO_TARGET

    def align_command(self, p: Pose2D) -> Command:
        """Command that brings the robot pose p (dock frame) to the pre-dock pose."""
        par = self._params
        dx = self.side * par.staging_distance - p.x
        dy = -p.y
        dist = math.hypot(dx, dy)
        yaw_error = normalize_angle(-p.yaw)
        if dist <= par.align_position_tolerance and abs(yaw_error) <= par.align_yaw_tolerance:
            self._stage = self._ROTATE_FINAL
            return Command(0.0, 0.0, True, False)

        if self._stage == self._ROTATE_FINAL:
            # Hysteresis: only leave the final rotation when the position error doubled
            if dist > 2.0 * par.align_position_tolerance:
                self._stage = self._ROTATE_TO_TARGET
        elif dist <= par.align_position_tolerance:
            self._stage = self._ROTATE_FINAL

        max_w = par.max_angular_speed
        if self._stage == self._ROTATE_FINAL:
            w = _clamp(par.k_heading * yaw_error, max_w)
            if abs(w) < min(_MIN_ANGULAR, max_w):
                w = math.copysign(min(_MIN_ANGULAR, max_w), yaw_error)
            return Command(0.0, w, False, False)

        heading_error = normalize_angle(math.atan2(dy, dx) - p.yaw)
        if self._stage == self._ROTATE_TO_TARGET and abs(heading_error) <= _ROTATE_EXIT:
            self._stage = self._DRIVE
        elif self._stage == self._DRIVE and abs(heading_error) > _ROTATE_ENTER:
            self._stage = self._ROTATE_TO_TARGET

        w = _clamp(par.k_heading * heading_error, max_w)
        if self._stage == self._ROTATE_TO_TARGET:
            if abs(w) < min(_MIN_ANGULAR, max_w):
                w = math.copysign(min(_MIN_ANGULAR, max_w), heading_error)
            return Command(0.0, w, False, False)
        v = _clamp(_POSITION_GAIN * dist, par.max_linear_speed)
        v = max(v, min(_MIN_LINEAR, par.max_linear_speed))
        return Command(v, w, False, False)


class DockController(Aligner):
    """Aligner plus the line-tracking insertion / undock commands and the pose error."""

    def _line_command(self, p: Pose2D, v: float) -> float:
        par = self._params
        theta_des = -_sign(v) * _clamp(par.k_lateral * p.y, _MAX_LINE_HEADING)
        return _clamp(par.k_heading * normalize_angle(theta_des - p.yaw), par.max_angular_speed)

    def insert_command(self, p: Pose2D) -> Command:
        par = self._params
        s = self.side
        speed = par.max_linear_speed if abs(p.x) > par.slow_down_distance else par.approach_speed
        v = -s * speed
        done = s * p.x <= 0.0
        if done:
            v = 0.0
        v = _clamp(v, par.max_linear_speed)
        w = self._line_command(p, v)
        failed = (abs(p.y) > par.max_insert_lateral_error
                  or abs(p.yaw) > par.max_insert_yaw_error)
        return Command(v, w, done, failed)

    def undock_command(self, p: Pose2D) -> Command:
        par = self._params
        s = self.side
        remaining = abs(s * par.staging_distance - p.x)
        speed = par.max_linear_speed if remaining > par.slow_down_distance else par.approach_speed
        v = s * speed
        done = s * p.x >= par.staging_distance - 0.01
        if done:
            v = 0.0
        v = _clamp(v, par.max_linear_speed)
        w = self._line_command(p, v)
        failed = (abs(p.y) > par.max_insert_lateral_error
                  or abs(p.yaw) > par.max_insert_yaw_error)
        return Command(v, w, done, failed)

    @staticmethod
    def pose_error(p: Pose2D) -> Tuple[float, float]:
        return math.hypot(p.x, p.y), abs(normalize_angle(p.yaw))


def _finite_pose(pose: Pose2D) -> bool:
    return math.isfinite(pose.x) and math.isfinite(pose.y) and math.isfinite(pose.yaw)


class DetectionFilter:
    """Gate and smooth detections of the charger pose (all poses in the odom frame)."""

    def __init__(self, max_jump: float, alpha: float):
        self._max_jump = max_jump
        self._alpha = alpha

    def update(self, estimate: Pose2D, detection: Pose2D) -> Tuple[bool, Pose2D]:
        """Return (accepted, new_estimate). A rejected detection leaves the estimate unchanged."""
        if not _finite_pose(detection) or not _finite_pose(estimate):
            return False, Pose2D(estimate.x, estimate.y, estimate.yaw)
        if math.hypot(detection.x - estimate.x, detection.y - estimate.y) > self._max_jump:
            return False, Pose2D(estimate.x, estimate.y, estimate.yaw)
        a = self._alpha
        return True, Pose2D(
            estimate.x + a * (detection.x - estimate.x),
            estimate.y + a * (detection.y - estimate.y),
            normalize_angle(estimate.yaw + a * normalize_angle(detection.yaw - estimate.yaw)))


class PhaseTimer:
    """
    Phase timeout on node time with a wall-clock fallback for a stalled time source.

    Time arguments are seconds. When the node time does not advance for clock_stall_timeout wall
    seconds, stalled() becomes true and the phase counts as expired. 0 disables the fallback.
    """

    def __init__(self, timeout: float, clock_stall_timeout: float):
        self._timeout = timeout
        self._clock_stall_timeout = clock_stall_timeout
        self._start = 0.0
        self._last_node = 0.0
        self._last_advance_wall = 0.0
        self._stalled = False

    def start(self, node_now: float, wall_now: float) -> None:
        self._start = node_now
        self._last_node = node_now
        self._last_advance_wall = wall_now
        self._stalled = False

    def expired(self, node_now: float, wall_now: float) -> bool:
        if node_now != self._last_node:
            self._last_node = node_now
            self._last_advance_wall = wall_now
        elif (self._clock_stall_timeout > 0.0
              and wall_now - self._last_advance_wall >= self._clock_stall_timeout):
            self._stalled = True
        return self._stalled or node_now - self._start >= self._timeout

    def stalled(self) -> bool:
        return self._stalled


@dataclass
class TrackerParams:
    max_retries: int = 2
    nav_max_retries: int = 2
    detection_mode: str = DETECTION_OPTIONAL
    dock_position_tolerance: float = 0.03
    dock_yaw_tolerance: float = 0.05


@dataclass
class Step:
    kind: str = STEP_FINISHED
    attempt: int = 0


class DockTracker:
    """
    Dock / undock state machine. No clock, no threads: the node drives it.

    next_step() is idempotent: it returns the current step until a report_* call advances it.
    """

    _MOTION_STEPS = (STEP_NAVIGATE_STAGING, STEP_ALIGN, STEP_INSERT, STEP_UNDOCK)

    def __init__(self, params: TrackerParams = None):
        self._params = params if params is not None else TrackerParams()
        self._reset_fields()
        self._state = STATE_UNDOCKED

    def _reset_fields(self) -> None:
        self._step = STEP_FINISHED
        self._attempt = 1
        self._outcome = OUTCOME_DOCKED
        self._docked = False
        self._detection_used = False
        self._position_error = -1.0
        self._yaw_error = -1.0
        self._message = ""
        self._moved = False

    # ----- read-only state -----
    @property
    def state(self) -> str:
        return self._state

    @property
    def attempt(self) -> int:
        return self._attempt

    @property
    def outcome(self) -> int:
        return self._outcome

    @property
    def docked(self) -> bool:
        return self._docked

    @property
    def message(self) -> str:
        return self._message

    @property
    def detection_used(self) -> bool:
        return self._detection_used

    @property
    def position_error(self) -> float:
        return self._position_error

    @property
    def yaw_error(self) -> float:
        return self._yaw_error

    @property
    def moved(self) -> bool:
        return self._moved

    @property
    def finished(self) -> bool:
        return self._step == STEP_FINISHED

    # ----- driving -----
    def start(self, command: int, docked: bool, near_staging: bool) -> None:
        """Begin a goal. Every per-goal field is reset."""
        self._reset_fields()
        self._docked = bool(docked)
        if command == COMMAND_DOCK:
            if docked:
                self._finish(OUTCOME_ALREADY_DOCKED, "already docked", docked=True)
            elif near_staging:
                self._after_staging()
            else:
                self._enter(STEP_NAVIGATE_STAGING)
        elif command == COMMAND_UNDOCK:
            if not docked:
                self._finish(OUTCOME_UNDOCKING_FAILED, "the robot is not docked")
            else:
                self._enter(STEP_UNDOCK)
        else:
            self._finish(OUTCOME_ERROR, "unknown command {}".format(command))

    def next_step(self) -> Step:
        if self._step in self._MOTION_STEPS:
            self._moved = True
            self._docked = False
        return Step(self._step, self._attempt)

    def report_nav(self, ok: bool) -> None:
        if self._step != STEP_NAVIGATE_STAGING:
            return
        if ok:
            self._after_staging()
        elif self._attempt >= self._params.nav_max_retries + 1:
            self._finish(OUTCOME_STAGING_FAILED, "the staging pose could not be reached")
        else:
            self._attempt += 1

    def report_detection(self, ok: bool) -> None:
        if self._step != STEP_WAIT_DETECTION:
            return
        if ok:
            self._detection_used = True
            self._enter(STEP_ALIGN)
        elif self._params.detection_mode == DETECTION_REQUIRED:
            self._finish(OUTCOME_DETECTION_FAILED, "no valid dock detection")
        else:
            self._message = "no dock detection, using the map pose only"
            self._enter(STEP_ALIGN)

    def report_phase(self, result: str) -> None:
        if self._step == STEP_ALIGN:
            if result == PHASE_OK:
                self._enter(STEP_INSERT)
            else:
                self._fail_attempt(result)
        elif self._step == STEP_INSERT:
            if result == PHASE_OK:
                self._enter(STEP_SETTLE_VERIFY)
            else:
                self._fail_attempt(result)
        elif self._step == STEP_UNDOCK:
            if result == PHASE_OK:
                self._finish(OUTCOME_UNDOCKED, "undocked")
            else:
                self._finish(OUTCOME_UNDOCKING_FAILED, "undocking failed: " + result)

    def report_verify(self, position_error: float, yaw_error: float) -> None:
        if self._step != STEP_SETTLE_VERIFY:
            return
        self._position_error = position_error
        self._yaw_error = yaw_error
        ok = (math.isfinite(position_error) and math.isfinite(yaw_error)
              and position_error <= self._params.dock_position_tolerance
              and yaw_error <= self._params.dock_yaw_tolerance)
        if ok:
            self._finish(OUTCOME_DOCKED, "docked", docked=True)
        else:
            self._fail_attempt("VERIFY")

    def request_cancel(self) -> None:
        if self.finished:
            return
        self._finish(OUTCOME_CANCELED, "canceled by the client")

    def abort(self, reason: str) -> None:
        if self.finished:
            return
        self._finish(OUTCOME_ERROR, reason)

    # ----- internals -----
    _STEP_STATES = {
        STEP_NAVIGATE_STAGING: STATE_NAVIGATING_TO_STAGING,
        STEP_WAIT_DETECTION: STATE_WAITING_FOR_DETECTION,
        STEP_ALIGN: STATE_ALIGNING,
        STEP_INSERT: STATE_DOCKING,
        STEP_SETTLE_VERIFY: STATE_DOCKING,
        STEP_UNDOCK: STATE_UNDOCKING,
    }

    def _enter(self, step: str) -> None:
        self._step = step
        self._state = self._STEP_STATES[step]

    def _after_staging(self) -> None:
        self._attempt = 1
        if self._params.detection_mode == DETECTION_NONE:
            self._enter(STEP_ALIGN)
        else:
            self._enter(STEP_WAIT_DETECTION)

    def _fail_attempt(self, reason: str) -> None:
        if self._attempt >= self._params.max_retries + 1:
            self._finish(OUTCOME_DOCKING_FAILED, "docking failed after {} attempts ({})".format(
                self._attempt, reason))
            return
        self._attempt += 1
        self._message = "attempt {} failed ({}), retrying".format(self._attempt - 1, reason)
        self._enter(STEP_ALIGN)

    def _finish(self, outcome: int, message: str, docked: bool = False) -> None:
        self._step = STEP_FINISHED
        self._outcome = outcome
        self._message = message
        self._docked = docked
        if outcome in (OUTCOME_DOCKED, OUTCOME_ALREADY_DOCKED):
            self._state = STATE_DOCKED
        elif outcome in (OUTCOME_UNDOCKED, OUTCOME_CANCELED):
            self._state = STATE_UNDOCKED
        else:
            self._state = STATE_FAILED
