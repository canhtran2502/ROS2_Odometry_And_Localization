#include "bumperbot_navigation/dock_logic.hpp"

#include <algorithm>
#include <cmath>

namespace bumperbot_navigation
{

const char * const DOCK_STATE_UNDOCKED = "UNDOCKED";
const char * const DOCK_STATE_NAVIGATING_TO_STAGING = "NAVIGATING_TO_STAGING";
const char * const DOCK_STATE_WAITING_FOR_DETECTION = "WAITING_FOR_DETECTION";
const char * const DOCK_STATE_ALIGNING = "ALIGNING";
const char * const DOCK_STATE_DOCKING = "DOCKING";
const char * const DOCK_STATE_DOCKED = "DOCKED";
const char * const DOCK_STATE_CHARGING = "CHARGING";
const char * const DOCK_STATE_UNDOCKING = "UNDOCKING";
const char * const DOCK_STATE_FAILED = "FAILED";

const char * const DETECTION_NONE = "none";
const char * const DETECTION_OPTIONAL = "optional";
const char * const DETECTION_REQUIRED = "required";

namespace
{
// Aligner tuning (fixed, not parameters): hysteresis of the rotate/drive switch and minimum speeds
constexpr double ROTATE_ENTER = 0.5;
constexpr double ROTATE_EXIT = 0.1;
constexpr double POSITION_GAIN = 1.0;
constexpr double MIN_LINEAR = 0.02;
constexpr double MIN_ANGULAR = 0.05;
constexpr double MAX_LINE_HEADING = 0.3;

double clamp_symmetric(double value, double limit)
{
  return std::max(-limit, std::min(limit, value));
}

double sign_of(double value)
{
  if (value > 0.0) {
    return 1.0;
  }
  if (value < 0.0) {
    return -1.0;
  }
  return 0.0;
}

bool finite_pose(const Pose2D & pose)
{
  return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.yaw);
}

using Quat = std::array<double, 4>;  // x, y, z, w

std::array<double, 3> rotate(const Quat & q, const std::array<double, 3> & v)
{
  const double tx = 2.0 * (q[1] * v[2] - q[2] * v[1]);
  const double ty = 2.0 * (q[2] * v[0] - q[0] * v[2]);
  const double tz = 2.0 * (q[0] * v[1] - q[1] * v[0]);
  return {
    v[0] + q[3] * tx + (q[1] * tz - q[2] * ty),
    v[1] + q[3] * ty + (q[2] * tx - q[0] * tz),
    v[2] + q[3] * tz + (q[0] * ty - q[1] * tx)};
}

Quat quaternion_multiply(const Quat & a, const Quat & b)
{
  return {
    a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
    a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
    a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
    a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2]};
}
}  // namespace

double normalize_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

Pose2D compose(const Pose2D & a, const Pose2D & b)
{
  const double c = std::cos(a.yaw);
  const double s = std::sin(a.yaw);
  return Pose2D{a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y, normalize_angle(a.yaw + b.yaw)};
}

Pose2D inverse(const Pose2D & a)
{
  const double c = std::cos(a.yaw);
  const double s = std::sin(a.yaw);
  return Pose2D{-c * a.x - s * a.y, s * a.x - c * a.y, normalize_angle(-a.yaw)};
}

Pose2D relative(const Pose2D & frame, const Pose2D & pose)
{
  return compose(inverse(frame), pose);
}

double quaternion_yaw(double qx, double qy, double qz, double qw)
{
  return std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
}

Pose2D project_pose(const Pose3 & frame_tf, const Pose3 & pose)
{
  const Quat q_frame{frame_tf[3], frame_tf[4], frame_tf[5], frame_tf[6]};
  const Quat q_pose{pose[3], pose[4], pose[5], pose[6]};
  const auto rotated = rotate(q_frame, {pose[0], pose[1], pose[2]});
  const Quat q = quaternion_multiply(q_frame, q_pose);
  return Pose2D{
    frame_tf[0] + rotated[0], frame_tf[1] + rotated[1], quaternion_yaw(q[0], q[1], q[2], q[3])};
}

Pose2D compute_dock_pose(const Pose2D & staging, double staging_distance, bool dock_backwards)
{
  const double sign = dock_backwards ? -1.0 : 1.0;
  return Pose2D{
    staging.x + sign * staging_distance * std::cos(staging.yaw),
    staging.y + sign * staging_distance * std::sin(staging.yaw),
    staging.yaw};
}

// ----- Aligner -----
Aligner::Aligner(const ControlParams & params)
: params_(params) {}

void Aligner::reset()
{
  stage_ = Stage::ROTATE_TO_TARGET;
}

Command Aligner::align_command(const Pose2D & p)
{
  const ControlParams & par = params_;
  const double dx = side() * par.staging_distance - p.x;
  const double dy = -p.y;
  const double dist = std::hypot(dx, dy);
  const double yaw_error = normalize_angle(-p.yaw);
  if (dist <= par.align_position_tolerance && std::fabs(yaw_error) <= par.align_yaw_tolerance) {
    stage_ = Stage::ROTATE_FINAL;
    return Command{0.0, 0.0, true, false};
  }

  if (stage_ == Stage::ROTATE_FINAL) {
    // Hysteresis: only leave the final rotation when the position error doubled
    if (dist > 2.0 * par.align_position_tolerance) {
      stage_ = Stage::ROTATE_TO_TARGET;
    }
  } else if (dist <= par.align_position_tolerance) {
    stage_ = Stage::ROTATE_FINAL;
  }

  const double max_w = par.max_angular_speed;
  const double min_w = std::min(MIN_ANGULAR, max_w);
  if (stage_ == Stage::ROTATE_FINAL) {
    double w = clamp_symmetric(par.k_heading * yaw_error, max_w);
    if (std::fabs(w) < min_w) {
      w = std::copysign(min_w, yaw_error);
    }
    return Command{0.0, w, false, false};
  }

  const double heading_error = normalize_angle(std::atan2(dy, dx) - p.yaw);
  if (stage_ == Stage::ROTATE_TO_TARGET && std::fabs(heading_error) <= ROTATE_EXIT) {
    stage_ = Stage::DRIVE;
  } else if (stage_ == Stage::DRIVE && std::fabs(heading_error) > ROTATE_ENTER) {
    stage_ = Stage::ROTATE_TO_TARGET;
  }

  double w = clamp_symmetric(par.k_heading * heading_error, max_w);
  if (stage_ == Stage::ROTATE_TO_TARGET) {
    if (std::fabs(w) < min_w) {
      w = std::copysign(min_w, heading_error);
    }
    return Command{0.0, w, false, false};
  }
  double v = clamp_symmetric(POSITION_GAIN * dist, par.max_linear_speed);
  v = std::max(v, std::min(MIN_LINEAR, par.max_linear_speed));
  return Command{v, w, false, false};
}

// ----- DockController -----
double DockController::line_command(const Pose2D & p, double v) const
{
  const double theta_des =
    -sign_of(v) * clamp_symmetric(params_.k_lateral * p.y, MAX_LINE_HEADING);
  return clamp_symmetric(
    params_.k_heading * normalize_angle(theta_des - p.yaw), params_.max_angular_speed);
}

Command DockController::insert_command(const Pose2D & p) const
{
  const ControlParams & par = params_;
  const double s = side();
  const double speed =
    std::fabs(p.x) > par.slow_down_distance ? par.max_linear_speed : par.approach_speed;
  double v = -s * speed;
  const bool done = s * p.x <= 0.0;
  if (done) {
    v = 0.0;
  }
  v = clamp_symmetric(v, par.max_linear_speed);
  const double w = line_command(p, v);
  const bool failed =
    std::fabs(p.y) > par.max_insert_lateral_error || std::fabs(p.yaw) > par.max_insert_yaw_error;
  return Command{v, w, done, failed};
}

Command DockController::undock_command(const Pose2D & p) const
{
  const ControlParams & par = params_;
  const double s = side();
  const double remaining = std::fabs(s * par.staging_distance - p.x);
  const double speed =
    remaining > par.slow_down_distance ? par.max_linear_speed : par.approach_speed;
  double v = s * speed;
  const bool done = s * p.x >= par.staging_distance - 0.01;
  if (done) {
    v = 0.0;
  }
  v = clamp_symmetric(v, par.max_linear_speed);
  const double w = line_command(p, v);
  const bool failed =
    std::fabs(p.y) > par.max_insert_lateral_error || std::fabs(p.yaw) > par.max_insert_yaw_error;
  return Command{v, w, done, failed};
}

std::pair<double, double> DockController::pose_error(const Pose2D & p)
{
  return {std::hypot(p.x, p.y), std::fabs(normalize_angle(p.yaw))};
}

// ----- DetectionFilter -----
DetectionFilter::DetectionFilter(double max_jump, double alpha)
: max_jump_(max_jump), alpha_(alpha) {}

std::pair<bool, Pose2D> DetectionFilter::update(
  const Pose2D & estimate, const Pose2D & detection) const
{
  if (!finite_pose(detection) || !finite_pose(estimate)) {
    return {false, estimate};
  }
  if (std::hypot(detection.x - estimate.x, detection.y - estimate.y) > max_jump_) {
    return {false, estimate};
  }
  const double a = alpha_;
  return {true, Pose2D{
      estimate.x + a * (detection.x - estimate.x),
      estimate.y + a * (detection.y - estimate.y),
      normalize_angle(estimate.yaw + a * normalize_angle(detection.yaw - estimate.yaw))}};
}

// ----- PhaseTimer -----
PhaseTimer::PhaseTimer(double timeout, double clock_stall_timeout)
: timeout_(timeout), clock_stall_timeout_(clock_stall_timeout) {}

void PhaseTimer::start(double node_now, double wall_now)
{
  start_ = node_now;
  last_node_ = node_now;
  last_advance_wall_ = wall_now;
  stalled_ = false;
}

bool PhaseTimer::expired(double node_now, double wall_now)
{
  if (node_now != last_node_) {
    last_node_ = node_now;
    last_advance_wall_ = wall_now;
  } else if (clock_stall_timeout_ > 0.0 && wall_now - last_advance_wall_ >= clock_stall_timeout_) {
    stalled_ = true;
  }
  return stalled_ || node_now - start_ >= timeout_;
}

// ----- names -----
const char * to_string(DockStepKind kind)
{
  switch (kind) {
    case DockStepKind::NAVIGATE_STAGING: return "NAVIGATE_STAGING";
    case DockStepKind::WAIT_DETECTION: return "WAIT_DETECTION";
    case DockStepKind::ALIGN: return "ALIGN";
    case DockStepKind::INSERT: return "INSERT";
    case DockStepKind::SETTLE_VERIFY: return "SETTLE_VERIFY";
    case DockStepKind::UNDOCK: return "UNDOCK";
    case DockStepKind::FINISHED: return "FINISHED";
  }
  return "FINISHED";
}

const char * to_string(PhaseResult result)
{
  switch (result) {
    case PhaseResult::OK: return "OK";
    case PhaseResult::TIMEOUT: return "TIMEOUT";
    case PhaseResult::STALL: return "STALL";
    case PhaseResult::SAFETY_TIMEOUT: return "SAFETY_TIMEOUT";
    case PhaseResult::MISALIGNED: return "MISALIGNED";
  }
  return "TIMEOUT";
}

// ----- DockTracker -----
DockTracker::DockTracker(const DockTrackerParams & params)
: params_(params)
{
  reset_fields();
  state_ = DOCK_STATE_UNDOCKED;
}

void DockTracker::reset_fields()
{
  step_ = DockStepKind::FINISHED;
  attempt_ = 1;
  outcome_ = DOCK_OUTCOME_DOCKED;
  docked_ = false;
  detection_used_ = false;
  position_error_ = -1.0;
  yaw_error_ = -1.0;
  message_.clear();
  moved_ = false;
}

void DockTracker::start(int command, bool docked, bool near_staging)
{
  reset_fields();
  docked_ = docked;
  if (command == COMMAND_DOCK) {
    if (docked) {
      finish(DOCK_OUTCOME_ALREADY_DOCKED, "already docked", true);
    } else if (near_staging) {
      after_staging();
    } else {
      enter(DockStepKind::NAVIGATE_STAGING);
    }
  } else if (command == COMMAND_UNDOCK) {
    if (!docked) {
      finish(DOCK_OUTCOME_UNDOCKING_FAILED, "the robot is not docked");
    } else {
      enter(DockStepKind::UNDOCK);
    }
  } else {
    finish(DOCK_OUTCOME_ERROR, "unknown command " + std::to_string(command));
  }
}

DockStep DockTracker::next_step()
{
  if (step_ == DockStepKind::NAVIGATE_STAGING || step_ == DockStepKind::ALIGN ||
    step_ == DockStepKind::INSERT || step_ == DockStepKind::UNDOCK)
  {
    moved_ = true;
    docked_ = false;
  }
  return DockStep{step_, attempt_};
}

void DockTracker::report_nav(bool ok)
{
  if (step_ != DockStepKind::NAVIGATE_STAGING) {
    return;
  }
  if (ok) {
    after_staging();
  } else if (attempt_ >= params_.nav_max_retries + 1) {
    finish(DOCK_OUTCOME_STAGING_FAILED, "the staging pose could not be reached");
  } else {
    ++attempt_;
  }
}

void DockTracker::report_detection(bool ok)
{
  if (step_ != DockStepKind::WAIT_DETECTION) {
    return;
  }
  if (ok) {
    detection_used_ = true;
    enter(DockStepKind::ALIGN);
  } else if (params_.detection_mode == DETECTION_REQUIRED) {
    finish(DOCK_OUTCOME_DETECTION_FAILED, "no valid dock detection");
  } else {
    message_ = "no dock detection, using the map pose only";
    enter(DockStepKind::ALIGN);
  }
}

void DockTracker::report_phase(PhaseResult result)
{
  if (step_ == DockStepKind::ALIGN) {
    if (result == PhaseResult::OK) {
      enter(DockStepKind::INSERT);
    } else {
      fail_attempt(to_string(result));
    }
  } else if (step_ == DockStepKind::INSERT) {
    if (result == PhaseResult::OK) {
      enter(DockStepKind::SETTLE_VERIFY);
    } else {
      fail_attempt(to_string(result));
    }
  } else if (step_ == DockStepKind::UNDOCK) {
    if (result == PhaseResult::OK) {
      finish(DOCK_OUTCOME_UNDOCKED, "undocked");
    } else {
      finish(DOCK_OUTCOME_UNDOCKING_FAILED, std::string("undocking failed: ") + to_string(result));
    }
  }
}

void DockTracker::report_verify(double position_error, double yaw_error)
{
  if (step_ != DockStepKind::SETTLE_VERIFY) {
    return;
  }
  position_error_ = position_error;
  yaw_error_ = yaw_error;
  const bool ok = std::isfinite(position_error) && std::isfinite(yaw_error) &&
    position_error <= params_.dock_position_tolerance && yaw_error <= params_.dock_yaw_tolerance;
  if (ok) {
    finish(DOCK_OUTCOME_DOCKED, "docked", true);
  } else {
    fail_attempt("VERIFY");
  }
}

void DockTracker::request_cancel()
{
  if (finished()) {
    return;
  }
  finish(DOCK_OUTCOME_CANCELED, "canceled by the client");
}

void DockTracker::abort(const std::string & reason)
{
  if (finished()) {
    return;
  }
  finish(DOCK_OUTCOME_ERROR, reason);
}

void DockTracker::enter(DockStepKind step)
{
  step_ = step;
  switch (step) {
    case DockStepKind::NAVIGATE_STAGING: state_ = DOCK_STATE_NAVIGATING_TO_STAGING; break;
    case DockStepKind::WAIT_DETECTION: state_ = DOCK_STATE_WAITING_FOR_DETECTION; break;
    case DockStepKind::ALIGN: state_ = DOCK_STATE_ALIGNING; break;
    case DockStepKind::INSERT: state_ = DOCK_STATE_DOCKING; break;
    case DockStepKind::SETTLE_VERIFY: state_ = DOCK_STATE_DOCKING; break;
    case DockStepKind::UNDOCK: state_ = DOCK_STATE_UNDOCKING; break;
    case DockStepKind::FINISHED: break;
  }
}

void DockTracker::after_staging()
{
  attempt_ = 1;
  if (params_.detection_mode == DETECTION_NONE) {
    enter(DockStepKind::ALIGN);
  } else {
    enter(DockStepKind::WAIT_DETECTION);
  }
}

void DockTracker::fail_attempt(const std::string & reason)
{
  if (attempt_ >= params_.max_retries + 1) {
    finish(
      DOCK_OUTCOME_DOCKING_FAILED,
      "docking failed after " + std::to_string(attempt_) + " attempts (" + reason + ")");
    return;
  }
  ++attempt_;
  message_ = "attempt " + std::to_string(attempt_ - 1) + " failed (" + reason + "), retrying";
  enter(DockStepKind::ALIGN);
}

void DockTracker::finish(int outcome, const std::string & message, bool docked)
{
  step_ = DockStepKind::FINISHED;
  outcome_ = outcome;
  message_ = message;
  docked_ = docked;
  if (outcome == DOCK_OUTCOME_DOCKED || outcome == DOCK_OUTCOME_ALREADY_DOCKED) {
    state_ = DOCK_STATE_DOCKED;
  } else if (outcome == DOCK_OUTCOME_UNDOCKED || outcome == DOCK_OUTCOME_CANCELED) {
    state_ = DOCK_STATE_UNDOCKED;
  } else {
    state_ = DOCK_STATE_FAILED;
  }
}

}  // namespace bumperbot_navigation
