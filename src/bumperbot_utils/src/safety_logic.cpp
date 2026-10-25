#include "bumperbot_utils/safety_logic.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace bumperbot_utils
{

namespace safety
{

namespace
{
constexpr double kBoundaryEps = 1e-12;
constexpr double kSelfHitEps = 1e-9;
constexpr double kAreaEps = 1e-12;
const double kNaN = std::numeric_limits<double>::quiet_NaN();
const double kInf = std::numeric_limits<double>::infinity();

double orient(double ax, double ay, double bx, double by, double cx, double cy)
{
  return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
}

bool on_segment(double ax, double ay, double bx, double by, double px, double py)
{
  return std::min(ax, bx) <= px && px <= std::max(ax, bx) &&
         std::min(ay, by) <= py && py <= std::max(ay, by);
}

bool segments_intersect(const Point & p1, const Point & p2, const Point & p3, const Point & p4)
{
  const double o1 = orient(p1.first, p1.second, p2.first, p2.second, p3.first, p3.second);
  const double o2 = orient(p1.first, p1.second, p2.first, p2.second, p4.first, p4.second);
  const double o3 = orient(p3.first, p3.second, p4.first, p4.second, p1.first, p1.second);
  const double o4 = orient(p3.first, p3.second, p4.first, p4.second, p2.first, p2.second);
  if (((o1 > 0 && o2 < 0) || (o1 < 0 && o2 > 0)) && ((o3 > 0 && o4 < 0) || (o3 < 0 && o4 > 0))) {
    return true;
  }
  if (o1 == 0 && on_segment(p1.first, p1.second, p2.first, p2.second, p3.first, p3.second)) {
    return true;
  }
  if (o2 == 0 && on_segment(p1.first, p1.second, p2.first, p2.second, p4.first, p4.second)) {
    return true;
  }
  if (o3 == 0 && on_segment(p3.first, p3.second, p4.first, p4.second, p1.first, p1.second)) {
    return true;
  }
  if (o4 == 0 && on_segment(p3.first, p3.second, p4.first, p4.second, p2.first, p2.second)) {
    return true;
  }
  return false;
}

double segment_distance(double px, double py, const Point & a, const Point & b)
{
  const double dx = b.first - a.first;
  const double dy = b.second - a.second;
  const double length_sq = dx * dx + dy * dy;
  if (length_sq <= 0.0) {
    return std::hypot(px - a.first, py - a.second);
  }
  double t = ((px - a.first) * dx + (py - a.second) * dy) / length_sq;
  t = std::max(0.0, std::min(1.0, t));
  return std::hypot(px - (a.first + t * dx), py - (a.second + t * dy));
}

double boundary_distance(double px, double py, const Polygon & poly)
{
  double best = kInf;
  const size_t n = poly.size();
  for (size_t i = 0; i < n; ++i) {
    best = std::min(best, segment_distance(px, py, poly[i], poly[(i + 1) % n]));
  }
  return best;
}

bool ray_inside(double px, double py, const Polygon & poly)
{
  bool inside = false;
  const size_t n = poly.size();
  for (size_t i = 0; i < n; ++i) {
    const double x1 = poly[i].first;
    const double y1 = poly[i].second;
    const double x2 = poly[(i + 1) % n].first;
    const double y2 = poly[(i + 1) % n].second;
    if ((y1 > py) != (y2 > py)) {
      const double x_cross = x1 + (py - y1) * (x2 - x1) / (y2 - y1);
      if (px < x_cross) {
        inside = !inside;
      }
    }
  }
  return inside;
}

double polygon_area(const Polygon & poly)
{
  double total = 0.0;
  const size_t n = poly.size();
  for (size_t i = 0; i < n; ++i) {
    const Point & a = poly[i];
    const Point & b = poly[(i + 1) % n];
    total += a.first * b.second - b.first * a.second;
  }
  return 0.5 * total;
}

std::string validate_footprint(const std::vector<double> & flat)
{
  if (flat.size() % 2 != 0) {
    return "footprint must have an even number of values";
  }
  if (flat.size() < 6) {
    return "footprint needs at least 3 points";
  }
  for (double value : flat) {
    if (!std::isfinite(value)) {
      return "footprint values must be finite";
    }
  }
  const Polygon poly = footprint_polygon(flat);
  if (!polygon_is_simple(poly)) {
    return "footprint must be a simple polygon";
  }
  if (std::fabs(polygon_area(poly)) <= kAreaEps) {
    return "footprint area must be > 0";
  }
  if (!point_in_polygon(0.0, 0.0, poly)) {
    return "footprint must contain the origin";
  }
  return "";
}

double finite_or_nan(double value)
{
  return std::isfinite(value) ? value : kNaN;
}

int zone_rank(const std::string & zone)
{
  if (zone == ZONE_WARNING) {
    return 1;
  }
  if (zone == ZONE_STOP) {
    return 2;
  }
  if (zone == ZONE_CONTACT) {
    return 3;
  }
  return 0;
}

bool is_blocking(const std::string & zone)
{
  return zone == ZONE_STOP || zone == ZONE_CONTACT;
}
}  // namespace

// ---------------------------------------------------------------------------------------------
// Geometry
// ---------------------------------------------------------------------------------------------

Polygon footprint_polygon(const std::vector<double> & flat)
{
  Polygon poly;
  for (size_t i = 0; i + 1 < flat.size(); i += 2) {
    poly.emplace_back(flat[i], flat[i + 1]);
  }
  return poly;
}

bool polygon_is_simple(const Polygon & poly)
{
  const size_t n = poly.size();
  if (n < 3) {
    return false;
  }
  for (size_t i = 0; i < n; ++i) {
    if (poly[i] == poly[(i + 1) % n]) {
      return false;
    }
  }
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = i + 1; j < n; ++j) {
      const Point & a1 = poly[i];
      const Point & a2 = poly[(i + 1) % n];
      const Point & b1 = poly[j];
      const Point & b2 = poly[(j + 1) % n];
      if (j == i + 1 || (i == 0 && j == n - 1)) {
        // Adjacent edges: they may only share their common vertex
        const Point * shared;
        const Point * other_a;
        const Point * other_b;
        if (j == i + 1) {
          shared = &a2;
          other_a = &a1;
          other_b = &b2;
        } else {
          shared = &a1;
          other_a = &a2;
          other_b = &b1;
        }
        const double cross = orient(
          other_a->first, other_a->second, shared->first, shared->second,
          other_b->first, other_b->second);
        const double dot =
          (other_a->first - shared->first) * (other_b->first - shared->first) +
          (other_a->second - shared->second) * (other_b->second - shared->second);
        if (cross == 0 && dot > 0) {
          return false;
        }
      } else if (segments_intersect(a1, a2, b1, b2)) {
        return false;
      }
    }
  }
  return true;
}

bool point_in_polygon(double px, double py, const Polygon & poly)
{
  if (boundary_distance(px, py, poly) <= kBoundaryEps) {
    return true;
  }
  return ray_inside(px, py, poly);
}

double distance_to_polygon(double px, double py, const Polygon & poly)
{
  if (point_in_polygon(px, py, poly)) {
    return 0.0;
  }
  return boundary_distance(px, py, poly);
}

Extents footprint_extents(const Polygon & poly)
{
  Extents e;
  e.x_front = -kInf;
  e.x_rear = kInf;
  e.half_width = 0.0;
  for (const Point & p : poly) {
    e.x_front = std::max(e.x_front, p.first);
    e.x_rear = std::min(e.x_rear, p.first);
    e.half_width = std::max(e.half_width, std::fabs(p.second));
  }
  return e;
}

ScanPoints scan_to_points(
  const std::vector<double> & ranges, double angle_min, double angle_increment,
  double range_min, double range_max, double tf_x, double tf_y, double tf_yaw)
{
  ScanPoints result;
  size_t invalid = 0;
  const size_t total = ranges.size();
  for (size_t i = 0; i < total; ++i) {
    const double r = ranges[i];
    if (std::isnan(r)) {
      ++invalid;
      continue;
    }
    if (std::isinf(r) || r > range_max) {
      continue;
    }
    // A finite range below range_min (including 0) is an obstacle inside the blind zone: keep
    // it at range_min along the beam (the farthest it can be), never drop it
    const double range = r < range_min ? range_min : r;
    const double angle = angle_min + static_cast<double>(i) * angle_increment + tf_yaw;
    result.points.emplace_back(tf_x + range * std::cos(angle), tf_y + range * std::sin(angle));
  }
  result.invalid_ratio =
    total > 0 ? static_cast<double>(invalid) / static_cast<double>(total) : 0.0;
  return result;
}

Evaluation evaluate_points(const std::vector<Point> & points, const Config & cfg)
{
  const Polygon poly = footprint_polygon(cfg.footprint);
  const Extents ext = footprint_extents(poly);
  const double band = ext.half_width + cfg.lateral_margin;
  Evaluation ev;
  double path_front = kInf;
  double path_rear = kInf;
  for (const Point & p : points) {
    const double x = p.first;
    const double y = p.second;
    if (!(std::isfinite(x) && std::isfinite(y))) {
      continue;
    }
    // Points strictly inside the footprint are self hits
    if (boundary_distance(x, y, poly) > kSelfHitEps && ray_inside(x, y, poly)) {
      continue;
    }
    const double d = distance_to_polygon(x, y, poly);
    if (d < ev.nearest_distance) {
      ev.nearest_distance = d;
      ev.nearest_bearing = std::atan2(y, x);
    }
    if (std::fabs(y) <= band) {
      if (x >= ext.x_front) {
        ev.front_clearance = std::min(ev.front_clearance, x - ext.x_front);
      }
      if (x <= ext.x_rear) {
        ev.rear_clearance = std::min(ev.rear_clearance, ext.x_rear - x);
      }
      // Contact flags use only points inside the swept path (|y| <= half width, no margin)
      if (std::fabs(y) <= ext.half_width) {
        if (x >= ext.x_front) {
          path_front = std::min(path_front, x - ext.x_front);
        }
        if (x <= ext.x_rear) {
          path_rear = std::min(path_rear, ext.x_rear - x);
        }
      }
    }
  }
  ev.contact_front = path_front <= cfg.contact_margin;
  ev.contact_rear = path_rear <= cfg.contact_margin;
  return ev;
}

std::string blind_zone_warning(
  const Config & cfg, double laser_x, double laser_y, double range_min)
{
  const Polygon poly = footprint_polygon(cfg.footprint);
  const Extents ext = footprint_extents(poly);
  const double band = ext.half_width + cfg.lateral_margin;
  const double rear_margin = cfg.dock_approach_enabled ? cfg.dock_stop_margin : cfg.stop_margin;
  struct Area
  {
    const char * name;
    double x_lo;
    double x_hi;
  };
  const Area areas[2] = {
    {"front", ext.x_front, ext.x_front + cfg.stop_margin},
    {"rear", ext.x_rear - rear_margin, ext.x_rear}};
  std::string parts;
  for (const Area & area : areas) {
    const double dx = std::max({area.x_lo - laser_x, 0.0, laser_x - area.x_hi});
    const double dy = std::max(std::fabs(laser_y) - band, 0.0);
    if (std::hypot(dx, dy) < range_min) {
      parts += (parts.empty() ? "" : " and ") + std::string(area.name);
    }
  }
  if (parts.empty()) {
    return "";
  }
  char buf[320];
  std::snprintf(
    buf, sizeof(buf),
    "laser range_min %.3f m blind zone (sensor at x=%.3f, y=%.3f) overlaps the %s "
    "stop corridor: a driver that reports no return below range_min leaves a blind "
    "spot there",
    range_min, laser_x, laser_y, parts.c_str());
  return buf;
}

// ---------------------------------------------------------------------------------------------
// Limits, zones, direction
// ---------------------------------------------------------------------------------------------

double speed_limit(double d, double margin, const Config & cfg)
{
  if (std::isnan(d) || d <= margin) {
    return 0.0;
  }
  if (std::isinf(d)) {
    return cfg.max_linear_speed;
  }
  const double s = d - margin;
  const double ramp = cfg.max_linear_speed * s / cfg.slowdown_distance;
  const double t = cfg.response_time;
  const double brake = cfg.max_decel * (-t + std::sqrt(t * t + 2.0 * s / cfg.max_decel));
  return std::min(cfg.max_linear_speed, std::min(ramp, brake));
}

std::string zone_for(
  double d, const std::string & prev_zone, double margin, const Config & cfg, bool allow_contact)
{
  if (std::isnan(d)) {
    return ZONE_STOP;
  }
  if (allow_contact && d <= cfg.contact_margin) {
    return ZONE_CONTACT;
  }
  if (d <= margin) {
    return ZONE_STOP;
  }
  if (is_blocking(prev_zone) && d <= margin + cfg.stop_release_hysteresis) {
    return ZONE_STOP;
  }
  if (d < margin + cfg.slowdown_distance) {
    return ZONE_WARNING;
  }
  return ZONE_CLEAR;
}

int travel_direction(double v_cmd, double v_odom, double stopped_speed)
{
  if (std::isfinite(v_odom) && std::fabs(v_odom) > stopped_speed) {
    return v_odom > 0.0 ? 1 : -1;
  }
  if (std::isfinite(v_cmd) && std::fabs(v_cmd) > stopped_speed) {
    return v_cmd > 0.0 ? 1 : -1;
  }
  return 0;
}

// ---------------------------------------------------------------------------------------------
// Config validation
// ---------------------------------------------------------------------------------------------

std::string validate_config(const Config & cfg)
{
  if (cfg.enforcement != ENFORCEMENT_FILTER && cfg.enforcement != ENFORCEMENT_LOCK_ONLY) {
    return "enforcement must be 'filter' or 'lock_only'";
  }
  const std::vector<std::pair<const char *, const std::string *>> text_params = {
    {"robot_frame", &cfg.robot_frame}, {"scan_topic", &cfg.scan_topic},
    {"odom_topic", &cfg.odom_topic}, {"cmd_vel_in_topic", &cfg.cmd_vel_in_topic},
    {"cmd_vel_out_topic", &cfg.cmd_vel_out_topic}, {"lock_topic", &cfg.lock_topic},
    {"status_topic", &cfg.status_topic}, {"estop_topic", &cfg.estop_topic},
    {"dock_status_topic", &cfg.dock_status_topic}};
  for (const auto & item : text_params) {
    if (item.second->empty()) {
      return std::string(item.first) + " must not be empty";
    }
  }
  if (cfg.cmd_vel_in_topic == cfg.cmd_vel_out_topic) {
    return "cmd_vel_in_topic and cmd_vel_out_topic must differ";
  }
  const std::string footprint_error = validate_footprint(cfg.footprint);
  if (!footprint_error.empty()) {
    return footprint_error;
  }
  const std::vector<std::pair<const char *, double>> numbers = {
    {"lateral_margin", cfg.lateral_margin}, {"stop_margin", cfg.stop_margin},
    {"stop_release_hysteresis", cfg.stop_release_hysteresis},
    {"slowdown_distance", cfg.slowdown_distance}, {"contact_margin", cfg.contact_margin},
    {"max_linear_speed", cfg.max_linear_speed}, {"max_angular_speed", cfg.max_angular_speed},
    {"response_time", cfg.response_time}, {"max_decel", cfg.max_decel},
    {"stopped_speed", cfg.stopped_speed}, {"brake_hold_time", cfg.brake_hold_time},
    {"scan_timeout", cfg.scan_timeout}, {"odom_timeout", cfg.odom_timeout},
    {"cmd_timeout", cfg.cmd_timeout}, {"recovery_time", cfg.recovery_time},
    {"max_invalid_beam_ratio", cfg.max_invalid_beam_ratio}, {"rate", cfg.rate},
    {"status_rate", cfg.status_rate}, {"dock_max_speed", cfg.dock_max_speed},
    {"dock_stop_margin", cfg.dock_stop_margin},
    {"dock_approach_radius", cfg.dock_approach_radius},
    {"dock_approach_max_time", cfg.dock_approach_max_time},
    {"dock_status_timeout", cfg.dock_status_timeout}};
  for (const auto & item : numbers) {
    if (!std::isfinite(item.second)) {
      return std::string(item.first) + " must be finite";
    }
  }
  if (cfg.lateral_margin < 0.0) {
    return "lateral_margin must be >= 0";
  }
  if (cfg.contact_margin < 0.0) {
    return "contact_margin must be >= 0";
  }
  if (cfg.stop_margin <= cfg.contact_margin) {
    return "stop_margin must be > contact_margin";
  }
  if (cfg.contact_debounce_scans < 1) {
    return "contact_debounce_scans must be >= 1";
  }
  if (cfg.stop_release_hysteresis < 0.0) {
    return "stop_release_hysteresis must be >= 0";
  }
  if (cfg.slowdown_distance <= 0.0) {
    return "slowdown_distance must be > 0";
  }
  if (cfg.max_linear_speed <= 0.0) {
    return "max_linear_speed must be > 0";
  }
  if (cfg.max_angular_speed <= 0.0) {
    return "max_angular_speed must be > 0";
  }
  if (cfg.response_time < 0.0) {
    return "response_time must be >= 0";
  }
  if (cfg.max_decel <= 0.0) {
    return "max_decel must be > 0";
  }
  if (cfg.stopped_speed < 0.0) {
    return "stopped_speed must be >= 0";
  }
  if (cfg.brake_hold_time < 0.0) {
    return "brake_hold_time must be >= 0";
  }
  if (cfg.scan_timeout <= 0.0) {
    return "scan_timeout must be > 0";
  }
  if (cfg.odom_timeout <= 0.0) {
    return "odom_timeout must be > 0";
  }
  if (cfg.cmd_timeout <= 0.0) {
    return "cmd_timeout must be > 0";
  }
  if (cfg.recovery_time < 0.0) {
    return "recovery_time must be >= 0";
  }
  if (cfg.max_invalid_beam_ratio < 0.0 || cfg.max_invalid_beam_ratio > 1.0) {
    return "max_invalid_beam_ratio must be within [0, 1]";
  }
  if (cfg.rate <= 0.0) {
    return "rate must be > 0";
  }
  if (cfg.status_rate <= 0.0) {
    return "status_rate must be > 0";
  }
  if (cfg.dock_max_speed <= 0.0 || cfg.dock_max_speed > cfg.max_linear_speed) {
    return "dock_max_speed must be within (0, max_linear_speed]";
  }
  if (cfg.dock_stop_margin < 0.0 || cfg.dock_stop_margin > cfg.stop_margin) {
    return "dock_stop_margin must be within [0, stop_margin]";
  }
  if (cfg.dock_approach_radius <= 0.0) {
    return "dock_approach_radius must be > 0";
  }
  if (cfg.dock_approach_max_time <= 0.0) {
    return "dock_approach_max_time must be > 0";
  }
  if (cfg.dock_status_timeout <= 0.0) {
    return "dock_status_timeout must be > 0";
  }
  if (cfg.debug_inject_tick_exceptions < 0) {
    return "debug_inject_tick_exceptions must be >= 0";
  }
  return "";
}

// ---------------------------------------------------------------------------------------------
// Dock approach gate
// ---------------------------------------------------------------------------------------------

DockGate::DockGate(const Config & cfg)
: cfg_(cfg)
{
  reset();
}

void DockGate::reset()
{
  in_episode_ = false;
  episode_start_ = 0.0;
  expired_ = false;
}

std::pair<bool, std::string> DockGate::update(
  double now, bool enabled, const std::string & status_state, double status_age, double distance)
{
  if (status_state == DOCK_STATE_DOCKING) {
    if (!in_episode_) {
      in_episode_ = true;
      episode_start_ = now;
      expired_ = false;
    } else if (now - episode_start_ >= cfg_.dock_approach_max_time) {
      expired_ = true;
    }
  } else {
    in_episode_ = false;
    expired_ = false;
  }
  if (!enabled) {
    return {false, "dock approach disabled"};
  }
  if (status_state.empty()) {
    return {false, "no dock status"};
  }
  const bool age_ok =
    std::isfinite(status_age) && status_age >= 0.0 && status_age < cfg_.dock_status_timeout;
  if (!age_ok) {
    return {false, "dock status stale"};
  }
  if (status_state != DOCK_STATE_DOCKING) {
    return {false, "dock state " + status_state};
  }
  if (expired_) {
    return {false, "dock approach time limit"};
  }
  if (!std::isfinite(distance)) {
    return {false, "dock TF unavailable"};
  }
  if (distance > cfg_.dock_approach_radius) {
    return {false, "dock out of radius"};
  }
  return {true, "dock approach"};
}

// ---------------------------------------------------------------------------------------------
// State machine
// ---------------------------------------------------------------------------------------------

SafetyCore::SafetyCore(const Config & cfg)
: cfg_(cfg)
{
  if (cfg_.require_reset_after_restart) {
    // An e-stop does not survive a restart: start latched until ~/reset is called
    estop_latched_ = true;
    latch_reason_ = "restart: reset required";
  }
}

void SafetyCore::shutdown()
{
  shutdown_ = true;
  brake_active_ = false;
}

void SafetyCore::latch(const std::string & reason)
{
  if (!estop_latched_) {
    estop_latched_ = true;
    latch_reason_ = reason;
  }
  brake_active_ = false;
}

void SafetyCore::trigger_estop(const std::string & reason)
{
  latch(reason);
}

std::pair<bool, std::string> SafetyCore::request_reset()
{
  if (!estop_latched_) {
    return {true, "not latched"};
  }
  if (last_estop_input_) {
    return {false, "estop input active"};
  }
  if (!last_problem_.empty()) {
    return {false, "fault: " + last_problem_};
  }
  const bool front = last_contact_front_;
  const bool rear = last_contact_rear_ && !last_dock_active_;
  if (front || rear) {
    // A contact (swept path, within contact_margin) allows a reset only when the speed
    // filter keeps blocking the motion toward it and the robot can reverse out: one side
    // only, nothing moving or commanded toward it. lock_only has no filter: refuse.
    const bool toward = front ? last_direction_ > 0 : last_direction_ < 0;
    if (cfg_.enforcement != ENFORCEMENT_FILTER || (front && rear) || toward) {
      return {false, std::string("obstacle in contact zone (") + (front ? "front" : "rear") + ")"};
    }
  }
  estop_latched_ = false;
  latch_reason_.clear();
  brake_active_ = false;
  fault_active_ = false;
  fault_reason_.clear();
  has_clear_since_ = false;
  return {true, "reset"};
}

std::pair<double, double> SafetyCore::limit_command(
  double v, double w, const Outputs & outputs) const
{
  if (outputs.locked) {
    return {0.0, 0.0};
  }
  if (!(std::isfinite(v) && std::isfinite(w))) {
    return {0.0, 0.0};
  }
  v = std::max(-outputs.max_reverse, std::min(outputs.max_forward, v));
  w = std::max(-cfg_.max_angular_speed, std::min(cfg_.max_angular_speed, w));
  return {v, w};
}

Outputs SafetyCore::update(double now, const Inputs & inp)
{
  if (shutdown_) {
    brake_active_ = false;
    Outputs out;
    out.state = STATE_SHUTDOWN;
    out.locked = true;
    out.estop_latched = estop_latched_;
    out.reason = "shutdown";
    return out;
  }

  // Input problems: not ready (INITIALIZING) and faults
  std::string init_reason;
  if (inp.scan_age < 0.0) {
    init_reason = "waiting for scan";
  } else if (!inp.tf_ok) {
    init_reason = "waiting for TF" +
      (inp.tf_description.empty() ? std::string() : " " + inp.tf_description);
  } else if (cfg_.use_odom && inp.odom_age < 0.0) {
    init_reason = "waiting for odom";
  }
  std::string fault_reason;
  if (!inp.tick_exception.empty()) {
    fault_reason = "internal error: " + inp.tick_exception;
  } else if (init_reason.empty()) {
    if (!inp.scan_invalid_reason.empty()) {
      fault_reason = inp.scan_invalid_reason;
    } else if (inp.scan_age > cfg_.scan_timeout) {
      fault_reason = "scan timeout";
    } else if (cfg_.use_odom && inp.odom_age > cfg_.odom_timeout) {
      fault_reason = "odom timeout";
    } else if (!inp.scan_ok) {
      fault_reason = "no valid scan";
    }
  }
  last_problem_ = fault_reason.empty() ? init_reason : fault_reason;
  last_estop_input_ = inp.estop_input;

  if (!fault_reason.empty()) {
    if (!fault_active_) {
      fault_active_ = true;
      brake_active_ = false;
      if (cfg_.latch_faults) {
        latch("fault: " + fault_reason);
      }
    }
    fault_reason_ = fault_reason;
    has_clear_since_ = false;
  } else if (fault_active_) {
    if (!init_reason.empty()) {
      has_clear_since_ = false;
    } else {
      if (!has_clear_since_) {
        has_clear_since_ = true;
        clear_since_ = now;
      }
      if (now - clear_since_ >= cfg_.recovery_time) {
        fault_active_ = false;
        fault_reason_.clear();
        has_clear_since_ = false;
      }
    }
  }

  // Dock profile (filter mode only)
  bool dock_active = inp.dock_active;
  std::string dock_reason = inp.dock_reason;
  if (cfg_.enforcement != ENFORCEMENT_FILTER) {
    dock_active = false;
    dock_reason = "dock profile unavailable in lock_only";
  }

  // Zones and limits per direction
  const bool usable = inp.scan_ok && inp.tf_ok && inp.scan_invalid_reason.empty() &&
    inp.scan_age >= 0.0 && inp.scan_age <= cfg_.scan_timeout;
  const Evaluation & ev = inp.evaluation;
  const std::string prev_front = prev_front_;
  // The rear margin changes with the dock profile: do not carry the old margin's STOP
  // hysteresis over, evaluate the rear zone afresh against the new margin
  const std::string prev_rear = (dock_active != last_dock_active_) ? ZONE_UNKNOWN : prev_rear_;
  std::string front_zone = ZONE_UNKNOWN;
  std::string rear_zone = ZONE_UNKNOWN;
  double fwd_limit = 0.0;
  double rev_limit = 0.0;
  // Contact debounce: N consecutive evaluated scans (an unconfirmed contact is a STOP)
  if (usable) {
    if (inp.scan_seq < 0 || !has_last_scan_seq_ || inp.scan_seq != last_scan_seq_) {
      contact_count_front_ =
        ev.contact_front ? std::min(contact_count_front_ + 1, cfg_.contact_debounce_scans) : 0;
      contact_count_rear_ =
        ev.contact_rear ? std::min(contact_count_rear_ + 1, cfg_.contact_debounce_scans) : 0;
    }
    has_last_scan_seq_ = true;
    last_scan_seq_ = inp.scan_seq;
  } else {
    contact_count_front_ = 0;
    contact_count_rear_ = 0;
    has_last_scan_seq_ = false;
  }
  const bool confirmed_front = contact_count_front_ >= cfg_.contact_debounce_scans;
  const bool confirmed_rear = contact_count_rear_ >= cfg_.contact_debounce_scans;
  if (usable) {
    front_zone = zone_for(ev.front_clearance, prev_front, cfg_.stop_margin, cfg_, confirmed_front);
    if (is_blocking(front_zone)) {
      fwd_limit = 0.0;
    } else {
      fwd_limit = speed_limit(ev.front_clearance, cfg_.stop_margin, cfg_);
    }
    if (dock_active) {
      rear_zone = zone_for(ev.rear_clearance, prev_rear, cfg_.dock_stop_margin, cfg_, false);
      if (rear_zone == ZONE_STOP) {
        rev_limit = 0.0;
      } else {
        rev_limit = std::min(
          cfg_.dock_max_speed, speed_limit(ev.rear_clearance, cfg_.dock_stop_margin, cfg_));
      }
    } else {
      rear_zone = zone_for(ev.rear_clearance, prev_rear, cfg_.stop_margin, cfg_, confirmed_rear);
      if (is_blocking(rear_zone)) {
        rev_limit = 0.0;
      } else {
        rev_limit = speed_limit(ev.rear_clearance, cfg_.stop_margin, cfg_);
      }
    }
  }
  prev_front_ = front_zone;
  prev_rear_ = rear_zone;
  last_contact_front_ = usable && ev.contact_front;
  last_contact_rear_ = usable && ev.contact_rear;
  last_dock_active_ = dock_active;

  // Motion: odometry decides when it exists, otherwise the command
  const double v_odom = cfg_.use_odom ? finite_or_nan(inp.v_odom) : kNaN;
  const bool cmd_fresh = inp.cmd_age >= 0.0 && inp.cmd_age <= cfg_.cmd_timeout;
  const double v_cmd = cmd_fresh ? finite_or_nan(inp.v_cmd) : kNaN;
  const int direction = travel_direction(v_cmd, v_odom, cfg_.stopped_speed);
  last_direction_ = direction;
  const bool odom_present = std::isfinite(v_odom);
  const bool moving = odom_present ? std::fabs(v_odom) > cfg_.stopped_speed : direction != 0;
  std::string zone_dir = ZONE_UNKNOWN;
  std::string prev_dir = ZONE_UNKNOWN;
  if (direction > 0) {
    zone_dir = front_zone;
    prev_dir = prev_front;
  } else if (direction < 0) {
    zone_dir = rear_zone;
    prev_dir = prev_rear;
  }

  if (inp.estop_input) {
    latch("estop input");
  }

  const bool healthy = usable && init_reason.empty() && !fault_active_;
  if (brake_active_) {
    const bool stopped = !odom_present || std::fabs(v_odom) < cfg_.stopped_speed;
    if (now - brake_start_ >= cfg_.brake_hold_time && stopped) {
      brake_active_ = false;
    }
  }
  if (healthy && moving && is_blocking(zone_dir)) {
    if (zone_dir == ZONE_CONTACT && cfg_.latch_on_contact) {
      latch(std::string("contact zone (") + (direction > 0 ? "front" : "rear") + ")");
    } else if (!brake_active_ && (odom_present || !is_blocking(prev_dir))) {
      brake_active_ = true;
      brake_start_ = now;
    }
  }

  // State, priority: ESTOP > INITIALIZING > FAULT > DOCK_APPROACH / ACTIVE
  std::string state;
  if (estop_latched_) {
    state = STATE_ESTOP;
  } else if (!init_reason.empty() && inp.tick_exception.empty()) {
    state = STATE_INITIALIZING;
  } else if (fault_active_) {
    state = STATE_FAULT;
  } else if (dock_active) {
    state = STATE_DOCK_APPROACH;
  } else {
    state = STATE_ACTIVE;
  }
  const bool running = state == STATE_ACTIVE || state == STATE_DOCK_APPROACH;
  if (!running) {
    brake_active_ = false;
  }

  Outputs out;
  out.state = state;
  out.braking = brake_active_;
  out.locked = !running || brake_active_;
  out.estop_latched = estop_latched_;
  out.front_zone = front_zone;
  out.rear_zone = rear_zone;
  out.dock_profile_active = running && dock_active;
  out.max_forward = running ? fwd_limit : 0.0;
  out.max_reverse = running ? rev_limit : 0.0;
  if (direction > 0) {
    out.active_zone = front_zone;
  } else if (direction < 0) {
    out.active_zone = rear_zone;
  } else if (front_zone == ZONE_UNKNOWN || rear_zone == ZONE_UNKNOWN) {
    out.active_zone = ZONE_UNKNOWN;
  } else {
    out.active_zone = zone_rank(front_zone) >= zone_rank(rear_zone) ? front_zone : rear_zone;
  }

  if (state == STATE_ESTOP) {
    out.reason = latch_reason_;
  } else if (state == STATE_INITIALIZING) {
    out.reason = init_reason;
  } else if (state == STATE_FAULT) {
    out.reason = fault_reason_;
  } else {
    std::vector<std::string> parts;
    if (front_zone != ZONE_CLEAR && front_zone != ZONE_UNKNOWN) {
      parts.push_back("front zone " + front_zone);
    }
    if (rear_zone != ZONE_CLEAR && rear_zone != ZONE_UNKNOWN) {
      if (dock_active) {
        parts.push_back("rear zone " + rear_zone + " (dock approach)");
      } else if (!dock_reason.empty()) {
        parts.push_back(dock_reason);
      } else {
        parts.push_back("rear zone " + rear_zone);
      }
    }
    if (dock_active && parts.empty()) {
      parts.push_back("dock approach");
    }
    if (brake_active_) {
      parts.push_back("brake pulse");
    }
    for (size_t i = 0; i < parts.size(); ++i) {
      out.reason += (i == 0 ? "" : "; ") + parts[i];
    }
  }
  return out;
}

}  // namespace safety

}  // namespace bumperbot_utils
