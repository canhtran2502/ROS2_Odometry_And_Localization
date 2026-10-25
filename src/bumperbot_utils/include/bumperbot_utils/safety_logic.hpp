#ifndef BUMPERBOT_UTILS__SAFETY_LOGIC_HPP_
#define BUMPERBOT_UTILS__SAFETY_LOGIC_HPP_

// Pure safety logic of the AGV safety supervisor (no ROS, no clock, no threads).
// The Python twin is bumperbot_utils/safety_logic.py (same API, formulas and strings).
//
// Conventions:
//   * Polygons are vectors of (x, y) pairs in the robot frame (the footprint parameter is flat).
//   * x_rear of footprint_extents() is the signed minimum x (a negative number for a normal robot).
//   * Clearances are distances in metres from the footprint bounding box edge, inf means none.

#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace bumperbot_utils
{

namespace safety
{

constexpr const char * STATE_INITIALIZING = "INITIALIZING";
constexpr const char * STATE_ACTIVE = "ACTIVE";
constexpr const char * STATE_DOCK_APPROACH = "DOCK_APPROACH";
constexpr const char * STATE_FAULT = "FAULT";
constexpr const char * STATE_ESTOP = "ESTOP";
constexpr const char * STATE_SHUTDOWN = "SHUTDOWN";

constexpr const char * ZONE_CLEAR = "CLEAR";
constexpr const char * ZONE_WARNING = "WARNING";
constexpr const char * ZONE_STOP = "STOP";
constexpr const char * ZONE_CONTACT = "CONTACT";
constexpr const char * ZONE_UNKNOWN = "UNKNOWN";

constexpr const char * ENFORCEMENT_FILTER = "filter";
constexpr const char * ENFORCEMENT_LOCK_ONLY = "lock_only";

constexpr const char * DOCK_STATE_DOCKING = "DOCKING";

using Point = std::pair<double, double>;
using Polygon = std::vector<Point>;

// All parameters of the supervisor (defaults equal config/safety_supervisor.yaml)
struct Config
{
  std::string enforcement = ENFORCEMENT_FILTER;
  std::string robot_frame = "base_footprint";
  std::string scan_topic = "scan";
  std::string odom_topic = "bumperbot_controller/odom";
  std::string cmd_vel_in_topic = "safety/cmd_vel_in";
  std::string cmd_vel_out_topic = "bumperbot_controller/cmd_vel_unstamped";
  std::string lock_topic = "safety_stop";
  std::string status_topic = "safety/status";
  std::string estop_topic = "safety/estop";
  std::string dock_status_topic = "dock/status";
  bool use_odom = true;
  std::vector<double> footprint = {0.10, 0.09, 0.10, -0.09, -0.10, -0.09, -0.10, 0.09};
  double lateral_margin = 0.03;
  double stop_margin = 0.05;
  double stop_release_hysteresis = 0.02;
  double slowdown_distance = 0.35;
  double contact_margin = 0.015;
  double max_linear_speed = 0.7;
  double max_angular_speed = 5.0;
  double response_time = 0.3;
  double max_decel = 0.5;
  double stopped_speed = 0.02;
  double brake_hold_time = 0.5;
  double scan_timeout = 0.6;
  double odom_timeout = 0.5;
  double cmd_timeout = 0.5;
  double recovery_time = 0.5;
  double max_invalid_beam_ratio = 0.5;
  bool latch_faults = false;
  bool latch_on_contact = true;
  double rate = 20.0;
  double status_rate = 5.0;
  bool dock_approach_enabled = true;
  double dock_max_speed = 0.06;
  double dock_stop_margin = 0.01;
  double dock_approach_radius = 0.8;
  double dock_approach_max_time = 30.0;
  double dock_status_timeout = 2.0;
  int64_t debug_inject_tick_exceptions = 0;
};

// Returns "" when valid, otherwise a description of the first error
std::string validate_config(const Config & cfg);

// Geometry
Polygon footprint_polygon(const std::vector<double> & flat);
bool polygon_is_simple(const Polygon & poly);
bool point_in_polygon(double px, double py, const Polygon & poly);
double distance_to_polygon(double px, double py, const Polygon & poly);

struct Extents
{
  double x_front = 0.0;
  double x_rear = 0.0;
  double half_width = 0.0;
};
Extents footprint_extents(const Polygon & poly);

struct ScanPoints
{
  std::vector<Point> points;
  double invalid_ratio = 0.0;
};
ScanPoints scan_to_points(
  const std::vector<double> & ranges, double angle_min, double angle_increment,
  double range_min, double range_max, double tf_x, double tf_y, double tf_yaw);

struct Evaluation
{
  double front_clearance = std::numeric_limits<double>::infinity();
  double rear_clearance = std::numeric_limits<double>::infinity();
  double nearest_distance = std::numeric_limits<double>::infinity();
  double nearest_bearing = 0.0;
  bool contact_front = false;
  bool contact_rear = false;
};
Evaluation evaluate_points(const std::vector<Point> & points, const Config & cfg);

// Limits, zones, direction
double speed_limit(double d, double margin, const Config & cfg);
std::string zone_for(
  double d, const std::string & prev_zone, double margin, const Config & cfg,
  bool allow_contact = true);
int travel_direction(double v_cmd, double v_odom, double stopped_speed);

// Dock approach gate: an episode starts on the transition into DOCKING, expires after
// dock_approach_max_time and re-arms only after the dock state leaves DOCKING.
class DockGate
{
public:
  explicit DockGate(const Config & cfg);
  void reset();
  // status_state: last known dock state ("" = no status yet)
  // status_age: seconds since it was received (negative or NaN = never)
  // distance: robot distance to the dock pose in m (NaN = TF unavailable)
  // Returns (active, reason).
  std::pair<bool, std::string> update(
    double now, bool enabled, const std::string & status_state, double status_age,
    double distance);

private:
  Config cfg_;
  bool in_episode_ = false;
  double episode_start_ = 0.0;
  bool expired_ = false;
};

// One snapshot of everything the node knows at a tick (defaults are fail-safe)
struct Inputs
{
  bool scan_ok = false;                 // a valid scan was evaluated (Evaluation is meaningful)
  double scan_age = -1.0;               // s since the last scan arrived, negative: never
  std::string scan_invalid_reason;      // non-empty: the last scan was invalid
  bool tf_ok = false;                   // robot_frame <- scan frame is known
  std::string tf_description;           // e.g. "base_footprint <- laser_link"
  Evaluation evaluation;
  double odom_age = -1.0;               // s, negative: never
  double v_odom = std::numeric_limits<double>::quiet_NaN();  // m/s, NaN: unknown
  double cmd_age = -1.0;                // s, negative: never
  double v_cmd = std::numeric_limits<double>::quiet_NaN();   // m/s, NaN: unknown
  bool estop_input = false;
  bool dock_active = false;             // DockGate result
  std::string dock_reason;
  std::string tick_exception;           // non-empty: the previous tick body raised
};

struct Outputs
{
  std::string state = STATE_INITIALIZING;
  bool locked = true;
  bool braking = false;
  bool estop_latched = false;
  std::string front_zone = ZONE_UNKNOWN;
  std::string rear_zone = ZONE_UNKNOWN;
  std::string active_zone = ZONE_UNKNOWN;
  double max_forward = 0.0;
  double max_reverse = 0.0;
  std::string reason;
  bool dock_profile_active = false;
};

// Safety state machine: zones, brake pulse, e-stop latch, watchdog and dock profile
class SafetyCore
{
public:
  explicit SafetyCore(const Config & cfg);
  Outputs update(double now, const Inputs & inp);
  std::pair<bool, std::string> request_reset();
  void trigger_estop(const std::string & reason);
  void shutdown();
  std::pair<double, double> limit_command(double v, double w, const Outputs & outputs) const;

private:
  void latch(const std::string & reason);

  Config cfg_;
  bool shutdown_ = false;
  bool estop_latched_ = false;
  std::string latch_reason_;
  bool brake_active_ = false;
  double brake_start_ = 0.0;
  bool fault_active_ = false;
  std::string fault_reason_;
  bool has_clear_since_ = false;
  double clear_since_ = 0.0;
  std::string prev_front_ = ZONE_UNKNOWN;
  std::string prev_rear_ = ZONE_UNKNOWN;
  std::string last_problem_ = "waiting for scan";
  bool last_estop_input_ = false;
  bool last_contact_front_ = false;
  bool last_contact_rear_ = false;
  bool last_dock_active_ = false;
};

}  // namespace safety

}  // namespace bumperbot_utils

#endif  // BUMPERBOT_UTILS__SAFETY_LOGIC_HPP_
