#ifndef BUMPERBOT_NAVIGATION__DOCK_LOGIC_HPP_
#define BUMPERBOT_NAVIGATION__DOCK_LOGIC_HPP_

// Pure docking logic (no ROS dependency): geometry, the closed-loop dock controller, the
// detection filter, the phase timer and the DockTracker state machine.
// The Python twin is bumperbot_navigation/dock_logic.py; both expose the same API and are
// checked against test/data/dock_cases.yaml.
//
// Conventions
//   * Poses are planar (x, y, yaw) with yaw in rad.
//   * The controller works on the robot pose p expressed in the dock frame D: the origin is the
//     charger pose (the base_footprint pose when docked) and the x axis is the docked heading.
//   * s = +1 for backwards docking (the robot backs into the charger), s = -1 for forwards
//     docking. The pre-dock pose is P = (s * L, 0, 0) with L = staging_distance, which is the
//     staging pose.

#include <array>
#include <string>
#include <utility>

namespace bumperbot_navigation
{

// Commands (same values as the DockRobot.action constants)
constexpr int COMMAND_DOCK = 0;
constexpr int COMMAND_UNDOCK = 1;

// Outcomes (same values as the DockRobot.action constants)
constexpr int DOCK_OUTCOME_DOCKED = 0;
constexpr int DOCK_OUTCOME_UNDOCKED = 1;
constexpr int DOCK_OUTCOME_ALREADY_DOCKED = 2;
constexpr int DOCK_OUTCOME_CANCELED = 3;
constexpr int DOCK_OUTCOME_STAGING_FAILED = 4;
constexpr int DOCK_OUTCOME_DETECTION_FAILED = 5;
constexpr int DOCK_OUTCOME_DOCKING_FAILED = 6;
constexpr int DOCK_OUTCOME_UNDOCKING_FAILED = 7;
constexpr int DOCK_OUTCOME_ERROR = 8;

// States (same strings as the DockStatus.msg constants)
extern const char * const DOCK_STATE_UNDOCKED;
extern const char * const DOCK_STATE_NAVIGATING_TO_STAGING;
extern const char * const DOCK_STATE_WAITING_FOR_DETECTION;
extern const char * const DOCK_STATE_ALIGNING;
extern const char * const DOCK_STATE_DOCKING;
extern const char * const DOCK_STATE_DOCKED;
extern const char * const DOCK_STATE_CHARGING;
extern const char * const DOCK_STATE_UNDOCKING;
extern const char * const DOCK_STATE_FAILED;

// Detection modes
extern const char * const DETECTION_NONE;
extern const char * const DETECTION_OPTIONAL;
extern const char * const DETECTION_REQUIRED;

struct Pose2D
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

// Wrap an angle to [-pi, pi]
double normalize_angle(double angle);
// Return a * b: the pose b (expressed in frame a) expressed in the parent frame of a
Pose2D compose(const Pose2D & a, const Pose2D & b);
Pose2D inverse(const Pose2D & a);
// Return the pose expressed in the given frame
Pose2D relative(const Pose2D & frame, const Pose2D & pose);
double quaternion_yaw(double qx, double qy, double qz, double qw);

// 3D pose as (x, y, z, qx, qy, qz, qw)
using Pose3 = std::array<double, 7>;
// Express a 3D pose in the parent frame of frame_tf (the transform from the pose's frame to the
// target frame) and project it to the plane.
Pose2D project_pose(const Pose3 & frame_tf, const Pose3 & pose);

// Charger pose: staging_distance behind (backwards) or ahead of the staging pose
Pose2D compute_dock_pose(const Pose2D & staging, double staging_distance, bool dock_backwards);

struct ControlParams
{
  double staging_distance = 0.5;
  bool dock_backwards = true;
  double max_linear_speed = 0.15;
  double max_angular_speed = 0.6;
  double approach_speed = 0.05;
  double slow_down_distance = 0.15;
  double k_lateral = 2.0;
  double k_heading = 2.0;
  double align_position_tolerance = 0.02;
  double align_yaw_tolerance = 0.03;
  double max_insert_lateral_error = 0.08;
  double max_insert_yaw_error = 0.3;
};

struct Command
{
  double v = 0.0;
  double w = 0.0;
  bool done = false;
  bool failed = false;
};

// Rotate towards the pre-dock pose, drive to it, rotate to the docked heading
class Aligner
{
public:
  explicit Aligner(const ControlParams & params = ControlParams());

  void reset();
  // Command that brings the robot pose p (dock frame) to the pre-dock pose
  Command align_command(const Pose2D & p);

protected:
  double side() const {return params_.dock_backwards ? 1.0 : -1.0;}

  ControlParams params_;

private:
  enum class Stage
  {
    ROTATE_TO_TARGET,
    DRIVE,
    ROTATE_FINAL
  };
  Stage stage_ = Stage::ROTATE_TO_TARGET;
};

// Aligner plus the line-tracking insertion / undock commands and the pose error
class DockController : public Aligner
{
public:
  explicit DockController(const ControlParams & params = ControlParams())
  : Aligner(params) {}

  Command insert_command(const Pose2D & p) const;
  Command undock_command(const Pose2D & p) const;
  // Returns (position error in m, absolute yaw error in rad)
  static std::pair<double, double> pose_error(const Pose2D & p);

private:
  double line_command(const Pose2D & p, double v) const;
};

// Gate and smooth detections of the charger pose (all poses in the odom frame)
class DetectionFilter
{
public:
  DetectionFilter(double max_jump, double alpha);

  // Returns (accepted, new_estimate). A rejected detection leaves the estimate unchanged.
  std::pair<bool, Pose2D> update(const Pose2D & estimate, const Pose2D & detection) const;

private:
  double max_jump_;
  double alpha_;
};

// Phase timeout on node time with a wall-clock fallback for a stalled time source. Time
// arguments are seconds. When the node time does not advance for clock_stall_timeout wall
// seconds, stalled() becomes true and the phase counts as expired. 0 disables the fallback.
class PhaseTimer
{
public:
  PhaseTimer(double timeout, double clock_stall_timeout);

  void start(double node_now, double wall_now);
  bool expired(double node_now, double wall_now);
  bool stalled() const {return stalled_;}

private:
  double timeout_;
  double clock_stall_timeout_;
  double start_ = 0.0;
  double last_node_ = 0.0;
  double last_advance_wall_ = 0.0;
  bool stalled_ = false;
};

// Step kinds
enum class DockStepKind
{
  NAVIGATE_STAGING,
  WAIT_DETECTION,
  ALIGN,
  INSERT,
  SETTLE_VERIFY,
  UNDOCK,
  FINISHED
};

// Phase results reported by the node
enum class PhaseResult
{
  OK,
  TIMEOUT,
  STALL,
  SAFETY_TIMEOUT,
  MISALIGNED
};

// Names used in the shared case table
const char * to_string(DockStepKind kind);
const char * to_string(PhaseResult result);

struct DockTrackerParams
{
  int max_retries = 2;
  int nav_max_retries = 2;
  std::string detection_mode = "optional";
  double dock_position_tolerance = 0.03;
  double dock_yaw_tolerance = 0.05;
};

struct DockStep
{
  DockStepKind kind = DockStepKind::FINISHED;
  int attempt = 0;
};

// Dock / undock state machine. No clock, no threads: the node drives it.
// next_step() is idempotent: it returns the current step until a report_* call advances it.
class DockTracker
{
public:
  explicit DockTracker(const DockTrackerParams & params = DockTrackerParams());

  // Begin a goal. Every per-goal field is reset.
  void start(int command, bool docked, bool near_staging);
  DockStep next_step();
  void report_nav(bool ok);
  void report_detection(bool ok);
  void report_phase(PhaseResult result);
  void report_verify(double position_error, double yaw_error);
  void request_cancel();
  void abort(const std::string & reason);

  const std::string & state() const {return state_;}
  int attempt() const {return attempt_;}
  int outcome() const {return outcome_;}
  bool docked() const {return docked_;}
  const std::string & message() const {return message_;}
  bool detection_used() const {return detection_used_;}
  double position_error() const {return position_error_;}
  double yaw_error() const {return yaw_error_;}
  bool moved() const {return moved_;}
  bool finished() const {return step_ == DockStepKind::FINISHED;}

private:
  void reset_fields();
  void enter(DockStepKind step);
  void after_staging();
  void fail_attempt(const std::string & reason);
  void finish(int outcome, const std::string & message, bool docked = false);

  DockTrackerParams params_;
  DockStepKind step_ = DockStepKind::FINISHED;
  int attempt_ = 1;
  int outcome_ = DOCK_OUTCOME_DOCKED;
  bool docked_ = false;
  bool detection_used_ = false;
  double position_error_ = -1.0;
  double yaw_error_ = -1.0;
  std::string message_;
  bool moved_ = false;
  std::string state_;
};

}  // namespace bumperbot_navigation

#endif  // BUMPERBOT_NAVIGATION__DOCK_LOGIC_HPP_
