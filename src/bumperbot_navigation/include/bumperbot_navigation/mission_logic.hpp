#ifndef BUMPERBOT_NAVIGATION__MISSION_LOGIC_HPP_
#define BUMPERBOT_NAVIGATION__MISSION_LOGIC_HPP_

// Pure mission logic (no ROS dependency): station loading and the MissionTracker state machine.
// The Python twin is bumperbot_navigation/mission_logic.py; both expose the same API and are
// checked against test/data/mission_cases.yaml.

#include <string>
#include <utility>
#include <vector>

namespace bumperbot_navigation
{

// Mission states (same strings as the MissionStatus.msg constants)
extern const char * const STATE_IDLE;
extern const char * const STATE_NAVIGATING;
extern const char * const STATE_DWELLING;
extern const char * const STATE_RETURNING_TO_DOCK;
extern const char * const STATE_COMPLETED;
extern const char * const STATE_ABORTED;
extern const char * const STATE_CANCELED;
extern const char * const STATE_DOCK_FAILED;

// Outcomes (same values as the ExecuteMission.action constants)
constexpr int OUTCOME_COMPLETED = 0;
constexpr int OUTCOME_COMPLETED_WITH_SKIPS = 1;
constexpr int OUTCOME_ABORTED = 2;
constexpr int OUTCOME_CANCELED = 3;
constexpr int OUTCOME_DOCK_FAILED = 4;

struct Station
{
  std::string name;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double dwell = 0.0;
};

struct StationSet
{
  std::string frame_id = "map";
  std::string dock_station = "dock";
  std::vector<Station> stations;

  // Returns nullptr when the station does not exist
  const Station * find(const std::string & name) const;
};

// Load and validate a stations file. Throws std::invalid_argument on any problem.
StationSet load_stations(const std::string & path);

// Returns an empty string when the mission is valid, otherwise the reason.
std::string validate_mission(
  const std::vector<std::string> & names,
  const StationSet & station_set);

// Returns (z, w) of a planar rotation.
std::pair<double, double> yaw_to_quaternion(double yaw);

double planar_distance(double ax, double ay, double bx, double by);

struct TrackerParams
{
  int max_retries = 2;
  int max_consecutive_failures = 2;
  int dock_max_retries = 3;
  bool return_to_dock = true;
  double goal_tolerance = 0.2;
};

enum class StepKind
{
  NAVIGATE,
  DWELL,
  FINISHED
};

struct Step
{
  StepKind kind = StepKind::FINISHED;
  std::string station;
  int attempt = 0;
  bool docking = false;
  double dwell = 0.0;
};

// Mission state machine. No clock, no threads: the node drives it.
// next_step() is idempotent: it returns the current step until a report_* call advances it.
class MissionTracker
{
public:
  MissionTracker(
    const StationSet & station_set, const std::vector<std::string> & names,
    const TrackerParams & params = TrackerParams());

  Step next_step();
  // distance_error NaN means the TF is unknown, which counts as a failure
  void report_navigation(bool succeeded, double distance_error);
  void report_dwell_done();
  void request_cancel();
  void abort(const std::string & reason);

  const std::string & state() const {return state_;}
  const std::string & current_station() const {return current_station_;}
  int current_index() const;
  int attempt() const {return attempt_;}
  double progress() const;
  const std::vector<std::string> & visited() const {return visited_;}
  const std::vector<std::string> & skipped() const {return skipped_;}
  int outcome() const {return outcome_;}
  bool docked() const {return docked_;}
  const std::string & message() const {return message_;}
  bool finished() const {return phase_ == Phase::DONE;}

private:
  enum class Phase
  {
    STATIONS,
    DWELL,
    DOCK,
    DONE
  };

  void advance();
  void begin_docking();
  void finish(bool dock_failed = false);

  StationSet set_;
  std::vector<std::string> names_;
  TrackerParams params_;
  Phase phase_ = Phase::STATIONS;
  size_t index_ = 0;
  int attempt_ = 1;
  int consecutive_ = 0;
  bool at_dock_ = false;
  bool cancel_ = false;
  bool abort_ = false;
  bool started_ = false;
  std::string current_station_;
  std::vector<std::string> visited_;
  std::vector<std::string> skipped_;
  int outcome_ = OUTCOME_COMPLETED;
  bool docked_ = false;
  std::string message_;
  std::string state_;
};

}  // namespace bumperbot_navigation

#endif  // BUMPERBOT_NAVIGATION__MISSION_LOGIC_HPP_
