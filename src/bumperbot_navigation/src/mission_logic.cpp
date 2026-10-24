#include "bumperbot_navigation/mission_logic.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace bumperbot_navigation
{

const char * const STATE_IDLE = "IDLE";
const char * const STATE_NAVIGATING = "NAVIGATING";
const char * const STATE_DWELLING = "DWELLING";
const char * const STATE_RETURNING_TO_DOCK = "RETURNING_TO_DOCK";
const char * const STATE_COMPLETED = "COMPLETED";
const char * const STATE_ABORTED = "ABORTED";
const char * const STATE_CANCELED = "CANCELED";
const char * const STATE_DOCK_FAILED = "DOCK_FAILED";

const Station * StationSet::find(const std::string & name) const
{
  for (const auto & station : stations) {
    if (station.name == name) {
      return &station;
    }
  }
  return nullptr;
}

namespace
{
double read_number(const YAML::Node & node, const std::string & what)
{
  if (!node.IsScalar()) {
    throw std::invalid_argument(what + " is not a number");
  }
  double value = 0.0;
  try {
    value = node.as<double>();
  } catch (const YAML::Exception &) {
    throw std::invalid_argument(what + " is not a number");
  }
  if (!std::isfinite(value)) {
    throw std::invalid_argument(what + " is not finite");
  }
  return value;
}
}  // namespace

StationSet load_stations(const std::string & path)
{
  YAML::Node data;
  try {
    data = YAML::LoadFile(path);
  } catch (const YAML::Exception & exc) {
    throw std::invalid_argument("cannot read stations file '" + path + "': " + exc.what());
  }
  if (!data.IsMap()) {
    throw std::invalid_argument("stations file '" + path + "' is not a mapping");
  }

  StationSet result;
  if (data["frame_id"]) {
    result.frame_id = data["frame_id"].as<std::string>();
  }
  if (data["dock_station"]) {
    result.dock_station = data["dock_station"].as<std::string>();
  }

  const YAML::Node stations = data["stations"];
  if (!stations || !stations.IsMap() || stations.size() == 0) {
    throw std::invalid_argument("'stations' is missing or empty");
  }

  for (auto it = stations.begin(); it != stations.end(); ++it) {
    const std::string name = it->first.as<std::string>();
    const YAML::Node entry = it->second;
    if (!entry.IsMap()) {
      throw std::invalid_argument("station '" + name + "' is not a mapping");
    }
    Station station;
    station.name = name;
    double values[4];
    const char * keys[4] = {"x", "y", "yaw", "dwell"};
    for (int i = 0; i < 4; ++i) {
      const YAML::Node field = entry[keys[i]];
      if (!field) {
        throw std::invalid_argument(
                "station '" + name + "' misses field '" + keys[i] + "'");
      }
      values[i] = read_number(
        field, "station '" + name + "' field '" + keys[i] + "'");
    }
    station.x = values[0];
    station.y = values[1];
    station.yaw = values[2];
    station.dwell = values[3];
    if (station.dwell < 0.0) {
      throw std::invalid_argument("station '" + name + "' has a negative dwell");
    }
    result.stations.push_back(station);
  }

  if (result.find(result.dock_station) == nullptr) {
    throw std::invalid_argument("dock station '" + result.dock_station + "' is not defined");
  }
  return result;
}

std::string validate_mission(const std::vector<std::string> & names, const StationSet & station_set)
{
  if (names.empty()) {
    return "empty station list";
  }
  for (const auto & name : names) {
    if (station_set.find(name) == nullptr) {
      return "unknown station '" + name + "'";
    }
  }
  return "";
}

std::pair<double, double> yaw_to_quaternion(double yaw)
{
  return {std::sin(yaw / 2.0), std::cos(yaw / 2.0)};
}

double planar_distance(double ax, double ay, double bx, double by)
{
  return std::hypot(ax - bx, ay - by);
}

MissionTracker::MissionTracker(
  const StationSet & station_set, const std::vector<std::string> & names,
  const TrackerParams & params)
: set_(station_set), names_(names), params_(params), state_(STATE_IDLE)
{
}

int MissionTracker::current_index() const
{
  if ((phase_ == Phase::STATIONS || phase_ == Phase::DWELL) && started_) {
    return static_cast<int>(index_);
  }
  return -1;
}

double MissionTracker::progress() const
{
  if (names_.empty()) {
    return 1.0;
  }
  const double done = static_cast<double>(visited_.size() + skipped_.size());
  return std::min(1.0, done / static_cast<double>(names_.size()));
}

Step MissionTracker::next_step()
{
  started_ = true;
  Step step;
  if (phase_ == Phase::STATIONS) {
    current_station_ = names_[index_];
    state_ = STATE_NAVIGATING;
    step.kind = StepKind::NAVIGATE;
    step.station = current_station_;
    step.attempt = attempt_;
  } else if (phase_ == Phase::DWELL) {
    current_station_ = names_[index_];
    state_ = STATE_DWELLING;
    step.kind = StepKind::DWELL;
    step.station = current_station_;
    step.attempt = attempt_;
    step.dwell = set_.find(current_station_)->dwell;
  } else if (phase_ == Phase::DOCK) {
    current_station_ = set_.dock_station;
    state_ = STATE_RETURNING_TO_DOCK;
    step.kind = StepKind::NAVIGATE;
    step.station = current_station_;
    step.attempt = attempt_;
    step.docking = true;
  }
  return step;
}

void MissionTracker::report_navigation(bool succeeded, double distance_error)
{
  if (phase_ != Phase::STATIONS && phase_ != Phase::DOCK) {
    return;
  }
  const bool docking = phase_ == Phase::DOCK;
  const std::string name = docking ? set_.dock_station : names_[index_];
  const bool reached = succeeded && !std::isnan(distance_error) &&
    distance_error <= params_.goal_tolerance;

  if (reached) {
    at_dock_ = (name == set_.dock_station);
    if (docking) {
      finish();
      return;
    }
    visited_.push_back(name);
    consecutive_ = 0;
    if (set_.find(name)->dwell > 0.0) {
      phase_ = Phase::DWELL;
    } else {
      advance();
    }
    return;
  }

  // Failed attempt: the robot may have moved, so it is no longer known to be at the dock.
  at_dock_ = false;
  const int limit = (docking ? params_.dock_max_retries : params_.max_retries) + 1;
  if (attempt_ < limit) {
    ++attempt_;
    return;
  }
  if (docking) {
    finish(true);
    return;
  }
  skipped_.push_back(name);
  ++consecutive_;
  const int max_fail = params_.max_consecutive_failures;
  if (max_fail > 0 && consecutive_ >= max_fail) {
    abort_ = true;
    message_ = "aborted after " + std::to_string(consecutive_) + " consecutive skipped stations";
    begin_docking();
    return;
  }
  advance();
}

void MissionTracker::report_dwell_done()
{
  if (phase_ == Phase::DWELL) {
    advance();
  }
}

void MissionTracker::request_cancel()
{
  if (phase_ == Phase::DONE || cancel_) {
    return;
  }
  cancel_ = true;
  if (message_.empty()) {
    message_ = "canceled by the client";
  }
  if (phase_ == Phase::DOCK) {
    return;
  }
  begin_docking();
}

void MissionTracker::abort(const std::string & reason)
{
  if (phase_ == Phase::DONE || abort_) {
    return;
  }
  abort_ = true;
  message_ = reason;
  if (phase_ == Phase::DOCK) {
    return;
  }
  begin_docking();
}

void MissionTracker::advance()
{
  ++index_;
  attempt_ = 1;
  if (index_ < names_.size()) {
    phase_ = Phase::STATIONS;
    return;
  }
  begin_docking();
}

void MissionTracker::begin_docking()
{
  attempt_ = 1;
  const bool forced = cancel_ || abort_;
  if ((!forced && !params_.return_to_dock) || at_dock_) {
    finish();
    return;
  }
  phase_ = Phase::DOCK;
}

void MissionTracker::finish(bool dock_failed)
{
  phase_ = Phase::DONE;
  if (dock_failed) {
    docked_ = false;
    outcome_ = OUTCOME_DOCK_FAILED;
    state_ = STATE_DOCK_FAILED;
    message_ = "the dock could not be reached";
    return;
  }
  docked_ = at_dock_;
  if (cancel_) {
    outcome_ = OUTCOME_CANCELED;
    state_ = STATE_CANCELED;
  } else if (abort_) {
    outcome_ = OUTCOME_ABORTED;
    state_ = STATE_ABORTED;
  } else if (!skipped_.empty()) {
    outcome_ = OUTCOME_COMPLETED_WITH_SKIPS;
    state_ = STATE_COMPLETED;
    message_ = "completed with skipped stations";
  } else {
    outcome_ = OUTCOME_COMPLETED;
    state_ = STATE_COMPLETED;
    message_ = "mission completed";
  }
}

}  // namespace bumperbot_navigation
