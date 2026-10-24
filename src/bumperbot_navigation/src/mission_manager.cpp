#include "bumperbot_navigation/mission_manager.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <thread>

#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

namespace bumperbot_navigation
{

namespace
{
using SteadyClock = std::chrono::steady_clock;

std::string join(const std::vector<std::string> & items)
{
  std::string result;
  for (size_t i = 0; i < items.size(); ++i) {
    result += (i == 0 ? "" : ", ") + items[i];
  }
  return result;
}
}  // namespace

MissionManager::MissionManager(const rclcpp::NodeOptions & options)
: Node("mission_manager", options)
{
  declare_parameter<std::string>("stations_file", "");
  declare_parameter<std::string>("nav_action_name", "navigate_to_pose");
  declare_parameter<std::string>("global_frame", "map");
  declare_parameter<std::string>("robot_frame", "base_footprint");
  declare_parameter<double>("goal_tolerance", 0.2);
  declare_parameter<int>("max_retries", 2);
  declare_parameter<double>("retry_delay", 1.0);
  declare_parameter<int>("max_consecutive_failures", 2);
  declare_parameter<int>("dock_max_retries", 3);
  declare_parameter<bool>("return_to_dock", true);
  declare_parameter<double>("nav_server_timeout", 10.0);
  declare_parameter<double>("nav_goal_timeout", 300.0);
  declare_parameter<double>("tf_timeout", 1.0);
  declare_parameter<double>("status_rate", 2.0);
  declare_parameter<std::string>("behavior_tree", "");

  std::string stations_file = get_parameter("stations_file").as_string();
  if (stations_file.empty()) {
    stations_file =
      ament_index_cpp::get_package_share_directory("bumperbot_navigation") +
      "/config/stations.yaml";
  }
  stations_ = load_stations(stations_file);
  RCLCPP_INFO(
    get_logger(), "Loaded %zu stations from %s", stations_.stations.size(),
    stations_file.c_str());

  global_frame_ = get_parameter("global_frame").as_string();
  robot_frame_ = get_parameter("robot_frame").as_string();
  if (stations_.frame_id != global_frame_) {
    // The tolerance check compares station coordinates with the TF in global_frame
    throw std::invalid_argument(
            "stations frame_id '" + stations_.frame_id + "' differs from global_frame '" +
            global_frame_ + "'");
  }
  retry_delay_ = get_parameter("retry_delay").as_double();
  nav_server_timeout_ = get_parameter("nav_server_timeout").as_double();
  nav_goal_timeout_ = get_parameter("nav_goal_timeout").as_double();
  tf_timeout_ = get_parameter("tf_timeout").as_double();
  behavior_tree_ = get_parameter("behavior_tree").as_string();
  tracker_params_.max_retries = get_parameter("max_retries").as_int();
  tracker_params_.max_consecutive_failures = get_parameter("max_consecutive_failures").as_int();
  tracker_params_.dock_max_retries = get_parameter("dock_max_retries").as_int();
  tracker_params_.return_to_dock = get_parameter("return_to_dock").as_bool();
  tracker_params_.goal_tolerance = get_parameter("goal_tolerance").as_double();

  group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);

  status_pub_ = create_publisher<bumperbot_msgs::msg::MissionStatus>(
    "/mission/status", rclcpp::QoS(1).reliable().transient_local());

  nav_client_ = rclcpp_action::create_client<NavigateToPose>(
    this, get_parameter("nav_action_name").as_string(), group_);

  action_server_ = rclcpp_action::create_server<ExecuteMission>(
    this, "execute_mission",
    std::bind(&MissionManager::goal_callback, this, _1, _2),
    std::bind(&MissionManager::cancel_callback, this, _1),
    std::bind(&MissionManager::accepted_callback, this, _1),
    rcl_action_server_get_default_options(), group_);

  publish_status();
  const double rate = get_parameter("status_rate").as_double();
  if (rate > 0.0) {
    status_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate), std::bind(&MissionManager::publish_status, this),
      group_);
  }

  RCLCPP_INFO(get_logger(), "Mission manager ready (action: /execute_mission)");
}

// ----- action server callbacks -----
rclcpp_action::GoalResponse MissionManager::goal_callback(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const ExecuteMission::Goal> goal)
{
  const std::string reason = validate_mission(goal->stations, stations_);
  if (!reason.empty()) {
    RCLCPP_WARN(get_logger(), "Mission rejected: %s", reason.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (active_) {
    RCLCPP_WARN(get_logger(), "Mission rejected: another mission is active");
    return rclcpp_action::GoalResponse::REJECT;
  }
  active_ = true;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MissionManager::cancel_callback(
  std::shared_ptr<MissionGoalHandle>)
{
  RCLCPP_INFO(get_logger(), "Mission cancel requested");
  cancel_requested_ = true;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MissionManager::accepted_callback(std::shared_ptr<MissionGoalHandle> goal_handle)
{
  // Run the mission in its own thread so the executor stays free
  std::thread{std::bind(&MissionManager::execute, this, _1), goal_handle}.detach();
}

// ----- status -----
bumperbot_msgs::msg::MissionStatus MissionManager::build_status()
{
  bumperbot_msgs::msg::MissionStatus msg;
  msg.stamp = get_clock()->now();
  std::lock_guard<std::mutex> lock(mutex_);
  if (!tracker_) {
    msg.state = bumperbot_msgs::msg::MissionStatus::STATE_IDLE;
    msg.current_index = -1;
    return msg;
  }
  msg.state = tracker_->state();
  msg.current_station = tracker_->current_station();
  msg.current_index = tracker_->current_index();
  msg.total_stations = mission_names_.size();
  msg.attempt = tracker_->attempt();
  msg.progress = static_cast<float>(tracker_->progress());
  msg.stations = mission_names_;
  msg.visited = tracker_->visited();
  msg.skipped = tracker_->skipped();
  msg.message = tracker_->message();
  return msg;
}

void MissionManager::publish_status()
{
  status_pub_->publish(build_status());
}

void MissionManager::publish_feedback()
{
  const auto status = build_status();
  std::shared_ptr<MissionGoalHandle> handle;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    handle = goal_handle_;
  }
  if (!handle) {
    return;
  }
  auto feedback = std::make_shared<ExecuteMission::Feedback>();
  feedback->state = status.state;
  feedback->current_station = status.current_station;
  feedback->current_index = status.current_index;
  feedback->total_stations = status.total_stations;
  feedback->attempt = status.attempt;
  feedback->progress = status.progress;
  feedback->distance_remaining = static_cast<float>(distance_remaining_.load());
  handle->publish_feedback(feedback);
}

void MissionManager::publish_all()
{
  publish_status();
  publish_feedback();
}

// ----- helpers -----
bool MissionManager::cancel_signaled()
{
  // A cancel can also be pending on the goal handle before the cancel callback has run
  if (!cancel_requested_ && goal_handle_ && goal_handle_->is_canceling()) {
    cancel_requested_ = true;
  }
  return cancel_requested_;
}

bool MissionManager::handle_cancel_if_pending()
{
  if (cancel_signaled() && !cancel_handled_) {
    cancel_handled_ = true;
    std::lock_guard<std::mutex> lock(mutex_);
    tracker_->request_cancel();
    return true;
  }
  return false;
}

bool MissionManager::wait_clock(double seconds, bool interruptible)
{
  const rclcpp::Time end = get_clock()->now() + rclcpp::Duration::from_seconds(seconds);
  while (rclcpp::ok() && get_clock()->now() < end) {
    std::this_thread::sleep_for(20ms);
    handle_cancel_if_pending();
    if (interruptible && cancel_handled_) {
      return true;
    }
  }
  return false;
}

MissionManager::NavigateToPose::Goal MissionManager::make_goal(const Station & station)
{
  NavigateToPose::Goal goal;
  goal.pose.header.frame_id = stations_.frame_id;
  goal.pose.header.stamp = get_clock()->now();
  goal.pose.pose.position.x = station.x;
  goal.pose.pose.position.y = station.y;
  const auto q = yaw_to_quaternion(station.yaw);
  goal.pose.pose.orientation.z = q.first;
  goal.pose.pose.orientation.w = q.second;
  goal.behavior_tree = behavior_tree_;
  return goal;
}

double MissionManager::distance_to_station(const Station & station)
{
  const auto deadline = SteadyClock::now() + std::chrono::duration<double>(tf_timeout_);
  while (true) {
    try {
      const auto transform = tf_buffer_->lookupTransform(
        global_frame_, robot_frame_, tf2::TimePointZero);
      return planar_distance(
        transform.transform.translation.x, transform.transform.translation.y,
        station.x, station.y);
    } catch (const std::exception &) {
      if (SteadyClock::now() >= deadline) {
        return std::numeric_limits<double>::quiet_NaN();
      }
      std::this_thread::sleep_for(50ms);
    }
  }
}

bool MissionManager::navigate(const Step & step, bool & interrupted)
{
  interrupted = false;
  const Station * station = stations_.find(step.station);
  distance_remaining_ = -1.0;

  rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
  options.feedback_callback =
    [this](NavGoalHandle::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> fb) {
      distance_remaining_ = fb->distance_remaining;
      publish_feedback();
    };

  // A rejection means the Nav2 lifecycle is probably not active yet: keep re-sending without
  // counting a failed attempt until nav_server_timeout of continuous rejection has passed.
  bool rejected_before = false;
  SteadyClock::time_point rejected_since;
  NavGoalHandle::SharedPtr nav_handle;
  while (true) {
    const auto server_deadline =
      SteadyClock::now() + std::chrono::duration<double>(nav_server_timeout_);
    while (!nav_client_->action_server_is_ready()) {
      if (SteadyClock::now() >= server_deadline) {
        RCLCPP_WARN(get_logger(), "Nav2 action server is not available");
        return false;
      }
      std::this_thread::sleep_for(100ms);
      if (handle_cancel_if_pending() && !step.docking) {
        interrupted = true;
        return false;
      }
    }

    auto send_future = nav_client_->async_send_goal(make_goal(*station), options);
    if (send_future.wait_for(std::chrono::duration<double>(nav_server_timeout_)) !=
      std::future_status::ready)
    {
      RCLCPP_WARN(get_logger(), "Nav2 did not answer the goal request");
      return false;
    }
    nav_handle = send_future.get();
    if (nav_handle) {
      break;
    }
    if (!rejected_before) {
      rejected_before = true;
      rejected_since = SteadyClock::now();
    }
    if (SteadyClock::now() - rejected_since >= std::chrono::duration<double>(nav_server_timeout_)) {
      RCLCPP_WARN(get_logger(), "Nav2 rejected the goal for '%s'", step.station.c_str());
      return false;
    }
    RCLCPP_INFO(
      get_logger(), "Nav2 rejected the goal for '%s', retrying (server not ready?)",
      step.station.c_str());
    if (wait_clock(std::max(retry_delay_, 0.1), !step.docking)) {
      interrupted = true;
      return false;
    }
  }

  auto result_future = nav_client_->async_get_result(nav_handle);
  const auto goal_deadline = SteadyClock::now() + std::chrono::duration<double>(nav_goal_timeout_);
  bool timed_out = false;
  while (result_future.wait_for(50ms) != std::future_status::ready) {
    if (!rclcpp::ok()) {
      return false;
    }
    if (cancel_signaled() && !cancel_handled_) {
      handle_cancel_if_pending();
      if (!step.docking) {
        interrupted = true;
        break;
      }
    }
    if (SteadyClock::now() >= goal_deadline) {
      RCLCPP_WARN(get_logger(), "Navigation to '%s' timed out", step.station.c_str());
      timed_out = true;
      break;
    }
  }

  if (interrupted || timed_out) {
    try {
      auto cancel_future = nav_client_->async_cancel_goal(nav_handle);
      cancel_future.wait_for(5s);
    } catch (const std::exception & exc) {
      // e.g. rclcpp_action::exceptions::UnknownGoalHandleError when the goal already ended
      RCLCPP_WARN(get_logger(), "Could not cancel the Nav2 goal: %s", exc.what());
    }
    result_future.wait_for(5s);
    return false;
  }

  return result_future.get().code == rclcpp_action::ResultCode::SUCCEEDED;
}

// ----- mission execution -----
void MissionManager::execute(std::shared_ptr<MissionGoalHandle> goal_handle)
{
  const std::vector<std::string> names = goal_handle->get_goal()->stations;
  RCLCPP_INFO(get_logger(), "Mission started: [%s]", join(names).c_str());
  cancel_handled_ = false;
  // cancel_requested_ is not cleared here: a cancel that arrived right after the goal was
  // accepted must still count. It is cleared at the end of the previous mission.
  {
    std::lock_guard<std::mutex> lock(mutex_);
    goal_handle_ = goal_handle;
    mission_names_ = names;
    tracker_ = std::make_unique<MissionTracker>(stations_, names, tracker_params_);
  }

  // Whatever happens below, the cleanup at the end must run (like a finally)
  try {
    publish_all();
    std::string error;
    try {
      run_mission();
    } catch (const std::exception & exc) {
      error = exc.what();
    } catch (...) {
      error = "unknown error";
    }
    bool recovery_failed = false;
    if (!error.empty()) {
      RCLCPP_ERROR(get_logger(), "Mission error: %s", error.c_str());
      try {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          tracker_->abort("internal error: " + error);
        }
        run_mission();
      } catch (...) {
        recovery_failed = true;
        RCLCPP_ERROR(get_logger(), "Mission recovery failed");
      }
    }

    auto result = std::make_shared<ExecuteMission::Result>();
    std::string state;
    std::string message;
    int outcome;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      outcome = tracker_->outcome();
      state = tracker_->state();
      message = tracker_->message();
      result->docked = tracker_->docked();
      result->visited = tracker_->visited();
      result->skipped = tracker_->skipped();
      if (recovery_failed || !tracker_->finished()) {
        outcome = OUTCOME_ABORTED;
        state = STATE_ABORTED;
        message = "internal error: " + error;
        result->docked = false;
      }
      result->outcome = outcome;
      result->message = message;
    }
    try {
      publish_all();
    } catch (...) {
      RCLCPP_WARN(get_logger(), "Could not publish the final status");
    }

    // A cancel in progress wins over the outcome: the action ends CANCELED even when the dock
    // return then failed (the outcome field still carries DOCK_FAILED)
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
    } else if (outcome == OUTCOME_ABORTED || outcome == OUTCOME_DOCK_FAILED) {
      goal_handle->abort(result);
    } else {
      goal_handle->succeed(result);
    }
    RCLCPP_INFO(get_logger(), "Mission finished: %s (%s)", state.c_str(), message.c_str());
  } catch (...) {
    RCLCPP_ERROR(get_logger(), "Could not report the mission result");
  }

  std::lock_guard<std::mutex> lock(mutex_);
  goal_handle_.reset();
  cancel_requested_ = false;
  cancel_handled_ = false;
  active_ = false;
}

void MissionManager::run_mission()
{
  while (rclcpp::ok()) {
    if (handle_cancel_if_pending()) {
      publish_all();
    }
    Step step;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      step = tracker_->next_step();
    }
    publish_all();
    if (step.kind == StepKind::FINISHED) {
      break;
    }

    if (step.kind == StepKind::DWELL) {
      RCLCPP_INFO(get_logger(), "Dwelling %.1f s at '%s'", step.dwell, step.station.c_str());
      if (!wait_clock(step.dwell, true)) {
        std::lock_guard<std::mutex> lock(mutex_);
        tracker_->report_dwell_done();
      }
      continue;
    }

    // NAVIGATE: wait between attempts, then navigate and check the tolerance
    if (step.attempt > 1 && wait_clock(retry_delay_, !step.docking)) {
      continue;
    }
    RCLCPP_INFO(
      get_logger(), "Navigating to '%s' (attempt %d%s)", step.station.c_str(), step.attempt,
      step.docking ? ", docking" : "");
    bool interrupted = false;
    const bool succeeded = navigate(step, interrupted);
    if (interrupted) {
      continue;
    }
    double error = std::numeric_limits<double>::quiet_NaN();
    if (succeeded) {
      error = distance_to_station(*stations_.find(step.station));
      if (!std::isnan(error) && error > tracker_params_.goal_tolerance) {
        RCLCPP_WARN(
          get_logger(), "'%s' reported reached but the robot is %.2f m away",
          step.station.c_str(), error);
      }
    }
    std::lock_guard<std::mutex> lock(mutex_);
    tracker_->report_navigation(succeeded, error);
  }
}

}  // namespace bumperbot_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<bumperbot_navigation::MissionManager>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 6);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
