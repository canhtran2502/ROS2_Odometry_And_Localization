#include "bumperbot_navigation/dock_manager.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <future>
#include <set>
#include <stdexcept>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "tf2_ros/buffer_interface.h"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

namespace bumperbot_navigation
{

namespace
{
using SteadyClock = std::chrono::steady_clock;

// Number of accepted detections collected at the pre-dock pose before the alignment starts
constexpr int MIN_DETECTIONS = 5;
// Pause between two Nav2 attempts (s)
constexpr double NAV_RETRY_PAUSE = 0.5;

double wall_now()
{
  return std::chrono::duration<double>(SteadyClock::now().time_since_epoch()).count();
}

void sleep_seconds(double seconds)
{
  std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
}

bool is_active_mission_state(const std::string & state)
{
  return state == bumperbot_msgs::msg::MissionStatus::STATE_NAVIGATING ||
         state == bumperbot_msgs::msg::MissionStatus::STATE_DWELLING ||
         state == bumperbot_msgs::msg::MissionStatus::STATE_RETURNING_TO_DOCK;
}
}  // namespace

DockManager::DockManager(const rclcpp::NodeOptions & options)
: Node("dock_manager", options)
{
  declare_parameter<std::string>("stations_file", "");
  declare_parameter<std::string>("dock_station", "");
  declare_parameter<std::string>("nav_action_name", "navigate_to_pose");
  declare_parameter<std::string>("global_frame", "map");
  declare_parameter<std::string>("odom_frame", "odom");
  declare_parameter<std::string>("robot_frame", "base_footprint");
  declare_parameter<std::string>("cmd_vel_topic", "dock_vel");
  declare_parameter<std::string>("detection_topic", "dock/detected_pose");
  declare_parameter<std::string>("safety_stop_topic", "safety_stop");
  declare_parameter<std::string>("battery_topic", "battery_state");
  declare_parameter<std::string>("mission_status_topic", "mission/status");
  declare_parameter<bool>("dock_backwards", true);
  declare_parameter<double>("staging_distance", 0.5);
  declare_parameter<double>("staging_tolerance", 0.25);
  declare_parameter<double>("staging_yaw_tolerance", 0.6);
  declare_parameter<int>("nav_max_retries", 2);
  declare_parameter<double>("nav_server_timeout", 10.0);
  declare_parameter<double>("nav_goal_timeout", 180.0);
  declare_parameter<std::string>("detection_mode", "optional");
  declare_parameter<double>("detection_timeout", 3.0);
  declare_parameter<double>("detection_max_age", 0.5);
  declare_parameter<double>("detection_max_jump", 0.3);
  declare_parameter<double>("detection_filter_alpha", 0.3);
  declare_parameter<double>("detection_offset_x", 0.0);
  declare_parameter<double>("detection_offset_y", 0.0);
  declare_parameter<double>("detection_offset_yaw", 0.0);
  declare_parameter<double>("align_position_tolerance", 0.02);
  declare_parameter<double>("align_yaw_tolerance", 0.03);
  declare_parameter<double>("dock_position_tolerance", 0.03);
  declare_parameter<double>("dock_yaw_tolerance", 0.05);
  declare_parameter<double>("max_linear_speed", 0.15);
  declare_parameter<double>("max_angular_speed", 0.6);
  declare_parameter<double>("approach_speed", 0.05);
  declare_parameter<double>("slow_down_distance", 0.15);
  declare_parameter<double>("k_lateral", 2.0);
  declare_parameter<double>("k_heading", 2.0);
  declare_parameter<double>("max_insert_lateral_error", 0.08);
  declare_parameter<double>("max_insert_yaw_error", 0.3);
  declare_parameter<int>("max_retries", 2);
  declare_parameter<double>("align_timeout", 30.0);
  declare_parameter<double>("docking_timeout", 40.0);
  declare_parameter<double>("undocking_timeout", 30.0);
  declare_parameter<double>("stall_timeout", 5.0);
  declare_parameter<double>("safety_stop_timeout", 10.0);
  declare_parameter<double>("settle_time", 0.5);
  declare_parameter<double>("undock_detect_distance", 0.10);
  declare_parameter<double>("tf_timeout", 1.0);
  declare_parameter<double>("tf_stale_timeout", 0.5);
  declare_parameter<double>("control_rate", 20.0);
  declare_parameter<double>("status_rate", 2.0);
  declare_parameter<double>("clock_stall_timeout", 5.0);

  load_configuration();

  tracker_ = DockTracker(tracker_params_);
  controller_ = DockController(control_params_);
  detection_filter_ = DetectionFilter(detection_max_jump_, detection_alpha_);
  goal_clock_ = PhaseTimer(1e12, clock_stall_timeout_);
  idle_state_ = DOCK_STATE_UNDOCKED;

  group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);

  status_pub_ = create_publisher<bumperbot_msgs::msg::DockStatus>(
    "/dock/status", rclcpp::QoS(1).reliable().transient_local());
  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = group_;
  detection_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    detection_topic_, 10, std::bind(&DockManager::detection_callback, this, _1), sub_options);
  safety_sub_ = create_subscription<std_msgs::msg::Bool>(
    safety_stop_topic_, 10,
    [this](std_msgs::msg::Bool::ConstSharedPtr msg) {safety_locked_ = msg->data;}, sub_options);
  battery_sub_ = create_subscription<sensor_msgs::msg::BatteryState>(
    battery_topic_, 10,
    [this](sensor_msgs::msg::BatteryState::ConstSharedPtr msg) {
      const double percentage = msg->percentage;
      battery_percentage_ = std::isfinite(percentage) ? percentage : -1.0;
      battery_charging_ =
      msg->power_supply_status == sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
    }, sub_options);
  mission_sub_ = create_subscription<bumperbot_msgs::msg::MissionStatus>(
    mission_status_topic_, rclcpp::QoS(1).reliable().transient_local(),
    [this](bumperbot_msgs::msg::MissionStatus::ConstSharedPtr msg) {
      const bool active = is_active_mission_state(msg->state);
      std::lock_guard<std::mutex> lock(mutex_);
      // A mission and a dock goal must not drive the robot at the same time
      if (active && !mission_active_ && active_) {
        mission_started_during_goal_ = true;
      }
      mission_active_ = active;
    }, sub_options);

  nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, nav_action_name_, group_);

  action_server_ = rclcpp_action::create_server<DockRobot>(
    this, "dock_robot",
    std::bind(&DockManager::goal_callback, this, _1, _2),
    std::bind(&DockManager::cancel_callback, this, _1),
    std::bind(&DockManager::accepted_callback, this, _1),
    rcl_action_server_get_default_options(), group_);

  publish_status();
  // Wall timer: the status keeps flowing when the node clock (sim time) stalls
  status_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / status_rate_),
    std::bind(&DockManager::status_timer_callback, this), group_);

  RCLCPP_INFO(
    get_logger(),
    "Dock manager ready (action: /dock_robot, staging (%.2f, %.2f, %.2f), charger (%.2f, %.2f, "
    "%.2f))",
    staging_.x, staging_.y, staging_.yaw, dock_pose_.x, dock_pose_.y, dock_pose_.yaw);
}

DockManager::~DockManager()
{
  shutdown_goal();
}

// ----- configuration -----
double DockManager::positive(const std::string & name)
{
  const double value = get_parameter(name).as_double();
  if (!(std::isfinite(value) && value > 0.0)) {
    throw std::invalid_argument(
            "parameter '" + name + "' must be a positive number, got " + std::to_string(value));
  }
  return value;
}

void DockManager::load_configuration()
{
  std::string stations_file = get_parameter("stations_file").as_string();
  if (stations_file.empty()) {
    stations_file =
      ament_index_cpp::get_package_share_directory("bumperbot_navigation") +
      "/config/stations.yaml";
  }
  stations_ = load_stations(stations_file);

  global_frame_ = get_parameter("global_frame").as_string();
  odom_frame_ = get_parameter("odom_frame").as_string();
  robot_frame_ = get_parameter("robot_frame").as_string();
  const std::set<std::string> frames{global_frame_, odom_frame_, robot_frame_};
  if (frames.size() != 3 || frames.count("") > 0) {
    throw std::invalid_argument(
            "global_frame, odom_frame and robot_frame must be 3 distinct non-empty names, got [" +
            global_frame_ + ", " + odom_frame_ + ", " + robot_frame_ + "]");
  }
  if (stations_.frame_id != global_frame_) {
    throw std::invalid_argument(
            "stations frame_id '" + stations_.frame_id + "' differs from global_frame '" +
            global_frame_ + "'");
  }
  std::string dock_station = get_parameter("dock_station").as_string();
  if (dock_station.empty()) {
    dock_station = stations_.dock_station;
  }
  const Station * station = stations_.find(dock_station);
  if (station == nullptr) {
    throw std::invalid_argument("dock station '" + dock_station + "' is not defined");
  }

  detection_mode_ = get_parameter("detection_mode").as_string();
  if (detection_mode_ != DETECTION_NONE && detection_mode_ != DETECTION_OPTIONAL &&
    detection_mode_ != DETECTION_REQUIRED)
  {
    throw std::invalid_argument(
            "detection_mode '" + detection_mode_ + "' is invalid (none, optional or required)");
  }

  nav_action_name_ = get_parameter("nav_action_name").as_string();
  cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();
  detection_topic_ = get_parameter("detection_topic").as_string();
  safety_stop_topic_ = get_parameter("safety_stop_topic").as_string();
  battery_topic_ = get_parameter("battery_topic").as_string();
  mission_status_topic_ = get_parameter("mission_status_topic").as_string();

  const double staging_distance = positive("staging_distance");
  staging_tolerance_ = positive("staging_tolerance");
  staging_yaw_tolerance_ = positive("staging_yaw_tolerance");
  nav_server_timeout_ = positive("nav_server_timeout");
  nav_goal_timeout_ = positive("nav_goal_timeout");
  detection_timeout_ = positive("detection_timeout");
  detection_max_age_ = positive("detection_max_age");
  detection_max_jump_ = positive("detection_max_jump");
  detection_alpha_ = get_parameter("detection_filter_alpha").as_double();
  if (!(std::isfinite(detection_alpha_) && detection_alpha_ > 0.0 && detection_alpha_ <= 1.0)) {
    throw std::invalid_argument(
            "detection_filter_alpha must be in (0, 1], got " + std::to_string(detection_alpha_));
  }
  detection_offset_.x = get_parameter("detection_offset_x").as_double();
  detection_offset_.y = get_parameter("detection_offset_y").as_double();
  detection_offset_.yaw = get_parameter("detection_offset_yaw").as_double();
  if (!std::isfinite(detection_offset_.x) || !std::isfinite(detection_offset_.y) ||
    !std::isfinite(detection_offset_.yaw))
  {
    throw std::invalid_argument("detection offsets must be finite");
  }
  align_timeout_ = positive("align_timeout");
  docking_timeout_ = positive("docking_timeout");
  undocking_timeout_ = positive("undocking_timeout");
  stall_timeout_ = positive("stall_timeout");
  safety_stop_timeout_ = positive("safety_stop_timeout");
  settle_time_ = positive("settle_time");
  undock_detect_distance_ = positive("undock_detect_distance");
  tf_timeout_ = positive("tf_timeout");
  tf_stale_timeout_ = positive("tf_stale_timeout");
  control_rate_ = positive("control_rate");
  status_rate_ = positive("status_rate");
  clock_stall_timeout_ = positive("clock_stall_timeout");
  for (const char * name : {"max_retries", "nav_max_retries"}) {
    if (get_parameter(name).as_int() < 0) {
      throw std::invalid_argument(std::string("parameter '") + name + "' must not be negative");
    }
  }

  control_params_.staging_distance = staging_distance;
  control_params_.dock_backwards = get_parameter("dock_backwards").as_bool();
  control_params_.max_linear_speed = positive("max_linear_speed");
  control_params_.max_angular_speed = positive("max_angular_speed");
  control_params_.approach_speed = positive("approach_speed");
  control_params_.slow_down_distance = positive("slow_down_distance");
  control_params_.k_lateral = positive("k_lateral");
  control_params_.k_heading = positive("k_heading");
  control_params_.align_position_tolerance = positive("align_position_tolerance");
  control_params_.align_yaw_tolerance = positive("align_yaw_tolerance");
  control_params_.max_insert_lateral_error = positive("max_insert_lateral_error");
  control_params_.max_insert_yaw_error = positive("max_insert_yaw_error");
  tracker_params_.max_retries = static_cast<int>(get_parameter("max_retries").as_int());
  tracker_params_.nav_max_retries = static_cast<int>(get_parameter("nav_max_retries").as_int());
  tracker_params_.detection_mode = detection_mode_;
  tracker_params_.dock_position_tolerance = positive("dock_position_tolerance");
  tracker_params_.dock_yaw_tolerance = positive("dock_yaw_tolerance");

  station_ = *station;
  staging_ = Pose2D{station->x, station->y, station->yaw};
  dock_pose_ = compute_dock_pose(staging_, staging_distance, control_params_.dock_backwards);
}

// ----- time and TF helpers -----
double DockManager::node_now()
{
  return get_clock()->now().seconds();
}

bool DockManager::lookup(
  const std::string & parent, const std::string & child, Pose2D & pose, double & stamp)
{
  try {
    const auto transform = tf_buffer_->lookupTransform(parent, child, tf2::TimePointZero);
    const auto & t = transform.transform.translation;
    const auto & q = transform.transform.rotation;
    pose = Pose2D{t.x, t.y, quaternion_yaw(q.x, q.y, q.z, q.w)};
    stamp = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9;
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

bool DockManager::can_transform(const std::string & parent, const std::string & child)
{
  // lookupTransform without a timeout never blocks and needs no dedicated TF thread
  Pose2D pose;
  double stamp;
  return lookup(parent, child, pose, stamp);
}

bool DockManager::near_staging()
{
  Pose2D pose;
  double stamp;
  if (!lookup(global_frame_, robot_frame_, pose, stamp)) {
    return false;
  }
  return std::hypot(pose.x - staging_.x, pose.y - staging_.y) <= staging_tolerance_ &&
         std::fabs(normalize_angle(pose.yaw - staging_.yaw)) <= staging_yaw_tolerance_;
}

// ----- subscriptions -----
void DockManager::detection_callback(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  // Validate a dock detection, bring it to odom and feed the filter
  if (detection_mode_ == DETECTION_NONE) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!accepting_) {
      return;
    }
  }
  Pose2D detection;
  try {
    detection = detection_to_odom(*msg);
  } catch (const std::exception & exc) {
    RCLCPP_DEBUG(get_logger(), "Detection ignored: %s", exc.what());
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (!accepting_ || !has_estimate_) {
    return;
  }
  const auto result = detection_filter_.update(dock_estimate_, detection);
  if (result.first) {
    dock_estimate_ = result.second;
    ++detection_count_;
  }
}

Pose2D DockManager::detection_to_odom(const geometry_msgs::msg::PoseStamped & msg)
{
  const auto & p = msg.pose.position;
  const auto & q = msg.pose.orientation;
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
    !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w))
  {
    throw std::runtime_error("non-finite pose");
  }
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (norm < 1e-6) {
    throw std::runtime_error("zero quaternion");
  }
  if (msg.header.frame_id.empty()) {
    throw std::runtime_error("empty frame id");
  }
  const double stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
  if (stamp != 0.0) {
    const double age = node_now() - stamp;
    if (age > detection_max_age_) {
      throw std::runtime_error("detection too old");
    }
  }
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(
      odom_frame_, msg.header.frame_id, tf2_ros::fromMsg(msg.header.stamp));
  } catch (const std::exception &) {
    // The stamp may be outside the TF history: fall back to the latest transform
    transform = tf_buffer_->lookupTransform(odom_frame_, msg.header.frame_id, tf2::TimePointZero);
  }
  const auto & t = transform.transform.translation;
  const auto & r = transform.transform.rotation;
  const Pose3 frame_tf{t.x, t.y, t.z, r.x, r.y, r.z, r.w};
  const Pose3 pose{p.x, p.y, p.z, q.x / norm, q.y / norm, q.z / norm, q.w / norm};
  return compose(project_pose(frame_tf, pose), detection_offset_);
}

// ----- status -----
bumperbot_msgs::msg::DockStatus DockManager::build_status()
{
  bumperbot_msgs::msg::DockStatus msg;
  msg.stamp = get_clock()->now();
  const bool charging_input = battery_charging_;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (active_) {
      msg.state = tracker_.state();
      msg.attempt = tracker_.attempt();
      msg.position_error = static_cast<float>(tracker_.position_error());
      msg.yaw_error = static_cast<float>(tracker_.yaw_error());
      msg.detection_used = tracker_.detection_used() || detection_count_ > 0;
      msg.message = tracker_.message();
    } else {
      msg.state = idle_state_;
      if (docked_ && charging_input) {
        msg.state = DOCK_STATE_CHARGING;
      }
      msg.attempt = last_attempt_;
      msg.position_error = static_cast<float>(last_position_error_);
      msg.yaw_error = static_cast<float>(last_yaw_error_);
      msg.detection_used = last_detection_used_;
      msg.message = idle_message_;
    }
    msg.docked = docked_;
    msg.charging = docked_ && charging_input;
  }
  msg.battery_percentage = static_cast<float>(battery_percentage_.load());
  msg.frame_id = global_frame_;
  msg.staging_pose.x = staging_.x;
  msg.staging_pose.y = staging_.y;
  msg.staging_pose.theta = staging_.yaw;
  msg.dock_pose.x = dock_pose_.x;
  msg.dock_pose.y = dock_pose_.y;
  msg.dock_pose.theta = dock_pose_.yaw;
  return msg;
}

void DockManager::publish_status()
{
  try {
    status_pub_->publish(build_status());
  } catch (const std::exception &) {
    // The context may already be gone during shutdown
  }
}

void DockManager::publish_feedback()
{
  std::shared_ptr<DockGoalHandle> handle;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    handle = goal_handle_;
  }
  if (!handle) {
    return;
  }
  const auto status = build_status();
  auto feedback = std::make_shared<DockRobot::Feedback>();
  feedback->state = status.state;
  feedback->attempt = status.attempt;
  feedback->distance_to_dock = static_cast<float>(live_distance_.load());
  feedback->yaw_error = static_cast<float>(live_yaw_error_.load());
  feedback->detection_used = status.detection_used;
  try {
    handle->publish_feedback(feedback);
  } catch (const std::exception &) {
    // The goal may already be terminal
  }
}

void DockManager::publish_all()
{
  publish_status();
  publish_feedback();
}

void DockManager::status_timer_callback()
{
  update_docked_watch();
  publish_status();
  bool active;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    active = active_;
  }
  if (active) {
    publish_feedback();
  }
}

void DockManager::update_docked_watch()
{
  // Clear the docked flag when the robot was driven away from the charger
  Pose2D reference;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (active_ || !docked_ || !has_docked_robot_odom_) {
      return;
    }
    reference = docked_robot_odom_;
  }
  Pose2D pose;
  double stamp;
  if (!lookup(odom_frame_, robot_frame_, pose, stamp)) {
    return;
  }
  if (std::hypot(pose.x - reference.x, pose.y - reference.y) > undock_detect_distance_) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_ && docked_) {
      docked_ = false;
      idle_state_ = DOCK_STATE_UNDOCKED;
      idle_message_ = "the robot left the dock";
      RCLCPP_INFO(get_logger(), "The robot left the dock");
    }
  }
}

// ----- action server callbacks -----
rclcpp_action::GoalResponse DockManager::reject(const std::string & reason)
{
  RCLCPP_WARN(get_logger(), "Dock goal rejected: %s", reason.c_str());
  return rclcpp_action::GoalResponse::REJECT;
}

rclcpp_action::GoalResponse DockManager::goal_callback(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const DockRobot::Goal> goal)
{
  const int command = goal->command;
  if (command != COMMAND_DOCK && command != COMMAND_UNDOCK) {
    return reject("unknown command " + std::to_string(command));
  }
  if (shutdown_) {
    return reject("the node is shutting down");
  }
  update_docked_watch();
  std::lock_guard<std::mutex> lock(mutex_);
  if (active_) {
    return reject("another dock goal is active");
  }
  const bool docked = docked_;
  if (command == COMMAND_UNDOCK && !docked) {
    return reject("UNDOCK while the robot is not docked");
  }
  if (command == COMMAND_DOCK && mission_running()) {
    return reject("DOCK while a mission is active");
  }
  if (!can_transform(global_frame_, robot_frame_)) {
    return reject("TF " + global_frame_ + " -> " + robot_frame_ + " is not available");
  }
  if (!can_transform(odom_frame_, robot_frame_)) {
    return reject("TF " + odom_frame_ + " -> " + robot_frame_ + " is not available");
  }
  if (command == COMMAND_DOCK && !docked && !near_staging() &&
    !nav_client_->action_server_is_ready())
  {
    return reject("the Nav2 action server is not ready");
  }
  active_ = true;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DockManager::cancel_callback(std::shared_ptr<DockGoalHandle>)
{
  // The worker polls is_canceling(), which also covers a cancel right after acceptance
  RCLCPP_INFO(get_logger(), "Dock cancel requested");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void DockManager::accepted_callback(std::shared_ptr<DockGoalHandle> goal_handle)
{
  // Run the goal in its own joinable thread so the executor stays free. The previous worker (if
  // any) has already released active_, so joining it here only waits for its last lines.
  if (worker_.joinable()) {
    worker_.join();
  }
  if (shutdown_) {
    std::lock_guard<std::mutex> lock(mutex_);
    active_ = false;
    auto result = std::make_shared<DockRobot::Result>();
    result->outcome = DOCK_OUTCOME_ERROR;
    result->message = "shutdown";
    result->position_error = -1.0F;
    result->yaw_error = -1.0F;
    goal_handle->abort(result);
    return;
  }
  worker_ = std::thread(std::bind(&DockManager::worker_main, this, goal_handle));
}

// ----- worker -----
void DockManager::worker_main(std::shared_ptr<DockGoalHandle> goal_handle)
{
  try {
    run_goal(goal_handle);
  } catch (const std::exception & exc) {
    RCLCPP_ERROR(get_logger(), "Dock goal error: %s", exc.what());
    std::lock_guard<std::mutex> lock(mutex_);
    tracker_.abort(std::string("internal error: ") + exc.what());
  } catch (...) {
    RCLCPP_ERROR(get_logger(), "Dock goal error: unknown");
    std::lock_guard<std::mutex> lock(mutex_);
    tracker_.abort("internal error: unknown error");
  }
  // Whatever happened above, the cleanup runs (like a finally)
  finish_goal(goal_handle);
}

void DockManager::finish_goal(const std::shared_ptr<DockGoalHandle> & goal_handle)
{
  // Stop the robot, build the result, reset the node state and end the goal once
  auto result = std::make_shared<DockRobot::Result>();
  result->outcome = DOCK_OUTCOME_ERROR;
  result->message = "internal error";
  result->position_error = -1.0F;
  result->yaw_error = -1.0F;
  try {
    for (int i = 0; i < 3; ++i) {
      publish_cmd(0.0, 0.0);
      sleep_seconds(0.01);
    }
    cancel_nav_goal(2.0);
    std::lock_guard<std::mutex> lock(mutex_);
    if (!tracker_.finished()) {
      tracker_.abort("internal error: goal ended unexpectedly");
    }
    result->outcome = static_cast<uint8_t>(tracker_.outcome());
    result->docked = tracker_.docked();
    result->attempts = tracker_.attempt();
    result->position_error = static_cast<float>(tracker_.position_error());
    result->yaw_error = static_cast<float>(tracker_.yaw_error());
    result->message = tracker_.message();
    docked_ = tracker_.docked();
    idle_state_ = tracker_.state();
    idle_message_ = tracker_.message();
    last_attempt_ = tracker_.attempt();
    last_position_error_ = tracker_.position_error();
    last_yaw_error_ = tracker_.yaw_error();
    last_detection_used_ = tracker_.detection_used() || detection_count_ > 0;
    if (tracker_.docked()) {
      if (tracker_.outcome() == DOCK_OUTCOME_DOCKED) {
        saved_dock_estimate_ = dock_estimate_;
        has_saved_estimate_ = has_estimate_;
        docked_robot_odom_ = last_robot_odom_;
        has_docked_robot_odom_ = has_robot_odom_;
      }
    } else {
      has_docked_robot_odom_ = false;
    }
    accepting_ = false;
  } catch (const std::exception & exc) {
    RCLCPP_ERROR(get_logger(), "Could not build the dock result: %s", exc.what());
  }
  {
    // From here on the node accepts the next goal; only locals are used below
    std::lock_guard<std::mutex> lock(mutex_);
    goal_handle_.reset();
    nav_handle_.reset();
    active_ = false;
  }
  publish_status();
  try {
    // A cancel after the terminal decision is ignored: a completed dock stays SUCCEEDED
    // (rcl_action allows CANCELING -> SUCCEEDED). Otherwise a cancel in progress wins.
    if (result->outcome == DOCK_OUTCOME_DOCKED || result->outcome == DOCK_OUTCOME_UNDOCKED ||
      result->outcome == DOCK_OUTCOME_ALREADY_DOCKED)
    {
      goal_handle->succeed(result);
    } else if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
    } else {
      goal_handle->abort(result);
    }
  } catch (const std::exception & exc) {
    RCLCPP_ERROR(get_logger(), "Could not report the dock result: %s", exc.what());
  }
  RCLCPP_INFO(
    get_logger(), "Dock goal finished: outcome %d (%s)", static_cast<int>(result->outcome),
    result->message.c_str());
}

void DockManager::run_goal(const std::shared_ptr<DockGoalHandle> & goal_handle)
{
  const int command = goal_handle->get_goal()->command;
  RCLCPP_INFO(get_logger(), "Dock goal started: %s", command == COMMAND_DOCK ? "DOCK" : "UNDOCK");
  update_docked_watch();
  const bool near = command == COMMAND_DOCK && near_staging();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    goal_handle_ = goal_handle;
    mission_started_during_goal_ = false;
    has_estimate_ = false;
    accepting_ = false;
    detection_count_ = 0;
    nav_handle_.reset();
    live_distance_ = -1.0;
    live_yaw_error_ = -1.0;
    tf_failing_ = false;
    has_robot_odom_ = false;
    controller_.reset();
    goal_clock_ = PhaseTimer(1e12, clock_stall_timeout_);
    goal_clock_.start(node_now(), wall_now());
    next_tick_time_ = wall_now();
    tracker_.start(command, docked_, near);
  }
  publish_all();

  std::string last_state;
  int last_attempt = -1;
  while (true) {
    check_interrupts();
    DockStep step;
    std::string state;
    int attempt;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      step = tracker_.next_step();
      state = tracker_.state();
      attempt = tracker_.attempt();
    }
    if (state != last_state || attempt != last_attempt) {
      last_state = state;
      last_attempt = attempt;
      publish_all();
    }
    if (step.kind == DockStepKind::FINISHED) {
      break;
    }
    switch (step.kind) {
      case DockStepKind::NAVIGATE_STAGING: run_navigate(); break;
      case DockStepKind::WAIT_DETECTION: run_wait_detection(); break;
      case DockStepKind::ALIGN: run_align(); break;
      case DockStepKind::INSERT: run_insert(); break;
      case DockStepKind::SETTLE_VERIFY: run_verify(); break;
      case DockStepKind::UNDOCK: run_undock(); break;
      case DockStepKind::FINISHED: break;
    }
  }
  publish_all();
}

// ----- interrupts and ticks -----
bool DockManager::check_interrupts()
{
  // Apply shutdown / cancel / clock stall to the tracker. Returns true when it ended.
  bool ended;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (tracker_.finished()) {
      return true;
    }
    if (shutdown_) {
      tracker_.abort("shutdown");
    } else if (mission_started_during_goal_) {
      tracker_.abort("a mission became active during the dock goal");
    } else if (goal_clock_.expired(node_now(), wall_now()) && goal_clock_.stalled()) {
      tracker_.abort("the node clock stalled");
    } else if (goal_handle_ && goal_handle_->is_canceling()) {
      tracker_.request_cancel();
    }
    ended = tracker_.finished();
  }
  if (ended) {
    publish_cmd(0.0, 0.0);
    publish_all();
  }
  return ended;
}

bool DockManager::tick()
{
  // Sleep until the next control period. Returns false when the goal ended.
  const double period = 1.0 / control_rate_;
  const double now = wall_now();
  next_tick_time_ += period;
  if (next_tick_time_ < now - period) {
    next_tick_time_ = now;
  }
  const double delay = next_tick_time_ - now;
  if (delay > 0.0) {
    sleep_seconds(delay);
  }
  return !check_interrupts();
}

bool DockManager::wait_interruptible(double seconds)
{
  // Wall wait that returns false as soon as the goal ended
  const double end = wall_now() + seconds;
  while (wall_now() < end) {
    sleep_seconds(0.02);
    if (check_interrupts()) {
      return false;
    }
  }
  return true;
}

bool DockManager::odom_pose(Pose2D & pose)
{
  // Robot pose in odom. Returns false (robot stopped) when the TF is stale or lost.
  double stamp = 0.0;
  double age = 0.0;
  bool found = lookup(odom_frame_, robot_frame_, pose, stamp);
  if (found) {
    age = std::max(0.0, node_now() - stamp);
    tf_failing_ = false;
  } else {
    if (!tf_failing_) {
      tf_failing_ = true;
      tf_fail_since_ = wall_now();
    } else if (wall_now() - tf_fail_since_ > tf_timeout_) {
      age = tf_timeout_ + 1.0;
    }
  }
  if (!found || age > tf_stale_timeout_) {
    publish_cmd(0.0, 0.0);
    if (age > tf_timeout_) {
      RCLCPP_ERROR(
        get_logger(), "The %s -> %s TF is lost", odom_frame_.c_str(), robot_frame_.c_str());
      {
        std::lock_guard<std::mutex> lock(mutex_);
        tracker_.abort("the odom TF is lost");
      }
      publish_all();
    }
    return false;
  }
  last_robot_odom_ = pose;
  has_robot_odom_ = true;
  return true;
}

// ----- command output -----
void DockManager::publish_cmd(double v, double w)
{
  geometry_msgs::msg::Twist msg;
  msg.linear.x = std::max(
    -control_params_.max_linear_speed, std::min(control_params_.max_linear_speed, v));
  msg.angular.z = std::max(
    -control_params_.max_angular_speed, std::min(control_params_.max_angular_speed, w));
  try {
    cmd_pub_->publish(msg);
  } catch (const std::exception &) {
    // The context may already be gone during shutdown
  }
}

// ----- dock estimate -----
bool DockManager::begin_approach(bool use_saved)
{
  // Set the charger pose estimate in odom (from the map or the saved docking)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (has_estimate_) {
      return true;
    }
    if (use_saved && has_saved_estimate_) {
      dock_estimate_ = saved_dock_estimate_;
      has_estimate_ = true;
      return true;
    }
  }
  Pose2D map_to_odom;
  double stamp;
  if (!lookup(global_frame_, odom_frame_, map_to_odom, stamp)) {
    RCLCPP_ERROR(
      get_logger(), "The %s -> %s TF is not available", global_frame_.c_str(),
      odom_frame_.c_str());
    std::lock_guard<std::mutex> lock(mutex_);
    tracker_.abort("the map to odom TF is not available");
    return false;
  }
  // Snapshot of map -> odom: the charger pose in odom stays fixed for the final approach
  const Pose2D estimate = compose(inverse(map_to_odom), dock_pose_);
  std::lock_guard<std::mutex> lock(mutex_);
  dock_estimate_ = estimate;
  has_estimate_ = true;
  accepting_ = detection_mode_ != DETECTION_NONE && !use_saved;
  return true;
}

Pose2D DockManager::robot_in_dock_frame(const Pose2D & odom)
{
  Pose2D estimate;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    estimate = dock_estimate_;
  }
  return relative(estimate, odom);
}

void DockManager::update_live(const Pose2D & p)
{
  const auto error = DockController::pose_error(p);
  live_distance_ = error.first;
  live_yaw_error_ = error.second;
}

// ----- Nav2 -----
DockManager::NavigateToPose::Goal DockManager::make_goal()
{
  NavigateToPose::Goal goal;
  goal.pose.header.frame_id = global_frame_;
  goal.pose.header.stamp = get_clock()->now();
  goal.pose.pose.position.x = staging_.x;
  goal.pose.pose.position.y = staging_.y;
  const auto q = yaw_to_quaternion(staging_.yaw);
  goal.pose.pose.orientation.z = q.first;
  goal.pose.pose.orientation.w = q.second;
  return goal;
}

void DockManager::cancel_nav_goal(double wait_seconds)
{
  // Best-effort cancel of the active Nav2 goal; never throws
  std::shared_ptr<NavGoalHandle> handle;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    handle = nav_handle_;
    nav_handle_.reset();
  }
  if (!handle) {
    return;
  }
  try {
    auto cancel_future = nav_client_->async_cancel_goal(handle);
    cancel_future.wait_for(std::chrono::duration<double>(wait_seconds));
  } catch (const std::exception & exc) {
    // e.g. rclcpp_action::exceptions::UnknownGoalHandleError when the goal already ended
    RCLCPP_WARN(get_logger(), "Could not cancel the Nav2 goal: %s", exc.what());
  } catch (...) {
    RCLCPP_WARN(get_logger(), "Could not cancel the Nav2 goal");
  }
}

bool DockManager::mission_running()
{
  // A latched mission status without a publisher is stale
  return mission_active_ && count_publishers(mission_status_topic_) > 0;
}

void DockManager::run_navigate()
{
  const int result = navigate();
  if (result < 0) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  tracker_.report_nav(result == 1);
}

int DockManager::navigate()
{
  int attempt;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    attempt = tracker_.attempt();
  }
  if (attempt > 1 && !wait_interruptible(NAV_RETRY_PAUSE)) {
    return -1;
  }
  RCLCPP_INFO(get_logger(), "Navigating to the staging pose (attempt %d)", attempt);

  bool rejected_before = false;
  double rejected_since = 0.0;
  std::shared_ptr<NavGoalHandle> handle;
  while (true) {
    const double server_deadline = wall_now() + nav_server_timeout_;
    while (!nav_client_->action_server_is_ready()) {
      if (wall_now() >= server_deadline) {
        RCLCPP_WARN(get_logger(), "Nav2 action server is not available");
        return 0;
      }
      if (!wait_interruptible(0.1)) {
        return -1;
      }
    }

    // Shared with the goal response callback: a goal accepted after the attempt was abandoned
    // is canceled exactly once (by the callback, or here when the answer came first)
    struct Request
    {
      std::mutex mutex;
      bool abandoned = false;
      std::shared_ptr<NavGoalHandle> handle;
    };
    auto request = std::make_shared<Request>();
    auto cancel_late = [this](const std::shared_ptr<NavGoalHandle> & late) {
        if (!late) {
          return;
        }
        try {
          nav_client_->async_cancel_goal(late);
        } catch (const std::exception & exc) {
          RCLCPP_WARN(get_logger(), "Could not cancel the Nav2 goal: %s", exc.what());
        }
      };
    auto abandon = [request, cancel_late](bool wait_for_answer, auto & future) {
        std::shared_ptr<NavGoalHandle> answered;
        {
          std::lock_guard<std::mutex> lock(request->mutex);
          request->abandoned = true;
          answered = request->handle;
        }
        if (answered) {
          cancel_late(answered);
        } else if (wait_for_answer) {
          // Bounded wait so that the late cancel is sent before the node shuts down
          future.wait_for(1s);
        }
      };
    rclcpp_action::Client<NavigateToPose>::SendGoalOptions send_options;
    send_options.goal_response_callback =
      [request, cancel_late](std::shared_ptr<NavGoalHandle> response) {
        bool abandoned;
        {
          std::lock_guard<std::mutex> lock(request->mutex);
          request->handle = response;
          abandoned = request->abandoned;
        }
        if (abandoned) {
          // The attempt was given up while the request was in flight: undo the goal
          cancel_late(response);
        }
      };
    auto send_future = nav_client_->async_send_goal(make_goal(), send_options);
    const double answer_deadline = wall_now() + nav_server_timeout_;
    while (send_future.wait_for(50ms) != std::future_status::ready) {
      if (check_interrupts()) {
        abandon(true, send_future);
        return -1;
      }
      if (wall_now() >= answer_deadline) {
        RCLCPP_WARN(get_logger(), "Nav2 did not answer the goal request");
        abandon(false, send_future);
        return 0;
      }
    }
    handle = send_future.get();
    if (handle) {
      break;
    }
    // A rejection means the Nav2 lifecycle is probably not active yet: re-send without counting
    // an attempt until nav_server_timeout of continuous rejection has passed
    if (!rejected_before) {
      rejected_before = true;
      rejected_since = wall_now();
    }
    if (wall_now() - rejected_since >= nav_server_timeout_) {
      RCLCPP_WARN(get_logger(), "Nav2 rejected the staging goal");
      return 0;
    }
    if (!wait_interruptible(0.5)) {
      return -1;
    }
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    nav_handle_ = handle;
  }
  auto result_future = nav_client_->async_get_result(handle);
  const double goal_deadline = wall_now() + nav_goal_timeout_;
  bool server_down = false;
  double server_down_since = 0.0;
  while (result_future.wait_for(50ms) != std::future_status::ready) {
    if (check_interrupts()) {
      cancel_nav_goal(2.0);
      return -1;
    }
    if (wall_now() >= goal_deadline) {
      RCLCPP_WARN(get_logger(), "Navigation to the staging pose timed out");
      cancel_nav_goal(5.0);
      return 0;
    }
    if (nav_client_->action_server_is_ready()) {
      server_down = false;
    } else if (!server_down) {
      server_down = true;
      server_down_since = wall_now();
    } else if (wall_now() - server_down_since >= nav_server_timeout_) {
      RCLCPP_ERROR(get_logger(), "Nav2 action server disappeared during the navigation");
      std::lock_guard<std::mutex> lock(mutex_);
      nav_handle_.reset();
      tracker_.abort("the Nav2 action server disappeared");
      return -1;
    }
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    nav_handle_.reset();
  }
  return result_future.get().code == rclcpp_action::ResultCode::SUCCEEDED ? 1 : 0;
}

// ----- phases -----
void DockManager::run_wait_detection()
{
  if (!begin_approach(false)) {
    return;
  }
  PhaseTimer timer(detection_timeout_, clock_stall_timeout_);
  timer.start(node_now(), wall_now());
  RCLCPP_INFO(get_logger(), "Waiting for a dock detection");
  int count = 0;
  while (true) {
    if (!tick()) {
      return;
    }
    publish_cmd(0.0, 0.0);
    Pose2D odom;
    if (!odom_pose(odom) && check_interrupts()) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      count = detection_count_;
    }
    if (count >= MIN_DETECTIONS) {
      break;
    }
    if (timer.expired(node_now(), wall_now())) {
      break;
    }
  }
  std::lock_guard<std::mutex> lock(mutex_);
  tracker_.report_detection(count >= 1);
}

void DockManager::run_align()
{
  if (!begin_approach(false)) {
    return;
  }
  controller_.reset();
  PhaseResult result;
  if (control_phase(
      align_timeout_, [this](const Pose2D & p) {return controller_.align_command(p);}, result))
  {
    std::lock_guard<std::mutex> lock(mutex_);
    tracker_.report_phase(result);
  }
}

void DockManager::run_insert()
{
  PhaseResult result;
  if (control_phase(
      docking_timeout_, [this](const Pose2D & p) {return controller_.insert_command(p);}, result))
  {
    std::lock_guard<std::mutex> lock(mutex_);
    tracker_.report_phase(result);
  }
}

void DockManager::run_undock()
{
  if (!begin_approach(true)) {
    return;
  }
  PhaseResult result;
  if (control_phase(
      undocking_timeout_, [this](const Pose2D & p) {return controller_.undock_command(p);},
      result))
  {
    std::lock_guard<std::mutex> lock(mutex_);
    tracker_.report_phase(result);
  }
}

bool DockManager::control_phase(
  double timeout, const std::function<Command(const Pose2D &)> & compute, PhaseResult & result)
{
  // Closed-loop control phase at control_rate. Returns false when the goal ended (cancel, abort)
  // meanwhile. The robot is stopped before every return.
  PhaseTimer timer(timeout, clock_stall_timeout_);
  timer.start(node_now(), wall_now());
  double locked_seconds = 0.0;
  double last_wall = wall_now();
  bool has_ref = false;
  Pose2D stall_ref;
  double stall_since = 0.0;
  while (true) {
    if (!tick()) {
      return false;
    }
    const double wall = wall_now();
    const double dt = wall - last_wall;
    last_wall = wall;
    Pose2D odom;
    if (!odom_pose(odom)) {
      has_ref = false;
      if (check_interrupts()) {
        return false;
      }
      continue;
    }
    const Pose2D p = robot_in_dock_frame(odom);
    update_live(p);
    const Command cmd = compute(p);
    if (cmd.done) {
      publish_cmd(0.0, 0.0);
      result = PhaseResult::OK;
      return true;
    }
    if (cmd.failed) {
      publish_cmd(0.0, 0.0);
      result = PhaseResult::MISALIGNED;
      return true;
    }

    if (safety_locked_) {
      locked_seconds += dt;
      publish_cmd(0.0, 0.0);
      has_ref = false;
      if (locked_seconds >= safety_stop_timeout_) {
        RCLCPP_WARN(get_logger(), "safety_stop held too long, attempt failed");
        result = PhaseResult::SAFETY_TIMEOUT;
        return true;
      }
    } else {
      publish_cmd(cmd.v, cmd.w);
      if (cmd.v == 0.0 && cmd.w == 0.0) {
        has_ref = false;
      } else if (!has_ref ||
        std::hypot(odom.x - stall_ref.x, odom.y - stall_ref.y) >= 0.01 ||
        std::fabs(normalize_angle(odom.yaw - stall_ref.yaw)) >= 0.02)
      {
        has_ref = true;
        stall_ref = odom;
        stall_since = wall;
      } else if (wall - stall_since >= stall_timeout_) {
        publish_cmd(0.0, 0.0);
        RCLCPP_WARN(get_logger(), "The robot does not move, attempt failed");
        result = PhaseResult::STALL;
        return true;
      }
    }

    if (timer.expired(node_now(), wall)) {
      publish_cmd(0.0, 0.0);
      if (timer.stalled()) {
        std::lock_guard<std::mutex> lock(mutex_);
        tracker_.abort("the node clock stalled");
        return false;
      }
      result = PhaseResult::TIMEOUT;
      return true;
    }
  }
}

void DockManager::run_verify()
{
  PhaseTimer timer(settle_time_, clock_stall_timeout_);
  timer.start(node_now(), wall_now());
  Pose2D odom;
  while (true) {
    if (!tick()) {
      return;
    }
    publish_cmd(0.0, 0.0);
    if (!odom_pose(odom)) {
      if (check_interrupts()) {
        return;
      }
      continue;
    }
    if (timer.expired(node_now(), wall_now())) {
      if (timer.stalled()) {
        std::lock_guard<std::mutex> lock(mutex_);
        tracker_.abort("the node clock stalled");
        return;
      }
      break;
    }
  }
  const Pose2D p = robot_in_dock_frame(odom);
  const auto error = DockController::pose_error(p);
  update_live(p);
  std::lock_guard<std::mutex> lock(mutex_);
  tracker_.report_verify(error.first, error.second);
}

// ----- shutdown -----
void DockManager::shutdown_goal()
{
  shutdown_ = true;
  const double deadline = wall_now() + 8.0;
  while (wall_now() < deadline) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!active_) {
        break;
      }
    }
    sleep_seconds(0.05);
  }
  if (worker_.joinable() && worker_.get_id() != std::this_thread::get_id()) {
    worker_.join();
  }
  // Let the executor deliver the final result to the client
  sleep_seconds(0.3);
  publish_cmd(0.0, 0.0);
}

}  // namespace bumperbot_navigation

namespace
{
std::atomic<bool> g_stop_requested{false};

void on_signal(int)
{
  g_stop_requested = true;
}
}  // namespace

int main(int argc, char ** argv)
{
  // Own signal handling: the context must stay valid while the worker stops the robot and
  // cancels the active Nav2 goal, so rclcpp's default shutdown-on-signal is disabled.
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
  int status = 0;
  try {
    auto node = std::make_shared<bumperbot_navigation::DockManager>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 6);
    executor.add_node(node);
    std::thread spin_thread([&executor]() {executor.spin();});
    while (!g_stop_requested && rclcpp::ok()) {
      std::this_thread::sleep_for(50ms);
    }
    // Stop the robot and end the goal while the executor still processes callbacks
    node->shutdown_goal();
    executor.cancel();
    spin_thread.join();
  } catch (const std::exception & exc) {
    RCLCPP_FATAL(rclcpp::get_logger("dock_manager"), "%s", exc.what());
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}
