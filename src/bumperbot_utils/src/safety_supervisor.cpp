#include "bumperbot_utils/safety_supervisor.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <functional>
#include <limits>
#include <thread>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

namespace bumperbot_utils
{

namespace
{
const double kNaN = std::numeric_limits<double>::quiet_NaN();
const char * const kFallbackLockTopic = "safety_stop";

std::string format_ratio(double ratio)
{
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.2f", ratio);
  return buffer;
}
}  // namespace

SafetySupervisor::SafetySupervisor(const rclcpp::NodeOptions & options)
: Node("safety_supervisor", options)
{
  try {
    setup();
  } catch (const StartupError &) {
    throw;
  } catch (const std::exception & exc) {
    fail_safe(std::string("startup failed: ") + exc.what());
  }
  started_ = true;
}

template<typename T>
T SafetySupervisor::declare_checked(const std::string & name, const T & default_value)
{
  try {
    return declare_parameter<T>(name, default_value);
  } catch (const std::exception & exc) {
    throw std::runtime_error("parameter '" + name + "': " + exc.what());
  }
}

safety::Config SafetySupervisor::load_config()
{
  safety::Config cfg;
  cfg.enforcement = declare_checked<std::string>("enforcement", cfg.enforcement);
  cfg.robot_frame = declare_checked<std::string>("robot_frame", cfg.robot_frame);
  cfg.scan_topic = declare_checked<std::string>("scan_topic", cfg.scan_topic);
  cfg.odom_topic = declare_checked<std::string>("odom_topic", cfg.odom_topic);
  cfg.cmd_vel_in_topic = declare_checked<std::string>("cmd_vel_in_topic", cfg.cmd_vel_in_topic);
  cfg.cmd_vel_out_topic =
    declare_checked<std::string>("cmd_vel_out_topic", cfg.cmd_vel_out_topic);
  cfg.lock_topic = declare_checked<std::string>("lock_topic", cfg.lock_topic);
  cfg.status_topic = declare_checked<std::string>("status_topic", cfg.status_topic);
  cfg.estop_topic = declare_checked<std::string>("estop_topic", cfg.estop_topic);
  cfg.dock_status_topic =
    declare_checked<std::string>("dock_status_topic", cfg.dock_status_topic);
  cfg.use_odom = declare_checked<bool>("use_odom", cfg.use_odom);
  cfg.latch_faults = declare_checked<bool>("latch_faults", cfg.latch_faults);
  cfg.latch_on_contact = declare_checked<bool>("latch_on_contact", cfg.latch_on_contact);
  cfg.require_reset_after_restart =
    declare_checked<bool>("require_reset_after_restart", cfg.require_reset_after_restart);
  cfg.contact_debounce_scans =
    declare_checked<int64_t>("contact_debounce_scans", cfg.contact_debounce_scans);
  cfg.dock_approach_enabled =
    declare_checked<bool>("dock_approach_enabled", cfg.dock_approach_enabled);
  cfg.footprint = declare_checked<std::vector<double>>("footprint", cfg.footprint);
  cfg.lateral_margin = declare_checked<double>("lateral_margin", cfg.lateral_margin);
  cfg.stop_margin = declare_checked<double>("stop_margin", cfg.stop_margin);
  cfg.stop_release_hysteresis =
    declare_checked<double>("stop_release_hysteresis", cfg.stop_release_hysteresis);
  cfg.slowdown_distance = declare_checked<double>("slowdown_distance", cfg.slowdown_distance);
  cfg.contact_margin = declare_checked<double>("contact_margin", cfg.contact_margin);
  cfg.max_linear_speed = declare_checked<double>("max_linear_speed", cfg.max_linear_speed);
  cfg.max_angular_speed = declare_checked<double>("max_angular_speed", cfg.max_angular_speed);
  cfg.response_time = declare_checked<double>("response_time", cfg.response_time);
  cfg.max_decel = declare_checked<double>("max_decel", cfg.max_decel);
  cfg.stopped_speed = declare_checked<double>("stopped_speed", cfg.stopped_speed);
  cfg.brake_hold_time = declare_checked<double>("brake_hold_time", cfg.brake_hold_time);
  cfg.scan_timeout = declare_checked<double>("scan_timeout", cfg.scan_timeout);
  cfg.odom_timeout = declare_checked<double>("odom_timeout", cfg.odom_timeout);
  cfg.cmd_timeout = declare_checked<double>("cmd_timeout", cfg.cmd_timeout);
  cfg.recovery_time = declare_checked<double>("recovery_time", cfg.recovery_time);
  cfg.max_invalid_beam_ratio =
    declare_checked<double>("max_invalid_beam_ratio", cfg.max_invalid_beam_ratio);
  cfg.rate = declare_checked<double>("rate", cfg.rate);
  cfg.status_rate = declare_checked<double>("status_rate", cfg.status_rate);
  cfg.dock_max_speed = declare_checked<double>("dock_max_speed", cfg.dock_max_speed);
  cfg.dock_stop_margin = declare_checked<double>("dock_stop_margin", cfg.dock_stop_margin);
  cfg.dock_approach_radius =
    declare_checked<double>("dock_approach_radius", cfg.dock_approach_radius);
  cfg.dock_approach_max_time =
    declare_checked<double>("dock_approach_max_time", cfg.dock_approach_max_time);
  cfg.dock_status_timeout =
    declare_checked<double>("dock_status_timeout", cfg.dock_status_timeout);
  cfg.debug_inject_tick_exceptions =
    declare_checked<int64_t>("debug_inject_tick_exceptions", cfg.debug_inject_tick_exceptions);
  return cfg;
}

void SafetySupervisor::fail_safe(const std::string & reason)
{
  // Publish the lock (true) on a startup error, log FATAL and abort the startup
  std::string topic = kFallbackLockTopic;
  try {
    const std::string value = get_parameter("lock_topic").as_string();
    if (!value.empty()) {
      topic = value;
    }
  } catch (const std::exception &) {
    // Parameter missing, undeclared or of another type: use the fallback topic
  }
  try {
    auto lock_pub = create_publisher<std_msgs::msg::Bool>(topic, 10);
    std::this_thread::sleep_for(200ms);
    for (int i = 0; i < 3; ++i) {
      std_msgs::msg::Bool msg;
      msg.data = true;
      lock_pub->publish(msg);
      std::this_thread::sleep_for(100ms);
    }
  } catch (const std::exception & exc) {
    RCLCPP_ERROR(get_logger(), "could not publish the lock: %s", exc.what());
  }
  RCLCPP_FATAL(get_logger(), "safety_supervisor cannot start: %s", reason.c_str());
  throw StartupError(reason);
}

void SafetySupervisor::setup()
{
  try {
    cfg_ = load_config();
  } catch (const std::runtime_error & exc) {
    fail_safe(exc.what());
  }
  const std::string error = safety::validate_config(cfg_);
  if (!error.empty()) {
    fail_safe("invalid configuration: " + error);
  }
  filter_mode_ = cfg_.enforcement == safety::ENFORCEMENT_FILTER;
  core_ = std::make_unique<safety::SafetyCore>(cfg_);
  dock_gate_ = std::make_unique<safety::DockGate>(cfg_);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  lock_pub_ = create_publisher<std_msgs::msg::Bool>(cfg_.lock_topic, 10);
  status_pub_ = create_publisher<bumperbot_msgs::msg::SafetyStatus>(
    cfg_.status_topic, rclcpp::QoS(1).reliable().transient_local());
  if (filter_mode_) {
    out_pub_ = create_publisher<geometry_msgs::msg::Twist>(cfg_.cmd_vel_out_topic, 10);
  }

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    cfg_.scan_topic, rclcpp::SensorDataQoS().keep_last(5),
    std::bind(&SafetySupervisor::on_scan, this, _1));
  if (cfg_.use_odom) {
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      cfg_.odom_topic, 10, std::bind(&SafetySupervisor::on_odom, this, _1));
  }
  if (filter_mode_) {
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cfg_.cmd_vel_in_topic, 10, std::bind(&SafetySupervisor::on_cmd, this, _1));
  } else {
    // lock_only: the command is only observed (direction of travel)
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cfg_.cmd_vel_out_topic, 10, std::bind(&SafetySupervisor::on_observed_cmd, this, _1));
  }
  // Two subscriptions: volatile (any publisher) and transient_local (replays the last value of
  // a latched publisher to a restarted supervisor)
  estop_sub_ = create_subscription<std_msgs::msg::Bool>(
    cfg_.estop_topic, 10, std::bind(&SafetySupervisor::on_estop, this, _1));
  estop_latched_sub_ = create_subscription<std_msgs::msg::Bool>(
    cfg_.estop_topic, rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&SafetySupervisor::on_estop, this, _1));
  dock_sub_ = create_subscription<bumperbot_msgs::msg::DockStatus>(
    cfg_.dock_status_topic, rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&SafetySupervisor::on_dock, this, _1));

  reset_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/reset", std::bind(&SafetySupervisor::on_reset, this, _1, _2));
  estop_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/trigger_estop", std::bind(&SafetySupervisor::on_trigger_estop, this, _1, _2));

  // Wall timer: keeps running when the ROS time source stalls
  tick_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / cfg_.rate)),
    std::bind(&SafetySupervisor::tick, this));
  RCLCPP_INFO(
    get_logger(), "safety_supervisor started (C++), enforcement=%s, scan=%s, lock=%s",
    cfg_.enforcement.c_str(), cfg_.scan_topic.c_str(), cfg_.lock_topic.c_str());
}

double SafetySupervisor::now_seconds() const
{
  return std::chrono::duration<double>(SteadyClock::now().time_since_epoch()).count();
}

void SafetySupervisor::note_exception(const std::string & where, const std::string & what)
{
  pending_exception_ = where + ": " + what;
  RCLCPP_ERROR_THROTTLE(
    get_logger(), *get_clock(), 2000, "exception in %s: %s", where.c_str(), what.c_str());
}

// ---------------------------------------------------------------------------------------------
// Callbacks (store only)
// ---------------------------------------------------------------------------------------------

void SafetySupervisor::on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  try {
    handle_scan(*msg);
    if (scan_pending_) {
      try_evaluate();
    }
  } catch (const std::exception & exc) {
    note_exception("scan callback", exc.what());
  } catch (...) {
    note_exception("scan callback", "unknown error");
  }
}

void SafetySupervisor::handle_scan(const sensor_msgs::msg::LaserScan & msg)
{
  scan_time_ = now_seconds();
  has_scan_ = true;
  scan_ok_ = false;
  scan_pending_.reset();
  const std::string & frame = msg.header.frame_id;
  if (frame.empty()) {
    scan_invalid_ = "invalid scan: empty frame_id";
    return;
  }
  if (!has_scan_frame_) {
    scan_frame_ = frame;
    has_scan_frame_ = true;
  } else if (frame != scan_frame_) {
    scan_invalid_ = "scan frame changed";
    return;
  }
  if (msg.ranges.empty()) {
    scan_invalid_ = "invalid scan: empty ranges";
    return;
  }
  if (!(std::isfinite(msg.angle_increment) && msg.angle_increment > 0.0F)) {
    scan_invalid_ = "invalid scan: angle_increment <= 0";
    return;
  }
  scan_invalid_.clear();
  scan_pending_ = std::make_shared<sensor_msgs::msg::LaserScan>(msg);
}

bool SafetySupervisor::lookup_laser_tf()
{
  try {
    const auto tf = tf_buffer_->lookupTransform(
      cfg_.robot_frame, scan_frame_, tf2::TimePointZero);
    const auto & q = tf.transform.rotation;
    tf_x_ = tf.transform.translation.x;
    tf_y_ = tf.transform.translation.y;
    tf_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    return true;
  } catch (const tf2::TransformException &) {
    return false;
  }
}

void SafetySupervisor::try_evaluate()
{
  // Evaluate the pending scan as soon as the laser TF is known (looked up with timeout 0)
  if (!scan_pending_) {
    return;
  }
  if (!has_tf_) {
    if (!has_scan_frame_) {
      return;
    }
    has_tf_ = lookup_laser_tf();
    if (!has_tf_) {
      return;
    }
  }
  const auto msg = scan_pending_;
  scan_pending_.reset();
  const std::vector<double> ranges(msg->ranges.begin(), msg->ranges.end());
  const auto result = safety::scan_to_points(
    ranges, msg->angle_min, msg->angle_increment, msg->range_min, msg->range_max,
    tf_x_, tf_y_, tf_yaw_);
  if (result.invalid_ratio > cfg_.max_invalid_beam_ratio) {
    scan_invalid_ = "invalid scan: nan ratio " + format_ratio(result.invalid_ratio);
    return;
  }
  evaluation_ = safety::evaluate_points(result.points, cfg_);
  ++evaluation_seq_;
  scan_ok_ = true;
  if (!blind_checked_) {
    blind_checked_ = true;
    const std::string warning =
      safety::blind_zone_warning(cfg_, tf_x_, tf_y_, msg->range_min);
    if (!warning.empty()) {
      RCLCPP_WARN(get_logger(), "%s", warning.c_str());
    }
  }
}

void SafetySupervisor::on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  odom_time_ = now_seconds();
  odom_v_ = msg->twist.twist.linear.x;
  has_odom_ = true;
}

void SafetySupervisor::on_cmd(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  try {
    cmd_time_ = now_seconds();
    cmd_v_ = msg->linear.x;
    has_cmd_ = true;
    const auto limited = core_->limit_command(msg->linear.x, msg->angular.z, last_outputs_);
    geometry_msgs::msg::Twist out;
    out.linear.x = limited.first;
    out.angular.z = limited.second;
    out_pub_->publish(out);
  } catch (const std::exception & exc) {
    note_exception("cmd callback", exc.what());
  } catch (...) {
    note_exception("cmd callback", "unknown error");
  }
}

void SafetySupervisor::on_observed_cmd(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  cmd_time_ = now_seconds();
  cmd_v_ = msg->linear.x;
  has_cmd_ = true;
}

void SafetySupervisor::on_estop(const std_msgs::msg::Bool::SharedPtr msg)
{
  estop_input_ = msg->data;
}

void SafetySupervisor::on_dock(const bumperbot_msgs::msg::DockStatus::SharedPtr msg)
{
  dock_state_ = msg->state;
  dock_time_ = now_seconds();
  has_dock_ = true;
  dock_frame_ = msg->frame_id;
  dock_x_ = msg->dock_pose.x;
  dock_y_ = msg->dock_pose.y;
}

void SafetySupervisor::on_reset(
  const std_srvs::srv::Trigger::Request::SharedPtr,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  try {
    const auto result = core_->request_reset();
    response->success = result.first;
    response->message = result.second;
    if (result.first && result.second == "reset") {
      RCLCPP_INFO(get_logger(), "e-stop reset");
      tick();
    }
  } catch (const std::exception & exc) {
    response->success = false;
    response->message = std::string("internal error: ") + exc.what();
  }
}

void SafetySupervisor::on_trigger_estop(
  const std_srvs::srv::Trigger::Request::SharedPtr,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  try {
    core_->trigger_estop("e-stop requested by service");
    RCLCPP_WARN(get_logger(), "e-stop latched by service");
    tick();
    response->success = true;
    response->message = "estop latched";
  } catch (const std::exception & exc) {
    response->success = false;
    response->message = std::string("internal error: ") + exc.what();
  }
}

// ---------------------------------------------------------------------------------------------
// Tick
// ---------------------------------------------------------------------------------------------

double SafetySupervisor::dock_distance()
{
  // Distance robot_frame to the dock pose through TF (NaN when unavailable)
  if (dock_frame_.empty()) {
    return kNaN;
  }
  try {
    const auto tf = tf_buffer_->lookupTransform(dock_frame_, cfg_.robot_frame, tf2::TimePointZero);
    const auto & stamp = tf.header.stamp;
    if (stamp.sec != 0 || stamp.nanosec != 0) {
      const double age = get_clock()->now().seconds() -
        (static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9);
      if (age > cfg_.dock_status_timeout) {
        return kNaN;
      }
    }
    return std::hypot(
      tf.transform.translation.x - dock_x_, tf.transform.translation.y - dock_y_);
  } catch (const tf2::TransformException &) {
    return kNaN;
  }
}

safety::Inputs SafetySupervisor::build_inputs(double now)
{
  try_evaluate();
  safety::Inputs inp;
  if (has_scan_) {
    inp.scan_age = now - scan_time_;
  }
  inp.scan_invalid_reason = scan_invalid_;
  inp.scan_ok = scan_ok_;
  inp.tf_ok = has_tf_;
  inp.tf_description = cfg_.robot_frame + " <- " + (has_scan_frame_ ? scan_frame_ : "<scan frame>");
  inp.evaluation = evaluation_;
  inp.scan_seq = evaluation_seq_;
  if (cfg_.use_odom && has_odom_) {
    inp.odom_age = now - odom_time_;
    inp.v_odom = odom_v_;
  }
  if (has_cmd_) {
    inp.cmd_age = now - cmd_time_;
    inp.v_cmd = cmd_v_;
  }
  inp.estop_input = estop_input_;
  const double dock_age = has_dock_ ? now - dock_time_ : -1.0;
  double distance = kNaN;
  if (cfg_.dock_approach_enabled && dock_state_ == safety::DOCK_STATE_DOCKING &&
    dock_age >= 0.0 && dock_age < cfg_.dock_status_timeout)
  {
    distance = dock_distance();
  }
  const auto gate = dock_gate_->update(
    now, cfg_.dock_approach_enabled, dock_state_, dock_age, distance);
  inp.dock_active = gate.first;
  inp.dock_reason = gate.second;
  return inp;
}

void SafetySupervisor::tick()
{
  const double now = now_seconds();
  safety::Inputs inp;
  try {
    ++tick_count_;
    if (tick_count_ <= cfg_.debug_inject_tick_exceptions) {
      throw std::runtime_error("injected tick exception " + std::to_string(tick_count_));
    }
    inp = build_inputs(now);
    if (!pending_exception_.empty()) {
      inp.tick_exception = pending_exception_;
    }
  } catch (const std::exception & exc) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000, "exception in tick: %s", exc.what());
    inp = safety::Inputs();
    inp.tick_exception = exc.what();
  } catch (...) {
    inp = safety::Inputs();
    inp.tick_exception = "unknown error";
  }
  pending_exception_.clear();
  safety::Outputs outputs;
  try {
    outputs = core_->update(now, inp);
  } catch (const std::exception & exc) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000, "exception in core: %s", exc.what());
    outputs.state = safety::STATE_FAULT;
    outputs.locked = true;
    outputs.reason = std::string("internal error: ") + exc.what();
  }
  last_outputs_ = outputs;
  last_inputs_ = inp;
  try {
    publish_outputs(outputs, inp, now);
  } catch (const std::exception & exc) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000, "exception while publishing: %s", exc.what());
  }
}

void SafetySupervisor::publish_outputs(
  const safety::Outputs & out, const safety::Inputs & inp, double now)
{
  std_msgs::msg::Bool lock;
  lock.data = out.locked;
  lock_pub_->publish(lock);
  if (filter_mode_) {
    const bool cmd_stale = !(inp.cmd_age >= 0.0 && inp.cmd_age <= cfg_.cmd_timeout);
    if (out.locked || cmd_stale) {
      out_pub_->publish(geometry_msgs::msg::Twist());
    }
  }
  const std::string key = out.state + "|" + out.active_zone + "|" + out.front_zone + "|" +
    out.rear_zone + "|" + (out.locked ? "1" : "0") + (out.braking ? "1" : "0") +
    (out.estop_latched ? "1" : "0") + (out.dock_profile_active ? "1" : "0") + "|" + out.reason;
  const double period = 1.0 / cfg_.status_rate;
  if (key != last_status_key_ || now - last_status_time_ >= period - 0.5 / cfg_.rate) {
    last_status_key_ = key;
    last_status_time_ = now;
    status_pub_->publish(build_status(out, inp));
  }
}

bumperbot_msgs::msg::SafetyStatus SafetySupervisor::build_status(
  const safety::Outputs & out, const safety::Inputs & inp)
{
  const auto & ev = inp.evaluation;
  const bool known = out.front_zone != safety::ZONE_UNKNOWN;
  bumperbot_msgs::msg::SafetyStatus msg;
  msg.stamp = get_clock()->now();
  msg.state = out.state;
  msg.enforcement = cfg_.enforcement;
  msg.active_zone = out.active_zone;
  msg.front_zone = out.front_zone;
  msg.rear_zone = out.rear_zone;
  msg.reason = out.reason;
  msg.locked = out.locked;
  msg.braking = out.braking;
  msg.estop_latched = out.estop_latched;
  msg.dock_profile_active = out.dock_profile_active;
  msg.nearest_obstacle_distance = known ? static_cast<float>(ev.nearest_distance) : -1.0F;
  msg.nearest_obstacle_bearing = known ? static_cast<float>(ev.nearest_bearing) : 0.0F;
  msg.front_clearance = known ? static_cast<float>(ev.front_clearance) : -1.0F;
  msg.rear_clearance = known ? static_cast<float>(ev.rear_clearance) : -1.0F;
  msg.max_forward_speed = static_cast<float>(out.max_forward);
  msg.max_reverse_speed = static_cast<float>(out.max_reverse);
  msg.scan_age = static_cast<float>(inp.scan_age);
  msg.odom_age = static_cast<float>(inp.odom_age);
  msg.cmd_age = static_cast<float>(inp.cmd_age);
  return msg;
}

// ---------------------------------------------------------------------------------------------
// Shutdown
// ---------------------------------------------------------------------------------------------

void SafetySupervisor::finalize()
{
  if (!started_) {
    return;
  }
  core_->shutdown();
  const auto out = core_->update(now_seconds(), safety::Inputs());
  for (int i = 0; i < 3; ++i) {
    try {
      std_msgs::msg::Bool lock;
      lock.data = true;
      lock_pub_->publish(lock);
      if (filter_mode_) {
        out_pub_->publish(geometry_msgs::msg::Twist());
      }
      auto status = build_status(out, last_inputs_);
      status.state = safety::STATE_SHUTDOWN;
      status.locked = true;
      status.max_forward_speed = 0.0F;
      status.max_reverse_speed = 0.0F;
      status_pub_->publish(status);
    } catch (const std::exception & exc) {
      RCLCPP_ERROR(get_logger(), "shutdown publish failed: %s", exc.what());
    }
    std::this_thread::sleep_for(50ms);
  }
  RCLCPP_INFO(get_logger(), "safety_supervisor stopped, lock engaged");
}

}  // namespace bumperbot_utils

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
  // Own signal handling: the context must stay valid while the final lock is published,
  // so rclcpp's default shutdown-on-signal is disabled.
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
  int status = 0;
  try {
    auto node = std::make_shared<bumperbot_utils::SafetySupervisor>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (!g_stop_requested && rclcpp::ok()) {
      try {
        executor.spin_once(50ms);
      } catch (const std::exception & exc) {
        RCLCPP_ERROR_THROTTLE(
          node->get_logger(), *node->get_clock(), 2000, "spin error: %s", exc.what());
      }
    }
    node->finalize();
  } catch (const bumperbot_utils::StartupError &) {
    status = 1;
  } catch (const std::exception & exc) {
    RCLCPP_FATAL(rclcpp::get_logger("safety_supervisor"), "%s", exc.what());
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}
