#ifndef BUMPERBOT_UTILS__SAFETY_SUPERVISOR_HPP_
#define BUMPERBOT_UTILS__SAFETY_SUPERVISOR_HPP_

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "bumperbot_msgs/msg/dock_status.hpp"
#include "bumperbot_msgs/msg/safety_status.hpp"
#include "bumperbot_utils/safety_logic.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace bumperbot_utils
{

// Thrown when the node cannot start (the lock has been published and FATAL logged)
class StartupError : public std::runtime_error
{
public:
  explicit StartupError(const std::string & what)
  : std::runtime_error(what) {}
};

// AGV safety supervisor: direction-aware speed limiting, latched e-stop and sensor watchdog.
// The Python twin is bumperbot_utils/safety_supervisor.py (same parameters and behavior).
// All decisions live in safety_logic (SafetyCore); this node only does the ROS I/O.
class SafetySupervisor : public rclcpp::Node
{
public:
  explicit SafetySupervisor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // Lock the robot: SHUTDOWN status, lock true and a zero command, 3 times
  void finalize();

private:
  using SteadyClock = std::chrono::steady_clock;

  template<typename T>
  T declare_checked(const std::string & name, const T & default_value);
  safety::Config load_config();
  [[noreturn]] void fail_safe(const std::string & reason);
  void setup();

  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void handle_scan(const sensor_msgs::msg::LaserScan & msg);
  bool lookup_laser_tf();
  void try_evaluate();
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void on_cmd(const geometry_msgs::msg::Twist::SharedPtr msg);
  void on_observed_cmd(const geometry_msgs::msg::Twist::SharedPtr msg);
  void on_estop(const std_msgs::msg::Bool::SharedPtr msg);
  void on_dock(const bumperbot_msgs::msg::DockStatus::SharedPtr msg);
  void on_reset(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);
  void on_trigger_estop(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);

  double now_seconds() const;
  void note_exception(const std::string & where, const std::string & what);
  double dock_distance();
  safety::Inputs build_inputs(double now);
  void tick();
  void publish_outputs(const safety::Outputs & out, const safety::Inputs & inp, double now);
  bumperbot_msgs::msg::SafetyStatus build_status(
    const safety::Outputs & out, const safety::Inputs & inp);

  safety::Config cfg_;
  bool filter_mode_ = true;
  bool started_ = false;
  std::unique_ptr<safety::SafetyCore> core_;
  std::unique_ptr<safety::DockGate> dock_gate_;

  // Inputs, written by the callbacks only
  bool has_scan_ = false;
  double scan_time_ = 0.0;
  bool has_scan_frame_ = false;
  std::string scan_frame_;
  std::string scan_invalid_;
  bool scan_ok_ = false;
  sensor_msgs::msg::LaserScan::SharedPtr scan_pending_;
  safety::Evaluation evaluation_;
  bool has_tf_ = false;
  double tf_x_ = 0.0;
  double tf_y_ = 0.0;
  double tf_yaw_ = 0.0;
  bool has_odom_ = false;
  double odom_time_ = 0.0;
  double odom_v_ = std::numeric_limits<double>::quiet_NaN();
  bool has_cmd_ = false;
  double cmd_time_ = 0.0;
  double cmd_v_ = std::numeric_limits<double>::quiet_NaN();
  bool estop_input_ = false;
  std::string dock_state_;
  bool has_dock_ = false;
  double dock_time_ = 0.0;
  std::string dock_frame_;
  double dock_x_ = 0.0;
  double dock_y_ = 0.0;
  std::string pending_exception_;
  int64_t tick_count_ = 0;
  safety::Outputs last_outputs_;
  safety::Inputs last_inputs_;
  std::string last_status_key_;
  double last_status_time_ = 0.0;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lock_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr out_pub_;
  rclcpp::Publisher<bumperbot_msgs::msg::SafetyStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<bumperbot_msgs::msg::DockStatus>::SharedPtr dock_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr estop_srv_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
};

}  // namespace bumperbot_utils

#endif  // BUMPERBOT_UTILS__SAFETY_SUPERVISOR_HPP_
