#ifndef BUMPERBOT_NAVIGATION__DOCK_MANAGER_HPP_
#define BUMPERBOT_NAVIGATION__DOCK_MANAGER_HPP_

// AGV dock manager: DockRobot action server (DOCK / UNDOCK) driving Nav2 and a closed-loop dock
// controller on odometry.
// The Python twin is bumperbot_navigation/dock_manager.py (same parameters and behavior).

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "bumperbot_msgs/action/dock_robot.hpp"
#include "bumperbot_msgs/msg/dock_status.hpp"
#include "bumperbot_msgs/msg/mission_status.hpp"
#include "bumperbot_navigation/dock_logic.hpp"
#include "bumperbot_navigation/mission_logic.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace bumperbot_navigation
{

class DockManager : public rclcpp::Node
{
public:
  using DockRobot = bumperbot_msgs::action::DockRobot;
  using DockGoalHandle = rclcpp_action::ServerGoalHandle<DockRobot>;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using NavGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  // Throws std::invalid_argument when the configuration is invalid
  explicit DockManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~DockManager() override;

  // Stop the robot and end the active goal (best effort), then join the worker thread. Safe to
  // call more than once; must be called while the executor is still spinning.
  void shutdown_goal();

private:
  // Configuration
  double positive(const std::string & name);
  void load_configuration();

  // Time and TF helpers
  double node_now();
  bool lookup(
    const std::string & parent, const std::string & child, Pose2D & pose, double & stamp);
  bool can_transform(const std::string & parent, const std::string & child);
  bool near_staging();

  // Subscriptions
  void detection_callback(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  Pose2D detection_to_odom(const geometry_msgs::msg::PoseStamped & msg);

  // Status and feedback
  bumperbot_msgs::msg::DockStatus build_status();
  void publish_status();
  void publish_feedback();
  void publish_all();
  void status_timer_callback();
  void update_docked_watch();

  // Action server callbacks
  rclcpp_action::GoalResponse goal_callback(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const DockRobot::Goal> goal);
  rclcpp_action::GoalResponse reject(const std::string & reason);
  rclcpp_action::CancelResponse cancel_callback(std::shared_ptr<DockGoalHandle> goal_handle);
  void accepted_callback(std::shared_ptr<DockGoalHandle> goal_handle);

  // Worker
  void worker_main(std::shared_ptr<DockGoalHandle> goal_handle);
  void run_goal(const std::shared_ptr<DockGoalHandle> & goal_handle);
  void finish_goal(const std::shared_ptr<DockGoalHandle> & goal_handle);

  // Interrupts and ticks
  bool check_interrupts();
  bool tick();
  bool wait_interruptible(double seconds);
  bool odom_pose(Pose2D & pose);

  // Command output and dock estimate
  void publish_cmd(double v, double w);
  bool begin_approach(bool use_saved);
  Pose2D robot_in_dock_frame(const Pose2D & odom);
  void update_live(const Pose2D & p);

  // Nav2
  NavigateToPose::Goal make_goal();
  void cancel_nav_goal(double wait_seconds);
  bool mission_running();
  void run_navigate();
  // Returns 1 (reached), 0 (failed) or -1 (the goal ended meanwhile)
  int navigate();

  // Phases
  void run_wait_detection();
  void run_align();
  void run_insert();
  void run_undock();
  // Returns false when the goal ended meanwhile (no result)
  bool control_phase(
    double timeout, const std::function<Command(const Pose2D &)> & compute, PhaseResult & result);
  void run_verify();

  // Configuration
  StationSet stations_;
  Station station_;
  Pose2D staging_;
  Pose2D dock_pose_;
  ControlParams control_params_;
  DockTrackerParams tracker_params_;
  std::string global_frame_;
  std::string odom_frame_;
  std::string robot_frame_;
  std::string nav_action_name_;
  std::string cmd_vel_topic_;
  std::string detection_topic_;
  std::string safety_stop_topic_;
  std::string battery_topic_;
  std::string mission_status_topic_;
  std::string detection_mode_;
  Pose2D detection_offset_;
  double staging_tolerance_;
  double staging_yaw_tolerance_;
  double nav_server_timeout_;
  double nav_goal_timeout_;
  double detection_timeout_;
  double detection_max_age_;
  double detection_max_jump_;
  double detection_alpha_;
  double align_timeout_;
  double docking_timeout_;
  double undocking_timeout_;
  double stall_timeout_;
  double safety_stop_timeout_;
  double settle_time_;
  double undock_detect_distance_;
  double tf_timeout_;
  double tf_stale_timeout_;
  double control_rate_;
  double status_rate_;
  double clock_stall_timeout_;

  // Goal state (guarded by mutex_)
  std::mutex mutex_;
  std::atomic<bool> shutdown_{false};
  std::thread worker_;
  bool active_ = false;
  std::shared_ptr<DockGoalHandle> goal_handle_;
  DockTracker tracker_;
  DockController controller_;
  DetectionFilter detection_filter_{0.3, 0.3};
  Pose2D dock_estimate_;
  bool has_estimate_ = false;
  bool accepting_ = false;
  int detection_count_ = 0;
  std::shared_ptr<NavGoalHandle> nav_handle_;
  std::atomic<double> live_distance_{-1.0};
  std::atomic<double> live_yaw_error_{-1.0};

  // Worker-thread state
  PhaseTimer goal_clock_{1e12, 5.0};
  Pose2D last_robot_odom_;
  bool has_robot_odom_ = false;
  double next_tick_time_ = 0.0;
  bool tf_failing_ = false;
  double tf_fail_since_ = 0.0;

  // Idle state (guarded by mutex_)
  bool docked_ = false;
  std::string idle_state_;
  std::string idle_message_;
  int last_attempt_ = 1;
  double last_position_error_ = -1.0;
  double last_yaw_error_ = -1.0;
  bool last_detection_used_ = false;
  Pose2D docked_robot_odom_;
  bool has_docked_robot_odom_ = false;
  Pose2D saved_dock_estimate_;
  bool has_saved_estimate_ = false;

  // Inputs
  std::atomic<bool> safety_locked_{false};
  std::atomic<double> battery_percentage_{-1.0};
  std::atomic<bool> battery_charging_{false};
  std::atomic<bool> mission_active_{false};
  // Set when a mission became active while a dock goal runs (guarded by mutex_)
  bool mission_started_during_goal_ = false;

  rclcpp::CallbackGroup::SharedPtr group_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<bumperbot_msgs::msg::DockStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr detection_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr safety_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Subscription<bumperbot_msgs::msg::MissionStatus>::SharedPtr mission_sub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp_action::Server<DockRobot>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace bumperbot_navigation

#endif  // BUMPERBOT_NAVIGATION__DOCK_MANAGER_HPP_
