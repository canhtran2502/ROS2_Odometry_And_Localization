#ifndef BUMPERBOT_NAVIGATION__MISSION_MANAGER_HPP_
#define BUMPERBOT_NAVIGATION__MISSION_MANAGER_HPP_

// AGV mission manager: ExecuteMission action server driving the Nav2 NavigateToPose action.
// The Python twin is bumperbot_navigation/mission_manager.py (same parameters and behavior).

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "bumperbot_msgs/action/execute_mission.hpp"
#include "bumperbot_msgs/msg/mission_status.hpp"
#include "bumperbot_navigation/mission_logic.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace bumperbot_navigation
{

class MissionManager : public rclcpp::Node
{
public:
  using ExecuteMission = bumperbot_msgs::action::ExecuteMission;
  using MissionGoalHandle = rclcpp_action::ServerGoalHandle<ExecuteMission>;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using NavGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  explicit MissionManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Action server callbacks
  rclcpp_action::GoalResponse goal_callback(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ExecuteMission::Goal> goal);
  rclcpp_action::CancelResponse cancel_callback(std::shared_ptr<MissionGoalHandle> goal_handle);
  void accepted_callback(std::shared_ptr<MissionGoalHandle> goal_handle);
  void execute(std::shared_ptr<MissionGoalHandle> goal_handle);
  void run_mission();

  // Status and feedback
  bumperbot_msgs::msg::MissionStatus build_status();
  void publish_status();
  void publish_feedback();
  void publish_all();

  // Helpers
  bool handle_cancel_if_pending();
  bool wait_clock(double seconds, bool interruptible);
  NavigateToPose::Goal make_goal(const Station & station);
  double distance_to_station(const Station & station);
  // Returns true when the navigation succeeded; interrupted is set when a cancel stopped it
  bool navigate(const Step & step, bool & interrupted);

  StationSet stations_;
  TrackerParams tracker_params_;
  std::string global_frame_;
  std::string robot_frame_;
  std::string behavior_tree_;
  double retry_delay_;
  double nav_server_timeout_;
  double nav_goal_timeout_;
  double tf_timeout_;

  rclcpp::CallbackGroup::SharedPtr group_;
  std::mutex mutex_;
  bool active_ = false;
  std::unique_ptr<MissionTracker> tracker_;
  std::vector<std::string> mission_names_;
  std::atomic<double> distance_remaining_{-1.0};
  std::shared_ptr<MissionGoalHandle> goal_handle_;
  std::atomic<bool> cancel_requested_{false};
  bool cancel_handled_ = false;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<bumperbot_msgs::msg::MissionStatus>::SharedPtr status_pub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp_action::Server<ExecuteMission>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace bumperbot_navigation

#endif  // BUMPERBOT_NAVIGATION__MISSION_MANAGER_HPP_
