#ifndef MISSION_CONTROL_HPP_
#define MISSION_CONTROL_HPP_

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/empty.hpp"

namespace moonpie_osamu
{

class MissionControl : public rclcpp::Node
{
public:
  explicit MissionControl();

private:
  // Mission state management
  enum class MissionState {
    IDLE,
    NAVIGATING,
    COMPLETED,
    FAILED
  };

  // Callbacks
  void onStartMission(const std_msgs::msg::Empty::SharedPtr msg);
  void sendNavigationGoal(const geometry_msgs::msg::PoseStamped& goal);
  void goalResponseCallback(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr & goal_handle);
  void feedbackCallback(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback);
  void resultCallback(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result);

  // Mission control functions
  void startMission();
  void handleMissionComplete();
  void handleMissionFailure(const std::string & reason);

  // Action clients
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;

  // Subscriptions
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr start_sub_;

  // Mission state
  MissionState current_state_;
  size_t current_waypoint_;
  std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
  bool mission_active_;
};

}  // namespace moonpie_osamu

#endif  // MISSION_CONTROL_HPP_ 