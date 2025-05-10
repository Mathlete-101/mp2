#ifndef MOONPIE_OSAMU_MISSION_CONTROL_HPP_
#define MOONPIE_OSAMU_MISSION_CONTROL_HPP_

#include <memory>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "moonpie_osamu/msg/behavior_status.hpp"

namespace moonpie_osamu
{

class MissionControl : public rclcpp::Node
{
public:
  MissionControl();

private:
  void onMissionCommand(const moonpie_osamu::msg::MissionCommand::SharedPtr msg);
  void onArucoPose(const geometry_msgs::msg::PoseArray::SharedPtr msg);
  void navigateToWorldCoordinates(double x, double y);
  void sendNavigationGoal(const geometry_msgs::msg::PoseStamped& goal);
  void goalResponseCallback(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr & goal_handle);
  void feedbackCallback(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback);
  void resultCallback(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result);

  // Action client for navigation
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;

  // Subscriptions
  rclcpp::Subscription<moonpie_osamu::msg::MissionCommand>::SharedPtr cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr aruco_sub_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<moonpie_osamu::msg::BehaviorStatus>::SharedPtr behavior_status_pub_;

  // Transform handling
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // State variables
  size_t current_waypoint_;
  bool mission_active_;
  rclcpp::Time last_distance_log_time_;
  bool localization_complete_;
  bool spinning_;
};

}  // namespace moonpie_osamu

#endif  // MOONPIE_OSAMU_MISSION_CONTROL_HPP_ 