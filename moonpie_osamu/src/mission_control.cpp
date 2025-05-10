#include "moonpie_osamu/mission_control.hpp"

namespace moonpie_osamu
{

MissionControl::MissionControl()
: Node("mission_control"),
  current_waypoint_(0)
{
  // Create the action client
  nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
    this, "navigate_to_pose");

  // Create the subscriptions
  start_sub_ = this->create_subscription<std_msgs::msg::Empty>(
    "mission/start", 10,
    std::bind(&MissionControl::onStartMission, this, std::placeholders::_1));

  stop_sub_ = this->create_subscription<std_msgs::msg::Empty>(
    "mission/stop", 10,
    std::bind(&MissionControl::onStopMission, this, std::placeholders::_1));

  // Initialize waypoints (these should be loaded from a config file in the future)
  geometry_msgs::msg::PoseStamped waypoint;
  waypoint.header.frame_id = "map";
  waypoint.pose.position.x = 1.0;
  waypoint.pose.position.y = 0.0;
  waypoint.pose.orientation.w = 1.0;
  waypoints_.push_back(waypoint);

  waypoint.pose.position.x = 0.0;
  waypoint.pose.position.y = 1.0;
  waypoints_.push_back(waypoint);
}

void MissionControl::onStartMission(const std_msgs::msg::Empty::SharedPtr)
{
  RCLCPP_INFO(this->get_logger(), "Starting mission...");
  current_waypoint_ = 0;
  if (!waypoints_.empty()) {
    sendNavigationGoal(waypoints_[current_waypoint_]);
  }
}

void MissionControl::onStopMission(const std_msgs::msg::Empty::SharedPtr)
{
  RCLCPP_INFO(this->get_logger(), "Stopping mission...");
  if (nav_client_) {
    // Cancel any active navigation goals
    nav_client_->async_cancel_all_goals();
  }
  current_waypoint_ = 0;  // Reset waypoint counter
}

// ... rest of the existing code ...
} 