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

  // Create the subscription
  start_sub_ = this->create_subscription<std_msgs::msg::Empty>(
    "mission/start", 10,
    std::bind(&MissionControl::onStartMission, this, std::placeholders::_1));

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

void MissionControl::sendNavigationGoal(const geometry_msgs::msg::PoseStamped& goal)
{
  if (!nav_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(this->get_logger(), "Navigation action server not available");
    return;
  }

  auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
  goal_msg.pose = goal;

  auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&MissionControl::goalResponseCallback, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
    std::bind(&MissionControl::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(&MissionControl::resultCallback, this, std::placeholders::_1);

  nav_client_->async_send_goal(goal_msg, send_goal_options);
}

void MissionControl::goalResponseCallback(
  const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  }
}

void MissionControl::feedbackCallback(
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
  const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
{
  RCLCPP_INFO(this->get_logger(), "Distance remaining: %.2f", feedback->distance_remaining);
}

void MissionControl::resultCallback(
  const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
      current_waypoint_++;
      if (current_waypoint_ < waypoints_.size()) {
        sendNavigationGoal(waypoints_[current_waypoint_]);
      } else {
        RCLCPP_INFO(this->get_logger(), "Mission complete!");
      }
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      break;
  }
}

}  // namespace moonpie_osamu

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<moonpie_osamu::MissionControl>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} 