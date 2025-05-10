#include "moonpie_osamu/mission_control.hpp"
#include "moonpie_osamu/msg/mission_command.hpp"

namespace moonpie_osamu
{

MissionControl::MissionControl()
: Node("mission_control"),
  current_waypoint_(0),
  mission_active_(false),
  last_distance_log_time_(this->now())
{
  RCLCPP_INFO(this->get_logger(), "Initializing Mission Control node...");

  // Create the action client
  nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
    this, "navigate_to_pose");

  // Create the command subscription
  cmd_sub_ = this->create_subscription<moonpie_osamu::msg::MissionCommand>(
    "mission/cmd", 10,
    std::bind(&MissionControl::onMissionCommand, this, std::placeholders::_1));

  // Initialize BehaviorStatus publisher
  behavior_status_pub_ = this->create_publisher<moonpie_osamu::msg::BehaviorStatus>(
    "mission/log", 10);
  RCLCPP_INFO(this->get_logger(), "Created mission log publisher on topic: mission/log");

  // Initialize waypoints (these should be loaded from a config file in the future)
  geometry_msgs::msg::PoseStamped waypoint;
  waypoint.header.frame_id = "map";
  waypoint.pose.position.x = 1.0;
  waypoint.pose.position.y = 0.0;
  waypoint.pose.orientation.w = 1.0;
  waypoints_.push_back(waypoint);

  waypoint.pose.position.x = 1.0;
  waypoint.pose.position.y = 1.0;
  waypoints_.push_back(waypoint);

  waypoint.pose.position.x = 0.0;
  waypoint.pose.position.y = 1.0;
  waypoints_.push_back(waypoint);

  waypoint.pose.position.x = 0.0;
  waypoint.pose.position.y = 0.0;
  waypoints_.push_back(waypoint);

  // Send ready message on startup
  auto status_msg = std::make_unique<moonpie_osamu::msg::BehaviorStatus>();
  status_msg->timestamp = this->now();
  status_msg->current_node = "mission_control";
  status_msg->status = "READY";
  status_msg->details = "Mission control node is ready";
  behavior_status_pub_->publish(std::move(status_msg));

  RCLCPP_INFO(this->get_logger(), "Mission Control node initialized successfully");
}

void MissionControl::onMissionCommand(const moonpie_osamu::msg::MissionCommand::SharedPtr msg)
{
  if (msg->command == "START") {
    RCLCPP_INFO(this->get_logger(), "Received START command");
    current_waypoint_ = 0;
    mission_active_ = true;
    sendNavigationGoal(waypoints_[current_waypoint_]);

    // Publish status update
    auto status_msg = std::make_unique<moonpie_osamu::msg::BehaviorStatus>();
    status_msg->timestamp = this->now();
    status_msg->current_node = "mission_control";
    status_msg->status = "NAVIGATING";
    status_msg->details = "Mission started and waypoint following initiated.";
    behavior_status_pub_->publish(std::move(status_msg));
  }
  else if (msg->command == "STOP") {
    RCLCPP_INFO(this->get_logger(), "Received STOP command");
    mission_active_ = false;
    nav_client_->async_cancel_all_goals();
    RCLCPP_INFO(this->get_logger(), "Navigation goals cancel request sent");

    // Publish status update
    auto status_msg = std::make_unique<moonpie_osamu::msg::BehaviorStatus>();
    status_msg->timestamp = this->now();
    status_msg->current_node = "mission_control";
    status_msg->status = "IDLE";
    status_msg->details = "Mission stopped and waypoint following canceled.";
    behavior_status_pub_->publish(std::move(status_msg));
  }
  else if (msg->command == "TEST") {
    RCLCPP_INFO(this->get_logger(), "Received TEST command");
    // Publish ready status
    auto status_msg = std::make_unique<moonpie_osamu::msg::BehaviorStatus>();
    status_msg->timestamp = this->now();
    status_msg->current_node = "mission_control";
    status_msg->status = "READY";
    status_msg->details = "Mission control node is ready";
    behavior_status_pub_->publish(std::move(status_msg));
  }
  else {
    RCLCPP_WARN(this->get_logger(), "Unknown command received: %s", msg->command.c_str());
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
  // Rate limit distance messages to 3 Hz
  auto now = this->now();
  double time_since_last = (now - last_distance_log_time_).seconds();
  if (time_since_last < 3.0) {
    return;  // Skip if less than 3 seconds have passed
  }
  last_distance_log_time_ = now;

  // Log to terminal at 3 Hz
  RCLCPP_INFO(this->get_logger(), "Distance remaining: %.2f meters", feedback->distance_remaining);

  // Create and publish the status message
  auto status_msg = std::make_unique<moonpie_osamu::msg::BehaviorStatus>();
  status_msg->timestamp = now;
  status_msg->current_node = "mission_control";
  status_msg->status = "NAVIGATING";
  status_msg->details = "Distance remaining: " + std::to_string(feedback->distance_remaining) + " meters";
  
  RCLCPP_INFO(this->get_logger(), "Publishing status message to mission/log");
  behavior_status_pub_->publish(std::move(status_msg));
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