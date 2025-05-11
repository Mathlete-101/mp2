#include "moonpie_osamu/mission_control.hpp"
#include "moonpie_osamu/msg/mission_command.hpp"
#include "aruco_opencv_msgs/msg/aruco_detection.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace moonpie_osamu
{

MissionControl::MissionControl()
: Node("mission_control"),
  current_waypoint_(0),
  mission_active_(false),
  last_distance_log_time_(this->now()),
  localization_complete_(false),
  spinning_(false)
{
  RCLCPP_INFO(this->get_logger(), "Initializing Mission Control node...");

  // Create the action client
  nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
    this, "navigate_to_pose");

  // Create the command subscription
  cmd_sub_ = this->create_subscription<moonpie_osamu::msg::MissionCommand>(
    "mission/cmd", 10,
    std::bind(&MissionControl::onMissionCommand, this, std::placeholders::_1));

  // Create ArUco pose subscription
  aruco_sub_ = this->create_subscription<aruco_opencv_msgs::msg::ArucoDetection>(
    "aruco_detections", 10,
    std::bind(&MissionControl::onArucoPose, this, std::placeholders::_1));

  // Create velocity publisher for spinning
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "cmd_vel", 10);

  // Initialize transform broadcaster and buffer
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Initialize BehaviorStatus publisher
  behavior_status_pub_ = this->create_publisher<moonpie_osamu::msg::BehaviorStatus>(
    "mission/log", 10);
  RCLCPP_INFO(this->get_logger(), "Created mission log publisher on topic: mission/log");

  // Send ready message on startup
  auto status_msg = std::make_unique<moonpie_osamu::msg::BehaviorStatus>();
  status_msg->timestamp = this->now();
  status_msg->current_node = "mission_control";
  status_msg->status = "READY";
  status_msg->details = "Mission control node is ready";
  behavior_status_pub_->publish(std::move(status_msg));

  // Initialize transform timer
  transform_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),  // Publish transform at 10Hz
    std::bind(&MissionControl::publishWorldTransform, this));

  RCLCPP_INFO(this->get_logger(), "Mission Control node initialized successfully");
}

void MissionControl::publishWorldTransform()
{
  if (localization_complete_) {
    // Update timestamp
    world_transform_.header.stamp = this->now();
    // Broadcast the transform
    tf_broadcaster_->sendTransform(world_transform_);
  }
}

void MissionControl::onArucoPose(const aruco_opencv_msgs::msg::ArucoDetection::SharedPtr msg)
{
  if (localization_complete_ || !mission_active_) {
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Received ArUco detection message with %zu markers", msg->markers.size());

  // Look for marker ID 42
  for (const auto& marker : msg->markers) {
    RCLCPP_INFO(this->get_logger(), "Checking marker ID: %d", marker.marker_id);
    if (marker.marker_id == 42) {
      RCLCPP_INFO(this->get_logger(), "Found marker 42!");
      
      // Stop spinning
      spinning_ = false;
      auto twist_msg = geometry_msgs::msg::Twist();
      cmd_vel_pub_->publish(twist_msg);

      // Calculate transform from map to world
      world_transform_.header.frame_id = "map";
      world_transform_.child_frame_id = "world";

      // The ArUco pose is in the camera frame, transform it to the map frame
      // For now, we'll assume the ArUco marker defines the origin of the world frame
      world_transform_.transform.translation.x = -marker.pose.position.x;
      world_transform_.transform.translation.y = -marker.pose.position.y;
      world_transform_.transform.translation.z = -marker.pose.position.z;

      // Invert the orientation
      tf2::Quaternion q;
      q.setX(-marker.pose.orientation.x);
      q.setY(-marker.pose.orientation.y);
      q.setZ(-marker.pose.orientation.z);
      q.setW(marker.pose.orientation.w);
      q.normalize();
      
      world_transform_.transform.rotation.x = q.x();
      world_transform_.transform.rotation.y = q.y();
      world_transform_.transform.rotation.z = q.z();
      world_transform_.transform.rotation.w = q.w();

      // Mark localization as complete
      localization_complete_ = true;

      // Publish status update
      auto status_msg = std::make_unique<moonpie_osamu::msg::BehaviorStatus>();
      status_msg->timestamp = this->now();
      status_msg->current_node = "mission_control";
      status_msg->status = "LOCALIZED";
      status_msg->details = "Found marker 42 and established world transform";
      behavior_status_pub_->publish(std::move(status_msg));

      // Create a timer to start navigation after a short delay
      nav_timer_ = this->create_wall_timer(
        std::chrono::seconds(2),  // Increased delay to 2 seconds
        [this]() {
          // Choose a random point in the excavation zone: x in [3.88, 6.88], y in [0, 2]
          double x_min = 3.88, x_max = 6.88, y_min = 0.0, y_max = 2.0;
          double x = x_min + static_cast<double>(rand()) / RAND_MAX * (x_max - x_min);
          double y = y_min + static_cast<double>(rand()) / RAND_MAX * (y_max - y_min);
          RCLCPP_INFO(this->get_logger(), "Starting navigation to random excavation zone point (%.2f, %.2f)", x, y);
          // If the point is occupied, another can be chosen later
          navigateToWorldCoordinates(x, y);
          nav_timer_.reset();  // Clear the timer after use
        }
      );

      return;
    }
  }
}

void MissionControl::navigateToWorldCoordinates(double x, double y)
{
  try {
    // Create a pose in world coordinates
    geometry_msgs::msg::PoseStamped world_pose;
    world_pose.header.frame_id = "world";
    world_pose.header.stamp = this->now();
    world_pose.pose.position.x = x;
    world_pose.pose.position.y = y;
    world_pose.pose.orientation.w = 1.0;  // Default orientation

    // Transform the pose to map coordinates using the latest available transform
    geometry_msgs::msg::PoseStamped map_pose;
    try {
      // First try with the current time
      map_pose = tf_buffer_->transform(world_pose, "map");
    } catch (const tf2::ExtrapolationException & ex) {
      // If that fails, try with a small time offset in the past
      world_pose.header.stamp = this->now() - rclcpp::Duration::from_seconds(0.1);
      map_pose = tf_buffer_->transform(world_pose, "map");
    }

    // Send navigation goal
    sendNavigationGoal(map_pose);

    // Publish status update
    auto status_msg = std::make_unique<moonpie_osamu::msg::BehaviorStatus>();
    status_msg->timestamp = this->now();
    status_msg->current_node = "mission_control";
    status_msg->status = "NAVIGATING";
    status_msg->details = "Navigating to world coordinates (" + std::to_string(x) + ", " + std::to_string(y) + ")";
    behavior_status_pub_->publish(std::move(status_msg));

  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(this->get_logger(), "Could not transform world coordinates to map: %s", ex.what());
    
    // Publish error status
    auto status_msg = std::make_unique<moonpie_osamu::msg::BehaviorStatus>();
    status_msg->timestamp = this->now();
    status_msg->current_node = "mission_control";
    status_msg->status = "ERROR";
    status_msg->details = "Failed to transform coordinates: " + std::string(ex.what());
    behavior_status_pub_->publish(std::move(status_msg));
  }
}

void MissionControl::onMissionCommand(const moonpie_osamu::msg::MissionCommand::SharedPtr msg)
{
  if (msg->command == "START") {
    RCLCPP_INFO(this->get_logger(), "Received START command");
    mission_active_ = true;
    
    if (!localization_complete_) {
      // Start spinning to find marker
      spinning_ = true;
      auto twist_msg = geometry_msgs::msg::Twist();
      twist_msg.angular.z = 0.5;  // Rotate at 0.5 rad/s
      cmd_vel_pub_->publish(twist_msg);

      // Publish status update
      auto status_msg = std::make_unique<moonpie_osamu::msg::BehaviorStatus>();
      status_msg->timestamp = this->now();
      status_msg->current_node = "mission_control";
      status_msg->status = "LOCALIZING";
      status_msg->details = "Spinning to find marker 42";
      behavior_status_pub_->publish(std::move(status_msg));
    }
  }
  else if (msg->command == "STOP") {
    RCLCPP_INFO(this->get_logger(), "Received STOP command");
    mission_active_ = false;
    spinning_ = false;
    
    // Stop any motion
    auto twist_msg = geometry_msgs::msg::Twist();
    cmd_vel_pub_->publish(twist_msg);
    
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
    
    // Publish error status
    auto status_msg = std::make_unique<moonpie_osamu::msg::BehaviorStatus>();
    status_msg->timestamp = this->now();
    status_msg->current_node = "mission_control";
    status_msg->status = "ERROR";
    status_msg->details = "Navigation goal was rejected by server";
    behavior_status_pub_->publish(std::move(status_msg));
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
  std::unique_ptr<moonpie_osamu::msg::BehaviorStatus> status_msg;
  
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
      
      // Publish success status
      status_msg = std::make_unique<moonpie_osamu::msg::BehaviorStatus>();
      status_msg->timestamp = this->now();
      status_msg->current_node = "mission_control";
      status_msg->status = "COMPLETE";
      status_msg->details = "Successfully reached target position";
      behavior_status_pub_->publish(std::move(status_msg));
      break;
      
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      
      // Publish error status
      status_msg = std::make_unique<moonpie_osamu::msg::BehaviorStatus>();
      status_msg->timestamp = this->now();
      status_msg->current_node = "mission_control";
      status_msg->status = "ERROR";
      status_msg->details = "Navigation goal was aborted";
      behavior_status_pub_->publish(std::move(status_msg));
      break;
      
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      
      // Publish status
      status_msg = std::make_unique<moonpie_osamu::msg::BehaviorStatus>();
      status_msg->timestamp = this->now();
      status_msg->current_node = "mission_control";
      status_msg->status = "CANCELED";
      status_msg->details = "Navigation goal was canceled";
      behavior_status_pub_->publish(std::move(status_msg));
      break;
      
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      
      // Publish error status
      status_msg = std::make_unique<moonpie_osamu::msg::BehaviorStatus>();
      status_msg->timestamp = this->now();
      status_msg->current_node = "mission_control";
      status_msg->status = "ERROR";
      status_msg->details = "Unknown navigation result code";
      behavior_status_pub_->publish(std::move(status_msg));
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