#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moonpie_osamu/msg/behavior_status.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"

class BackwardNavigateToPose : public BT::SyncActionNode {
public:
  BackwardNavigateToPose(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config), node_(rclcpp::Node::make_shared("backward_navigate_to_pose")) {
    pub_ = node_->create_publisher<moonpie_osamu::msg::BehaviorStatus>("mission/log", 10);
    nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node_, "navigate_to_pose");
    speed_pub_ = node_->create_publisher<nav2_msgs::msg::SpeedLimit>("/speed_limit", 10);
  }
  BT::NodeStatus tick() override {
    geometry_msgs::msg::PoseStamped goal;
    if (!getInput("goal", goal)) {
      RCLCPP_ERROR(node_->get_logger(), "No goal provided to BackwardNavigateToPose");
      return BT::NodeStatus::FAILURE;
    }
    // Set orientation to face backwards (180 degrees from x-axis)
    goal.pose.orientation.x = 0.0;
    goal.pose.orientation.y = 0.0;
    goal.pose.orientation.z = 1.0;
    goal.pose.orientation.w = 0.0;

    // Speed modulation: get current pose from blackboard if available
    geometry_msgs::msg::PoseStamped current_pose;
    if (getInput("robot_pose", current_pose)) {
      double dx = goal.pose.position.x - current_pose.pose.position.x;
      double dy = goal.pose.position.y - current_pose.pose.position.y;
      double dist = std::sqrt(dx*dx + dy*dy);
      nav2_msgs::msg::SpeedLimit speed_msg;
      speed_msg.header.stamp = node_->now();
      speed_msg.header.frame_id = "base_link";
      speed_msg.percentage = false;
      speed_msg.speed_limit = (dist < 0.5) ? 0.1 : 1.0;
      speed_pub_->publish(speed_msg);
    } else {
      // Default to 1.0 m/s if no pose available
      nav2_msgs::msg::SpeedLimit speed_msg;
      speed_msg.header.stamp = node_->now();
      speed_msg.header.frame_id = "base_link";
      speed_msg.percentage = false;
      speed_msg.speed_limit = 1.0;
      speed_pub_->publish(speed_msg);
    }

    // Log start
    auto msg = moonpie_osamu::msg::BehaviorStatus();
    msg.timestamp = node_->now();
    msg.current_node = "BackwardNavigateToPose";
    msg.status = "NAVIGATING_BACKWARD";
    msg.details = "Navigating backwards to dump zone";
    msg.message = "";
    pub_->publish(msg);

    // Wait for action server
    if (!nav_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(node_->get_logger(), "NavigateToPose action server not available");
      return BT::NodeStatus::FAILURE;
    }

    // Send goal
    nav2_msgs::action::NavigateToPose::Goal nav_goal;
    nav_goal.pose = goal;
    auto future = nav_client_->async_send_goal(nav_goal);
    if (rclcpp::spin_until_future_complete(node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to send navigation goal");
      return BT::NodeStatus::FAILURE;
    }
    auto result_future = nav_client_->async_get_result(future.get());
    if (rclcpp::spin_until_future_complete(node_, result_future) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to get navigation result");
      return BT::NodeStatus::FAILURE;
    }

    // Log complete
    msg.status = "COMPLETE";
    msg.details = "Backward navigation complete";
    pub_->publish(msg);

    return BT::NodeStatus::SUCCESS;
  }
  static BT::PortsList providedPorts() {
    return { BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
             BT::InputPort<geometry_msgs::msg::PoseStamped>("robot_pose") };
  }
private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<moonpie_osamu::msg::BehaviorStatus>::SharedPtr pub_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;
  rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_pub_;
}; 