#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "moonpie_osamu/msg/behavior_status.hpp"

class SetDumpGoal : public BT::SyncActionNode {
public:
  SetDumpGoal(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config), node_(rclcpp::Node::make_shared("set_dump_goal")) {
    pub_ = node_->create_publisher<moonpie_osamu::msg::BehaviorStatus>("mission/log", 10);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  BT::NodeStatus tick() override {
    // Get the desired dump zone center from the blackboard
    geometry_msgs::msg::PoseStamped dump_zone_center;
    if (!getInput("dump_zone_center", dump_zone_center)) {
      RCLCPP_ERROR(node_->get_logger(), "No dump_zone_center provided to SetDumpGoal");
      return BT::NodeStatus::FAILURE;
    }

    // Log start
    auto msg = moonpie_osamu::msg::BehaviorStatus();
    msg.timestamp = node_->now();
    msg.current_node = "SetDumpGoal";
    msg.status = "CALCULATING";
    msg.details = "Calculating dump goal with tf2 offset";
    msg.message = "";
    pub_->publish(msg);

    // Look up the transform from base_link to dump_link
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform("base_link", "dump_link", tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to lookup transform: %s", ex.what());
      return BT::NodeStatus::FAILURE;
    }

    // Apply the transform to the dump zone center to get the goal for base_link
    geometry_msgs::msg::PoseStamped dump_goal;
    dump_goal.header = dump_zone_center.header;
    dump_goal.pose.position.x = dump_zone_center.pose.position.x - transform.transform.translation.x;
    dump_goal.pose.position.y = dump_zone_center.pose.position.y - transform.transform.translation.y;
    dump_goal.pose.position.z = dump_zone_center.pose.position.z - transform.transform.translation.z;
    dump_goal.pose.orientation = dump_zone_center.pose.orientation;

    // Set the dump_goal on the blackboard
    setOutput("dump_goal", dump_goal);

    // Log complete
    msg.status = "COMPLETE";
    msg.details = "Dump goal set with tf2 offset";
    pub_->publish(msg);

    return BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts() {
    return { BT::InputPort<geometry_msgs::msg::PoseStamped>("dump_zone_center"),
             BT::OutputPort<geometry_msgs::msg::PoseStamped>("dump_goal") };
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<moonpie_osamu::msg::BehaviorStatus>::SharedPtr pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
}; 