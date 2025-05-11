#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "moonpie_osamu/msg/behavior_status.hpp"
#include <chrono>
#include <thread>

class ScanDumpZone : public BT::SyncActionNode {
public:
  ScanDumpZone(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config), node_(rclcpp::Node::make_shared("scan_dump_zone")) {
    pub_ = node_->create_publisher<moonpie_osamu::msg::BehaviorStatus>("mission/log", 10);
    twist_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }
  BT::NodeStatus tick() override {
    // Log start
    auto msg = moonpie_osamu::msg::BehaviorStatus();
    msg.timestamp = node_->now();
    msg.current_node = "ScanDumpZone";
    msg.status = "SCANNING";
    msg.details = "Rotating to scan dump zone";
    msg.message = "";
    pub_->publish(msg);

    // Rotate in place for 3 seconds
    geometry_msgs::msg::Twist twist;
    twist.angular.z = 0.5;
    rclcpp::Rate rate(10);
    for (int i = 0; i < 30; ++i) {
      twist_pub_->publish(twist);
      rate.sleep();
    }
    // Stop rotation
    twist.angular.z = 0.0;
    twist_pub_->publish(twist);

    // Log complete
    msg.status = "COMPLETE";
    msg.details = "Scan complete";
    pub_->publish(msg);

    return BT::NodeStatus::SUCCESS;
  }
  static BT::PortsList providedPorts() { return {}; }
private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<moonpie_osamu::msg::BehaviorStatus>::SharedPtr pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
}; 