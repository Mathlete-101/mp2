#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "moonpie_osamu/msg/behavior_status.hpp"
#include "moonpie_osamu/msg/mission_command.hpp"

class WaitForStartCommand : public BT::SyncActionNode {
public:
  WaitForStartCommand(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config), node_(rclcpp::Node::make_shared("wait_for_start_command")), started_(false) {
    sub_ = node_->create_subscription<std_msgs::msg::Empty>(
      "/mission/start", 10, [this](const std_msgs::msg::Empty::SharedPtr) { started_ = true; });
    sub_cmd_ = node_->create_subscription<moonpie_osamu::msg::MissionCommand>(
      "/mission/cmd", 10, [this](const moonpie_osamu::msg::MissionCommand::SharedPtr msg) {
        if (msg->command == "START") started_ = true;
      });
    spin_thread_ = std::thread([this]() { rclcpp::spin(node_); });
    pub_ = node_->create_publisher<moonpie_osamu::msg::BehaviorStatus>("mission/log", 10);
    // Publish waiting status
    auto msg = moonpie_osamu::msg::BehaviorStatus();
    msg.timestamp = node_->now();
    msg.current_node = "WaitForStartCommand";
    msg.status = "WAITING";
    msg.details = "Waiting for start command";
    msg.message = "";
    pub_->publish(msg);
  }
  ~WaitForStartCommand() { node_->shutdown(); if (spin_thread_.joinable()) spin_thread_.join(); }
  BT::NodeStatus tick() override {
    if (started_) {
      auto msg = moonpie_osamu::msg::BehaviorStatus();
      msg.timestamp = node_->now();
      msg.current_node = "WaitForStartCommand";
      msg.status = "STARTED";
      msg.details = "Received start command";
      msg.message = "";
      pub_->publish(msg);
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }
  static BT::PortsList providedPorts() { return {}; }
private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_;
  rclcpp::Subscription<moonpie_osamu::msg::MissionCommand>::SharedPtr sub_cmd_;
  std::atomic<bool> started_;
  std::thread spin_thread_;
  rclcpp::Publisher<moonpie_osamu::msg::BehaviorStatus>::SharedPtr pub_;
}; 