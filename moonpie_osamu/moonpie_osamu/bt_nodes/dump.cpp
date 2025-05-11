#include "behaviortree_cpp_v3/action_node.h"
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "moonpie_osamu/msg/behavior_status.hpp"

class Dump : public BT::StatefulActionNode {
public:
  Dump(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config), node_(rclcpp::Node::make_shared("dump")), dumping_(false) {
    pub_ = node_->create_publisher<moonpie_osamu::msg::BehaviorStatus>("mission/log", 10);
    arduino_pub_ = node_->create_publisher<std_msgs::msg::String>("arduino_command", 10);
  }

  BT::NodeStatus onStart() override {
    dumping_ = true;
    // Log start
    auto msg = moonpie_osamu::msg::BehaviorStatus();
    msg.timestamp = node_->now();
    msg.current_node = "Dump";
    msg.status = "DUMPING";
    msg.details = "Activating dump belt for 5 seconds...";
    msg.message = "";
    pub_->publish(msg);
    // Activate dump belt
    std_msgs::msg::String cmd_on;
    cmd_on.data = R"({\"cmd\": true, \"dump_belt\": 1})";
    arduino_pub_->publish(cmd_on);
    // Start timer
    timer_ = node_->create_wall_timer(
      std::chrono::seconds(5),
      [this]() {
        // Deactivate dump belt
        std_msgs::msg::String cmd_off;
        cmd_off.data = R"({\"cmd\": true, \"dump_belt\": 0})";
        arduino_pub_->publish(cmd_off);
        // Log complete
        auto msg = moonpie_osamu::msg::BehaviorStatus();
        msg.timestamp = node_->now();
        msg.current_node = "Dump";
        msg.status = "COMPLETE";
        msg.details = "Dump complete.";
        msg.message = "";
        pub_->publish(msg);
        dumping_ = false;
        timer_->cancel();
      }
    );
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override {
    return dumping_ ? BT::NodeStatus::RUNNING : BT::NodeStatus::SUCCESS;
  }

  void onHalted() override {
    if (timer_) timer_->cancel();
    dumping_ = false;
    // Ensure dump belt is off
    std_msgs::msg::String cmd_off;
    cmd_off.data = R"({\"cmd\": true, \"dump_belt\": 0})";
    arduino_pub_->publish(cmd_off);
  }

  static BT::PortsList providedPorts() { return {}; }
private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<moonpie_osamu::msg::BehaviorStatus>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr arduino_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool dumping_;
}; 