#include "behaviortree_cpp_v3/action_node.h"
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "moonpie_osamu/msg/behavior_status.hpp"

class Dig : public BT::StatefulActionNode {
public:
  Dig(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config), node_(rclcpp::Node::make_shared("dig")), phase_(0), phase_done_(false), t_(0) {
    pub_ = node_->create_publisher<moonpie_osamu::msg::BehaviorStatus>("mission/log", 10);
    arduino_pub_ = node_->create_publisher<std_msgs::msg::String>("arduino_command", 10);
    twist_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

  BT::NodeStatus onStart() override {
    phase_ = 0;
    t_ = 0;
    phase_done_ = false;
    startPhase();
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override {
    rclcpp::spin_some(node_);
    return phase_done_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
  }

  void onHalted() override {
    if (timer_) timer_->cancel();
    phase_done_ = true;
    // Stop all belts and actuator
    std_msgs::msg::String cmd;
    cmd.data = R"({\"cmd\": true, \"dig_belt\": 0, \"dump_belt\": 0, \"actuator_extend\": false, \"actuator_retract\": false})";
    arduino_pub_->publish(cmd);
    geometry_msgs::msg::Twist twist;
    twist.linear.x = 0.0;
    twist_pub_->publish(twist);
  }

  static BT::PortsList providedPorts() { return {}; }
private:
  void startPhase() {
    auto msg = moonpie_osamu::msg::BehaviorStatus();
    msg.timestamp = node_->now();
    msg.current_node = "Dig";
    msg.status = "DIGGING";
    msg.message = "";
    switch (phase_) {
      case 0: // Lower actuator, dig belt on
        msg.details = "Lowering actuator and starting dig belt";
        pub_->publish(msg);
        sendCmd(R"({\"cmd\": true, \"dig_belt\": 1, \"actuator_extend\": true})");
        timer_ = node_->create_wall_timer(std::chrono::milliseconds(6700), [this]() { nextPhase(); });
        break;
      case 1: // Stop actuator extend, keep dig belt on, start creeping forward
        msg.details = "Creeping forward, dig belt on, pulsing dump belt";
        pub_->publish(msg);
        sendCmd(R"({\"cmd\": true, \"dig_belt\": 1, \"actuator_extend\": false, \"dump_belt\": 0})");
        t_ = 0;
        timer_ = node_->create_wall_timer(std::chrono::seconds(1), [this]() { creepForward(); });
        break;
      case 2: // Retract actuator, dig belt on for 4s
        msg.details = "Retracting actuator, dig belt on";
        pub_->publish(msg);
        sendCmd(R"({\"cmd\": true, \"dig_belt\": 1, \"actuator_retract\": true})");
        geometry_msgs::msg::Twist twist;
        twist.linear.x = 0.0;
        twist_pub_->publish(twist);
        timer_ = node_->create_wall_timer(std::chrono::milliseconds(4000), [this]() { nextPhase(); });
        break;
      case 3: // Stop all
        msg.details = "Dig complete.";
        pub_->publish(msg);
        sendCmd(R"({\"cmd\": true, \"dig_belt\": 0, \"dump_belt\": 0, \"actuator_extend\": false, \"actuator_retract\": false})");
        phase_done_ = true;
        break;
    }
  }

  void nextPhase() {
    if (timer_) timer_->cancel();
    phase_++;
    startPhase();
  }

  void creepForward() {
    // Creep forward for 20s, pulse dump belt at t=0,10
    if (t_ == 0 || t_ == 10) {
      sendCmd(R"({\"cmd\": true, \"dig_belt\": 1, \"dump_belt\": 1})");
    } else {
      sendCmd(R"({\"cmd\": true, \"dig_belt\": 1, \"dump_belt\": 0})");
    }
    geometry_msgs::msg::Twist twist;
    twist.linear.x = 0.075;
    twist_pub_->publish(twist);
    t_++;
    if (t_ == 1 || t_ == 11) {
      // After 1s pulse, turn off dump belt
      auto self = this;
      node_->create_wall_timer(std::chrono::seconds(1), [self]() {
        self->sendCmd(R"({\"cmd\": true, \"dig_belt\": 1, \"dump_belt\": 0})");
      });
    }
    if (t_ < 20) {
      timer_ = node_->create_wall_timer(std::chrono::seconds(1), [this]() { creepForward(); });
    } else {
      nextPhase();
    }
  }

  void sendCmd(const std::string& data) {
    std_msgs::msg::String cmd;
    cmd.data = data;
    arduino_pub_->publish(cmd);
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<moonpie_osamu::msg::BehaviorStatus>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr arduino_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int phase_;
  bool phase_done_;
  int t_;
}; 