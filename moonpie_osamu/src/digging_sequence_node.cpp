#include <memory>
#include <chrono>
#include <functional>
#include <string>
#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "moonpie_osamu/msg/behavior_status.hpp"
#include "moonpie_osamu/msg/mission_command.hpp"

using namespace std::chrono_literals;
using json = nlohmann::json;

// Timing constants from arduino_control.py
constexpr double ACTUATOR_EXTEND_S = 6.7;
constexpr double DRIVE_FORWARD_S = 20.0;
constexpr double DUMP_ROTATE_EVERY_S = 10.0;
constexpr double DUMP_ROTATE_PERIOD_S = 1.0;
constexpr double ACTUATOR_RETRACT_S = 4.0;
constexpr double DRIVE_AND_DIG_SPEED_MPS = 0.075;

// PID constants
constexpr double Kp = 4.0;
constexpr double Ki = 0.2;

class DiggingSequenceNode : public rclcpp::Node
{
public:
  DiggingSequenceNode() : Node("digging_sequence_node")
  {
    // Create publishers
    arduino_command_pub_ = this->create_publisher<std_msgs::msg::String>("arduino_command", 10);
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    status_pub_ = this->create_publisher<moonpie_osamu::msg::BehaviorStatus>("mission/log", 10);
    cmd_pub_ = this->create_publisher<moonpie_osamu::msg::MissionCommand>("mission/cmd", 10);

    // Create subscription for dig command
    dig_command_sub_ = this->create_subscription<moonpie_osamu::msg::MissionCommand>(
      "mission/cmd", 10,
      std::bind(&DiggingSequenceNode::dig_command_callback, this, std::placeholders::_1));

    // Create timer for sequence control
    sequence_timer_ = this->create_wall_timer(
      100ms, std::bind(&DiggingSequenceNode::sequence_timer_callback, this));

    // Initialize control message
    control_msg_ = {
      {"cmd", true},
      {"dump_belt", 0},
      {"dig_belt", 0},
      {"actuator_extend", false},
      {"actuator_retract", false},
      {"dpad", {{"x", 0}, {"y", 0}}},
      {"Kp", Kp},
      {"Ki", Ki}
    };

    // Initialize previous state
    prev_control_msg_ = control_msg_;

    // Publish initial status
    publishStatus("IDLE", "Digging sequence node initialized");

    RCLCPP_INFO(this->get_logger(), "Digging sequence node initialized");
  }

private:
  void dig_command_callback(const moonpie_osamu::msg::MissionCommand::SharedPtr msg)
  {
    if (msg->command == "START_DIG" && !is_sequence_running_)
    {
      RCLCPP_INFO(this->get_logger(), "Starting digging sequence");
      is_sequence_running_ = true;
      current_step_ = 0;
      step_start_time_ = this->now();
      // Reset control message to initial state
      control_msg_ = {
        {"cmd", true},
        {"dump_belt", 0},
        {"dig_belt", 0},
        {"actuator_extend", false},
        {"actuator_retract", false},
        {"dpad", {{"x", 0}, {"y", 0}}},
        {"Kp", Kp},
        {"Ki", Ki}
      };
      prev_control_msg_ = control_msg_;
      publishStatus("DIGGING", "Starting digging sequence");

      // Send disable manual control command
      auto disable_msg = std::make_unique<moonpie_osamu::msg::MissionCommand>();
      disable_msg->command = "DISABLE_MANUAL";
      cmd_pub_->publish(std::move(disable_msg));
    }
    else if (msg->command == "STOP_DIG" && is_sequence_running_)
    {
      RCLCPP_INFO(this->get_logger(), "Stopping digging sequence");
      is_sequence_running_ = false;
      // Stop all actuators and movement
      control_msg_["dig_belt"] = 0;
      control_msg_["dump_belt"] = 0;
      control_msg_["actuator_extend"] = false;
      control_msg_["actuator_retract"] = false;
      sendControlMessage();
      // Stop the robot
      auto twist_msg = geometry_msgs::msg::Twist();
      cmd_vel_pub_->publish(twist_msg);
      publishStatus("IDLE", "Digging sequence stopped by user");

      // Send enable manual control command
      auto enable_msg = std::make_unique<moonpie_osamu::msg::MissionCommand>();
      enable_msg->command = "ENABLE_MANUAL";
      cmd_pub_->publish(std::move(enable_msg));
    }
  }

  void sendControlMessage()
  {
    // Only send messages if we're running a sequence
    if (!is_sequence_running_) {
      return;
    }

    // Check if any values have changed
    bool has_changes = false;
    
    // Check simple values
    if (control_msg_["cmd"] != prev_control_msg_["cmd"] ||
        control_msg_["dump_belt"] != prev_control_msg_["dump_belt"] ||
        control_msg_["dig_belt"] != prev_control_msg_["dig_belt"] ||
        control_msg_["actuator_extend"] != prev_control_msg_["actuator_extend"] ||
        control_msg_["actuator_retract"] != prev_control_msg_["actuator_retract"] ||
        control_msg_["Kp"] != prev_control_msg_["Kp"] ||
        control_msg_["Ki"] != prev_control_msg_["Ki"]) {
      has_changes = true;
    }
    
    // Check dpad values
    if (control_msg_["dpad"]["x"] != prev_control_msg_["dpad"]["x"] ||
        control_msg_["dpad"]["y"] != prev_control_msg_["dpad"]["y"]) {
      has_changes = true;
    }

    // Only send message if there are changes
    if (has_changes) {
      auto msg = std_msgs::msg::String();
      msg.data = control_msg_.dump();
      arduino_command_pub_->publish(msg);
      
      // Update previous state
      prev_control_msg_ = control_msg_;
    }
  }

  void publishStatus(const std::string& status, const std::string& details)
  {
    auto status_msg = moonpie_osamu::msg::BehaviorStatus();
    status_msg.timestamp = this->now();
    status_msg.current_node = "digging_sequence";
    status_msg.status = status;
    status_msg.details = details;
    status_pub_->publish(status_msg);
  }

  void sequence_timer_callback()
  {
    if (!is_sequence_running_)
    {
      return;
    }

    auto current_time = this->now();
    auto elapsed = (current_time - step_start_time_).seconds();

    switch (current_step_)
    {
      case 0: // Extend actuator with dig belt on
        if (elapsed < ACTUATOR_EXTEND_S)
        {
          // Only update and send if values are different
          if (!control_msg_["actuator_extend"] || control_msg_["dig_belt"] != 1 || control_msg_["dump_belt"] != 0)
          {
            control_msg_["actuator_extend"] = true;
            control_msg_["dig_belt"] = 1;
            control_msg_["dump_belt"] = 0;
            sendControlMessage();
          }
          // Stop the robot
          auto twist_msg = geometry_msgs::msg::Twist();
          cmd_vel_pub_->publish(twist_msg);
        }
        else
        {
          current_step_++;
          step_start_time_ = current_time;
        }
        break;

      case 1: // Drive forward with dig belt on
        if (elapsed < DRIVE_FORWARD_S)
        {
          // Keep dig belt on
          bool dump_belt_on = fmod(elapsed, DUMP_ROTATE_EVERY_S) < DUMP_ROTATE_PERIOD_S;
          
          // Only update and send if values are different
          if (control_msg_["dig_belt"] != 1 || 
              control_msg_["actuator_extend"] || 
              control_msg_["dump_belt"] != (dump_belt_on ? 1 : 0))
          {
            control_msg_["dig_belt"] = 1;
            control_msg_["actuator_extend"] = false;
            control_msg_["dump_belt"] = dump_belt_on ? 1 : 0;
            sendControlMessage();
          }
          // Drive forward
          auto twist_msg = geometry_msgs::msg::Twist();
          twist_msg.linear.x = DRIVE_AND_DIG_SPEED_MPS;
          cmd_vel_pub_->publish(twist_msg);
        }
        else
        {
          // Stop driving
          auto twist_msg = geometry_msgs::msg::Twist();
          cmd_vel_pub_->publish(twist_msg);
          current_step_++;
          step_start_time_ = current_time;
        }
        break;

      case 2: // Retract actuator with dig belt on
        if (elapsed < ACTUATOR_RETRACT_S)
        {
          // Only update and send if values are different
          if (control_msg_["dig_belt"] != 1 || 
              !control_msg_["actuator_retract"] || 
              control_msg_["dump_belt"] != 0)
          {
            control_msg_["dig_belt"] = 1;
            control_msg_["actuator_retract"] = true;
            control_msg_["dump_belt"] = 0;
            sendControlMessage();
          }
          // Stop the robot
          auto twist_msg = geometry_msgs::msg::Twist();
          cmd_vel_pub_->publish(twist_msg);
        }
        else
        {
          // Stop everything
          if (control_msg_["dig_belt"] != 0 || 
              control_msg_["actuator_retract"] || 
              control_msg_["dump_belt"] != 0)
          {
            control_msg_["dig_belt"] = 0;
            control_msg_["actuator_retract"] = false;
            control_msg_["dump_belt"] = 0;
            sendControlMessage();
          }
          
          // Stop the robot
          auto twist_msg = geometry_msgs::msg::Twist();
          cmd_vel_pub_->publish(twist_msg);
          
          // Sequence complete
          is_sequence_running_ = false;
          publishStatus("IDLE", "Digging sequence completed");
          //here
          RCLCPP_INFO(this->get_logger(), "Digging sequence completed");

          // Send enable manual control command
          auto enable_msg = std::make_unique<moonpie_osamu::msg::MissionCommand>();
          enable_msg->command = "ENABLE_MANUAL";
          cmd_pub_->publish(std::move(enable_msg));
        }
        break;
    }
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr arduino_command_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<moonpie_osamu::msg::BehaviorStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<moonpie_osamu::msg::MissionCommand>::SharedPtr cmd_pub_;
  rclcpp::Subscription<moonpie_osamu::msg::MissionCommand>::SharedPtr dig_command_sub_;
  rclcpp::TimerBase::SharedPtr sequence_timer_;

  bool is_sequence_running_ = false;
  int current_step_ = 0;
  rclcpp::Time step_start_time_;
  json control_msg_;
  json prev_control_msg_;  // Track previous state for change detection
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DiggingSequenceNode>());
  rclcpp::shutdown();
  return 0;
} 