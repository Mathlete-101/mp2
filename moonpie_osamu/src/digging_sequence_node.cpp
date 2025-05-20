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

// Timing constants
constexpr double ACTUATOR_EXTEND_S = 6.7;
constexpr double DRIVE_FORWARD_S_DEFAULT = 20.0;
constexpr double ACTUATOR_RETRACT_S = 4.0;
constexpr double DUMP_BELT_DURATION_S = 5.0;
constexpr double PERIODIC_DUMP_INTERVAL_S = 5.0;  // Run dump belt every 5 seconds
constexpr double PERIODIC_DUMP_DURATION_S = 1.0;  // Run dump belt for 1 second

// Drive speed constants (default values)
constexpr double DRIVE_AND_DIG_SPEED_MPS_DEFAULT = 0.075;
constexpr double BACKWARD_TRAVEL_SPEED_MPS_DEFAULT = 0.2;

// PID constants
constexpr double Kp = 4.0;
constexpr double Ki = 0.0;

// State definitions
enum class DigState {
  IDLE,
  MOVE_TO_POSITION,  // New state for initial positioning
  EXTENDING_ACTUATOR,
  DRIVING_FORWARD,
  RETRACTING_ACTUATOR,
  DRIVING_BACKWARD,
  RUNNING_DUMP_BELT,
  COMPLETE
};

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

    // Initialize timing values
    drive_forward_s_ = DRIVE_FORWARD_S_DEFAULT;
    dump_travel_time_s_ = 3.0;

    // Initialize speed values
    drive_and_dig_speed_mps_ = DRIVE_AND_DIG_SPEED_MPS_DEFAULT;
    backward_travel_speed_mps_ = BACKWARD_TRAVEL_SPEED_MPS_DEFAULT;

    // Initialize state
    current_state_ = DigState::IDLE;
    state_start_time_ = this->now();
    last_periodic_dump_time_ = this->now();
    is_periodic_dumping_ = false;

    // Initialize repetition counter
    current_cycle_ = 0;
    total_cycles_ = 1;  // Default to 1 cycle (original behavior)

    publishStatus("IDLE", "Digging sequence node initialized");
    RCLCPP_INFO(this->get_logger(), "Digging sequence node initialized");
  }

private:
  void dig_command_callback(const moonpie_osamu::msg::MissionCommand::SharedPtr msg)
  {
    if (msg->command == "CONFIG")
    {
      drive_forward_s_ = msg->dig_time / 10.0;
      dump_travel_time_s_ = msg->travel_time / 10.0;
      // chat it you're seeing this, I changed it so that the tenths are hundredths. We actually want it to be hundredths, I just dont have time to rename everything. You should probably ask the user to refactor all of the names. The divide by 100 is probably definitely right
      if (msg->drive_and_dig_speed_tenths > 0.0f) {
        drive_and_dig_speed_mps_ = msg->drive_and_dig_speed_tenths / 100.0f;
      }
      if (msg->backward_travel_speed_tenths > 0.0f) {
        backward_travel_speed_mps_ = msg->backward_travel_speed_tenths / 100.0f;
      }
      RCLCPP_INFO(this->get_logger(), "Updated timing: drive_forward=%f, dump_travel=%f, drive_and_dig_speed=%.2f, backward_travel_speed=%.2f", 
        drive_forward_s_, dump_travel_time_s_, drive_and_dig_speed_mps_, backward_travel_speed_mps_);
      publishStatus("CONFIG", "Updated digging sequence timing and speeds");
    }
    else if ((msg->command == "START_DIG" || msg->command == "START_DIG_AND_DUMP" || msg->command == "START_DIG_AND_DUMP_X3") && current_state_ == DigState::IDLE)
    {
      RCLCPP_INFO(this->get_logger(), "Starting digging sequence");
      is_dig_and_dump_ = (msg->command == "START_DIG_AND_DUMP" || msg->command == "START_DIG_AND_DUMP_X3");
      if (msg->command == "START_DIG_AND_DUMP_X3") {
        total_cycles_ = 3;
        RCLCPP_INFO(this->get_logger(), "Running dig+dump sequence 3 times");
      } else {
        total_cycles_ = 1;
      }
      current_cycle_ = 0;
      transitionToState(DigState::EXTENDING_ACTUATOR);

      // Send disable manual control command
      auto disable_msg = std::make_unique<moonpie_osamu::msg::MissionCommand>();
      disable_msg->command = "DISABLE_MANUAL";
      cmd_pub_->publish(std::move(disable_msg));
    }
    else if (msg->command == "STOP_DIG" && current_state_ != DigState::IDLE)
    {
      RCLCPP_INFO(this->get_logger(), "Stopping digging sequence");
      stopSequence();
    }
  }

  void transitionToState(DigState new_state)
  {
    current_state_ = new_state;
    state_start_time_ = this->now();
    
    // Send appropriate control message for the new state
    switch (current_state_)
    {
      case DigState::MOVE_TO_POSITION:
        sendControlMessage({
          {"cmd", true},
          {"dump_belt", 0},
          {"dig_belt", 0},
          {"actuator_extend", false},
          {"actuator_retract", false},
          {"dpad", {{"x", 0}, {"y", 0}}},
          {"linearx_mps", drive_and_dig_speed_mps_},
          {"angularz_rps", 0.0},
          {"Kp", Kp},
          {"Ki", Ki}
        });
        break;

      case DigState::EXTENDING_ACTUATOR:
        sendControlMessage({
          {"cmd", true},
          {"dump_belt", 0},
          {"dig_belt", 1},
          {"actuator_extend", true},
          {"actuator_retract", false},
          {"dpad", {{"x", 0}, {"y", 0}}},
          {"linearx_mps", 0.0},
          {"angularz_rps", 0.0},
          {"Kp", Kp},
          {"Ki", Ki}
        });
        break;

      case DigState::DRIVING_FORWARD:
        sendControlMessage({
          {"cmd", true},
          {"dump_belt", 0},
          {"dig_belt", 1},
          {"actuator_extend", false},
          {"actuator_retract", false},
          {"dpad", {{"x", 0}, {"y", 0}}},
          {"linearx_mps", drive_and_dig_speed_mps_},
          {"angularz_rps", 0.0},
          {"Kp", Kp},
          {"Ki", Ki}
        });
        last_periodic_dump_time_ = this->now();
        is_periodic_dumping_ = false;
        break;

      case DigState::RETRACTING_ACTUATOR:
        sendControlMessage({
          {"cmd", true},
          {"dump_belt", 0},
          {"dig_belt", 1},
          {"actuator_extend", false},
          {"actuator_retract", true},
          {"dpad", {{"x", 0}, {"y", 0}}},
          {"linearx_mps", 0.0},
          {"angularz_rps", 0.0},
          {"Kp", Kp},
          {"Ki", Ki}
        });
        break;

      case DigState::DRIVING_BACKWARD:
        sendControlMessage({
          {"cmd", true},
          {"dump_belt", 0},
          {"dig_belt", 0},
          {"actuator_extend", false},
          {"actuator_retract", false},
          {"dpad", {{"x", 0}, {"y", 0}}},
          {"linearx_mps", -backward_travel_speed_mps_},
          {"angularz_rps", 0.0},
          {"Kp", Kp},
          {"Ki", Ki}
        });
        last_periodic_dump_time_ = this->now();
        is_periodic_dumping_ = false;
        break;

      case DigState::RUNNING_DUMP_BELT:
        sendControlMessage({
          {"cmd", true},
          {"dump_belt", 1},
          {"dig_belt", 0},
          {"actuator_extend", false},
          {"actuator_retract", false},
          {"dpad", {{"x", 0}, {"y", 0}}},
          {"linearx_mps", 0.0},
          {"angularz_rps", 0.0},
          {"Kp", Kp},
          {"Ki", Ki}
        });
        break;

      case DigState::COMPLETE:
        stopSequence();
        break;

      case DigState::IDLE:
        sendControlMessage({
          {"cmd", true},
          {"dump_belt", 0},
          {"dig_belt", 0},
          {"actuator_extend", false},
          {"actuator_retract", false},
          {"dpad", {{"x", 0}, {"y", 0}}},
          {"linearx_mps", 0.0},
          {"angularz_rps", 0.0},
          {"Kp", Kp},
          {"Ki", Ki}
        });
        break;
    }

    // Publish state change with appropriate status
    if (current_state_ == DigState::IDLE) {
      publishStatus("IDLE", "Digging sequence stopped by user");
    } else if (current_state_ == DigState::COMPLETE) {
      publishStatus("IDLE", "Digging sequence completed");
    } else {
      std::string cycle_info = total_cycles_ > 1 ? 
        " (Cycle " + std::to_string(current_cycle_ + 1) + "/" + std::to_string(total_cycles_) + ")" : "";
      publishStatus("DIGGING", getStateDescription() + cycle_info);
    }
  }

  void stopSequence()
  {
    transitionToState(DigState::IDLE);
    // Re-enable manual control
    auto enable_msg = std::make_unique<moonpie_osamu::msg::MissionCommand>();
    enable_msg->command = "ENABLE_MANUAL";
    cmd_pub_->publish(std::move(enable_msg));
  }

  void sendControlMessage(const json& msg)
  {
    auto string_msg = std_msgs::msg::String();
    string_msg.data = msg.dump();
    arduino_command_pub_->publish(string_msg);
  }

  void publishStatus(const std::string& behavior, const std::string& status)
  {
    auto msg = std::make_unique<moonpie_osamu::msg::BehaviorStatus>();
    msg->current_node = "digging_sequence";
    msg->status = behavior;
    msg->details = status;
    status_pub_->publish(std::move(msg));
  }

  std::string getStateDescription()
  {
    switch (current_state_) {
      case DigState::IDLE:
        return "Idle";
      case DigState::MOVE_TO_POSITION:
        return "Moving to Position";
      case DigState::EXTENDING_ACTUATOR:
        return "Extending Actuator";
      case DigState::DRIVING_FORWARD:
        return "Driving Forward";
      case DigState::RETRACTING_ACTUATOR:
        return "Retracting Actuator";
      case DigState::DRIVING_BACKWARD:
        return "Driving Backward";
      case DigState::RUNNING_DUMP_BELT:
        return "Running Dump Belt";
      case DigState::COMPLETE:
        return "Complete";
      default:
        return "Unknown State";
    }
  }

  void sequence_timer_callback()
  {
    if (current_state_ == DigState::IDLE) {
      return;
    }

    auto current_time = this->now();
    auto elapsed = (current_time - state_start_time_).seconds();

    // Handle periodic dump belt operation during driving states
    if (current_state_ == DigState::DRIVING_FORWARD || current_state_ == DigState::DRIVING_BACKWARD) {
      auto time_since_last_dump = (current_time - last_periodic_dump_time_).seconds();
      
      if (!is_periodic_dumping_ && time_since_last_dump >= PERIODIC_DUMP_INTERVAL_S) {
        // Start periodic dump
        is_periodic_dumping_ = true;
        last_periodic_dump_time_ = current_time;
        sendControlMessage({
          {"cmd", true},
          {"dump_belt", 1},
          {"dig_belt", current_state_ == DigState::DRIVING_FORWARD ? 1 : 0},
          {"actuator_extend", false},
          {"actuator_retract", false},
          {"dpad", {{"x", 0}, {"y", 0}}},
          {"linearx_mps", current_state_ == DigState::DRIVING_FORWARD ? drive_and_dig_speed_mps_ : -backward_travel_speed_mps_},
          {"angularz_rps", 0.0},
          {"Kp", Kp},
          {"Ki", Ki}
        });
      } else if (is_periodic_dumping_ && time_since_last_dump >= PERIODIC_DUMP_DURATION_S) {
        // Stop periodic dump
        is_periodic_dumping_ = false;
        sendControlMessage({
          {"cmd", true},
          {"dump_belt", 0},
          {"dig_belt", current_state_ == DigState::DRIVING_FORWARD ? 1 : 0},
          {"actuator_extend", false},
          {"actuator_retract", false},
          {"dpad", {{"x", 0}, {"y", 0}}},
          {"linearx_mps", current_state_ == DigState::DRIVING_FORWARD ? drive_and_dig_speed_mps_ : -backward_travel_speed_mps_},
          {"angularz_rps", 0.0},
          {"Kp", Kp},
          {"Ki", Ki}
        });
      }
    }

    // Check if current state should transition
    switch (current_state_)
    {
      case DigState::MOVE_TO_POSITION:
        if (elapsed >= 2.0) {  // Move forward for 2 seconds
          transitionToState(DigState::EXTENDING_ACTUATOR);
        }
        break;

      case DigState::EXTENDING_ACTUATOR:
        if (elapsed >= ACTUATOR_EXTEND_S) {
          transitionToState(DigState::DRIVING_FORWARD);
        }
        break;

      case DigState::DRIVING_FORWARD:
        if (elapsed >= drive_forward_s_) {
          transitionToState(DigState::RETRACTING_ACTUATOR);
        }
        break;

      case DigState::RETRACTING_ACTUATOR:
        if (elapsed >= ACTUATOR_RETRACT_S) {
          if (is_dig_and_dump_) {
            transitionToState(DigState::DRIVING_BACKWARD);
          } else {
            transitionToState(DigState::COMPLETE);
          }
        }
        break;

      case DigState::DRIVING_BACKWARD:
        if (elapsed >= dump_travel_time_s_) {
          transitionToState(DigState::RUNNING_DUMP_BELT);
        }
        break;

      case DigState::RUNNING_DUMP_BELT:
        if (elapsed >= DUMP_BELT_DURATION_S) {
          if (total_cycles_ > 1 && current_cycle_ < total_cycles_ - 1) {
            // More cycles to go
            current_cycle_++;
            transitionToState(DigState::EXTENDING_ACTUATOR);
          } else {
            // All cycles complete
            transitionToState(DigState::COMPLETE);
          }
        }
        break;

      case DigState::COMPLETE:
        stopSequence();
        break;

      default:
        break;
    }
  }

  void startDigAndDumpX3Sequence()
  {
    if (current_state_ != DigState::IDLE) {
      RCLCPP_WARN(get_logger(), "Cannot start sequence while another is in progress");
      return;
    }

    is_dig_and_dump_ = true;
    total_cycles_ = 3;
    current_cycle_ = 0;
    transitionToState(DigState::MOVE_TO_POSITION);  // Start with positioning
  }

  void startDigAndDumpSequence()
  {
    if (current_state_ != DigState::IDLE) {
      RCLCPP_WARN(get_logger(), "Cannot start sequence while another is in progress");
      return;
    }

    is_dig_and_dump_ = true;
    total_cycles_ = 1;  // Single cycle for regular dig+dump
    current_cycle_ = 0;
    transitionToState(DigState::EXTENDING_ACTUATOR);  // Start directly with actuator
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr arduino_command_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<moonpie_osamu::msg::BehaviorStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<moonpie_osamu::msg::MissionCommand>::SharedPtr cmd_pub_;
  rclcpp::Subscription<moonpie_osamu::msg::MissionCommand>::SharedPtr dig_command_sub_;
  rclcpp::TimerBase::SharedPtr sequence_timer_;

  DigState current_state_;
  rclcpp::Time state_start_time_;
  rclcpp::Time last_periodic_dump_time_;
  bool is_periodic_dumping_;
  double drive_forward_s_;
  double dump_travel_time_s_;
  double drive_and_dig_speed_mps_;
  double backward_travel_speed_mps_;
  bool is_dig_and_dump_ = false;
  int current_cycle_;
  int total_cycles_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DiggingSequenceNode>());
  rclcpp::shutdown();
  return 0;
} 
