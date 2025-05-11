#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "aruco_opencv_msgs/msg/aruco_detection.hpp"
#include "moonpie_osamu/msg/behavior_status.hpp"

class FindAruco : public BT::SyncActionNode {
public:
  FindAruco(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config), node_(rclcpp::Node::make_shared("find_aruco")), found_(false) {
    int marker_id;
    getInput("marker_id", marker_id);
    sub_ = node_->create_subscription<aruco_opencv_msgs::msg::ArucoDetection>(
      "/aruco_detections", 10, [this, marker_id](const aruco_opencv_msgs::msg::ArucoDetection::SharedPtr msg) {
        for (const auto& marker : msg->markers) {
          if (marker.marker_id == marker_id) found_ = true;
        }
      });
    pub_ = node_->create_publisher<moonpie_osamu::msg::BehaviorStatus>("mission/log", 10);
    // Publish searching status
    auto msg = moonpie_osamu::msg::BehaviorStatus();
    msg.timestamp = node_->now();
    msg.current_node = "FindAruco";
    msg.status = "SEARCHING";
    msg.details = "Searching for ArUco marker";
    msg.message = "";
    pub_->publish(msg);
    spin_thread_ = std::thread([this]() { rclcpp::spin(node_); });
  }
  ~FindAruco() { node_->shutdown(); if (spin_thread_.joinable()) spin_thread_.join(); }
  BT::NodeStatus tick() override {
    if (found_) {
      auto msg = moonpie_osamu::msg::BehaviorStatus();
      msg.timestamp = node_->now();
      msg.current_node = "FindAruco";
      msg.status = "FOUND";
      msg.details = "Found ArUco marker";
      msg.message = "";
      pub_->publish(msg);
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }
  static BT::PortsList providedPorts() {
    return { BT::InputPort<int>("marker_id") };
  }
private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<aruco_opencv_msgs::msg::ArucoDetection>::SharedPtr sub_;
  rclcpp::Publisher<moonpie_osamu::msg::BehaviorStatus>::SharedPtr pub_;
  std::atomic<bool> found_;
  std::thread spin_thread_;
}; 