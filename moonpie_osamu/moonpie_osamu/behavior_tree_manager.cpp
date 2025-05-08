#include "moonpie_osamu/behavior_tree_manager.hpp"

namespace moonpie_osamu
{

BehaviorTreeManager::BehaviorTreeManager()
: Node("behavior_tree_manager"),
  mission_active_(false)
{
  // Get the behavior tree file paths from parameters
  default_bt_xml_ = this->declare_parameter(
    "default_bt_xml",
    "navigate_w_replanning_and_recovery.xml");
  mission_bt_xml_ = this->declare_parameter(
    "mission_bt_xml",
    "mission_tree.xml");

  // Subscribe to mission start topic
  mission_start_sub_ = this->create_subscription<std_msgs::msg::Empty>(
    "mission/start",
    10,
    std::bind(&BehaviorTreeManager::missionStartCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Behavior Tree Manager initialized");
}

void BehaviorTreeManager::missionStartCallback(const std_msgs::msg::Empty::SharedPtr)
{
  if (!mission_active_) {
    RCLCPP_INFO(this->get_logger(), "Starting mission sequence");
    switchToMissionTree();
    mission_active_ = true;
  }
}

void BehaviorTreeManager::switchToMissionTree()
{
  // TODO: Implement tree switching logic
  // This will require modifying the Nav2 behavior tree navigator to support
  // dynamic tree switching, or creating a custom behavior tree navigator
  RCLCPP_INFO(this->get_logger(), "Switching to mission behavior tree");
}

void BehaviorTreeManager::switchToDefaultTree()
{
  // TODO: Implement tree switching logic
  RCLCPP_INFO(this->get_logger(), "Switching to default behavior tree");
  mission_active_ = false;
}

}  // namespace moonpie_osamu

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<moonpie_osamu::BehaviorTreeManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} 