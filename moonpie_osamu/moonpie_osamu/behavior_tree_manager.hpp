#ifndef BEHAVIOR_TREE_MANAGER_HPP_
#define BEHAVIOR_TREE_MANAGER_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "nav2_bt_navigator/bt_navigator.hpp"

namespace moonpie_osamu
{

class BehaviorTreeManager : public rclcpp::Node
{
public:
  explicit BehaviorTreeManager();

private:
  void missionStartCallback(const std_msgs::msg::Empty::SharedPtr msg);
  void switchToMissionTree();
  void switchToDefaultTree();

  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr mission_start_sub_;
  std::shared_ptr<nav2_bt_navigator::BtNavigator> bt_navigator_;
  std::string default_bt_xml_;
  std::string mission_bt_xml_;
  bool mission_active_;
};

}  // namespace moonpie_osamu

#endif  // BEHAVIOR_TREE_MANAGER_HPP_ 