#pragma once

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include "moonpie_osamu/mission_state.hpp"

namespace moonpie_osamu {

class BehaviorTreeManager {
public:
    BehaviorTreeManager(rclcpp::Node::SharedPtr node);
    
    // Tree management
    bool switchToMissionTree();
    bool switchToDefaultTree();
    bool isMissionTreeActive() const;
    
    // Mission control
    void startMission();
    void stopMission();
    bool isMissionRunning() const;
    
    // Tree execution
    void tick();
    void reset();
    
private:
    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<BT::BehaviorTreeFactory> factory_;
    std::unique_ptr<BT::Tree> current_tree_;
    std::unique_ptr<BT::Tree> mission_tree_;
    std::unique_ptr<BT::Tree> default_tree_;
    std::shared_ptr<MissionState> mission_state_;
    bool mission_running_;
    
    // Tree loading and validation
    bool loadTrees();
    bool validateTree(const BT::Tree& tree);
    void registerNodes();
    
    // Logging
    void logTreeSwitch(const std::string& tree_name);
    void logTreeError(const std::string& error_msg);
};

} // namespace moonpie_osamu 