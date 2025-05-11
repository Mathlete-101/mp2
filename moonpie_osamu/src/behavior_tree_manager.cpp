#include "moonpie_osamu/behavior_tree_manager.hpp"
#include <filesystem>

namespace moonpie_osamu {

BehaviorTreeManager::BehaviorTreeManager(rclcpp::Node::SharedPtr node)
    : node_(node),
      factory_(std::make_unique<BT::BehaviorTreeFactory>()),
      mission_state_(std::make_shared<MissionState>(node)),
      mission_running_(false)
{
    registerNodes();
    if (!loadTrees()) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to load behavior trees");
        return;
    }
    current_tree_ = std::move(default_tree_);
    RCLCPP_INFO(node_->get_logger(), "BehaviorTreeManager initialized");
}

bool BehaviorTreeManager::switchToMissionTree() {
    if (!mission_tree_) {
        logTreeError("Mission tree not loaded");
        return false;
    }
    
    current_tree_ = std::move(mission_tree_);
    logTreeSwitch("Mission");
    return true;
}

bool BehaviorTreeManager::switchToDefaultTree() {
    if (!default_tree_) {
        logTreeError("Default tree not loaded");
        return false;
    }
    
    current_tree_ = std::move(default_tree_);
    logTreeSwitch("Default");
    return true;
}

bool BehaviorTreeManager::isMissionTreeActive() const {
    return current_tree_ == mission_tree_;
}

void BehaviorTreeManager::startMission() {
    if (!mission_running_) {
        mission_running_ = true;
        mission_state_->startMission();
        RCLCPP_INFO(node_->get_logger(), "Mission started");
    }
}

void BehaviorTreeManager::stopMission() {
    if (mission_running_) {
        mission_running_ = false;
        mission_state_->endMission();
        RCLCPP_INFO(node_->get_logger(), "Mission stopped");
    }
}

bool BehaviorTreeManager::isMissionRunning() const {
    return mission_running_;
}

void BehaviorTreeManager::tick() {
    if (!current_tree_ || !mission_running_) {
        return;
    }
    
    auto status = current_tree_->tickRoot();
    
    switch (status) {
        case BT::NodeStatus::SUCCESS:
            mission_state_->incrementCycles();
            break;
        case BT::NodeStatus::FAILURE:
            mission_state_->incrementFailedAttempts();
            break;
        case BT::NodeStatus::RUNNING:
            // Continue execution
            break;
        default:
            RCLCPP_WARN(node_->get_logger(), "Unexpected tree status");
            break;
    }
}

void BehaviorTreeManager::reset() {
    if (current_tree_) {
        current_tree_->haltTree();
    }
    mission_state_->setState(MissionStateEnum::IDLE);
    RCLCPP_INFO(node_->get_logger(), "Behavior tree reset");
}

bool BehaviorTreeManager::loadTrees() {
    try {
        std::string mission_tree_path = "config/mission_tree.xml";
        std::string default_tree_path = "config/default_tree.xml";
        
        if (!std::filesystem::exists(mission_tree_path) || 
            !std::filesystem::exists(default_tree_path)) {
            RCLCPP_ERROR(node_->get_logger(), "Tree XML files not found");
            return false;
        }
        
        mission_tree_ = std::make_unique<BT::Tree>(factory_->createTreeFromFile(mission_tree_path));
        default_tree_ = std::make_unique<BT::Tree>(factory_->createTreeFromFile(default_tree_path));
        
        if (!validateTree(*mission_tree_) || !validateTree(*default_tree_)) {
            return false;
        }
        
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to load trees: %s", e.what());
        return false;
    }
}

bool BehaviorTreeManager::validateTree(const BT::Tree& tree) {
    // Basic validation - check if tree has a root node
    if (tree.rootNode() == nullptr) {
        RCLCPP_ERROR(node_->get_logger(), "Tree validation failed: No root node");
        return false;
    }
    return true;
}

void BehaviorTreeManager::registerNodes() {
    // Register custom nodes here
    // Example: factory_->registerNodeType<CustomNode>("CustomNode");
}

void BehaviorTreeManager::logTreeSwitch(const std::string& tree_name) {
    RCLCPP_INFO(node_->get_logger(), "Switched to %s tree", tree_name.c_str());
}

void BehaviorTreeManager::logTreeError(const std::string& error_msg) {
    RCLCPP_ERROR(node_->get_logger(), "Tree error: %s", error_msg.c_str());
}

} // namespace moonpie_osamu 