#include "moonpie_osamu/mission_state.hpp"
#include <rclcpp/rclcpp.hpp>

namespace moonpie_osamu {

MissionState::MissionState(rclcpp::Node::SharedPtr node)
    : node_(node),
      current_state_(MissionStateEnum::IDLE),
      cycles_completed_(0),
      failed_attempts_(0),
      mission_active_(false)
{
    RCLCPP_INFO(node_->get_logger(), "MissionState initialized");
}

void MissionState::setState(MissionStateEnum new_state) {
    if (!isValidStateTransition(new_state)) {
        RCLCPP_WARN(node_->get_logger(), "Invalid state transition from %s to %s",
                    stateToString(current_state_).c_str(),
                    stateToString(new_state).c_str());
        return;
    }
    
    MissionStateEnum old_state = current_state_;
    current_state_ = new_state;
    logStateTransition(old_state, new_state);
}

MissionStateEnum MissionState::getCurrentState() const {
    return current_state_;
}

std::string MissionState::getStateString() const {
    return stateToString(current_state_);
}

void MissionState::incrementCycles() {
    cycles_completed_++;
    RCLCPP_INFO(node_->get_logger(), "Cycle completed. Total cycles: %d", cycles_completed_);
}

void MissionState::incrementFailedAttempts() {
    failed_attempts_++;
    RCLCPP_WARN(node_->get_logger(), "Failed attempt recorded. Total failures: %d", failed_attempts_);
}

void MissionState::startMission() {
    if (!mission_active_) {
        mission_active_ = true;
        mission_start_time_ = std::chrono::steady_clock::now();
        RCLCPP_INFO(node_->get_logger(), "Mission started");
    }
}

void MissionState::endMission() {
    if (mission_active_) {
        mission_active_ = false;
        RCLCPP_INFO(node_->get_logger(), "Mission ended. Duration: %.2f seconds", getMissionDuration());
    }
}

int MissionState::getCyclesCompleted() const {
    return cycles_completed_;
}

int MissionState::getFailedAttempts() const {
    return failed_attempts_;
}

double MissionState::getMissionDuration() const {
    if (!mission_active_) {
        return 0.0;
    }
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration<double>(now - mission_start_time_).count();
}

bool MissionState::isValidStateTransition(MissionStateEnum new_state) const {
    // Define valid state transitions
    switch (current_state_) {
        case MissionStateEnum::IDLE:
            return new_state == MissionStateEnum::LOCALIZING;
        case MissionStateEnum::LOCALIZING:
            return new_state == MissionStateEnum::EXCAVATING || 
                   new_state == MissionStateEnum::FAILED;
        case MissionStateEnum::EXCAVATING:
            return new_state == MissionStateEnum::DUMPING || 
                   new_state == MissionStateEnum::RECOVERING ||
                   new_state == MissionStateEnum::FAILED;
        case MissionStateEnum::DUMPING:
            return new_state == MissionStateEnum::EXCAVATING || 
                   new_state == MissionStateEnum::RECOVERING ||
                   new_state == MissionStateEnum::COMPLETED ||
                   new_state == MissionStateEnum::FAILED;
        case MissionStateEnum::RECOVERING:
            return new_state == MissionStateEnum::EXCAVATING || 
                   new_state == MissionStateEnum::FAILED;
        case MissionStateEnum::COMPLETED:
        case MissionStateEnum::FAILED:
            return new_state == MissionStateEnum::IDLE;
        default:
            return false;
    }
}

void MissionState::logStateTransition(MissionStateEnum old_state, MissionStateEnum new_state) {
    RCLCPP_INFO(node_->get_logger(), 
                "State transition: %s -> %s",
                stateToString(old_state).c_str(),
                stateToString(new_state).c_str());
}

std::string MissionState::stateToString(MissionStateEnum state) const {
    switch (state) {
        case MissionStateEnum::IDLE: return "IDLE";
        case MissionStateEnum::LOCALIZING: return "LOCALIZING";
        case MissionStateEnum::EXCAVATING: return "EXCAVATING";
        case MissionStateEnum::DUMPING: return "DUMPING";
        case MissionStateEnum::RECOVERING: return "RECOVERING";
        case MissionStateEnum::COMPLETED: return "COMPLETED";
        case MissionStateEnum::FAILED: return "FAILED";
        default: return "UNKNOWN";
    }
}

} // namespace moonpie_osamu 