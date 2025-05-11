#pragma once

#include <string>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace moonpie_osamu {

enum class MissionStateEnum {
    IDLE,
    LOCALIZING,
    EXCAVATING,
    DUMPING,
    RECOVERING,
    COMPLETED,
    FAILED
};

class MissionState {
public:
    MissionState(rclcpp::Node::SharedPtr node);
    
    // State management
    void setState(MissionStateEnum new_state);
    MissionStateEnum getCurrentState() const;
    std::string getStateString() const;
    
    // Statistics tracking
    void incrementCycles();
    void incrementFailedAttempts();
    void startMission();
    void endMission();
    
    // Getters for statistics
    int getCyclesCompleted() const;
    int getFailedAttempts() const;
    double getMissionDuration() const;
    
    // State validation
    bool isValidStateTransition(MissionStateEnum new_state) const;
    
private:
    rclcpp::Node::SharedPtr node_;
    MissionStateEnum current_state_;
    int cycles_completed_;
    int failed_attempts_;
    std::chrono::time_point<std::chrono::steady_clock> mission_start_time_;
    bool mission_active_;
    
    void logStateTransition(MissionStateEnum old_state, MissionStateEnum new_state);
    std::string stateToString(MissionStateEnum state) const;
};

} // namespace moonpie_osamu 