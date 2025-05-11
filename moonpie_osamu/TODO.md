# Moonpie Osamu TODO List

## System Overview

### System Goals
- Autonomous excavation and material transport system
- Continuous operation with error recovery
- Clear mission state tracking and logging
- Robust behavior tree-based control system

### Currently Implemented Features
- Basic behavior tree structure with nodes for:
  - ArUco marker detection and localization
  - Navigation to excavation and dump zones
  - Basic digging sequence with actuator control
  - Dump mechanism control
- Transform-based goal calculations for navigation
- Basic node status reporting
- Hardware control interfaces for:
  - Actuator control
  - Drive system
  - Dump mechanism
  - Camera system

### System Architecture
- ROS2-based control system
- Behavior tree for mission sequencing
- Transform-based navigation
- Hardware abstraction layer for actuators and sensors

## 1. Behavior Tree Management & Mission Loop

### Behavior Tree Structure
- [ ] Modify mission_tree.xml to include infinite loop
  - Add Repeat node for excavation cycle
  - Add CheckMissionComplete node
  - Ensure proper sequence of operations

### Tree Switching Implementation
- [ ] Complete BehaviorTreeManager implementation
  - [ ] Implement switchToMissionTree()
  - [ ] Implement switchToDefaultTree()
  - [ ] Add tree validation and error handling
  - [ ] Add logging for tree switching events

### Mission Loop Control
- [ ] Create CheckMissionComplete node
  - [ ] Track number of cycles completed
  - [ ] Implement mission completion criteria
  - [ ] Add logging for cycle completion
  - [ ] Add ability to break loop on completion

## 2. Error Handling and Recovery

### Recovery Sequence Implementation
- [ ] Create RecoverySequence node
  - [ ] Implement retry logic
  - [ ] Add timeout handling
  - [ ] Add logging for recovery attempts

### Navigation Recovery
- [ ] Implement navigation recovery behaviors
  - [ ] Back up behavior
  - [ ] Rotate in place
  - [ ] Path replanning
  - [ ] Add logging for navigation failures

### Digging Recovery
- [ ] Implement digging recovery behaviors
  - [ ] Actuator retraction
  - [ ] Material clearing
  - [ ] Parameter adjustment
  - [ ] Add logging for digging failures

### Dumping Recovery
- [ ] Implement dumping recovery behaviors
  - [ ] Blockage clearing
  - [ ] Position adjustment
  - [ ] Add logging for dumping failures

## 3. Mission State Management

### State Tracking
- [ ] Create MissionState class
  - [ ] Define state enum (IDLE, LOCALIZING, EXCAVATING, DUMPING, RECOVERING, COMPLETED, FAILED)
  - [ ] Implement state transitions
  - [ ] Add state change logging

### Statistics Tracking
- [ ] Implement mission statistics
  - [ ] Track cycles completed
  - [ ] Track failed attempts
  - [ ] Track mission duration
  - [ ] Add logging for statistics updates

### State-Dependent Behavior
- [ ] Add state checks to existing nodes
  - [ ] Navigation state validation
  - [ ] Digging state validation
  - [ ] Dumping state validation
  - [ ] Add logging for state-dependent actions

## 4. Logging System

### Status Messages
- [ ] Enhance behavior status messages
  - [ ] Add detailed state information
  - [ ] Include recovery attempts
  - [ ] Add timing information
  - [ ] Format for output panel display

### Error Logging
- [ ] Implement comprehensive error logging
  - [ ] Log all recovery attempts
  - [ ] Log state transitions
  - [ ] Log mission progress
  - [ ] Format for output panel display

### Mission Progress Logging
- [ ] Add mission progress tracking
  - [ ] Log cycle completion
  - [ ] Log material movement
  - [ ] Log mission milestones
  - [ ] Format for output panel display

## Implementation Notes

### Required Parameters
- Maximum recovery attempts per operation
- Recovery timeouts
- Success criteria for recovery actions
- Mission completion criteria

### State Transition Rules
- Cycle completion criteria
- Recovery trigger conditions
- Mission completion conditions

### Logging Format
- Use consistent timestamp format
- Include node name in all logs
- Include state information
- Include relevant statistics 