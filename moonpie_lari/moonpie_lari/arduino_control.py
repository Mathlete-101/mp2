# arduino_control.py
# manages remote control inputs and builds commands for arduino
import time
from queue import Queue
from threading import Thread
from sensor_msgs.msg import Joy
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import json
from moonpie_osamu.msg import MissionCommand, BehaviorStatus

# Timing constants
DRIVE_FORWARD_S_DEFAULT = 20.0
DUMP_BELT_DURATION_S = 5.0
PERIODIC_DUMP_INTERVAL_S = 5.0
PERIODIC_DUMP_DURATION_S = 1.0

# Drive speed constants
DRIVE_AND_DIG_SPEED_MPS_DEFAULT = 0.075
BACKWARD_TRAVEL_SPEED_MPS_DEFAULT = 0.2

# PID constants
Kp = 4.0
Ki = 0.0

# State definitions
class DigState:
    IDLE = 0
    TELEOP = 1
    MOVE_TO_START = 2  # New state for positioning before 3x dig
    MOVE_TO_POSITION = 3
    EXTENDING_ACTUATOR = 4
    RESET_DRIVE_TIMER = 5  # New state to reset drive timer
    DRIVING_FORWARD = 6
    DRIVING_FORWARD_WITH_DUMP = 7
    RETRACTING_ACTUATOR = 8
    DRIVING_BACKWARD = 9
    RUNNING_DUMP_BELT = 10

    @staticmethod
    def to_string(state):
        states = {
            DigState.IDLE: "IDLE",
            DigState.TELEOP: "TELEOP",
            DigState.MOVE_TO_START: "MOVE_TO_START",
            DigState.MOVE_TO_POSITION: "MOVE_TO_POSITION",
            DigState.EXTENDING_ACTUATOR: "EXTENDING_ACTUATOR",
            DigState.RESET_DRIVE_TIMER: "RESET_DRIVE_TIMER",
            DigState.DRIVING_FORWARD: "DRIVING_FORWARD",
            DigState.DRIVING_FORWARD_WITH_DUMP: "DRIVING_FORWARD_WITH_DUMP",
            DigState.RETRACTING_ACTUATOR: "RETRACTING_ACTUATOR",
            DigState.DRIVING_BACKWARD: "DRIVING_BACKWARD",
            DigState.RUNNING_DUMP_BELT: "RUNNING_DUMP_BELT"
        }
        return states.get(state, "UNKNOWN")

class ArduinoControl(Node):
    def __init__(self):
        super().__init__('arduino_control')
        # Create publishers
        self.arduino_command_publisher = self.create_publisher(String, 'arduino_command', 10)
        self.status_publisher = self.create_publisher(BehaviorStatus, 'mission/log', 10)
        self.cmd_publisher = self.create_publisher(MissionCommand, 'mission/cmd', 10)
        self.resends = 0

        # Create subscriptions
        self.twist_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel,
            10
        )
        self.joy_subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy,
            10
        )
        self.dig_command_subscription = self.create_subscription(
            MissionCommand,
            'mission/cmd',
            self.dig_command_callback,
            10
        )

        # Initialize control message
        self.control_message = {
            "cmd": True,
            "dump_belt": 0,
            "dig_belt": 0,
            "actuator_extend": False,
            "actuator_retract": False,
            "dpad": {"x": 0, "y": 0},
            "linearx_mps": 0.0,
            "angularz_rps": 0.0,
            "Kp": Kp,
            "Ki": Ki
        }
        
        # Initialize digging sequence variables
        self.current_state = DigState.IDLE
        self.state_start_time = 0.0
        self.drive_start_time = 0.0  # Track when we started driving forward
        self.last_periodic_dump_time = 0.0
        self.is_periodic_dumping = False
        self.drive_forward_s = DRIVE_FORWARD_S_DEFAULT
        self.dump_travel_time_s = 3.0
        self.drive_and_dig_speed_mps = DRIVE_AND_DIG_SPEED_MPS_DEFAULT
        self.backward_travel_speed_mps = BACKWARD_TRAVEL_SPEED_MPS_DEFAULT
        self.forward_move_time_s = 2.0  # Default 2.0 seconds
        self.actuator_lower_time_s = 4.0  # Default 4.0 seconds
        self.is_dig_and_dump = False
        self.current_cycle = 0
        self.total_cycles = 1

        # Teleop state tracking
        self.dig_belt_on = False
        self.prev_dig_button = False
        self.x_button_forward = False  # Flag for X button forward movement
        self.y_button_backward = False  # Flag for Y button backward movement
        self.prev_y_button = False  # Track previous Y button state

        # Create timer for sequence control
        self.sequence_timer = self.create_timer(0.1, self.sequence_timer_callback)
        
        self.get_logger().info('Arduino Control Node has started')
        self.transition_to_state(DigState.TELEOP)  # Start in teleop mode

    def dig_command_callback(self, msg):
        if msg.command == "START_DIG":
            self.is_dig_and_dump = False
            self.total_cycles = 1
            self.current_cycle = 0
            self.transition_to_state(DigState.EXTENDING_ACTUATOR)  # Skip MOVE_TO_START for single dig
        elif msg.command == "START_DIG_AND_DUMP":
            self.is_dig_and_dump = True
            self.total_cycles = 1
            self.current_cycle = 0
            self.transition_to_state(DigState.EXTENDING_ACTUATOR)  # Skip MOVE_TO_START for single dig+dump
        elif msg.command == "START_DIG_AND_DUMP_X3":
            self.is_dig_and_dump = True
            self.total_cycles = 3
            self.current_cycle = 0
            self.transition_to_state(DigState.MOVE_TO_POSITION)  # Only use MOVE_TO_START for 3x dig+dump
        elif msg.command == "STOP_DIG":
            self.stop_sequence()
        elif msg.command == "CONFIG":
            self.dig_time_s = msg.dig_time / 10.0
            self.travel_time_s = msg.travel_time / 10.0
            self.drive_and_dig_speed_mps = msg.drive_and_dig_speed_tenths / 100.0
            self.backward_travel_speed_mps = msg.backward_travel_speed_tenths / 100.0
            self.forward_move_time_s = msg.forward_move_time / 10.0
            self.actuator_lower_time_s = msg.actuator_lower_time / 10.0
            self.drive_forward_s = self.dig_time_s
            self.dump_travel_time_s = self.travel_time_s

    def transition_to_state(self, new_state):
        self.current_state = new_state
        self.state_start_time = time.time()
        self.resends = 0
        
        # Reset drive timer when entering RESET_DRIVE_TIMER state
        if new_state == DigState.RESET_DRIVE_TIMER:
            self.drive_start_time = time.time()
        
        # Update control message based on state
        if new_state == DigState.TELEOP:
            # In teleop, we keep the current control values but ensure we're not in any automated mode
            self.update_control_message(
                dump_belt=0,
                dig_belt=0,
                actuator_extend=False,
                actuator_retract=False,
                linearx_mps=0.0,
                angularz_rps=0.0
            )
        elif new_state == DigState.MOVE_TO_POSITION:
            self.update_control_message(dig_belt=0, dump_belt=0, actuator_extend=False, 
                                     actuator_retract=False, linearx_mps=self.backward_travel_speed_mps)
        elif new_state == DigState.EXTENDING_ACTUATOR:
            self.update_control_message(dig_belt=1, dump_belt=0, actuator_extend=True, 
                                     actuator_retract=False, linearx_mps=0.0)
        elif new_state == DigState.RESET_DRIVE_TIMER:
            self.update_control_message(dig_belt=1, dump_belt=0, actuator_extend=False, 
                                     actuator_retract=False, linearx_mps=0.0)
        elif new_state == DigState.DRIVING_FORWARD:
            self.update_control_message(dig_belt=1, dump_belt=0, actuator_extend=False, 
                                     actuator_retract=False, linearx_mps=self.drive_and_dig_speed_mps)
        elif new_state == DigState.DRIVING_FORWARD_WITH_DUMP:
            self.update_control_message(dig_belt=1, dump_belt=1, actuator_extend=False, 
                                     actuator_retract=False, linearx_mps=self.drive_and_dig_speed_mps)
        elif new_state == DigState.RETRACTING_ACTUATOR:
            self.update_control_message(dig_belt=1, dump_belt=0, actuator_extend=False, 
                                     actuator_retract=True, linearx_mps=0.0)
        elif new_state == DigState.DRIVING_BACKWARD:
            self.update_control_message(dig_belt=0, dump_belt=0, actuator_extend=False, 
                                     actuator_retract=False, linearx_mps=-self.backward_travel_speed_mps)
        elif new_state == DigState.RUNNING_DUMP_BELT:
            self.update_control_message(dig_belt=0, dump_belt=1, actuator_extend=False, 
                                     actuator_retract=False, linearx_mps=0.0)

        # Publish state change
        cycle_info = f" (Cycle {self.current_cycle + 1}/{self.total_cycles})" if self.total_cycles > 1 and new_state not in [DigState.IDLE, DigState.TELEOP] else ""
        self.publish_status(DigState.to_string(new_state), DigState.to_string(new_state) + cycle_info)

    def update_control_message(self, **kwargs):
        self.control_message.update(kwargs)
        self.send_control_message()

    def send_control_message(self):
        cmd = json.dumps(self.control_message)
        msg = String()
        msg.data = cmd
        self.arduino_command_publisher.publish(msg)

    def publish_status(self, behavior, status):
        msg = BehaviorStatus()
        msg.current_node = "arduino_control"
        msg.status = behavior
        msg.details = status
        self.status_publisher.publish(msg)

    def stop_sequence(self):
        # Immediately stop all motors and actuators
        self.update_control_message(
            dig_belt=0,
            dump_belt=0,
            actuator_extend=False,
            actuator_retract=False,
            linearx_mps=0.0,
            angularz_rps=0.0
        )
        self.transition_to_state(DigState.TELEOP)

    def sequence_timer_callback(self):
        if self.current_state in [DigState.IDLE, DigState.TELEOP]:
            self.send_control_message()
        else:
            if self.resends < 10 & self.resends % 3 == 0:
                self.send_control_message()
            self.resends+=1

        current_time = time.time()
        elapsed = current_time - self.state_start_time
        total_drive_time = current_time - self.drive_start_time if self.current_state in [DigState.DRIVING_FORWARD, DigState.DRIVING_FORWARD_WITH_DUMP] else 0

        # Debug logging for drive states
        if self.current_state in [DigState.DRIVING_FORWARD, DigState.DRIVING_FORWARD_WITH_DUMP]:
            self.get_logger().info(f'State: {DigState.to_string(self.current_state)}, Total Drive Time: {total_drive_time:.1f}s, Target: {self.drive_forward_s:.1f}s')

        # State transitions
        if self.current_state == DigState.MOVE_TO_START and elapsed >= self.forward_move_time_s:
            if self.total_cycles == 3:  # Only go to MOVE_TO_POSITION for 3x dig and dump
                self.transition_to_state(DigState.MOVE_TO_POSITION)
            else:
                self.transition_to_state(DigState.EXTENDING_ACTUATOR)
        elif self.current_state == DigState.MOVE_TO_POSITION and elapsed >= self.forward_move_time_s:
            self.transition_to_state(DigState.EXTENDING_ACTUATOR)
        elif self.current_state == DigState.EXTENDING_ACTUATOR and elapsed >= self.actuator_lower_time_s:
            self.transition_to_state(DigState.RESET_DRIVE_TIMER)
        elif self.current_state == DigState.RESET_DRIVE_TIMER:
            self.transition_to_state(DigState.DRIVING_FORWARD)
        elif self.current_state == DigState.DRIVING_FORWARD:
            if total_drive_time >= self.drive_forward_s:
                self.get_logger().info(f'Drive time complete: {total_drive_time:.1f}s >= {self.drive_forward_s:.1f}s')
                self.transition_to_state(DigState.RETRACTING_ACTUATOR)
            elif elapsed >= PERIODIC_DUMP_INTERVAL_S:
                self.transition_to_state(DigState.DRIVING_FORWARD_WITH_DUMP)
        elif self.current_state == DigState.DRIVING_FORWARD_WITH_DUMP:
            if total_drive_time >= self.drive_forward_s:
                self.get_logger().info(f'Drive time complete: {total_drive_time:.1f}s >= {self.drive_forward_s:.1f}s')
                self.transition_to_state(DigState.RETRACTING_ACTUATOR)
            elif elapsed >= PERIODIC_DUMP_DURATION_S:
                self.transition_to_state(DigState.DRIVING_FORWARD)
        elif self.current_state == DigState.RETRACTING_ACTUATOR and elapsed >= self.actuator_lower_time_s:
            if self.is_dig_and_dump:
                self.transition_to_state(DigState.DRIVING_BACKWARD)
            else:
                self.transition_to_state(DigState.TELEOP)
        elif self.current_state == DigState.DRIVING_BACKWARD and elapsed >= self.dump_travel_time_s:
            self.transition_to_state(DigState.RUNNING_DUMP_BELT)
        elif self.current_state == DigState.RUNNING_DUMP_BELT and elapsed >= DUMP_BELT_DURATION_S:
            if self.total_cycles > 1 and self.current_cycle < self.total_cycles - 1:
                self.current_cycle += 1
                self.transition_to_state(DigState.EXTENDING_ACTUATOR)
            else:
                self.transition_to_state(DigState.TELEOP)

    def cmd_vel(self, msg):
        if self.current_state != DigState.TELEOP:
            return

        # Update velocity message
        if self.x_button_forward:
            linearx = self.drive_and_dig_speed_mps
        elif self.y_button_backward:
            linearx = -self.drive_and_dig_speed_mps
        else:
            linearx = msg.linear.x

        # Only update movement-related fields
        self.control_message.update({
            "linearx_mps": linearx,
            "angularz_rps": msg.angular.z * -1,
        })
        # Send the complete control message after all updates
        self.send_control_message()

    def joy(self, msg):
        if self.current_state != DigState.TELEOP:
            return

        # Handle dig belt toggle
        dig_button = msg.buttons[0]
        if dig_button and not self.prev_dig_button:  # Button just pressed
            self.dig_belt_on = not self.dig_belt_on  # Toggle state
        self.prev_dig_button = dig_button

        # Update X and Y button movement flags
        x_button = msg.buttons[2]  # X button
        # just disable the y button for now
        y_button = msg.buttons[3]  # Y button (index 3)
        
        # Handle X button state changes
        self.x_button_forward = x_button

        # Handle Y button state changes
        self.y_button_backward = y_button  # Direct assignment like X button

        # Update control message for manual control (only non-movement controls)
        self.control_message.update({
            "dump_belt": msg.buttons[1],
            "dig_belt": 1 if self.dig_belt_on else 0,
            "actuator_extend": msg.axes[7] < 0,
            "actuator_retract": msg.axes[7] > 0,
            "dpad": {"x": msg.axes[6], "y": msg.axes[7]},
        })

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
