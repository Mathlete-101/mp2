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
ACTUATOR_EXTEND_S = 4.0
DRIVE_FORWARD_S_DEFAULT = 20.0
ACTUATOR_RETRACT_S = 4.3
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
    DRIVING_FORWARD = 5
    RETRACTING_ACTUATOR = 6
    DRIVING_BACKWARD = 7
    RUNNING_DUMP_BELT = 8

    @staticmethod
    def to_string(state):
        states = {
            DigState.IDLE: "IDLE",
            DigState.TELEOP: "TELEOP",
            DigState.MOVE_TO_START: "MOVE_TO_START",
            DigState.MOVE_TO_POSITION: "MOVE_TO_POSITION",
            DigState.EXTENDING_ACTUATOR: "EXTENDING_ACTUATOR",
            DigState.DRIVING_FORWARD: "DRIVING_FORWARD",
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
        self.last_periodic_dump_time = 0.0
        self.is_periodic_dumping = False
        self.drive_forward_s = DRIVE_FORWARD_S_DEFAULT
        self.dump_travel_time_s = 3.0
        self.drive_and_dig_speed_mps = DRIVE_AND_DIG_SPEED_MPS_DEFAULT
        self.backward_travel_speed_mps = BACKWARD_TRAVEL_SPEED_MPS_DEFAULT
        self.is_dig_and_dump = False
        self.current_cycle = 0
        self.total_cycles = 1

        # Create timer for sequence control
        self.sequence_timer = self.create_timer(0.1, self.sequence_timer_callback)
        
        self.get_logger().info('Arduino Control Node has started')
        self.transition_to_state(DigState.TELEOP)  # Start in teleop mode

    def dig_command_callback(self, msg):
        if msg.command == "START_DIG":
            self.is_dig_and_dump = False
            self.total_cycles = 1
            self.current_cycle = 0
            self.transition_to_state(DigState.MOVE_TO_POSITION)
        elif msg.command == "START_DIG_AND_DUMP":
            self.is_dig_and_dump = True
            self.total_cycles = 1
            self.current_cycle = 0
            self.transition_to_state(DigState.MOVE_TO_POSITION)
        elif msg.command == "START_DIG_AND_DUMP_X3":
            self.is_dig_and_dump = True
            self.total_cycles = 3
            self.current_cycle = 0
            self.transition_to_state(DigState.MOVE_TO_START)  # Start with positioning for 3x dig
        elif msg.command == "STOP_DIG":
            self.stop_sequence()
        elif msg.command == "CONFIG":
            self.dig_time_s = msg.dig_time / 10.0
            self.travel_time_s = msg.travel_time / 10.0
            self.drive_and_dig_speed_mps = msg.drive_and_dig_speed_tenths / 10.0
            self.backward_travel_speed_mps = msg.backward_travel_speed_tenths / 10.0
            self.drive_forward_s = self.dig_time_s
            self.dump_travel_time_s = self.travel_time_s

    def transition_to_state(self, new_state):
        self.current_state = new_state
        self.state_start_time = time.time()
        
        # Update control message based on state
        if new_state == DigState.TELEOP:
            # In teleop, we keep the current control values but ensure we're not in any automated mode
            self.update_control_message(
                actuator_extend=False,
                actuator_retract=False,
                linearx_mps=0.0,
                angularz_rps=0.0
            )
        elif new_state == DigState.MOVE_TO_POSITION:
            self.update_control_message(dig_belt=0, dump_belt=0, actuator_extend=False, 
                                     actuator_retract=False, linearx_mps=self.drive_and_dig_speed_mps)
        elif new_state == DigState.EXTENDING_ACTUATOR:
            self.update_control_message(dig_belt=1, dump_belt=0, actuator_extend=True, 
                                     actuator_retract=False, linearx_mps=0.0)
        elif new_state == DigState.DRIVING_FORWARD:
            self.update_control_message(dig_belt=1, dump_belt=0, actuator_extend=False, 
                                     actuator_retract=False, linearx_mps=self.drive_and_dig_speed_mps)
            self.last_periodic_dump_time = time.time()
            self.is_periodic_dumping = False
        elif new_state == DigState.RETRACTING_ACTUATOR:
            self.update_control_message(dig_belt=1, dump_belt=0, actuator_extend=False, 
                                     actuator_retract=True, linearx_mps=0.0)
        elif new_state == DigState.DRIVING_BACKWARD:
            self.update_control_message(dig_belt=0, dump_belt=0, actuator_extend=False, 
                                     actuator_retract=False, linearx_mps=-self.backward_travel_speed_mps)
            self.last_periodic_dump_time = time.time()
            self.is_periodic_dumping = False
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
        self.transition_to_state(DigState.TELEOP)

    def sequence_timer_callback(self):
        if self.current_state in [DigState.IDLE, DigState.TELEOP]:
            return

        current_time = time.time()
        elapsed = current_time - self.state_start_time

        # Handle periodic dump belt operation during driving states
        if self.current_state in [DigState.DRIVING_FORWARD, DigState.DRIVING_BACKWARD]:
            time_since_last_dump = current_time - self.last_periodic_dump_time
            
            if not self.is_periodic_dumping and time_since_last_dump >= PERIODIC_DUMP_INTERVAL_S:
                # Start periodic dump
                self.is_periodic_dumping = True
                self.last_periodic_dump_time = current_time
                self.update_control_message(
                    dump_belt=1,
                    dig_belt=1 if self.current_state == DigState.DRIVING_FORWARD else 0,
                    linearx_mps=self.drive_and_dig_speed_mps if self.current_state == DigState.DRIVING_FORWARD else -self.backward_travel_speed_mps
                )
            elif self.is_periodic_dumping and time_since_last_dump >= PERIODIC_DUMP_DURATION_S:
                # Stop periodic dump
                self.is_periodic_dumping = False
                self.update_control_message(
                    dump_belt=0,
                    dig_belt=1 if self.current_state == DigState.DRIVING_FORWARD else 0,
                    linearx_mps=self.drive_and_dig_speed_mps if self.current_state == DigState.DRIVING_FORWARD else -self.backward_travel_speed_mps
                )

        # State transitions
        if self.current_state == DigState.MOVE_TO_START and elapsed >= 2.0:  # Move forward for 2 seconds
            self.transition_to_state(DigState.MOVE_TO_POSITION)
        elif self.current_state == DigState.MOVE_TO_POSITION and elapsed >= 2.0:
            self.transition_to_state(DigState.EXTENDING_ACTUATOR)
        elif self.current_state == DigState.EXTENDING_ACTUATOR and elapsed >= ACTUATOR_EXTEND_S:
            self.transition_to_state(DigState.DRIVING_FORWARD)
        elif self.current_state == DigState.DRIVING_FORWARD and elapsed >= self.drive_forward_s:
            self.transition_to_state(DigState.RETRACTING_ACTUATOR)
        elif self.current_state == DigState.RETRACTING_ACTUATOR and elapsed >= ACTUATOR_RETRACT_S:
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

        # Continuously send the appropriate control message for the current state
        if self.current_state == DigState.MOVE_TO_START:
            self.update_control_message(dig_belt=0, dump_belt=0, actuator_extend=False, 
                                     actuator_retract=False, linearx_mps=self.drive_and_dig_speed_mps)
        elif self.current_state == DigState.MOVE_TO_POSITION:
            self.update_control_message(dig_belt=0, dump_belt=0, actuator_extend=False, 
                                     actuator_retract=False, linearx_mps=self.drive_and_dig_speed_mps)
        elif self.current_state == DigState.EXTENDING_ACTUATOR:
            self.update_control_message(dig_belt=1, dump_belt=0, actuator_extend=True, 
                                     actuator_retract=False, linearx_mps=0.0)
        elif self.current_state == DigState.DRIVING_FORWARD:
            self.update_control_message(dig_belt=1, dump_belt=0, actuator_extend=False, 
                                     actuator_retract=False, linearx_mps=self.drive_and_dig_speed_mps)
        elif self.current_state == DigState.RETRACTING_ACTUATOR:
            self.update_control_message(dig_belt=1, dump_belt=0, actuator_extend=False, 
                                     actuator_retract=True, linearx_mps=0.0)
        elif self.current_state == DigState.DRIVING_BACKWARD:
            self.update_control_message(dig_belt=0, dump_belt=0, actuator_extend=False, 
                                     actuator_retract=False, linearx_mps=-self.backward_travel_speed_mps)
        elif self.current_state == DigState.RUNNING_DUMP_BELT:
            self.update_control_message(dig_belt=0, dump_belt=1, actuator_extend=False, 
                                     actuator_retract=False, linearx_mps=0.0)

    def joy(self, msg):
        if self.current_state != DigState.TELEOP:
            return

        # Update control message for manual control
        self.control_message.update({
            "dump_belt": msg.buttons[1],
            "dig_belt": msg.buttons[0],
            "actuator_extend": msg.axes[7] < 0,
            "actuator_retract": msg.axes[7] > 0,
            "dpad": {"x": msg.axes[6], "y": msg.axes[7]},
        })
        self.send_control_message()

    def cmd_vel(self, msg):
        if self.current_state != DigState.TELEOP:
            return

        # Update velocity message
        self.control_message.update({
            "linearx_mps": msg.linear.x,
            "angularz_rps": msg.angular.z * -1,
        })
        self.send_control_message()

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
