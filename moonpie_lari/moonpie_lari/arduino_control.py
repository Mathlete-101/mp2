# arduino_control.py
# manages remote control inputs and builds commands for arduino
# automated digging operation
import time
from queue import Queue
from threading import Thread
from sensor_msgs.msg import Joy
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import json
from moonpie_osamu.msg import BehaviorStatus, MissionCommand

# timing for automated digging sequence
# The actuator will extend with the dig belt on for time
# The rover will drive forward at a slow constand speed with the dig belt on for time
# While driving and digging, the dump belt will rotate for time every time to more evenly load regolith
# After driving and digging, the rover will stop, then retract the dig belt for time with the dig belt on
# Everything stops

ACTUATOR_EXTEND_S = 6.7
#ACTUATOR_EXTEND_S = 6.8
DRIVE_FORWARD_S = 20
DUMP_ROTATE_EVERY_S = 10
DUMP_ROTATE_PERIOD_S = 1
ACTUATOR_RETRACT_S = 4
DRIVE_AND_DIG_SPEED_MPS = 0.075
#DRIVE_AND_DIG_SPEED_MPS = 0.1

# for autonomy, Ki must be small or the rover will slowly increase speed 
Ki = 0.2
Kp = 4

class ArduinoControl(Node):
    def __init__(self):
        super().__init__('arduino_control')
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
        self.mission_cmd_subscription = self.create_subscription(
            MissionCommand,
            'mission/cmd',
            self.mission_cmd_callback,
            10
        )
        self.arduino_command_publisher = self.create_publisher(String, 'arduino_command', 10)
        self.status_pub = self.create_publisher(BehaviorStatus, 'behavior_status', 10)
        
        # Split message into velocity and control components
        self.velocity_message = {
            "cmd": True,
            "linearx_mps": 0.0,
            "angularz_rps": 0.0
        }
        
        self.control_message = {
            "cmd": True,
            "dump_belt": 0,
            "dig_belt": 0,
            "actuator_extend": False,
            "actuator_retract": False,
            "dpad": {"x": 0, "y": 0},
            "Kp": Kp,
            "Ki": Ki
        }
        
        # Track if control values have changed
        self.control_changed = False
        
        # Manual control state
        self.manual_control_enabled = True
        
        self.get_logger().info('Arduino Control Node has started')

        # Set up separate timers for velocity and control messages
        self.velocity_timer = self.create_timer(0.02, self.write_velocity_message)
        self.control_timer = self.create_timer(0.1, self.write_control_message)

        self.autonomous_active = False
        self.autonomous_start_time = 0.0
        self.dump_belt_last_time = 0.0
        self.dump_belt_on_period = False

    def mission_cmd_callback(self, msg):
        if msg.command == "START_DIG":
            self.manual_control_enabled = False
            self.get_logger().info("Manual control disabled for digging sequence")
            self.publish_status("MANUAL_DISABLED", "Manual control disabled for digging sequence")
        elif msg.command == "STOP_DIG":
            self.manual_control_enabled = True
            self.get_logger().info("Manual control re-enabled")
            self.publish_status("MANUAL_ENABLED", "Manual control re-enabled")
        elif msg.command == "DISABLE_MANUAL":
            self.manual_control_enabled = False
            self.get_logger().info("Manual control disabled")
            self.publish_status("MANUAL_DISABLED", "Manual control disabled")
        elif msg.command == "ENABLE_MANUAL":
            self.manual_control_enabled = True
            self.get_logger().info("Manual control enabled")
            self.publish_status("MANUAL_ENABLED", "Manual control enabled")

    def publish_status(self, status, details):
        """Publish status to mission log"""
        msg = BehaviorStatus()
        msg.timestamp = self.get_clock().now().to_msg()
        msg.current_node = "arduino_control"
        msg.status = status
        msg.details = details
        self.status_pub.publish(msg)

    def joy(self, msg):
        if not self.manual_control_enabled:
            return

        # Update control message and mark as changed
        self.control_message.update({
            "dump_belt": msg.buttons[1],
            "dig_belt": msg.buttons[0],
            "actuator_extend": msg.axes[7] < 0,
            "actuator_retract": msg.axes[7] > 0,
            "dpad": {"x": msg.axes[6], "y": msg.axes[7]},
        })
        self.control_changed = True

        # start autonomy if left dpad_x is pressed
        if msg.axes[6] == 1.0 and not self.autonomous_active:
            self.start_autonomous()
        # kill autonomy if right dpad_x is pressed
        if msg.axes[6] == -1.0 and self.autonomous_active:
            self.kill_autonomous()

    def cmd_vel(self, msg):
        if not self.manual_control_enabled:
            return

        # Update velocity message
        self.velocity_message.update({
            "linearx_mps": msg.linear.x * 0.5,
            "angularz_rps": msg.angular.z * -2,
        })

    def write_velocity_message(self):
        if not self.manual_control_enabled and not self.autonomous_active:
            return

        # Always send velocity messages when manual control is enabled or autonomy is active
        cmd = json.dumps(self.velocity_message)
        msg = String()
        msg.data = cmd
        self.arduino_command_publisher.publish(msg)

    def write_control_message(self):
        if not self.manual_control_enabled and not self.autonomous_active:
            return

        # Only send control messages when values change or autonomy is active
        if self.control_changed or self.autonomous_active:
            cmd = json.dumps(self.control_message)
            msg = String()
            msg.data = cmd
            self.arduino_command_publisher.publish(msg)
            self.control_changed = False

    def start_autonomous(self):
        self.get_logger().info("Autonomous process started")
        self.autonomous_active = True
        self.manual_control_enabled = False
        self.autonomous_start_time = time.time()
        self.control_message["actuator_extend"] = True
        self.control_message["dig_belt"] = 1
        self.dump_belt_last_time = time.time()
        self.control_changed = True

    def kill_autonomous(self):
        self.get_logger().info("Autonomous process killed")
        self.autonomous_active = False
        self.manual_control_enabled = True
        self.control_message["actuator_extend"] = False
        self.control_message["dig_belt"] = 0
        self.control_changed = True

    def run_autonomous(self):
        current_time = time.time()
        elapsed_time = current_time - self.autonomous_start_time

        if elapsed_time < ACTUATOR_EXTEND_S:
            self.control_message["actuator_extend"] = True
            self.velocity_message["linearx_mps"] = 0.0
            self.control_message["dig_belt"] = 1
        elif elapsed_time < ACTUATOR_EXTEND_S + DRIVE_FORWARD_S:
            self.control_message["actuator_extend"] = False
            self.velocity_message["linearx_mps"] = DRIVE_AND_DIG_SPEED_MPS
            self.control_message["dig_belt"] = 1

            # Rotate dump belt at intervals
            if (elapsed_time - ACTUATOR_EXTEND_S) % DUMP_ROTATE_EVERY_S < DUMP_ROTATE_PERIOD_S:
                self.control_message["dump_belt"] = 1
            else:
                self.control_message["dump_belt"] = 0
        elif elapsed_time < ACTUATOR_EXTEND_S + DRIVE_FORWARD_S + ACTUATOR_RETRACT_S:
            self.velocity_message["linearx_mps"] = 0.0
            self.control_message["dig_belt"] = 1
            self.control_message["actuator_retract"] = True
        else:
            # Stop everything
            self.control_message["actuator_retract"] = False
            self.control_message["dig_belt"] = 0
            self.autonomous_active = False
            self.manual_control_enabled = True
            self.get_logger().info("Autonomous process finished")


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
