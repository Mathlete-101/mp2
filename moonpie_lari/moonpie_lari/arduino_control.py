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

# timing for automated digging sequence
# The actuator will extend with the dig belt on for time
# The rover will drive forward at a slow constand speed with the dig belt on for time
# While driving and digging, the dump belt will rotate for time every time to more evenly load regolith
# After driving and digging, the rover will stop and stop digging, then retract the dig belt for time

ACTUATOR_EXTEND_S = 2
DRIVE_FORWARD_S = 20
DUMP_ROTATE_EVERY_S = 1
DUMP_ROTATE_PERIOD_S = 1
ACTUATOR_RETRACT_S = 4
DRIVE_AND_DIG_SPEED_MPS = 0.05

# for autonomy, Ki must be zero or the rover will slowly increase speed 
Ki = 0

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
        self.arduino_command_publisher = self.create_publisher(String, 'arduino_command', 10)
        self.message = {
            "cmd": True,
            "linearx_mps": 0.0,
            "angularz_rps": 0.0,
            "dump_belt": 0,
            "dig_belt": 0,
            "actuator_extend": False,
            "actuator_retract": False,
            "dpad": {"x": 0, "y": 0},
            "Ki": Ki,
        }

        self.get_logger().info('Arduino Control Node has started')

        self.send_mode = 0
        self.comms_timer = self.create_timer(0.1, self.write_message)

        self.autonomous_active = False
        self.autonomous_start_time = 0.0
        self.dump_belt_last_time = 0.0

    def joy(self, msg):
        self.message = dict(self.message, **{
            "dump_belt": msg.buttons[0],
            "dig_belt": msg.buttons[1],
            "actuator_extend": msg.axes[7] < 0,
            "actuator_retract": msg.axes[7] > 0,
            "dpad": {"x": msg.axes[6], "y": msg.axes[7]},
        })

        # start autonomy if left dpad_x is pressed
        if msg.axes[6] == 1.0 and not self.autonomous_active:
            self.start_autonomous()
        # kill autonomy if right dpad_x is pressed
        if msg.axes[6] == -1.0 and self.autonomous_active:
            self.kill_autonomous()

    def cmd_vel(self, msg):
        self.message = dict(self.message, **{
            "linearx_mps": msg.linear.x,
            "angularz_rps": msg.angular.z,
        })

    def write_message(self):
        # autonomous message is written after remote control message
        # appendages not controlled by autonomy can still be controlled while 'autonomous' if the button is pressed and it is not controlled by autonomy
        if self.autonomous_active:
            self.run_autonomous()

        if self.send_mode == 0:
            cmd = json.dumps(self.message)
            self.send_mode = 0
        elif self.send_mode == 1:
            cmd = '{"cmd": false, "drive_train": {"set_angularz_rps": null, "set_linearx_mps": null}, "actuator": {"get_state": null}, "dig_belt": {"get_state": null}, "dump_belt": {"get_state": null}}'
            self.send_mode = 0
        else:
            self.get_logger().error("invalid send mode")
        self.get_logger().info(f"sending message: {self.send_mode}\n{cmd}")
        msg = String()
        msg.data = cmd
        self.arduino_command_publisher.publish(msg)

    # start the actuator and dig belt to start autonomous digging 
    def start_autonomous(self):
        self.get_logger().info("Autonomous process started")
        self.autonomous_active = True
        self.autonomous_start_time = time.time()
        self.message["actuator_extend"] = True
        self.message["dig_belt"] = 1
        self.dump_belt_last_time = time.time() 

    # stop everything in autonomy and exit auto mdoe
    def kill_autonomous(self):
        self.get_logger().info("Autonomous process killed")
        self.autonomous_active = False
        self.message["actuator_extend"] = False
        self.message["dig_belt"] = 0
        self.message["linearx_mps"] = 0.0
        self.message["dump_belt"] = 0
        self.message["actuator_retract"] = False

    # run autonomy
    # using ROS timers made things super weird when I tried it, but would probably be better
    def run_autonomous(self):
        current_time = time.time()
        elapsed_time = current_time - self.autonomous_start_time

        if elapsed_time < ACTUATOR_EXTEND_S:
            self.message["actuator_extend"] = True
            self.message["linearx_mps"] = 0.0
            self.message["dig_belt"] = 1
        elif ACTUATOR_EXTEND_S <= elapsed_time < ACTUATOR_EXTEND_S + DRIVE_FORWARD_S:
            self.message["actuator_extend"] = False
            self.message["linearx_mps"] = DRIVE_AND_DIG_SPEED_MPS
            self.message["dig_belt"] = 1
            if current_time - self.dump_belt_last_time >= DUMP_ROTATE_EVERY_S:
                self.message["dump_belt"] = 1
                self.dump_belt_last_time = current_time
            if current_time - self.dump_belt_last_time >= DUMP_ROTATE_PERIOD_S and self.message["dump_belt"] == 1:
                self.message["dump_belt"] = 0

        elif elapsed_time >= ACTUATOR_EXTEND_S + DRIVE_FORWARD_S:
            self.message["linearx_mps"] = 0.0
            self.message["dig_belt"] = 0
            self.message["actuator_retract"] = True
            if elapsed_time >= ACTUATOR_EXTEND_S + DRIVE_FORWARD_S + ACTUATOR_RETRACT_S:
                self.message["actuator_retract"] = False
                self.autonomous_active = False
                self.get_logger().info("Autonomous process finished")

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()