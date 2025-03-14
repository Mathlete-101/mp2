#ros2 node for communicating with arduino


import time
from queue import Queue
from threading import Thread
from sensor_msgs.msg import Joy
import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import json

class ArduinoControl(Node):
    def serial_logger(self):
        class p:
            def __init__(self, logger):
                self.logger = logger
            def write(self, message):
                self.logger.info(str(message))
            def __getattr__(self, name):
                pass
            def reset_input_buffer(self):
                pass
            def reset_output_buffer(self):
                pass
        return p(self.get_logger())
        
    def __init__(self):
        super().__init__('arduino_control')
        try:
            self.serial_connection = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=5)
            time.sleep(2)
        except serial.SerialException as e:
            self.get_logger().warning('could not connect to arduino, messages will be logged instead')
            self.get_logger().warning(str(e))
            self.serial_connection = self.serial_logger()
            self.serial_connection.reset_input_buffer()
        self.twist_subscription = self.create_subscription(
            Twist,              # Message type
            'cmd_vel',           # Topic name
            self.cmd_vel,  # Callback function
            10                    # Queue size
        )
        self.joy_subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy,
            10

        )
        self.message = {
            "cmd": True,
			"linearx_mps": 0.0,
			"angularz_rps": 0.0,
			"dump_belt": 0,
			"dig_belt": 0,
			"actuator_extend": False,
			"actuator_retract": False,
			
			#params -- ignore
			#"Kp": 0.75,
			#"Ki": 0.02,
			#"dutyA": 50,
			#"dutyB": 60,
			#"dutyC": 40,
			
			#dont use
			"dpad": {"x": 0, "y": 0},
        }

        self.get_logger().info('Arduino Control Node has started')

        def ard_msg_thread(ctrl, msg, log, conn):
            try:
                log.info(f"thread_started")
                while ctrl.empty():
                    msg.put(conn.readline())
            except KeyboardInterrupt:
                log.info("message thread interrupted")
        self.amt_ctrl = Queue()
        self.amt_msg = Queue()
        self.amt_tmr = self.create_timer(0.01, self.publish_messages)
        self.amt = Thread(target=ard_msg_thread, args=(self.amt_ctrl, self.amt_msg, self.get_logger(), self.serial_connection))
        self.amt.start()
        
        self.send_mode = 0
        self.comms_timer = self.create_timer(0.1, self.write_message)


    def destroy_node(self):
        super().destroy_node()
        self.amt_ctrl.put(0) # stop the message thread
         

    def publish_messages(self):
        while not self.amt_msg.empty():
            msg = self.amt_msg.get().decode()
            try:
                result = json.loads(msg)
                #publish whatever messages need to be published
            except:
                self.get_logger().info(f"arduino: {msg}");
                
    def joy(self, msg):
        
        self.message = dict(self.message, **{
            "dump_belt": msg.buttons[0],
            "dig_belt": msg.buttons[1],
            "actuator_extend": msg.axes[7] < 0,
            "actuator_retract": msg.axes[7] > 0,
        })
        

    def cmd_vel(self, msg):
        #self.get_logger().info(f'Received cmd_vel: "{msg}"')
        self.message = dict(self.message, **{
           "linearx_mps": msg.linear.x,
           "angularz_rps": msg.angular.z,
        })

    def write_message(self):
        if self.send_mode == 0:
            cmd = json.dumps(self.message).encode() + b"\n"
            self.send_mode = 0

        #request data. this strat is probably bad
        elif self.send_mode == 1:
            cmd = b'{"cmd": false, "drive_train": {"set_angularz_rps": null, "set_linearx_mps": null}, "actuator": {"get_state": null}, "dig_belt": {"get_state": null}, "dump_belt": {"get_state": null}}\n'
            self.send_mode = 0
        else:
            self.get_logger().error("invalid send mode")
        self.get_logger().info(f"sending message: {self.send_mode}\n{cmd}")
        try:
            self.serial_connection.write(cmd)
        except serial.serialutil.SerialTimeoutException as e:
            self.get_logger().info(f'write timeout {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
