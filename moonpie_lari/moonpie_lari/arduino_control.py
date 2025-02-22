#ros2 node for communicating with arduino


import time
from queue import Queue
from threading import Thread
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
        self.subscription = self.create_subscription(
            Twist,              # Message type
            'cmd_vel',           # Topic name
            self.cmd_vel,  # Callback function
            10                    # Queue size
        )

        self.get_logger().info('Arduino Control Node has started')

        def ard_msg_thread(ctrl, msg, log, conn):
            log.info(f"thread_started")
            while ctrl.empty():
                l = conn.readline()
                log.info(f"arduino: {l.decode()}");
                msg.put(l)
        self.amt_ctrl = Queue()
        self.amt_msg = Queue()
        self.amt = Thread(target=ard_msg_thread, args=(self.amt_ctrl, self.amt_msg, self.get_logger(), self.serial_connection))
        self.amt.start()


    def destroy_node():
        super().destroy_node()
        self.amt_ctrl.put(0) # stop the message thread
         

    def cmd_vel(self, msg):
        #self.get_logger().info(f'Received: "{msg}"')
        cmd = json.dumps({
           "linearx_mps": msg.linear.x,
           "angularz_rps": msg.angular.z,
           "buttons": {"A": 0, "B": 0, "X": 0, "Y": 0},
           "dpad": {"x": 0, "y": 0},
           "Kp": 2.5,
           "Ki": 0.05,
        }) + "\n"
        try:
            self.serial_connection.write(cmd.encode())
        except serial.serialutil.SerialTimeoutException as e:
            self.get_logger().info(f'write timeout {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

