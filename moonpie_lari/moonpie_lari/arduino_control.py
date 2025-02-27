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
           "linearx_mps": 0.0,
           "angularz_rps": 0.0,
           "buttons": {"A": 0, "B": 0, "X": 0, "Y": 0},
           "dpad": {"x": 0, "y": 0},
           "Kp": 2.5,
           "Ki": 0.05,
        }

        self.get_logger().info('Arduino Control Node has started')

        def ard_msg_thread(ctrl, msg, log, conn):
            log.info(f"thread_started")
            while ctrl.empty():
                msg.put(conn.readline())
        self.amt_ctrl = Queue()
        self.amt_msg = Queue()
        self.amt_tmr = Node.create_timer(0.01, self.publish_messages)
        self.amt = Thread(target=ard_msg_thread, args=(self.amt_ctrl, self.amt_msg, self.get_logger(), self.serial_connection))
        self.amt.start()


    def destroy_node():
        super().destroy_node()
        self.amt_ctrl.put(0) # stop the message thread
         

    def publish_messages():
        while not self.amt_msg.empty():
            msg = self.amt_msg.get().decode()
            try:
                result = json.loads(msg)
                #publish whatever messages need to be published
            except:
                log.info(f"arduino: {msg}");
                
    def joy(self, msg):
        self.message = dict(self.message, **{a: b for a, b in zip(("mining_belt", "dig_belt", "mining_actuator"), msg.buttons[:3])})
        self.write_message()
        

    def cmd_vel(self, msg):
        #self.get_logger().info(f'Received cmd_vel: "{msg}"')
        self.message = dict(self.message, **{
           "linearx_mps": msg.linear.x,
           "angularz_rps": msg.angular.z,
        })
        self.write_message()

    def write_message(self):
        cmd = json.dumps(self.message.encode()) + "\n"
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
