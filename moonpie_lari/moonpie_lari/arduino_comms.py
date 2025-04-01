# arduino_comms.py
# manages sending and receiving data to and from the arduino
import time
from queue import Queue
from threading import Thread
import rclpy
from rclpy.node import Node
import serial
import json
from std_msgs.msg import String

class ArduinoComms(Node):
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
        super().__init__('arduino_comms')
        # set up serial connection
        try:
            self.serial_connection = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=5)
            time.sleep(2)
        except serial.SerialException as e:
            self.get_logger().warning('could not connect to arduino, messages will be logged instead')
            self.get_logger().warning(str(e))
            self.serial_connection = self.serial_logger()
            self.serial_connection.reset_input_buffer()

        # subscribe to the remote control/autonomous node 
        self.arduino_command_subscription = self.create_subscription(
            String,
            'arduino_command',
            self.arduino_command_callback,
            10
        )

        self.message_queue = Queue()

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
        self.get_logger().info('Arduino Comms Node has started')

    def destroy_node(self):
        super().destroy_node()
        self.amt_ctrl.put(0) # stop the message thread

    def arduino_command_callback(self, msg):
        self.message_queue.put(msg.data)

    # print what arduino sends
    def publish_messages(self):
        while not self.amt_msg.empty():
            msg = self.amt_msg.get().decode()
            try:
                result = json.loads(msg)
                #publish whatever messages need to be published
            except:
                self.get_logger().info(f"arduino: {msg}")
    
    # send the messages to the arduino at the specified interval
    def write_message(self, message):
        cmd = message.encode() + b"\n"
        self.get_logger().info(f"sending message: \n{cmd}")
        try:
            self.serial_connection.write(cmd)
        except serial.serialutil.SerialTimeoutException as e:
            self.get_logger().info(f'write timeout {e}')

    def timer_callback(self):
        while not self.message_queue.empty():
            message = self.message_queue.get()
            self.write_message(message)

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoComms()
    timer = node.create_timer(0.1, node.timer_callback)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()