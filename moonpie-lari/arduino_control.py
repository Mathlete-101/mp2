#ros2 node for communicating with arduino


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class ArduinoControl(Node):
    def __init__(self):
        super().__init__('arduino_control')
        self.subscription = self.create_subscription(
            Twist,              # Message type
            'cmd_vel',           # Topic name
            self.cmd_vel,  # Callback function
            10                    # Queue size
        )
        self.get_logger().info('Arduino Control Node has started')

    def cmd_vel(self, msg):
        self.get_logger().info(f'Received: "{msg}"')

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

