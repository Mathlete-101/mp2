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

# timing for automated digging sequence
# The actuator will extend with the dig belt on for time
# The rover will drive forward at a slow constand speed with the dig belt on for time
# While driving and digging, the dump belt will rotate for time every time to more evenly load regolith
# After driving and digging, the rover will stop and stop digging, then retract the dig belt for time

ACTUATOR_EXTEND_S = 7
DRIVE_FORWARD_S = 20
DUMP_ROTATE_EVERY_S = 5
DUMP_ROTATE_PERIOD_S = 0.5
ACTUATOR_RETRACT_S = 7
DRIVE_AND_DIG_SPEED_MPS = 0.1

Kp = 2.5
Ki = 0.5
extend_speed = 50
retract_speed = 100
dig_speed = 100
dump_speed = 100

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
            Twist,               # Message type
            'cmd_vel',           # Topic name
            self.cmd_vel,        # Callback function
            10                   # Queue size
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
            "Kp": Kp,
            "Ki": Ki,
            #"dutyA": {"extend_speed": extend_speed, "retract_speed": retract_speed},   # actuators speeds
            #"dutyB": dig_speed,                                          # dig belt speed
            #"dutyD": dump_speed,                                          # dump belt speed
            
            # not used
            "dpad": {"x": 0, "y": 0}, # x=1: kill, x=-1: start autonomy
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

        self.autonomous_active = False
        self.autonomous_start_time = 0.0
        self.dump_belt_last_time = 0.0

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
                self.get_logger().info(f"arduino: {msg}")
                
    def joy(self, msg):
        # don't send controller input if autonomy is active
        
        self.message = dict(self.message, **{
            "dump_belt": msg.buttons[0],
            "dig_belt": msg.buttons[1],
            "actuator_extend": msg.axes[7] < 0,
            "actuator_retract": msg.axes[7] > 0,
            "dpad": {"x": msg.axes[6], "y": msg.axes[7]},
        })

        # start autonomy if left dpad is pressed
        if msg.axes[6] == 1.0 and not self.autonomous_active:
            self.start_autonomous()
        # kill autonomy is right dpad is pressed and autonomy is active
        if msg.axes[6] == -1.0 and self.autonomous_active:
            self.kill_autonomous()

    def cmd_vel(self, msg):
        #self.get_logger().info(f'Received cmd_vel: "{msg}"')
        # don't send controller input if autonomy is active
        
        self.message = dict(self.message, **{
            "linearx_mps": msg.linear.x,
            "angularz_rps": msg.angular.z,
        })

    def write_message(self):
        # run autonomy, generate messages
        # called after the remote control inputs are generated, so some inputs from remote control could be processed and others are overwritten
        if self.autonomous_active:
            self.run_autonomous()

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


    def start_autonomous(self):
        self.get_logger().info("Autonomous process started")
        self.autonomous_active = True
        self.autonomous_start_time = time.time()
        self.message["actuator_extend"] = True
        self.message["dig_belt"] = True
        self.message["Ki"] = 0                          # no integral control term while autonomous (are encoders actually working???)
        self.dump_belt_last_time = time.time()

    def kill_autonomous(self):
        self.get_logger().info("Autonomous process killed")
        self.autonomous_active = False
        self.message["actuator_extend"] = False
        self.message["dig_belt"] = False
        self.message["linearx_mps"] = 0.0
        self.message["dump_belt"] = False
        self.message["Ki"] = Ki                         # reinstate Ki for remote control
        self.message["actuator_retract"] = False

    # extends actuator with dig belt on, drives forward with dig belt on and rotates dump belt periodically, stops driving and dig belt and retracts actuator
    def run_autonomous(self):
        current_time = time.time()
        elapsed_time = current_time - self.autonomous_start_time

        if elapsed_time < ACTUATOR_EXTEND_S:
            self.message["actuator_extend"] = True
            self.message["linearx_mps"] = 0.0
            self.message["dig_belt"] = 1
        elif ACTUATOR_EXTEND_S <= elapsed_time < ACTUATOR_EXTEND_S + DRIVE_FORWARD_S:
            self.message["actuator_extend"] = False
            self.message["linearx_mps"] = DRIVE_AND_DIG_SPEED_MPS #slow speed
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