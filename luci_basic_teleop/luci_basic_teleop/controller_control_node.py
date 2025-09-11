import rclpy
from rclpy.node import Node
from luci_messages.msg import LuciJoystick
from sensor_msgs.msg import Joy
import sys
from std_msgs.msg import String, Int32
from std_srvs.srv import Empty
import signal, time
from enum import Enum


# Constants
UP_KEY_MAX = 100
DOWN_KEY_MAX = -100
LR_KEY_MAX = 100

JS_FRONT = 0
JS_LEFT = 3
JS_RIGHT = 4
JS_BACK = 7
JS_ORIGIN = 8

REMOTE = 5

class State(Enum):
    IDLE = 0
    CONTROLLED = 1
    NAV = 2
    OVERRIDE = 3


class ControllerPublisher(Node):
    def __init__(self):
        super().__init__('controller_control_node')
        self.publisher_ = self.create_publisher(LuciJoystick, 'luci/remote_joystick', 10)
        # self.set_auto_input_client = self.create_client(Empty, '/luci/set_auto_remote_input')
        # self.rm_auto_input_client = self.create_client(Empty, '/luci/remove_auto_remote_input')
        self.set_shared_input_client = self.create_client(Empty, '/luci/set_shared_remote_input')
        self.rm_shared_input_client = self.create_client(Empty, '/luci/remove_shared_remote_input')

        while not self.set_shared_input_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /luci/set_shared_control service...')
        
        # Joystick button subscriber 
        self.joystick_subscriber = self.create_subscription(
            LuciJoystick,
            '/luci/joystick_position',
            self.joystick_callback,
            10)
        
        
        # # Override button subscriber 
        # self.override_subscriber = self.create_subscription(
        #     Int32,
        #     '/luci/override_button_press_count_data',
        #     self.override_callback,
        #     10)
        
        # Contorller subscriber
        self.joy_subscriber = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)
        
        # Button States
        self.mode = State.IDLE
        self.override_state = False
        self.drive_enabled = False
        self.b_button_prev = 0
        self.right_trigger_prev = 0
        self.set_shared_service() #enable shared remote input
    
    # def set_auto_service(self):
    #     # Call this to enable auto remote input (remote joystick control)
    #     req = Empty.Request()
    #     future = self.set_auto_input_client.call_async(req)
    #     future.add_done_callback(self.handle_response)

    # def rm_auto_service(self):
    #     # Call this to disable auto remote input (remote joystick control)
    #     req = Empty.Request()
    #     future = self.rm_auto_input_client.call_async(req)
    #     future.add_done_callback(self.handle_response)

    def set_shared_service(self):
        # Call this to enable auto remote input (remote joystick control)
        req = Empty.Request()
        future = self.set_shared_input_client.call_async(req)
        future.add_done_callback(self.handle_response)
    
    def rm_shared_service(self):
        # Call this to enable auto remote input (remote joystick control)
        req = Empty.Request()
        future = self.rm_shared_input_client.call_async(req)
        future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        # Handler for service calls
        try:
            future.result()  # Empty service has no response fields
            self.get_logger().info('Service call succeeded!')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def joystick_callback(self, joystick_msg:LuciJoystick):
        if joystick_msg.joystick_zone != 8 and self.mode != State.OVERRIDE:
            self.mode = State.OVERRIDE
            ControllerPublisher.rm_shared_service(self)
        elif joystick_msg.joystick_zone == 8 and self.mode == State.OVERRIDE:
            self.mode = State.IDLE
            ControllerPublisher.set_shared_service(self)
    
    # # How to interact with LUCI override button 
    # def override_callback(self, override_msg:Int32):
    #     if override_msg.data == 1:
    #         self.mode = State.OVERRIDE

    #     if self.mode == State.OVERRIDE:
    #         ControllerPublisher.rm_shared_service(self)
    #     else:
    #         ControllerPublisher.set_shared_service(self)


    def joy_callback(self, joy_msg: Joy):
        msg = LuciJoystick()
        msg.input_source = REMOTE

        forward_back_axis = joy_msg.axes[1] 
        left_right_axis = joy_msg.axes[0]   
        right_trigger = joy_msg.axes[5]
        b_button = joy_msg.buttons[1]

        if right_trigger and b_button and self.mode != State.CONTROLLED and self.mode == State.IDLE:
            self.drive_enabled = not self.drive_enabled
            self.mode = State.CONTROLLED
        self.a_button_prev = b_button
        self.right_trigger_prev = right_trigger


        if self.mode == State.CONTROLLED:
            msg.forward_back = int(forward_back_axis * UP_KEY_MAX)
            msg.left_right = int(-left_right_axis * LR_KEY_MAX)

            if msg.forward_back > 10:
                msg.joystick_zone = JS_FRONT
                dir_char = 'F'
            elif msg.forward_back < -10:
                msg.joystick_zone = JS_BACK
                dir_char = 'B'
            elif msg.left_right < 10:
                msg.joystick_zone = JS_LEFT
                dir_char = 'L'
            elif msg.left_right > -10:
                msg.joystick_zone = JS_RIGHT
                dir_char = 'R'
            else:
                msg.joystick_zone = JS_ORIGIN
                dir_char = '?'
        else:
            msg.forward_back = 0
            msg.left_right = 0
            msg.joystick_zone = JS_ORIGIN
            dir_char = '-'

        self.publisher_.publish(msg)
        self.get_logger().info('dir: {} js_zone:{}| Publishing: {} {}'.format(dir_char, msg.joystick_zone, msg.forward_back, msg.left_right))


def main(args=None):
    rclpy.init(args=args)

    controller_publisher = ControllerPublisher()
    rclpy.spin(controller_publisher)

    controller_publisher.rm_shared_service()
    controller_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()