import rclpy
from rclpy.node import Node
from luci_messages.msg import LuciJoystick
from sensor_msgs.msg import Joy
import sys
from std_msgs.msg import String
from std_srvs.srv import Empty
import signal, time

# Constants
UP_KEY_MAX = 100
DOWN_KEY_MAX = -100
LR_KEY_MAX = 100

JS_FRONT = 0
JS_LEFT = 3
JS_RIGHT = 4
JS_BACK = 7
JS_ORIGIN = 8

REMOTE = 1


class ControllerPublisher(Node):
    def __init__(self):
        super().__init__('controller_control_node')
        self.publisher_ = self.create_publisher(LuciJoystick, 'luci/remote_joystick', 10)
        self.set_auto_input_client = self.create_client(Empty, '/luci/set_auto_remote_input')
        self.rm_auto_input_client = self.create_client(Empty, '/luci/remove_auto_remote_input')
        while not self.set_auto_input_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /luci/set_auto_remote_input service...')
        
        self.joy_subscriber = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)
        self.set_auto_service() #enable auto remote input
    
    def set_auto_service(self):
        # Call this to enable auto remote input (remote joystick control)
        req = Empty.Request()
        future = self.set_auto_input_client.call_async(req)
        future.add_done_callback(self.handle_response)

    def rm_auto_service(self):
        # Call this to disable auto remote input (remote joystick control)
        req = Empty.Request()
        future = self.rm_auto_input_client.call_async(req)
        future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        # Handler for service calls
        try:
            future.result()  # Empty service has no response fields
            self.get_logger().info('Service call succeeded!')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def joy_callback(self, joy_msg: Joy):
        msg = LuciJoystick()
        msg.input_source = REMOTE

        forward_back_axis = joy_msg.axes[1]  # Left stick Y
        left_right_axis = joy_msg.axes[0]   # Left stick X

        drive_enabled = True

        if drive_enabled:
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

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller_publisher.rm_auto_service()
    controller_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()