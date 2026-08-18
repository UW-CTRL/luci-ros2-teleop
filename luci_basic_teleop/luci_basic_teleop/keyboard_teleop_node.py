import rclpy
from rclpy.node import Node
from luci_messages.msg import LuciJoystick
from luci_basic_teleop.wait_for_key import read_single_keypress
import sys
from std_msgs.msg import String
from std_srvs.srv import Empty
import signal, time


UP_KEY_MAX = 100
DOWN_KEY_MAX = -100
LR_KEY_MAX = 100

UP_KEY_STRING = r"'\x1b', '[', 'A'"
DOWN_KEY_STRING = r"'\x1b', '[', 'B'"
LEFT_KEY_STRING = r"'\x1b', '[', 'D'"
RIGHT_KEY_STRING = r"'\x1b', '[', 'C'"
CTRL_c_KEY_STRING = r"('\x03',)"
q_KEY_STRING = r"('q',)"

# NOTE:
# Joystick Zones and Input Sources 
# should match with what is
# defined in the gRPC proto file:

# enum JoystickZone {
#     Front = 0;
#     FrontLeft = 1;
#     FrontRight = 2;
#     Left = 3;
#     Right = 4;
#     BackLeft = 5;
#     BackRight = 6;
#     Back = 7;
#     Origin = 8;
# }

# enum InputSource {
#     RampAssist = 0;
#     Remote = 1;
#     WDI = 2;
#     ChairVirtual = 3;
#     ChairPhysical = 4;
# }

JS_FRONT = 0
JS_RIGHT = 4
JS_LEFT = 3
JS_BACK = 7
JS_ORIGIN = 8

REMOTE = 1


def key_timer_handler(signum, frame):
    # raise timeout error for input key timer.
    raise TimeoutError()


class KeyboardPublisher(Node):

    def __init__(self):
        super().__init__('keyboard_control_node')
        self.publisher_ = self.create_publisher(LuciJoystick, 'luci/remote_joystick', 10)
        self.set_auto_input_client = self.create_client(Empty, '/luci/set_auto_remote_input')
        self.rm_auto_input_client = self.create_client(Empty, '/luci/remove_auto_remote_input')
        while not self.set_auto_input_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /luci/set_auto_remote_input service...')

        self.set_auto_service() #enable auto remote input
        timer_period = 0.05  # Rate at which to send joystick commands in seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

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

    def timer_callback(self):
        msg = LuciJoystick()
        msg.input_source = REMOTE

        # Set a timer that times out if no keys are pressed
        signal.signal(signal.SIGALRM, key_timer_handler)
        signal.setitimer(signal.ITIMER_REAL, 0.5)

        try:
            # Wait for a key to be pressed
            keyboard_data = str(read_single_keypress())

        except TimeoutError:
            # If no key is pressed send zeros to the joystick
            msg = LuciJoystick()
            dir_char = ' '
            msg.forward_back = 0
            msg.left_right = 0
            msg.joystick_zone = JS_ORIGIN
            self.publisher_.publish(msg)
            self.get_logger().info('dir: {} js_zone:{}| Publishing: {} {}'.format(dir_char, msg.joystick_zone, msg.forward_back, msg.left_right))
            return

        finally:
            # Disable the timer when the program terminates
            signal.setitimer(signal.ITIMER_REAL, 0)

        print(keyboard_data)

        # Keyboard input is converted to joytick commands
        # q or ctrl+c to stop keyboard and sends zeros to chair
        if keyboard_data == q_KEY_STRING or keyboard_data == CTRL_c_KEY_STRING:
            msg.forward_back = 0
            msg.left_right = 0
            msg.joystick_zone = JS_ORIGIN
            self.publisher_.publish(msg)
            self.rm_auto_service()
            sys.exit("\n keyboard control ended")

        # UP -> Forward
        elif UP_KEY_STRING in keyboard_data:
            msg.forward_back = UP_KEY_MAX
            msg.left_right = 0
            dir_char = 'F'
            msg.joystick_zone = JS_FRONT

        # DOWN -> Backwards
        elif DOWN_KEY_STRING in keyboard_data:
            msg.forward_back = DOWN_KEY_MAX
            msg.left_right = 0
            dir_char = 'B'
            msg.joystick_zone = JS_BACK

        # LEFT -> Left turn
        elif LEFT_KEY_STRING in keyboard_data:
            msg.forward_back = 0
            # negative number goes left
            msg.left_right = -1 * LR_KEY_MAX
            dir_char = 'L'
            msg.joystick_zone = JS_LEFT

        # RIGHT -> Right turn
        elif RIGHT_KEY_STRING in keyboard_data:
            msg.forward_back = 0
            msg.left_right = LR_KEY_MAX
            dir_char = 'R'
            msg.joystick_zone = JS_RIGHT

        # All other inputs will stop the motors
        else:
            dir_char = '?'
            msg.forward_back = 0
            msg.left_right = 0
            msg.joystick_zone = JS_ORIGIN

        # Publish joystick commands
        self.publisher_.publish(msg)
        self.get_logger().info('dir: {} js_zone:{}| Publishing: {} {}'.format(dir_char, msg.joystick_zone, msg.forward_back, msg.left_right))

def main(args=None):
    rclpy.init(args=args)

    keyboard_publisher = KeyboardPublisher()
    rclpy.spin(keyboard_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    keyboard_publisher.rm_auto_service()
    keyboard_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()