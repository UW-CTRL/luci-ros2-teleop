import rclpy
from rclpy.node import Node
from luci_messages.msg import LuciJoystick, LuciDriveMode
from luci_basic_teleop.wait_for_key import read_single_keypress
import sys
from std_msgs.msg import String


UP_KEY_MAX = 100
DOWN_KEY_MAX = -100
LR_KEY_MAX = 100

UP_KEY_STRING = r"'\x1b', '[', 'A'"
DOWN_KEY_STRING = r"'\x1b', '[', 'B'"
LEFT_KEY_STRING = r"'\x1b', '[', 'D'"
RIGHT_KEY_STRING = r"'\x1b', '[', 'C'"
CTRL_c_KEY_STRING = r"('\x03',)"
q_KEY_STRING = r"('q',)"


class KeyboardPublisher(Node):

    def __init__(self):
        super().__init__('keyboard_control_node')
        self.publisher_ = self.create_publisher(
            LuciJoystick, 'luci/remote_joystick', 10)
        self.mode_publisher_ = self.create_publisher(
            LuciDriveMode, 'luci/drive_mode', 1)
        timer_period = 0.05  # Seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        keyboard_data = str(read_single_keypress())
        print(keyboard_data)
        msg = LuciJoystick()
        dir_char = ''

        # Keyboard input is converted to joytick commands
        # q or ctrl+c to stop keyboard and sends zeros to chair
        if keyboard_data == q_KEY_STRING or keyboard_data == CTRL_c_KEY_STRING:
            msg.forward_back = 0
            msg.left_right = 0
            self.publisher_.publish(msg)
            sys.exit("\n keyboard control ended")

        # UP -> Forward
        elif UP_KEY_STRING in keyboard_data:
            msg.forward_back = UP_KEY_MAX
            msg.left_right = 0
            dir_char = 'F'

        # DOWN -> Backwards
        elif DOWN_KEY_STRING in keyboard_data:
            msg.forward_back = DOWN_KEY_MAX
            msg.left_right = 0
            dir_char = 'B'

        # LEFT -> Left turn
        elif LEFT_KEY_STRING in keyboard_data:
            msg.forward_back = 0
            # negative number goes left
            msg.left_right = -1 * LR_KEY_MAX
            dir_char = 'L'

        # RIGHT -> Right turn
        elif RIGHT_KEY_STRING in keyboard_data:
            msg.forward_back = 0
            msg.left_right = LR_KEY_MAX
            dir_char = 'R'

        # All other input wil stop the motors
        else:
            msg.forward_back = 0
            msg.left_right = 0

        # Publish joystick commands
        self.publisher_.publish(msg)
        self.get_logger().info('dir: {} | Publishing: {} {}'.format(dir_char,
                                                                    msg.forward_back,
                                                                    msg.left_right))


def main(args=None):
    rclpy.init(args=args)

    keyboard_publisher = KeyboardPublisher()
    mode_msg = LuciDriveMode()
    mode_msg.mode = LuciDriveMode.AUTO
    keyboard_publisher.mode_publisher_.publish(mode_msg)

    rclpy.spin(keyboard_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    keyboard_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
