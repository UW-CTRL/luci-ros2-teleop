import rclpy
from rclpy.node import Node
from luci_messages.msg import LuciJoystick
from sensor_msgs.msg import Joy

# Constants
UP_KEY_MAX = 100
LR_KEY_MAX = 100

JS_FRONT = 0
JS_LEFT = 3
JS_RIGHT = 4
JS_BACK = 7
JS_ORIGIN = 8

REMOTE = 5


class XboxPublisher(Node):
    def __init__(self):
        super().__init__('xbox_teleop_node')

        self.joy_subscriber = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)

        self.xbox_publisher = self.create_publisher(LuciJoystick, '/xbox', 10)
        self.get_logger().info("Xbox Teleop Node Started")

    def joy_callback(self, joy_msg: Joy):
        msg = LuciJoystick()
        msg.input_source = REMOTE

        forward_back_axis = joy_msg.axes[1] if len(joy_msg.axes) > 1 else 0.0
        left_right_axis = joy_msg.axes[0] if len(joy_msg.axes) > 0 else 0.0

        msg.forward_back = int(forward_back_axis * UP_KEY_MAX)
        msg.left_right = int(-left_right_axis * LR_KEY_MAX)

        if msg.forward_back > 10:
            msg.joystick_zone = JS_FRONT
        elif msg.forward_back < -10:
            msg.joystick_zone = JS_BACK
        elif msg.left_right < 10:
            msg.joystick_zone = JS_LEFT
        elif msg.left_right > -10:
            msg.joystick_zone = JS_RIGHT
        else:
            msg.joystick_zone = JS_ORIGIN

        self.xbox_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    xbox_publisher = XboxPublisher()
    try:
        rclpy.spin(xbox_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        xbox_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()