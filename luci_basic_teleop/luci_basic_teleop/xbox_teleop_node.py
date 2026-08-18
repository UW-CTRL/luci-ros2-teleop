import rclpy
from rclpy.node import Node
from luci_messages.msg import LuciJoystick
from sensor_msgs.msg import Joy
from std_msgs.msg import String, Int32
from std_srvs.srv import Empty


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


class XboxPublisher(Node):
    def __init__(self):
        super().__init__('xbox_teleop_node')
        self.publisher_ = self.create_publisher(LuciJoystick, 'luci/remote_joystick', 10)

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
        
        
        # Override button subscriber 
        self.override_subscriber = self.create_subscription(
            Int32,
            '/luci/override_button_press_count_data',
            self.override_callback,
            10)
        
        # Contorller subscriber
        self.joy_subscriber = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)
        
        # External state is managed by luci_state_manager_node.
        self.current_state = 'IDLE'
        self.state_subscriber = self.create_subscription(
            String,
            '/luci/control_state',
            self.state_callback,
            10)
        self.overrride_data = 0
        self.drive_enabled = False
        self.set_shared_service() #enable shared remote input

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
    
    def state_callback(self, msg: String):
        self.current_state = msg.data

    # How to interact with LUCI override button
    def override_callback(self, override_msg:Int32):
        self.overrride_data = override_msg.data

    def joystick_callback(self, joystick_msg:LuciJoystick):
        if joystick_msg.joystick_zone != JS_ORIGIN:
            self.current_state = 'INTERRUPT'

    def joy_callback(self, joy_msg: Joy):
        msg = LuciJoystick()
        msg.input_source = REMOTE

        forward_back_axis = joy_msg.axes[1] if len(joy_msg.axes) > 1 else 0.0
        left_right_axis = joy_msg.axes[0] if len(joy_msg.axes) > 0 else 0.0

        if self.current_state != 'TELEOP':
            msg.forward_back = 0
            msg.left_right = 0
            msg.joystick_zone = JS_ORIGIN
            self.publisher_.publish(msg)
            return

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

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    xbox_publisher = XboxPublisher()
    rclpy.spin(xbox_publisher)

    xbox_publisher.rm_shared_service()
    xbox_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()