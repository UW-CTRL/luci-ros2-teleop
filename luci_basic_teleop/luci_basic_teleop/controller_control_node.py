import rclpy
from rclpy.node import Node
from luci_messages.msg import LuciJoystick
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_srvs.srv import SetBool
from enum import Enum

UP_KEY_MAX = 100
LR_KEY_MAX = 100
JS_FRONT, JS_LEFT, JS_RIGHT, JS_BACK, JS_ORIGIN = 0, 3, 4, 7, 8
REMOTE = 5

class ControllerPublisher(Node):
    def __init__(self):
        super().__init__('controller_control_node')
        
        self.publisher_ = self.create_publisher(LuciJoystick, 'luci/remote_joystick', 10)
        self.mode_client = self.create_client(SetBool, '/luci/request_controlled_mode')
        
        self.state_subscriber = self.create_subscription(
            String, 'luci/control_state', self.state_callback, 10)
        
        self.joy_subscriber = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10)
        
        self.current_mode = "IDLE"
        self.last_b_state = 0

    def state_callback(self, msg):
        self.current_mode = msg.data

    def request_mode(self, enable: bool):
        if not self.mode_client.wait_for_service(timeout_sec=1.0):
            return
        req = SetBool.Request()
        req.data = enable
        self.mode_client.call_async(req)

    def joy_callback(self, joy_msg: Joy):
        # 1. Handle Toggle Request (Right Trigger + B Button)
        rt_pressed = joy_msg.axes[5] < -0.5
        b_pressed = joy_msg.buttons[1]

        if rt_pressed and b_pressed and not self.last_b_state:
            target = (self.current_mode == "IDLE")
            self.request_mode(target)
        self.last_b_state = b_pressed

        # 2. Movement Logic - STRICTLY SILENT IF NOT IN CONTROL
        if self.current_mode == "CONTROLLED":
            msg = LuciJoystick()
            msg.input_source = REMOTE
            
            fb_val = int(joy_msg.axes[1] * UP_KEY_MAX)
            lr_val = int(-joy_msg.axes[0] * LR_KEY_MAX)

            msg.forward_back = fb_val
            msg.left_right = lr_val

            if fb_val > 10: msg.joystick_zone = JS_FRONT
            elif fb_val < -10: msg.joystick_zone = JS_BACK
            elif lr_val > 10: msg.joystick_zone = JS_LEFT
            elif lr_val < -10: msg.joystick_zone = JS_RIGHT
            else: msg.joystick_zone = JS_ORIGIN

            self.publisher_.publish(msg)
            
        # NO ELSE STATEMENT. If not in control, do not fight the passthrough!

def main(args=None):
    rclpy.init(args=args)
    node = ControllerPublisher()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.request_mode(False)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()