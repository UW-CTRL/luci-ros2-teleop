import rclpy
from rclpy.node import Node
from luci_messages.msg import LuciJoystick
from std_msgs.msg import String, Int32, Bool
from std_srvs.srv import Empty
import sys
import select
import termios
import tty
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

# Terminal settings for reading keystrokes
settings = termios.tcgetattr(sys.stdin)

def getKey():
    """Reads a single keypress from the terminal non-blockingly."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1) # 0.1 second timeout
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_control_node')
        self.publisher_ = self.create_publisher(LuciJoystick, 'luci/remote_joystick', 10)
        self.state_publisher_ = self.create_publisher(String, 'luci/control_state', 10)
        self.intervention_publisher_ = self.create_publisher(Bool, 'luci/intervention_alert', 10)
        
        self.rm_auto_input_client = self.create_client(Empty, '/luci/remove_auto_remote_input')
        self.set_shared_input_client = self.create_client(Empty, '/luci/set_shared_remote_input')
        self.rm_shared_input_client = self.create_client(Empty, '/luci/remove_shared_remote_input')

        while not self.set_shared_input_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /luci/set_shared_remote_input service...')
        
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
        
        # Initial States
        self.mode = State.IDLE
        self.override_data = 0
        self.set_shared_service() #enable shared remote input

    def rm_auto_service(self):
        req = Empty.Request()
        future = self.rm_auto_input_client.call_async(req)
        future.add_done_callback(self.handle_response)

    def set_shared_service(self):
        req = Empty.Request()
        future = self.set_shared_input_client.call_async(req)
        future.add_done_callback(self.handle_response)
    
    def rm_shared_service(self):
        req = Empty.Request()
        future = self.rm_shared_input_client.call_async(req)
        future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        try:
            future.result()
            self.get_logger().info('Service call succeeded!')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
    
    def override_callback(self, override_msg:Int32):
        self.override_data = override_msg.data
    
    def joystick_callback(self, joystick_msg:LuciJoystick):
        if joystick_msg.joystick_zone != JS_ORIGIN and self.mode != State.OVERRIDE:
            self.mode = State.OVERRIDE
            self.rm_shared_service()
        elif joystick_msg.joystick_zone == JS_ORIGIN and self.mode == State.OVERRIDE:
            self.mode = State.IDLE
        self.state_publisher_.publish(String(data=f'Mode: {self.mode}'))

def main(args=None):
    rclpy.init(args=args)
    keyboard_publisher = KeyboardPublisher()
    
    msg = """
    ---------------------------
    LUCI Keyboard Teleop Node
    ---------------------------
    Controls:
      W : Forward
      S : Backward
      A : Left
      D : Right
      T : Toggle CONTROLLED / IDLE Mode
      
      CTRL-C to quit
    ---------------------------
    """
    print(msg)

    try:
        while rclpy.ok():
            key = getKey()
            
            # 1. Handle Mode Toggle (Replaces Right Trigger + B Button)
            if key.lower() == 't':
                if keyboard_publisher.mode == State.IDLE:
                    keyboard_publisher.set_shared_service()
                    keyboard_publisher.mode = State.CONTROLLED
                    print("\n[Mode Switched to CONTROLLED]")
                elif keyboard_publisher.mode == State.CONTROLLED:
                    keyboard_publisher.mode = State.IDLE
                    print("\n[Mode Switched to IDLE]")

            # 2. Handle Ctrl-C termination
            if key == '\x03':
                break

            # 3. Create and populate Joystick Message
            joy_msg = LuciJoystick()
            joy_msg.input_source = REMOTE
            
            forward_back_val = 0
            left_right_val = 0

            # Only allow movement if in CONTROLLED state
            if keyboard_publisher.mode == State.CONTROLLED:
                if key.lower() == 'w':
                    forward_back_val = UP_KEY_MAX
                elif key.lower() == 's':
                    forward_back_val = DOWN_KEY_MAX
                elif key.lower() == 'a':
                    left_right_val = LR_KEY_MAX
                elif key.lower() == 'd':
                    left_right_val = -LR_KEY_MAX

                joy_msg.forward_back = forward_back_val
                joy_msg.left_right = left_right_val

                # Determine Joystick Zone
                if joy_msg.forward_back > 10:
                    joy_msg.joystick_zone = JS_FRONT
                elif joy_msg.forward_back < -10:
                    joy_msg.joystick_zone = JS_BACK
                elif joy_msg.left_right < 10 and joy_msg.left_right != 0: 
                    # Assuming negative is left based on your original logic. 
                    # Adjust if your robot maps positive to left!
                    joy_msg.joystick_zone = JS_RIGHT
                elif joy_msg.left_right > -10 and joy_msg.left_right != 0:
                    joy_msg.joystick_zone = JS_LEFT
                else:
                    joy_msg.joystick_zone = JS_ORIGIN
            else:
                # If not controlled, zero out everything
                joy_msg.forward_back = 0
                joy_msg.left_right = 0
                joy_msg.joystick_zone = JS_ORIGIN

            # Publish the message
            keyboard_publisher.publisher_.publish(joy_msg)

            # Spin once to process incoming callbacks (override, joystick position, services)
            rclpy.spin_once(keyboard_publisher, timeout_sec=0)

    except Exception as e:
        print(f"Error: {e}")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        keyboard_publisher.rm_shared_service()
        keyboard_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()