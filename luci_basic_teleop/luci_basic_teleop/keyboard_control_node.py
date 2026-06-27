import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
import sys
import select
import termios
import tty

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
        super().__init__('keyboard_teleop_node')
        
        # Publishing to cmd_vel for standard ROS routing
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Service client to talk to the LuciStateManager
        self.mode_client = self.create_client(SetBool, '/luci/request_controlled_mode')
        
        # Safety limits
        self.linear_speed = 0.5  # m/s
        self.angular_speed = 0.4 # rad/s
        self.is_controlled = False

    def toggle_mode(self):
        if not self.mode_client.wait_for_service(timeout_sec=1.0):
            print("\n[Error: State Manager service not available! Is it running?]")
            return

        # Toggle local state
        self.is_controlled = not self.is_controlled
        
        # Send the request to the State Manager
        req = SetBool.Request()
        req.data = self.is_controlled
        
        # Use call_async so we don't freeze the keyboard while waiting
        future = self.mode_client.call_async(req)
        future.add_done_callback(self.mode_response_callback)
        
        state_str = "CONTROLLED" if self.is_controlled else "IDLE"
        print(f"\n[Requesting Mode Switch to: {state_str}]")

    def mode_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                print(f"\n[Success: {response.message}]")
            else:
                print(f"\n[Rejected: {response.message}]")
                # Revert local state if the State Manager rejected us
                self.is_controlled = not self.is_controlled 
        except Exception as e:
            print(f"\n[Service call failed: {e}]")
            self.is_controlled = not self.is_controlled

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardPublisher()
    
    msg = """
    ---------------------------
    Standard Twist Keyboard Teleop
    ---------------------------
    Controls:
      W : Forward
      S : Backward
      A : Turn Left
      D : Turn Right
      T : Toggle CONTROLLED / IDLE Mode
      
      CTRL-C to quit
    ---------------------------
    """
    print(msg)

    try:
        while rclpy.ok():
            key = getKey()
            
            # Handle Ctrl-C termination
            if key == '\x03':
                break

            # Handle Mode Toggle
            if key.lower() == 't':
                node.toggle_mode()

            # Create a blank Twist message (defaults all values to 0.0)
            twist_msg = Twist()

            # Map keys to standard velocities ONLY if we are in CONTROLLED mode
            if node.is_controlled:
                if key.lower() == 'w':
                    twist_msg.linear.x = node.linear_speed
                elif key.lower() == 's':
                    twist_msg.linear.x = -node.linear_speed
                elif key.lower() == 'a':
                    twist_msg.angular.z = node.angular_speed
                elif key.lower() == 'd':
                    twist_msg.angular.z = -node.angular_speed

            # Always publish. If not controlled or no key pressed, it publishes 0.0
            node.publisher_.publish(twist_msg)

            # Spin once to process incoming service responses
            rclpy.spin_once(node, timeout_sec=0)

    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        
        # Try to drop to IDLE before shutting down for safety
        if node.is_controlled and node.mode_client.wait_for_service(timeout_sec=0.5):
            req = SetBool.Request()
            req.data = False
            node.mode_client.call_async(req)
            
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()