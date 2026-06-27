import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_srvs.srv import SetBool

class ControllerPublisher(Node):
    def __init__(self):
        super().__init__('controller_control_node')
        
        # Publishing to cmd_vel for standard ROS routing
        self.publisher_ = self.create_publisher(Twist, '/xbox_twist', 10)
        self.mode_client = self.create_client(SetBool, '/luci/request_controlled_mode')
        
        self.state_subscriber = self.create_subscription(
            String, 'luci/control_state', self.state_callback, 10)
        
        self.joy_subscriber = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10)
        
        self.current_mode = "IDLE"
        self.last_b_state = 0

        # Safety limits (matching keyboard node)
        self.max_linear_speed = 0.5  # m/s
        self.max_angular_speed = 0.4 # rad/s

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
            twist_msg = Twist()
            
            # Map joystick axes directly to Twist velocities.
            # Assuming standard ROS joy mapping:
            # axes[1] is Left Stick Up/Down (Forward = positive)
            # axes[0] is Left Stick Left/Right (Left = positive)
            twist_msg.linear.x = joy_msg.axes[1] * self.max_linear_speed
            twist_msg.angular.z = joy_msg.axes[0] * self.max_angular_speed

            self.publisher_.publish(twist_msg)
            

def main(args=None):
    rclpy.init(args=args)
    node = ControllerPublisher()
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt: 
        pass
    finally:
        node.request_mode(False)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()