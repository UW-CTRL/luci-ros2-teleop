# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from luci_messages.msg import LuciJoystick

# class TwistToLuciNode(Node):
#     def __init__(self):
#         super().__init__('twist_to_luci_node')
        
#         # Subscribe to standard ROS velocity commands
#         self.xbox_twist_sub = self.create_subscription(
#             Twist,
#             '/xbox_twist',
#             self.twist_callback,
#             10)
        
#         self.state_subscriber = self.create_subscription(
#             LuciJoystick, '/luci/control_state', self.state_callback, 10)
        
#         self.ams_vel_subscriber = self.create_subscription(
#             Twist, '/luci/joystick_position', self.position_callback, 10)
            
#         # Publish to the LUCI State Manager's passthrough topic
#         # self.publisher = self.create_publisher(LuciJoystick, '/luci/remote_joystick', 10)
        
#         # Wheelchair hardware limits (MUST BE CALIBRATED)
#         # self.max_linear_speed = 2.68224 # meters per second
#         self.max_linear_speed = 0.5
#         self.max_angular_speed = 0.9 * 0.44704/ 0.4699 # radians per second

#     def twist_callback(self, msg: Twist):
#             luci_msg = LuciJoystick()
#             luci_msg.input_source = 5 # SharedRemote
            
#             # 1. Convert to raw percentages
#             linear_pct = (msg.linear.x / self.max_linear_speed) * 100.0
#             angular_pct = (msg.angular.z / self.max_angular_speed) * 100.0
            
#             # 2. THE DEADBAND BOOST (New Code)
#             # If Nav2 wants to move at all, force it past the 10% idle zone!
#             if 0.0 < linear_pct <= 10.0: linear_pct = 12.0
#             elif -10.0 <= linear_pct < 0.0: linear_pct = -12.0
            
#             if 0.0 < angular_pct <= 10.0: angular_pct = 12.0
#             elif -10.0 <= angular_pct < 0.0: angular_pct = -12.0
            
#             # 3. Clamp values
#             luci_msg.forward_back = int(max(min(linear_pct, 100), -100))
#             luci_msg.left_right = int(max(min(angular_pct, 100), -100))
            
#             # 4. Set Joystick Zone
#             if luci_msg.forward_back > 10: luci_msg.joystick_zone = 0
#             elif luci_msg.forward_back < -10: luci_msg.joystick_zone = 7
#             elif luci_msg.left_right > 10: luci_msg.joystick_zone = 3
#             elif luci_msg.left_right < -10: luci_msg.joystick_zone = 4
#             else: luci_msg.joystick_zone = 8
            
#             self.publisher.publish(luci_msg)
#             self.publisher.publish(luci_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = TwistToLuciNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()