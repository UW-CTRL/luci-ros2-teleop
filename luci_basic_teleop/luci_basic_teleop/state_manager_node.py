import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

class StateManagerNode(Node):
    def __init__(self):
        super().__init__('state_manager_node')
        ## Publishers 

        self.publisher_ = self.create_publisher(Bool, 'luci/interrupt', 10)
        
        ## Subscriptions 
        self.subscription = self.create_subscription(
            String,
            '/luci/control_state',
            self.control_state_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def control_state_callback(self, msg):
        if msg.data == "OVERRIDE":
            self.get_logger().info('OVERRIDE state detected. Publishing interrupt signal.')
            interrupt_msg = Bool()
            interrupt_msg.data = True
            self.publisher_.publish(interrupt_msg)
        else:
            self.get_logger().info('State: {}. No interrupt signal published.'.format(msg.data))
            interrupt_msg = Bool()
            interrupt_msg.data = False
            self.publisher_.publish(interrupt_msg)

def main(args=None):
    rclpy.init(args=args)
    node = StateManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()