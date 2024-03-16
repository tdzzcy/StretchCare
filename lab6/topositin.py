import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import time
from std_srvs.srv import Trigger

class BoolPublisherNode(Node):

    def __init__(self):
        super().__init__('bool_publisher_node')
        self.cli = self.create_client(Trigger, 'switch_to_position_mode')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again')
        self.cla = self.create_client(Trigger, 'switch_to_navigation_mode')
        while not self.cla.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again')
        self.req = Trigger.Request()

    def publish_ounce(self):
        self.cla.call_async(self.req)
        time.sleep(3.0)
        return None

def main(args=None):
    rclpy.init(args=args)
    bool_publisher_node = BoolPublisherNode()
    
    #rclpy.spin_once(bool_publisher_node)  # Run the node once
    bool_publisher_node.publish_ounce()
    rclpy.shutdown()

if __name__ == '__main__':
    main()