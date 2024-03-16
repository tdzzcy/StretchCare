import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import time

class BoolPublisherNode(Node):

    def __init__(self):
        super().__init__('bool_publisher_node')
        self.align_sub = self.create_subscription(Bool, 'align', self.publish_do_align, 10)
        self.search_sub = self.create_subscription(Bool, 'search', self.publish_pos, 10)
        self.doalignpublisher_ = self.create_publisher(Bool, 'do_align', 10)
        self.pospublisher_ = self.create_publisher(Bool, 'do_pos', 10)
        self.bool_msg = Bool()
        self.bool_msg.data = True  # Set the boolean value to True initially

    def publish_do_align(self,need):
        print("align")
        if need:
            time.sleep(1.0)
            self.doalignpublisher_.publish(self.bool_msg)
            
            self.get_logger().info('Publishing: {}'.format(self.bool_msg.data))
            # self.destroy_node()  # Shutdown the node after publishing once
    
    def publish_pos(self, need):

        print("pos")
        if need:
            
            time.sleep(1.0)
            self.pospublisher_.publish(self.bool_msg)
            self.get_logger().info('Publishing: {}'.format(self.bool_msg.data))
            # self.destroy_node()  # Shutdown the node after publishing once

def main(args=None):
    rclpy.init(args=args)
    bool_publisher_node = BoolPublisherNode()
    rclpy.spin(bool_publisher_node)  # Run the node once
    # rclpy.shutdown()

if __name__ == '__main__':
    main()