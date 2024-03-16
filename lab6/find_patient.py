import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import time

class BoolPublisherNode(Node):

    def __init__(self):
        super().__init__('bool_publisher_node')
        self.mes_sub = self.create_subscription(Bool, 'needmes', self.publish_mes, 10)
        self.back_sub = self.create_subscription(Bool, 'needback', self.publish_back, 10)
        self.mespublisher_ = self.create_publisher(Bool, 'do_mes', 10)
        self.backpublisher_ = self.create_publisher(Bool, 'do_back', 10)
        self.bool_msg = Bool()
        self.bool_msg.data = True  # Set the boolean value to True initially

    def publish_mes(self,need):
        print("mes")
        if need:
            time.sleep(1.0)
            self.mespublisher_.publish(self.bool_msg)
            
            self.get_logger().info('Publishing: {}'.format(self.bool_msg.data))
            # self.destroy_node()  # Shutdown the node after publishing once
    
    def publish_back(self, need):
        print("back")
        if need:
            time.sleep(1.0)
            self.backpublisher_.publish(self.bool_msg)
            self.get_logger().info('Publishing: {}'.format(self.bool_msg.data))
            # self.destroy_node()  # Shutdown the node after publishing once

def main(args=None):
    rclpy.init(args=args)
    bool_publisher_node = BoolPublisherNode()
    rclpy.spin(bool_publisher_node)  # Run the node once
    # rclpy.shutdown()

if __name__ == '__main__':
    main()