import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import time
from std_srvs.srv import Trigger

class BoolPublisherNode(Node):

    def __init__(self):
        super().__init__('bool_publisher_node')
        self.publisher_ = self.create_publisher(Bool, 'do_mes', 10)
        self.bool_msg = Bool()
        self.bool_msg.data = True  # Set the boolean value to True initially
        self.timer_ = self.create_timer(1.0, self.publish_ounce)
        
        self.req = Trigger.Request()

    def publish_ounce(self):
        # self.cla.call_async(self.req)
        # time.sleep(3.0)
        # return None
        print("ac")
        self.publisher_.publish(self.bool_msg)
        self.get_logger().info('Publishing: {}'.format(self.bool_msg.data))
        time.sleep(5.0)

def main(args=None):
    rclpy.init(args=args)
    bool_publisher_node = BoolPublisherNode()
    
    #rclpy.spin_once(bool_publisher_node)  # Run the node once
    bool_publisher_node.publish_ounce()
    rclpy.shutdown()

if __name__ == '__main__':
    main()