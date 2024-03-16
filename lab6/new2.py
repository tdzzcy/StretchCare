import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
# We're going to subscribe to a JointState message type, so we need to import
# the definition for it
from sensor_msgs.msg import JointState
import numpy as np
import ros2_numpy as rn
from geometry_msgs.msg import PointStamped, Point
import tf2_ros
import tf2_geometry_msgs.tf2_geometry_msgs
from visualization_msgs.msg import Marker
import stretch_body.robot
import ikpy.chain


class executor(Node):

    def __init__(self):
        super().__init__('executor')
        self.publisher_ = self.create_publisher(Marker, 'relative_point', 10)
        self.get_logger().info("Publishing relative point...")

        self.declare_parameter('target_frame', 'base_link')
        self.target_frame = self.get_parameter(
            'target_frame').get_parameter_value().string_value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        time_period = 1.0 # seconds
        self.timer = self.create_timer(time_period, self.publish_point)

    def publish_point(self):
        file1 = 'gp.npy'
        array3 = np.load(file1)
        marker_msg = Marker()
        marker_msg.header.frame_id = 'base_link'  # Set the frame ID
        marker_msg.type = Marker.SPHERE  # Set the marker type to a sphere
        marker_msg.action = Marker.ADD  # Set the action to ADD
        marker_msg.pose.position = Point(x=array3[0], y=array3[1], z=array3[2])  # Set the position
        marker_msg.pose.orientation.x = 0.0
        marker_msg.pose.orientation.y = 0.0
        marker_msg.pose.orientation.z = 0.0
        marker_msg.pose.orientation.w = 1.0
        marker_msg.scale.x = 0.02  # Set the scale (diameter)
        marker_msg.scale.y = 0.02
        marker_msg.scale.z = 0.02
        marker_msg.color.r = 1.0  # Set the color (red)
        marker_msg.color.g = 0.0
        marker_msg.color.b = 0.0
        marker_msg.color.a = 1.0  # Set the alpha (transparency)
        marker_msg.lifetime.sec = 10  # Set the lifetime in seconds

        # Publish the Marker message
        self.publisher_.publish(marker_msg)
        


        


        #msg = PointStamped()
        #msg.header.frame_id = "base_link"  # Set the frame ID
        #msg.point.x = 1.0  # Set the x-coordinate
        #msg.point.y = 2.0  # Set the y-coordinate
        #msg.point.z = 0.0  # Set the z-coordinate
        #self.publisher_.publish(msg)
        #self.get_logger().info("Published relative point.")

def main(args=None):
    rclpy.init()
    point_publisher = executor()
    try:
        rclpy.spin(point_publisher)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()