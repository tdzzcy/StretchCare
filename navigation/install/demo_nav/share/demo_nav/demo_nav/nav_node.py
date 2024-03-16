import json

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped

from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, qos_profile_system_default
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

import json
import numpy as np

from geometry_msgs.msg import PoseStamped
from BasicNavigator import BasicNavigator, TaskResult

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool

class GoToLocationSubscriber(Node):
    def __init__(self):
        super().__init__('go_to_location_subscriber')
        self.curr_pos = None
        self.filename = "./locations.json"
        self.navigator = BasicNavigator()

        self.send_pos_sub = self.create_subscription(Bool, 'do_what', self.send_position_callback, 10)

          # callback to store the location
        
    def send_position_callback(self, work):
        print("check")
        # open the json file and append the location to it
        if work:
            x = 0 
            y = 0
            # file1 = 'patientlocation.npy'
            # array3 = np.load(file1)
            x=1.0#array3[0]
            y=-2.0#array3[1]

            final_pose = PoseStamped()
            final_pose.header.frame_id = 'map'
            final_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            final_pose.pose.position.x = x
            final_pose.pose.position.y = y
            final_pose.pose.orientation.z = 0.0
            final_pose.pose.orientation.w = 1.0

            self.navigator.goToPose(final_pose)

            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print('Goal succeeded!')
            elif result == TaskResult.CANCELED:
                print('Goal was canceled!')
            elif result == TaskResult.FAILED:
                print('Goal failed!')
            else:
                print('Goal has an invalid return status!')


def main(args=None):
    rclpy.init(args=args)

    send_location_poses_subs = GoToLocationSubscriber()

    rclpy.spin(send_location_poses_subs)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    send_location_poses_subs.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

    