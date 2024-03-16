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
import time

class GoToLocationSubscriber(Node):
    def __init__(self):
        super().__init__('go_to_location_subscriber')
        self.curr_pos = None
        self.filename = "./locations.json"
        self.navigator = BasicNavigator()

        self.send_pos_sub = self.create_subscription(Bool, 'do_mes', self.send_position_callback, 10)
        self.send_pos_sub = self.create_subscription(Bool, 'do_back', self.back_position_callback, 10)
        self.configpublisher_ = self.create_publisher(Bool, 'config', 10)
        self.alignpublisher_ = self.create_publisher(Bool, 'do_align', 10)
        self.bool_msg = Bool()
        self.bool_msg.data = True  # Set the boolean value to True initially

          # callback to store the location
        
    def send_position_callback(self, work):
        print("check")
        # open the json file and append the location to it
        if work:
            initial_pose = PoseStamped()
            initial_pose.header.frame_id = 'map'
            initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            initial_pose.pose.position.x = 1.274
            initial_pose.pose.position.y = 1.589
            initial_pose.pose.orientation.z = -0.4263
            initial_pose.pose.orientation.w = 0.90456
            self.navigator.setInitialPose(initial_pose)
            
            time.sleep(5.0)
            
            x = 0 
            y = 0
            # file1 = 'patientlocation.npy'
            # array3 = np.load(file1)
            x=1.0#array3[0]
            y=-2.0#array3[1]

            final_pose = PoseStamped()
            final_pose.header.frame_id = 'map'
            final_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            final_pose.pose.position.x = -1.1968371868133545
            final_pose.pose.position.y =  -1.221876859664917
            final_pose.pose.orientation.z = 0.8786305785179138
            final_pose.pose.orientation.w = 0.47735342383384705

            self.navigator.goToPose(final_pose)
            while not self.navigator.isTaskComplete():
                rclpy.spin_once(self.navigator)
            if self.navigator.isTaskComplete():
                time.sleep(3.0)
                self.alignpublisher_.publish(self.bool_msg)
                print("arrived")
            print("?")
            
    
    def back_position_callback(self, work):
        print("check")
        # open the json file and append the location to it
        if work:

            final_pose = PoseStamped()
            final_pose.header.frame_id = 'map'
            final_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            final_pose.pose.position.x = 1.274
            final_pose.pose.position.y =  1.589
            final_pose.pose.orientation.z = -0.4263
            final_pose.pose.orientation.w = 0.90456

            self.navigator.goToPose(final_pose)
            while not self.navigator.isTaskComplete():
                rclpy.spin_once(self.navigator)
            if self.navigator.isTaskComplete():
                self.alignpublisher_.publish(self.bool_msg)
                print("arrived")
            print("?")



def main(args=None):
    rclpy.init(args=args)

    send_location_poses_subs = GoToLocationSubscriber()
    try:
        while True:
            rclpy.spin_once(send_location_poses_subs)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    except KeyboardInterrupt:
        send_location_poses_subs.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

    