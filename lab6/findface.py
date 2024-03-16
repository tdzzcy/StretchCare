import numpy as np
import time
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import JointState
from stretch_teleop_interface_msgs.action import HeadScan

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.action.server import ServerGoalHandle
from std_srvs.srv import Trigger
from rclpy.callback_groups import ReentrantCallbackGroup



class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.exec_callback_group = ReentrantCallbackGroup()
        self.cli = ActionClient(self,HeadScan, 'head_scan')
        while not self.cli.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.markers = self.create_publisher(MarkerArray, '/aruco/marker_array', 1)
        self.cla = self.create_client(Trigger, 'switch_to_position_mode')
        while not self.cla.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again')
        self.req = Trigger.Request()

    def send_request(self):
        self.future = self.cla.call_async(self.req)
        head_goal = HeadScan.Goal()
        time.sleep(3.0)
        head_goal.name = "target_object1"
        print("abc")
        future = self.cli.send_goal_async(head_goal)
        while True:
            print(future.done())
            time.sleep(3.0)
        print("seee")        
        
        return self.future.result()


def main():
    rclpy.init()
    minimal_client = MinimalClientAsync()

    
    response = minimal_client.send_request()

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()