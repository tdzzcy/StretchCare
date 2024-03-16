#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
class MultiPointCommand(Node):
    def __init__(self):
        super().__init__('stretch_multipoint_command')
        self.trajectory_client = ActionClient(self, FollowJointTrajectory, '/stretch_controller/follow_joint_trajectory')
        server_reached = self.trajectory_client.wait_for_server(timeout_sec=60.0)
        if not server_reached:
            self.get_logger().error('Unable to connect to arm action server. Timeout exceeded.')
            sys.exit()
        self.subscription = self.create_subscription(JointState, '/stretch/joint_states', self.joint_states_callback, 1)
        self.subscription
        self.joint_state = None
        self.get_logger().info('issuing multipoint command...')
    def joint_states_callback(self, joint_state):
        print("abcdefg")
        self.joint_state = joint_state
    def issue_multipoint_command(self):
        joint_state = self.joint_state
        duration0 = Duration(seconds=0.0)
        duration1 = Duration(seconds=2.0)
        duration2 = Duration(seconds=4.0)
        duration3 = Duration(seconds=6.0)
        duration4 = Duration(seconds=8.0)
        duration5 = Duration(seconds=10.0)

        joint_value1 = joint_state.position[1] # joint_lift is at index 1
        joint_value2 = joint_state.position[0] # wrist_extension is at index 0
        joint_value3 = joint_state.position[8] # joint_wrist_yaw is at index 8
        point0 = JointTrajectoryPoint()
        point0.positions = [joint_value1, joint_value2, joint_value3]
        point0.velocities = [0.2, 0.2, 2.5]
        point0.accelerations = [1.0, 1.0, 3.5]
        point0.time_from_start = duration0.to_msg()
        point1 = JointTrajectoryPoint()
        point1.positions = [0.3, 0.1, 2.0]
        point1.time_from_start = duration1.to_msg()
        point2 = JointTrajectoryPoint()
        point2.positions = [0.5, 0.2, -1.0]
        point2.time_from_start = duration2.to_msg()
        point3 = JointTrajectoryPoint()
        point3.positions = [0.6, 0.3, 0.0]
        point3.time_from_start = duration3.to_msg()
        point4 = JointTrajectoryPoint()
        point4.positions = [0.8, 0.2, 1.0]
        point4.time_from_start = duration4.to_msg()
        point5 = JointTrajectoryPoint()
        point5.positions = [0.5, 0.1, 0.0]
        point5.time_from_start = duration5.to_msg()
        trajectory_goal = FollowJointTrajectory.Goal()
        trajectory_goal.trajectory.joint_names = ['joint_lift', 'wrist_extension', 'joint_wrist_yaw']
        trajectory_goal.trajectory.points = [point0, point1]#, point2, point3]#, #point4]#, point5]
        trajectory_goal.trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory_goal.trajectory.header.frame_id = 'base_link'
        self.trajectory_client.send_goal_async(trajectory_goal)
        self.get_logger().info('Sent stow goal = {0}'.format(trajectory_goal))
def main(args=None):
    rclpy.init(args=args)
    multipoint_command = MultiPointCommand()
    rclpy.spin_once(multipoint_command)
    multipoint_command.issue_multipoint_command()
    rclpy.spin(multipoint_command)
    multipoint_command.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()