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
from geometry_msgs.msg import PointStamped, TransformStamped
from tf2_geometry_msgs import do_transform_point
from std_msgs.msg import Bool
from rclpy.action import ActionClient
from rclpy.duration import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import time
from std_srvs.srv import Trigger
from stretch_teleop_interface_msgs.action import HeadScan






class Recorder(Node):

    def __init__(self):
        super().__init__('recorder')

        self.declare_parameter('target_frame', 'toy')
        self.target_frame = self.get_parameter(
            'target_frame').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        time_period = 1.0 # seconds
        
        self.align_subscribe = self.create_subscription(Bool, 'do_align', self.align, 10)
        self.trajectory_client = ActionClient(self, FollowJointTrajectory, '/stretch_controller/follow_joint_trajectory')
        
        self.req = Trigger.Request()
        self.cle = ActionClient(self,HeadScan, 'head_scan')
        while not self.cle.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.exepublisher_ = self.create_publisher(Bool, 'do_pos', 10)
        self.bool_msg = Bool()
        self.bool_msg.data = True



    def align(self, do):
        if do:
            print("aligning")
            self.cli = self.create_client(Trigger, 'switch_to_position_mode')
            while not self.cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again')
            self.cli.call_async(self.req)
            time.sleep(3.0)
            from_frame_rel = 'base_link'
            to_frame_rel = self.target_frame

            while True:
                try:
                    rclpy.spin_once(self)
                    now = Time()
                    trans = self.tf_buffer.lookup_transform(
                        to_frame_rel,
                        from_frame_rel,
                        now)
                    break
                except TransformException as ex:
                    self.get_logger().info(
                        f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')

            self.get_logger().info(
                            f'the pose of target frame {from_frame_rel} with reference to {to_frame_rel} is: {trans}')
            point = PointStamped()
            point.header.frame_id = 'link_grasp_center'
            point.point.x = 0.0
            point.point.y = 0.0
            point.point.z = 0.0
            transformed_point = do_transform_point(point, trans)
            rotate_point = JointTrajectoryPoint()
        
            rotate_point.positions = [-3.625/2]
            rotate_point.time_from_start = Duration(seconds=5.0).to_msg()

            rotate_goal = FollowJointTrajectory.Goal()
            rotate_goal.trajectory.joint_names = ['rotate_mobile_base']
            rotate_goal.trajectory.points = [rotate_point]
            self.trajectory_client.send_goal_async(rotate_goal)
            print(trans._transform.translation.y)
            step = abs(trans._transform.translation.y)-0.1
            print(trans._transform.rotation.z)
            time.sleep(5.0)

            translate_point = JointTrajectoryPoint()
            translate_point.positions = [step]
            translate_point.time_from_start = Duration(seconds=5.0).to_msg()

            translate_goal = FollowJointTrajectory.Goal()
            translate_goal.trajectory.joint_names = ['translate_mobile_base']
            translate_goal.trajectory.points = [translate_point]
            self.trajectory_client.send_goal_async(translate_goal)
            time.sleep(5.0)
            rotate_point1 = JointTrajectoryPoint()
        
            rotate_point1.positions = [3.625*10/18.0]
            rotate_point1.time_from_start = Duration(seconds=5.0).to_msg()

            rotate_goal1 = FollowJointTrajectory.Goal()
            rotate_goal1.trajectory.joint_names = ['rotate_mobile_base']
            rotate_goal1.trajectory.points = [rotate_point1]
            self.trajectory_client.send_goal_async(rotate_goal1)
            print(trans._transform.translation.y)
            step = abs(trans._transform.translation.y)
            print(trans._transform.rotation.z)
            time.sleep(5.0)
            head_goal = HeadScan.Goal()
            head_goal.name = "target_object1"
            print("abc")
            self.cle.send_goal_async(head_goal)
            time.sleep(20.0)
            print("done")
            self.exepublisher_.publish(self.bool_msg)
            






        """
		print_states function to deal with the incoming JointState messages.
		:param self: The self reference.
		:param joints: A list of string values of joint names.
		# Create an empty list that will store the positions of the requested joints
        joint_positions = []

		# Use of forloop to parse the names of the requested joints list.
		# The index() function returns the index at the first occurrence of
		# the name of the requested joint in the self.joint_states.name list
        for joint in joints:
            if joint == "wrist_extension":
                index = self.joint_states.name.index('joint_arm_l0')
                joint_positions.append(4*self.joint_states.position[index])
                continue
			
            index = self.joint_states.name.index(joint)
            joint_positions.append(self.joint_states.position[index])

        #####
        
        #######
		# Print the joint position values to the terminal
        print("name: " + str(joints))
        print("position: " + str(joint_positions))

		# Sends a signal to rclpy to shutdown the ROS interfaces
        rclpy.shutdown()

		# Exit the Python interpreter
        sys.exit(0)		
        """



def main(args=None):
    rclpy.init()
    node = Recorder()
    try:
        while True:
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()