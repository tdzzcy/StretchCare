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



class Recorder(Node):

    def __init__(self):
        super().__init__('recorder')

        self.declare_parameter('target_frame', 'face')
        self.target_frame = self.get_parameter(
            'target_frame').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        time_period = 1.0 # seconds
        self.timer = self.create_timer(time_period, self.on_timer)

    def on_timer(self):
        from_frame_rel = 'link_grasp_center'
        to_frame_rel = self.target_frame

        try:
            now = Time()
            trans = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                now)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        self.get_logger().info(
                        f'the pose of target frame {from_frame_rel} with reference to {to_frame_rel} is: {trans}')
        point = PointStamped()
        point.header.frame_id = 'link_grasp_center'
        point.point.x = 0.0
        point.point.y = 0.0
        point.point.z = 0.0
        transformed_point = do_transform_point(point, trans)
        mat=[transformed_point.point.x, transformed_point.point.y,transformed_point.point.z]
        np.save('/home/hello-robot/cse481/team5/CSE481C/lab6/mat5.npy', mat)





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
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()