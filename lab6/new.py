import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
# We're going to subscribe to a JointState message type, so we need to import
# the definition for it
import numpy as np
from tf2_geometry_msgs import PointStamped,  TransformStamped, Point
import ikpy.chain
import stretch_body.robot
import time
import geometry_msgs.msg
import tf2_ros
from tf2_ros import TransformException
from rclpy.time import Time
from geometry_msgs.msg import Point


file = 'gp.npy'

arr = np.load(file)
urdf_path = '/home/hello-robot/cse481/team5/lab6/stretch.urdf'
chain = ikpy.chain.Chain.from_urdf_file(urdf_path)
robot = stretch_body.robot.Robot()
if not robot.startup():
    print("Failed to open connection to the robot")
if not robot.is_calibrated():
    robot.home()
def bound_range(name, value):
    names = [l.name for l in chain.links]
    index = names.index(name)
    bounds = chain.links[index].bounds
    return min(max(value, bounds[0]), bounds[1])

q_base = 0.0
q_lift = bound_range('joint_lift', robot.lift.status['pos'])
q_arml = bound_range('joint_arm_l0', robot.arm.status['pos'] / 4.0)
q_yaw = bound_range('joint_wrist_yaw', robot.end_of_arm.status['wrist_yaw']['pos'])
q_pitch = bound_range('joint_wrist_pitch', robot.end_of_arm.status['wrist_pitch']['pos'])
q_roll = bound_range('joint_wrist_roll', robot.end_of_arm.status['wrist_roll']['pos'])
q_init=[0.0, q_base, 0.0, q_lift, 0.0, q_arml, q_arml, q_arml, q_arml, q_yaw, 0.0, q_pitch, q_roll, 0.0, 0.0]
target_point = [arr[0], arr[1], arr[2]]
q_soln = chain.inverse_kinematics(target_point, initial_position=q_init)
q_base = q_soln[1]
q_lift = q_soln[3]
q_arm = q_soln[5] + q_soln[6] + q_soln[7] + q_soln[8]
q_yaw = q_soln[9]
q_pitch = q_soln[11]
q_roll = q_soln[12]
robot.base.translate_by(q_base)
robot.push_command()
time.sleep(2.0)

robot.lift.move_to(q_lift)
robot.push_command()
time.sleep(2.0)

robot.arm.move_to(q_arm)
robot.push_command()
time.sleep(2.0)

robot.end_of_arm.move_to('wrist_yaw', q_yaw)
robot.push_command()
time.sleep(2.0)

robot.end_of_arm.move_to('wrist_pitch', q_pitch)
robot.push_command()
time.sleep(2.0)

robot.end_of_arm.move_to('wrist_roll', q_roll)

robot.push_command()
time.sleep(2.0)

robot.end_of_arm.move_to('stretch_gripper',-20)
time.sleep(2.0)
robot.push_command()
time.sleep(2.0)
robot.lift.move_by(0.1)
time.sleep(2.0)
robot.push_command()


#robot.stop()




        #msg = PointStamped()
        #msg.header.frame_id = "base_link"  # Set the frame ID
        #msg.point.x = 1.0  # Set the x-coordinate
        #msg.point.y = 2.0  # Set the y-coordinate
        #msg.point.z = 0.0  # Set the z-coordinate
        #self.publisher_.publish(msg)
        #self.get_logger().info("Published relative point.")
