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
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
import ikpy.chain
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.duration import Duration
import ipdb
import time
from std_srvs.srv import Trigger
from std_msgs.msg import Bool


class executor(Node):

    def __init__(self):
        super().__init__('executor')
        # self.publisher_ = self.create_publisher(Marker, 'relative_point', 10)
        # self.get_logger().info("Publishing relative point...")

        self.declare_parameter('target_frame', 'base_link')
        self.target_frame = self.get_parameter(
            'target_frame').get_parameter_value().string_value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.not_find = True
        self.joint_state = None
        self.bool_msg = Bool()
        self.bool_msg.data = True
        self.trajectory_client = ActionClient(self, FollowJointTrajectory, '/stretch_controller/follow_joint_trajectory')
        server_reached = self.trajectory_client.wait_for_server(timeout_sec=60.0)
        if not server_reached:
            self.get_logger().error('Unable to connect to arm action server. Timeout exceeded.')
            exit(1)
        # self.subscription2 = self.create_subscription(JointState, '/stretch/mea_or_not', self.joint_states_callback, 1)
        self.subscription = self.create_subscription(JointState, '/stretch/joint_states', self.joint_states_callback, 1)
        # self.get_aruco_transform = self.create_timer(1.0, self.making_record)
        self.joint_names = ['translate_mobile_base', 'wrist_extension', 'joint_lift', 'joint_wrist_yaw', 'joint_wrist_pitch', 'joint_wrist_roll', 'joint_gripper_finger_left']
        self.cli = self.create_client(Trigger, 'switch_to_position_mode')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again')
        self.cla = self.create_client(Trigger, 'switch_to_navigation_mode')
        while not self.cla.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again')
        self.req = Trigger.Request()
        self.measure_subscribe = self.create_subscription(Bool, 'do_pos', self.arrivd, 10)
        self.backpublisher_ = self.create_publisher(Bool, 'do_back', 10)
        

    def send_request(self):
        print("complete")
        self.future = self.cli.call_async(self.req)
        time.sleep(3.0)
        
        return self.future
    
    def joint_states_callback(self, joint_state):
        self.joint_state = joint_state
        # self.get_logger().info(f'{self.joint_state}')

    # def measure(self):
    
    def arrivd(self, arriv):
        print("call")
        if arriv:
            self.send_request()
            self.making_record()
            print("switchback")
            self.future = self.cla.call_async(self.req)
            time.sleep(5.0)
            self.backpublisher_.publish(self.bool_msg)


    def making_record(self):
        file1 = '/home/hello-robot/cse481/team5/CSE481C/lab6/average.npy'
        array3 = np.load(file1)
        tpoint = PointStamped()
        tpoint.header.frame_id = 'target_object1'
        tpoint.point.x = array3[0]
        tpoint.point.y = array3[1]
        tpoint.point.z = array3[2]
        print(self.tf_buffer.all_frames_as_yaml())
        tires = 0
        try:
            if self.not_find:
                transformed_message = self.tf_buffer.transform(tpoint, 'base_link')
                mat=[transformed_message.point.x, transformed_message.point.y,transformed_message.point.z]
            # np.save('/home/hello-robot/cse481/team5/lab6/gp.npy', mat)
            # self.execute(mat)
            # exit(0)
        except TransformException as ex:
            tires+=1
            if tires > 50:
                return
            
        self.execute(mat)
        

        #msg = PointStamped()
        #msg.header.frame_id = "base_link"  # Set the frame ID
        #msg.point.x = 1.0  # Set the x-coordinate
        #msg.point.y = 2.0  # Set the y-coordinate
        #msg.point.z = 0.0  # Set the z-coordinate
        #self.publisher_.publish(msg)
        #self.get_logger().info("Published relative point.")
    
    def get_ik_state(self,q_lift, q_arml3, q_arml2, q_arml1, q_arml0, q_yaw, q_pitch, q_roll):
            q_base = 0.0
            return [0.0, q_base, 0.0, q_lift, 0.0, q_arml3, q_arml2, q_arml1, q_arml0, q_yaw, 0.0, q_pitch, q_roll, 0.0, 0.0]
    
    def pose_from_ik_state(self,ik_state):
        return {
            "translate_mobile_base": ik_state[1],
            "wrist_extension": ik_state[5] + ik_state[6] + ik_state[7] + ik_state[8],
            "joint_lift": ik_state[3],
            "joint_wrist_yaw": 0.00001,#ik_state[9],
            "joint_wrist_pitch": 0.00001,
            #ik_state[11],
            "joint_wrist_roll": 0.00001, #ik_state[12],
            "joint_gripper_finger_left": -0.2
        }

    def execute(self, mat):
        self.get_logger().info(f'{mat}')
        self.get_logger().info('Computing trajectory...')

        urdf_path = '/home/hello-robot/cse481/team5/CSE481C/lab6/stretch.urdf'
        chain = ikpy.chain.Chain.from_urdf_file(urdf_path)
        # ipdb.set_trace()
        #
        q_init = self.get_ik_state(
            self.joint_state.position[1],
            self.joint_state.position[2],
            self.joint_state.position[3],
            self.joint_state.position[4],
            self.joint_state.position[5],
            self.joint_state.position[8],
            self.joint_state.position[9],
            self.joint_state.position[10]
        )
        q_soln = chain.inverse_kinematics(mat, initial_position=q_init)
        # ipdb.set_trace()
        state_initial=self.pose_from_ik_state(q_init)
        state_final = self.pose_from_ik_state(q_soln)
        # forwa=[]
        # bawa = []
        # self.stepper(state_initial,state_final,forwa, bawa)
        positions_0 = []
        state_final["translate_mobile_base"] -=0.1
        state_final["wrist_extension"] += 0.1
        positions_0 = self.gothrough(state_final, positions_0)
        point_0 = JointTrajectoryPoint()
        point_0.positions = positions_0
        point_0.time_from_start = Duration(seconds=0.0).to_msg()
        positions_1 = []
        state_initial["translate_mobile_base"] = -state_final["translate_mobile_base"]
        state_initial["wrist_extension"] = 0.055
        positions_1 = self.gothrough(state_initial, positions_1)
        point_1 = JointTrajectoryPoint()
        point_1.positions = positions_1
        point_1.time_from_start = Duration(seconds=0.0).to_msg()
        

        # point_a = JointTrajectoryPoint()
        # positions_a = []
        
        # for name, value in state_initial.items():
        #     positions_a.append(value)
        # point_a.positions = positions_a
        # point_a.time_from_start = Duration(seconds=0.0).to_msg()
            
        trajectory_goal = FollowJointTrajectory.Goal()
        trajectory_goal.trajectory.header.frame_id = 'base_link'
        trajectory_goal.trajectory.joint_names = self.joint_names
        trajectory_goal.trajectory.points = [point_0]
        # trajectory_goal.trajectory.header.stamp = self.get_clock().now().to_msg()
        
        # trajectory_goal.trajectory.header.frame_id = 'base_link'
        self.trajectory_client.send_goal_async(trajectory_goal)
        time.sleep(15.0)
        # trajectory_goal1 = FollowJointTrajectory.Goal()
        # trajectory_goal1.trajectory.header.frame_id = 'base_link'
        # trajectory_goal1.trajectory.joint_names = self.joint_names
        # trajectory_goal.trajectory.points = [point_0, point_1]
        # # trajectory_goal.trajectory.header.stamp = self.get_clock().now().to_msg()
        
        # # trajectory_goal.trajectory.header.frame_id = 'base_link'
        # self.trajectory_client.send_goal_async(trajectory_goal)
        # print("complete")
        trajectory_goal1 = FollowJointTrajectory.Goal()
        trajectory_goal1.trajectory.header.frame_id = 'base_link'
        trajectory_goal1.trajectory.joint_names = self.joint_names
        trajectory_goal1.trajectory.points = [point_1]
        # trajectory_goal.trajectory.header.stamp = self.get_clock().now().to_msg()
        
        # trajectory_goal.trajectory.header.frame_id = 'base_link'
        self.trajectory_client.send_goal_async(trajectory_goal1)
        print("done")
        time.sleep(10.0)
    
    # def stepper(self, state_i, state_f,foward, backward):
    #     # base
    #     positions_0 = []
    #     state_i["translate_mobile_base"] = state_f["translate_mobile_base"]
    #     positions_0 = self.gothrough(state_i, positions_0)
    #     point_0 = JointTrajectoryPoint()
    #     point_0.positions = positions_0
    #     point_0.time_from_start = Duration(seconds=5.0).to_msg()
    #     foward.append(point_0)
    #     state_i["translate_mobile_base"] = 0.0
    #     #lift
    #     positions_1 = []
    #     state_i["joint_lift"] = state_f["joint_lift"]
    #     positions_1 = self.gothrough(state_i, positions_1)
    #     point_1 = JointTrajectoryPoint()
    #     point_1.positions = positions_1
    #     point_1.time_from_start = Duration(seconds=1.0).to_msg()
    #     foward.append(point_1)
    #     # #wrist
    #     positions_2 = []
    #     state_i["wrist_extension"] = state_f["wrist_extension"]
    #     positions_2 = self.gothrough(state_i, positions_2)
    #     point_2 = JointTrajectoryPoint()
    #     point_2.positions = positions_2
    #     point_2.time_from_start = Duration(seconds=1.0).to_msg()
    #     foward.append(point_2)

    def gothrough(self, state,position):
        for name, value in state.items():
            position.append(value)
        return position

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