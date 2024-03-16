#!/usr/bin/env python3

import rospy
import tf2_ros
import actionlib
import dynamic_reconfigure.client
from geometry_msgs.msg import TransformStamped, Pose, Transform
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from stretch_teleop_interface_msgs.srv import NavigateToAruco, RelativePose
from stretch_teleop_interface_msgs.msg import ArucoNavigationState
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import sqrt, fabs, atan2
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectoryPoint
from visualization_msgs.msg import MarkerArray
from tf2_geometry_msgs import PoseStamped
import numpy as np
from sensor_msgs.msg import JointState

class ArucoNavigation:
    def __init__(self):
        # Service for enabling aruco navigation
        self.navigation_service = rospy.Service('navigate_to_aruco', NavigateToAruco, self.navigate_to_aruco_callback)
        self.relative_pose_service = rospy.Service('get_relative_pose', RelativePose, self.get_relative_pose_callback)
        self.navigation_state = rospy.Publisher('/navigate_to_aruco/state', ArucoNavigationState, queue_size=1)
        self.state = ArucoNavigationState()

        self.navigation_complete = False
        self.reconfigure_client = dynamic_reconfigure.client.Client("move_base")
        self.aruco_name = ''
        self.tilt_range = [-1.0, 0.0]
        self.pan_range = [-4.07, 1.74]
        self.pan = None
        self.tilt = None
        self.num_tries = 0
        self.max_tries = 3

        self.tf2_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf2_buffer)
        self.tf2_broadcaster = tf2_ros.TransformBroadcaster()

        self.action_client = actionlib.SimpleActionClient(
            "move_base",
            MoveBaseAction
        )

        self.trajectory_client = actionlib.SimpleActionClient(
            "stretch_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction
        )

        self.aruco_marker_array = rospy.Subscriber('aruco/marker_array', MarkerArray, self.aruco_callback)
        self.joint_states = rospy.Subscriber('joint_states', JointState, self.joint_state_callback)

    def joint_state_callback(self, msg):
        for name, position in zip(msg.name, msg.position):
            if name == "joint_head_pan":
                self.pan = position
            if name == "joint_head_tilt":
                self.tilt = position
                
    def aruco_callback(self, msg):
        self.markers = msg.markers

    def broadcast_tf(self, trans, name, ref):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = ref
        t.child_frame_id = name
        t.transform = trans
        return t

    def calculate_diff(self, trans1, trans2):
        angle1 = euler_from_quaternion([trans1.transform.rotation.x,
                                        trans1.transform.rotation.y,
                                        trans1.transform.rotation.z,
                                        trans1.transform.rotation.w])
        angle2 = euler_from_quaternion([trans2.transform.rotation.x,
                                        trans2.transform.rotation.y,
                                        trans2.transform.rotation.z,
                                        trans2.transform.rotation.w])
        translation_diff = sqrt((trans1.transform.translation.x - trans2.transform.translation.x)**2 + \
                    (trans1.transform.translation.y - trans2.transform.translation.y)**2)
        angle_diff = fabs(angle1[2] - angle2[2])
        return translation_diff, angle_diff
        
    def set_pan_tilt_camera(self, pan, tilt):
        point = JointTrajectoryPoint()
        point.time_from_start = rospy.Duration(1.0)
        point.positions = [pan, tilt]

        head_goal = FollowJointTrajectoryGoal()
        head_goal.trajectory.joint_names = ['joint_head_pan', 'joint_head_tilt']
        head_goal.trajectory.points = [point]
        head_goal.trajectory.header.stamp = rospy.Time.now()
        head_goal.trajectory.header.frame_id = 'base_link'

        # self.trajectory_client.cancel_all_goals()
        self.trajectory_client.send_goal(head_goal)
        self.trajectory_client.wait_for_result()

    def scan_for_marker(self):
        for marker in self.markers:
            if self.aruco_name == marker.text:
                print("print marker found")
                return True
        
        self.state.state = "Searching for marker..."
        self.state.alert_type = "info"
        self.navigation_state.publish(self.state)

        if self.pan != None and self.tilt != None:
            print('looking around')
            tilt_angles = [
                max(self.tilt_range[0], self.tilt - 0.5), 
                max(self.tilt_range[0], self.tilt - 0.25),
                min(self.tilt_range[1], self.tilt + 0.25),
                min(self.tilt_range[1], self.tilt + 0.5)]
            pan_angles = [
                max(self.pan_range[0], self.pan - 0.5), 
                max(self.pan_range[0], self.pan - 0.25),
                min(self.pan_range[1], self.pan + 0.25),
                min(self.pan_range[1], self.pan + 0.5)]
            for tilt in tilt_angles[::-1]:
                self.tilt = tilt
                pan_angles = pan_angles[::-1]
                for pan in pan_angles:
                    self.pan = pan
                    self.set_pan_tilt_camera(self.pan, self.tilt)
                    print('looking')
                    rospy.sleep(0.2)
                    for marker in self.markers:
                        print(self.aruco_name, marker.text)
                        if self.aruco_name == marker.text:
                            print("print marker found")
                            return True

        tilt_angles = np.linspace(self.tilt_range[0], self.tilt_range[1], 5)
        pan_angles = np.linspace(self.pan_range[0], self.pan_range[1], 15)
        for tilt in tilt_angles[::-1]:
            self.tilt = tilt
            pan_angles = pan_angles[::-1]
            for pan in pan_angles:
                self.pan = pan
                self.set_pan_tilt_camera(self.pan, self.tilt)
                rospy.sleep(0.2)
                for marker in self.markers:
                    print(self.aruco_name, marker.text)
                    if self.aruco_name == marker.text:
                        print("print marker found")
                        return True
        # self.state.state = ""
        # self.state.alert_type = ""
        # self.navigation_state.publish(self.state)
        return False

    def get_relative_pose_callback(self, req):
        print('in callback')
        self.aruco_name = req.name
        marker_found = self.scan_for_marker()
        if not marker_found: return None

        try:
            map_pose = self.tf2_buffer.lookup_transform('map', 'base_link', rospy.Time(), rospy.Duration(1.0))
            pose_stamped =  PoseStamped()
            pose_stamped.header = map_pose.header
            pose_stamped.pose.position.x = map_pose.transform.translation.x
            pose_stamped.pose.position.y = map_pose.transform.translation.y
            pose_stamped.pose.position.z = map_pose.transform.translation.z
            pose_stamped.pose.orientation.x = map_pose.transform.rotation.x
            pose_stamped.pose.orientation.y = map_pose.transform.rotation.y
            pose_stamped.pose.orientation.z = map_pose.transform.rotation.z
            pose_stamped.pose.orientation.w = map_pose.transform.rotation.w

            trans = self.tf2_buffer.transform(pose_stamped, self.aruco_name, rospy.Duration(1.0))
        
            transform = Transform()
            # transform.transform.header = trans.header
            transform.translation.x = trans.pose.position.x
            transform.translation.y = trans.pose.position.y
            transform.translation.z = trans.pose.position.z
            transform.rotation.x = trans.pose.orientation.x
            transform.rotation.y = trans.pose.orientation.y
            transform.rotation.z = trans.pose.orientation.z
            transform.rotation.w = trans.pose.orientation.w

            # tran = self.broadcast_tf(trans.transform, 'relative_pose', self.aruco_name)
            # self.tf2_broadcaster.sendTransform(tran)
            print(transform)
            return transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo("Could not publish pose to tf")
            pass

    def average_transforms(self, transforms):
        trans = transforms[0]
        trans.transform.translation.z = 0
        angles = euler_from_quaternion([trans.transform.rotation.x,
                                        trans.transform.rotation.y,
                                        trans.transform.rotation.z,
                                        trans.transform.rotation.w])
        angle = angles[2]

        for i in range(1, len(transforms)):
            trans.transform.translation.x += transforms[i].transform.translation.x
            trans.transform.translation.y += transforms[i].transform.translation.y
            angles = euler_from_quaternion([transforms[i].transform.rotation.x,
                                            transforms[i].transform.rotation.y,
                                            transforms[i].transform.rotation.z,
                                            transforms[i].transform.rotation.w])
            angle += angles[2]

        trans.transform.translation.x /= len(transforms)
        trans.transform.translation.y /= len(transforms)
        angle /= len(transforms)
        
        q = quaternion_from_euler(0, 0, angle)
        trans.transform.rotation.x = q[0]
        trans.transform.rotation.y = q[1]
        trans.transform.rotation.z = q[2]
        trans.transform.rotation.w = q[3]

        return trans

    def navigate_to_marker(self):
        self.num_tries += 1
        print(self.relative_pose)
        tran = self.broadcast_tf(self.relative_pose, 'relative_pose', self.aruco_name)
        print(tran)
        self.tf2_broadcaster.sendTransform(tran)
        transforms = []
        self.state.state = "Navigating to marker..."
        self.state.alert_type = "info"
        self.navigation_state.publish(self.state)
        while len(transforms) < 10:
            try:
                trans = self.tf2_buffer.lookup_transform('map', 'relative_pose', rospy.Time(), rospy.Duration(1.0))
                transforms.append(trans)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.loginfo("Could not publish pose to tf")
                self.state.state = 'Navigation failed, please try again.'
                self.state.alert_type = "error"
                self.navigation_state.publish(self.state)
                return 'Navigation failed, please try again.'

        # tilt_angles = [
        #     max(self.tilt_range[0], self.tilt - 0.25), 
        #     max(self.tilt_range[0], self.tilt - 0.15),
        #     max(self.tilt_range[0], self.tilt - 0.05),
        #     min(self.tilt_range[1], self.tilt + 0.05),
        #     min(self.tilt_range[1], self.tilt + 0.15),
        #     min(self.tilt_range[1], self.tilt + 0.25)]
        # pan_angles = [
        #     max(self.pan_range[0], self.pan - 0.25), 
        #     max(self.pan_range[0], self.pan - 0.15),
        #     max(self.pan_range[0], self.pan - 0.05),
        #     min(self.pan_range[1], self.pan + 0.05),
        #     min(self.pan_range[1], self.pan + 0.15),
        #     min(self.pan_range[1], self.pan + 0.25)]
        # for tilt in tilt_angles[::-1]:
        #     pan_angles = pan_angles[::-1]
        #     for pan in pan_angles:
        #         self.set_pan_tilt_camera(pan, tilt)
        #         rospy.sleep(0.3)
        #         try:
        #             trans = self.tf2_buffer.lookup_transform('map', 'relative_pose', rospy.Time(), rospy.Duration(1.0))
        #             transforms.append(trans)
        #         except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #             rospy.loginfo("Could not publish pose to tf")

        # Bring predock_frame at base_link level
        # angles = euler_from_quaternion([trans.transform.rotation.x,
        #                                 trans.transform.rotation.y,
        #                                 trans.transform.rotation.z,
        #                                 trans.transform.rotation.w])
        # q = quaternion_from_euler(0, 0, angles[2])
        # trans.transform.translation.z = 0
        # trans.transform.rotation.x = q[0]
        # trans.transform.rotation.y = q[1]
        # trans.transform.rotation.z = q[2]
        # trans.transform.rotation.w = q[3]
        # trans.header.stamp = rospy.Time.now()
        self.relative_pose_tf = self.average_transforms(transforms)

        rospy.loginfo("Published relative pose")

        goal_pose = Pose()
        goal_pose.position.x = self.relative_pose_tf.transform.translation.x
        goal_pose.position.y = self.relative_pose_tf.transform.translation.y
        goal_pose.position.z = self.relative_pose_tf.transform.translation.z
        goal_pose.orientation.x = self.relative_pose_tf.transform.rotation.x
        goal_pose.orientation.y = self.relative_pose_tf.transform.rotation.y
        goal_pose.orientation.z = self.relative_pose_tf.transform.rotation.z
        goal_pose.orientation.w = self.relative_pose_tf.transform.rotation.w

        action_goal = MoveBaseGoal()
        action_goal.target_pose.header.frame_id = "map"
        action_goal.target_pose.header.stamp = rospy.Time.now()
        action_goal.target_pose.pose = goal_pose

        result = self.action_client.send_goal_and_wait(action_goal)
        
        return self.move_base_aruco_callback(result)
        

        # saved_pose = Transform()
        # saved_pose.translation.x = 0.0
        # saved_pose.translation.y = -1.0
        # saved_pose.translation.z = 0.47
        # saved_pose.rotation.x = -0.382 # -0.356
        # saved_pose.rotation.y = -0.352 # -0.356
        # saved_pose.rotation.z = -0.604 # -0.611
        # saved_pose.rotation.w = 0.604 # 0.611

    def move_base_aruco_callback(self, result):
        if self.num_tries == self.max_tries:
            self.num_tries = 0
            self.state.state = 'Navigation succeeded!'
            self.state.alert_type = "success"
            self.navigation_state.publish(self.state)
            return 'Navigation succeeded!'

        rospy.sleep(0.1)
        marker_found = self.scan_for_marker()
        if not marker_found: 
            self.state.state = 'Could not find Aruco Marker. Stretch may be too far away, try moving it closer to the marker.'
            self.state.alert_type = "error"
            self.navigation_state.publish(self.state)
            return 'Could not find Aruco Marker. Stretch may be too far away, try moving it closer to the marker.'
        # self.state.state = ""
        # self.state.alert_type = ""
        # self.navigation_state.publish(self.state)
        self.reconfigure_client.update_configuration({"controller_frequency": 18.0})

        try:
            new_map_to_aruco = self.tf2_buffer.lookup_transform('map', self.aruco_name, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo("Could not publish pose to tf")
            self.state.state = 'Navigation failed, please try again.'
            self.state.alert_type = "error"
            self.navigation_state.publish(self.state)
            return 'Navigation failed, please try again.'

        translation_diff, angle_diff = self.calculate_diff(self.map_to_aruco, new_map_to_aruco)
        if (translation_diff > 0.075 or angle_diff > 0.075):
            self.map_to_aruco = new_map_to_aruco
            # self.reconfigure_client.update_configuration({"controller_frequency": 10.0})
            self.navigate_to_marker()
        # elif (translation_diff > 0.05 or angle_diff > 0.05):
        #     self.map_to_aruco = new_map_to_aruco
        #     self.reconfigure_client.update_configuration({"controller_frequency": 5.0})
        #     self.navigate_to_marker()
        # elif (translation_diff > 0.025 or angle_diff > 0.025):
        #     self.map_to_aruco = new_map_to_aruco
        #     self.reconfigure_client.update_configuration({"controller_frequency": 1.0})
        #     self.navigate_to_marker()

        self.state.state = 'Navigation succeeded!'
        self.state.alert_type = "success"
        self.navigation_state.publish(self.state)
        return 'Navigation succeeded!'

    def navigate_to_aruco_callback(self, req):
        print('in navigate callback')
        self.aruco_name = req.name
        print(req.pose)
        self.relative_pose = req.pose
        marker_found = self.scan_for_marker()
        if not marker_found: 
            self.state.state = 'Could not find Aruco Marker. Stretch may be too far away, try moving it closer to the marker.'
            self.state.alert_type = "error"
            self.navigation_state.publish(self.state)
            return 'Could not find Aruco Marker. Stretch may be too far away, try moving it closer to the marker.'

        self.reconfigure_client.update_configuration({"controller_frequency": 20.0})
                
        self.marker = self.tf2_buffer.lookup_transform('base_link', self.aruco_name, rospy.Time())
        self.map_to_aruco = self.tf2_buffer.lookup_transform('map', self.aruco_name, rospy.Time())
        self.aruco_tf = self.broadcast_tf(self.map_to_aruco.transform, self.aruco_name, 'map')
        rospy.loginfo("{} pose published to tf".format(self.aruco_name))

        return self.navigate_to_marker()

if __name__ == '__main__':
    rospy.init_node('aruco_navigation')
    node = ArucoNavigation()
    r = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        # state = ArucoNavigationState()
        # state.state = node.state
        # node.navigation_state.publish(node.state)
        r.sleep()    