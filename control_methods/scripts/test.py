#!/usr/bin/env python

import rospy
import tf.transformations
import numpy as np
import os
from pyquaternion import Quaternion

from geometry_msgs.msg import PoseStamped
from franka_msgs.msg import FrankaState


class RemoteControl():

    # Define positions limits for the end effector: [[min_x, max_x], [min_y, max_y], [min_z, max_z]]
    position_limits = [[-0.6, 0.6], [-0.6, 0.6], [0.05, 0.9]]

    # Class constructor
    def __init__(self):

        # Define variables
        self.marker_pose = PoseStamped()
        self.initial_pose_robot_found = False
        self.initial_pose_marker_found = False
        self.pose_pub = None
        self.pose_sub = None

        # Load the hand-eye calibration
        dirname = os.path.dirname(__file__)
        filename = os.path.join(dirname, '../config/base2tracking.txt')
        handEye = np.genfromtxt(filename, dtype=np.double)
        base2tracking = handEye[0:3, 0:3]
        self.tracking2base = np.transpose(base2tracking)
        print('Loaded hand-eye calibration data:')
        print(self.tracking2base) 

        # Initialize ROS
        rospy.init_node("optitrack_pose_node")
        listener = tf.TransformListener()
        link_name = rospy.get_param("~link_name")

        # Get initial poseof the robots endeffector
        initial_state_sub_robot = rospy.Subscriber("franka_state_controller/franka_states",
                                    FrankaState, self.franka_initial_state_callback)
        while not self.initial_pose_robot_found:
            rospy.sleep(1)
        initial_state_sub_robot.unregister()
        print('\n Robot Initial State:')
        print('R = ')
        print(self.R_initial_robot)
        print('x = ')
        print(self.x_initial_robot)


        # Get initial pose from the optitrack system
        initial_state_sub_optitrack = rospy.Subscriber("/vrpn_client_node/RigidBody1/pose",
                                    PoseStamped, self.optitrack_initial_state_callback)
        while not self.initial_pose_marker_found:
            rospy.sleep(1)
        initial_state_sub_optitrack.unregister()
        print('\n OptiTrack Initial State:')
        print('R = ')
        print(self.R_initial_marker)
        print('x = ')
        print(self.x_initial_marker)

        # Initialize pose publisher and subscriber
        self.pose_pub = rospy.Publisher(
           "equilibrium_pose", PoseStamped, queue_size=10)
        self.pose_sub = rospy.Subscriber("/vrpn_client_node/RigidBody1/pose", PoseStamped, self.optitrack_state_callback)

        # Set the timer
        rospy.Timer(rospy.Duration(0.005),
              lambda msg: self.posePublisher(msg, link_name))

        # Spin it
        rospy.spin()


    # Define the franka state callback function for retrieving initial pose information
    def franka_initial_state_callback(self, msg):

        # Get the quaternions
        initial_quaternion = \
            tf.transformations.quaternion_from_matrix(
                np.transpose(np.reshape(msg.O_T_EE,
                                        (4, 4))))
        initial_quaternion = initial_quaternion / np.linalg.norm(initial_quaternion)

        # Define the pose stamped message
        initial_pose_robot = PoseStamped()
        initial_pose_robot.pose.orientation.x = initial_quaternion[0]
        initial_pose_robot.pose.orientation.y = initial_quaternion[1]
        initial_pose_robot.pose.orientation.z = initial_quaternion[2]
        initial_pose_robot.pose.orientation.w = initial_quaternion[3]
        initial_pose_robot.pose.position.x = msg.O_T_EE[12]
        initial_pose_robot.pose.position.y = msg.O_T_EE[13]
        initial_pose_robot.pose.position.z = msg.O_T_EE[14] 

        # Get rotation matrix and position vector
        self.R_initial_robot, self.x_initial_robot = self.get_orientation_and_position(initial_pose_robot)
        self.initial_pose_robot_found = True


    # The OptiTrack callback function to retrieve the initial pose
    def optitrack_initial_state_callback(self, msg):

        # Get rotation matrix and position vector
        R, x = self.get_orientation_and_position(msg)

        # Transform to base coordinates
        self.R_initial_marker, self.x_initial_marker = self.rotate_tracking2base(R, x)
        self.initial_pose_marker_found = True
    

    # Pose publisher
    def posePublisher(self, msg, link_name):
        self.marker_pose.header.frame_id = link_name
        self.marker_pose.header.stamp = rospy.Time(0)
        self.pose_pub.publish(self.marker_pose)


    # The OptiTrack callback function, to retrieve current pose of the marker object for remote control
    def optitrack_state_callback(self, msg):

        # Transform to rotation matrix and position vector
        R, x = self.get_orientation_and_position(msg)

        # Rotate to message to base coordinates
        R_base, x_base = self.rotate_tracking2base(R, x)
    
        # Get the current marker object pose, adjust the information, check also the limits
        self.marker_pose.pose.position.x = max([min([x_base[0] - self.x_initial_marker[0],
                                          self.position_limits[0][1]]),
                                          self.position_limits[0][0]]) + self.x_initial_robot[0] 
        self.marker_pose.pose.position.y = max([min([x_base[1] - self.x_initial_marker[1],
                                          self.position_limits[1][1]]),
                                          self.position_limits[1][0]]) + self.x_initial_robot[1] 
        self.marker_pose.pose.position.z = max([min([x_base[2] - self.x_initial_marker[2],
                                          self.position_limits[2][1]]),
                                          self.position_limits[2][0]]) + self.x_initial_robot[2] 

        # Calculate the rotation
        R_diff = np.matmul(self.R_initial_robot, np.transpose(self.R_initial_marker))
        R_des = np.matmul(R_diff, R_base)

        Q, R = np.linalg.qr(R_des)
        q = Quaternion(matrix=self.R_initial_robot)

        self.marker_pose.pose.orientation.w = q[0]; 
        self.marker_pose.pose.orientation.x = q[1]; 
        self.marker_pose.pose.orientation.y = q[2]; 
        self.marker_pose.pose.orientation.z = q[3]; 


    # Returns from a PoseStamped message the according rotation matrix and the position vector
    def get_orientation_and_position(self, msg):
        quat = Quaternion(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z)
        R = quat.rotation_matrix
        x = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        return R, x


    # Transform R and x to base coordinates
    def rotate_tracking2base(self, R, x):
        R_base = np.matmul(self.tracking2base, R)
        x_base = np.matmul(self.tracking2base, x)
        return R_base, x_base





if __name__ == "__main__":

    remoteControl = RemoteControl()
    




