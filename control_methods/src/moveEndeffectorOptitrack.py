#!/usr/bin/env python2

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
import numpy as np

class moveOptitrack:
    group = None
    lastPose = None
    lastPosition = None
    homePosition = np.array([0.6, 0.0, 0.7])
    homeOrientation = np.array([0.0, 0.0, 0.0, 0.0])
    robotPosition = None
    transform = None

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_panda_python_interface', anonymous=True)
        rospy.Subscriber("/vrpn_client_node/RigidBody1/pose", PoseStamped, self.optitrackCallback)
        

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
    
        group_name = "panda_arm"
        self.group = moveit_commander.MoveGroupCommander(group_name)
    
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', 
                                    moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        
        print(self.homePosition)
        self.moveRobot(self.homePosition, self.homeOrientation)
        print("________________________________________________________________")
        print("initialized")

    def optitrackCallback(self, data):
        #print(data.pose.position)
        self.lastPose = data.pose
        self.lastPosition = np.array([data.pose.position.x,data.pose.position.y,data.pose.position.z])
        if self.transform is None:
            self.transform = np.array([-data.pose.position.x,-data.pose.position.y,-data.pose.position.z])

    def calcNewPosition(self):
        relativePosition = self.lastPosition+self.transform
        if np.linalg.norm(relativePosition) > 0.2:
            print("not in bounds")
            return self.homePosition
        else:
            print(relativePosition)
            return self.homePosition+relativePosition

    def moveRobot(self, goalPosition, goalOrientation):
        # Move the robot
        print('Before movement robot state: ')
        print self.group.get_current_pose().pose
        
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = goalOrientation[0] 
        pose_goal.orientation.y = goalOrientation[1]
        pose_goal.orientation.z = goalOrientation[2]
        pose_goal.orientation.w = goalOrientation[3]
        pose_goal.position.x = goalPosition[0]
        pose_goal.position.y = goalPosition[1]
        pose_goal.position.z = goalPosition[2]

        self.group.set_pose_target(pose_goal)
        plan = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()

        print('After movement robot state: ')
        print self.group.get_current_pose().pose
        self.robotPosition = np.array([self.group.get_current_pose().pose.position.x, self.group.get_current_pose().pose.position.y, self.group.get_current_pose().pose.position.z])

        self.group.stop()

if __name__ == '__main__':
    optiTrack = moveOptitrack()

    r = rospy.Rate(0.1)
    while not rospy.is_shutdown():
        print("Planning new!")
        position = optiTrack.calcNewPosition()
        optiTrack.moveRobot(position, optiTrack.homeOrientation)
        r.sleep()
