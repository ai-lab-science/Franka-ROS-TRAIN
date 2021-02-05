#!/usr/bin/env python2

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
import copy
import time

class MoveTrajectory:
    trajectory = np.array([])
    trajOrientation = None
    startPosition = None
    startOrientation = None
    group = None
    prevPlan = None
    
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_panda_python_interface', anonymous=True)
        

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
    
        group_name = "panda_arm"
        self.group = moveit_commander.MoveGroupCommander(group_name)
    
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', 
                                    moveit_msgs.msg.DisplayTrajectory, queue_size=20)


    def goToHome(self):
        # Move the robot
        print('Joint positions before movement: ')
        joint_goal = self.group.get_current_joint_values()
        for i in range(7):
            print(joint_goal[i])
        
        joint_goal[0] = 0.0         # -2.8973 - +2.8973
        joint_goal[1] = 0.0         # -1.7628 - +1.7628
        joint_goal[2] = 0.0         # -2.8973 - +2.8973
        joint_goal[3] = -1.501      # -3.0718 - -0.0698
        joint_goal[4] = 0.0         # -2.8973 - +2.8973
        joint_goal[5] = 1.8675      # -0.0175 - +3.7525
        joint_goal[6] = 0.0         # -2.8973 - +2.8973

        self.group.go(joint_goal, wait=True)

        print('Joint positions after movement: ')
        joint_goal = self.group.get_current_joint_values()
        for i in range(7):
            print(joint_goal[i])

        self.group.stop()
    
    def moveRobot(self, goalPosition, goalOrientation):
        # Move the robot
        print('Before movement robot state: ')
        print self.group.get_current_pose().pose
        
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = goalPosition[0]
        pose_goal.position.y = goalPosition[1]
        pose_goal.position.z = goalPosition[2]
        pose_goal.orientation.w = goalOrientation

        self.group.set_pose_target(pose_goal)
        plan = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()

        print('After movement robot state: ')
        print self.group.get_current_pose().pose

        self.group.stop()

    def calcTrajectory(self, traj, orient = None):
        self.trajectory = traj
        self.trajOrientation = orient
        if self.startPosition is None:
            self.startPosition = np.array([self.group.get_current_pose().pose.position.x, self.group.get_current_pose().pose.position.y, self.group.get_current_pose().pose.position.z])
        if self.startOrientation is None:
            self.startOrientation = self.group.get_current_pose().pose.orientation.w
        if self.trajOrientation is None:
            self.trajOrientation = self.startOrientation

        waypoints = []
        for checkpoint in self.trajectory:
            wpose = self.group.get_current_pose().pose
            wpose.position.x = self.startPosition[0]+checkpoint[0] 
            wpose.position.y = self.startPosition[1]+checkpoint[1] 
            wpose.position.z = self.startPosition[2]+checkpoint[2]
            waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
        (plan, fraction) = self.group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold

        plan = self.group.retime_trajectory(self.group.get_current_state(), plan, velocity_scaling_factor=1.0, acceleration_scaling_factor=1.0, algorithm="time_optimal_trajectory_generation")

        #if self.prevPlan is None:
        #    self.prevPlan = plan.joint_trajectory.points.pop()
        #    return plan
        
        #plan.joint_trajectory.points[0] = self.prevPlan 
        #self.prevPlan = plan.joint_trajectory.points.pop()

        #print(plan.joint_trajectory.points)
        return plan

    def followPlan(self, plan):
        self.group.execute(plan, wait=True)



def createCircleTraj(r):
    circle = [] 
    for i in range(100):
        point = np.array([0.0, 0.1*np.cos(i*2*np.pi/100), 0.1*np.sin(i*2*np.pi/100)])
        circle.append(point)
    return circle


if __name__ == '__main__':
    orientation = 0.0
    #upRight = np.array([0.0, 0.1, 0.1])
    #upLeft = np.array([0.0, -0.1, 0.1])
    #downRight = np.array([0.0, 0.1, -0.1])
    #downLeft = np.array([0.0, -0.1, -0.1])
    #square = np.array([upRight, upLeft, downLeft, downRight])
    circle = createCircleTraj(0.1)

    trajMover = MoveTrajectory()
    trajMover.goToHome()

    for p in circle:
        plan = trajMover.calcTrajectory([p])
        trajMover.followPlan(plan)

    #trajMover.goToHome()