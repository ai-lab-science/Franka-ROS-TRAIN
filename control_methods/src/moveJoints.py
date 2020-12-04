#!/usr/bin/env python2

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_panda_python_interface', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
   
    group_name = "panda_arm"
    group = moveit_commander.MoveGroupCommander(group_name)
  
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', 
                                moveit_msgs.msg.DisplayTrajectory, queue_size=20)

    # Move the robot
    print('Joint positions before movement: ')
    joint_goal = group.get_current_joint_values()
    for i in range(7):
        print(joint_goal[i])
    
    joint_goal[0] = 0.0         # -2.8973 - +2.8973
    joint_goal[1] = 0.0         # -1.7628 - +1.7628
    joint_goal[2] = 0.0         # -2.8973 - +2.8973
    joint_goal[3] = -1.501      # -3.0718 - -0.0698
    joint_goal[4] = 0.0         # -2.8973 - +2.8973
    joint_goal[5] = 1.8675      # -0.0175 - +3.7525
    joint_goal[6] = 0.0         # -2.8973 - +2.8973

    group.go(joint_goal, wait=True)

    print('Joint positions after movement: ')
    joint_goal = group.get_current_joint_values()
    for i in range(7):
        print(joint_goal[i])

    group.stop()