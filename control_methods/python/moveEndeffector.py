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
    print('Before movement robot state: ')
    print group.get_current_pose().pose
    
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4

    group.set_pose_target(pose_goal)
    plan = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

    print('After movement robot state: ')
    print group.get_current_pose().pose

    group.stop()