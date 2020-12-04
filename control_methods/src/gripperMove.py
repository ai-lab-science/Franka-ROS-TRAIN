#!/usr/bin/env python2

import rospy
import actionlib
from franka_gripper.msg import MoveGoal, MoveAction

if __name__ == '__main__':
    rospy.init_node('Franka_gripper_move_action')
    client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
    client.wait_for_server()
    goal = MoveGoal(width = 0.08, speed = 0.2)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))