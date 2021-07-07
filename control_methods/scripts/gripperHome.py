#!/usr/bin/env python2

import rospy
import actionlib
from franka_gripper.msg import MoveGoal, MoveAction, GraspAction, StopAction, GraspGoal, StopGoal
import curses
from std_msgs.msg import Float32

class joint:
    def __init__(self):
        rospy.Subscriber("/force_joints/hand/index_finger_joint_1", Float32, self.callback)
        self.data = Float32()
        self.flag = False
        

    def callback(self, msg):
        self.grasping(msg)

    def grasping(self,data):
        set_width = 0.1
        max_open = 0.1
        min_open = 0
        grasp = 0.07
        release = 0.09 

        print(data)
        if float(data.data) < 1.0:
            print('open')
            if (set_width <= max_open):
                self.flag = False
                client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
                client.wait_for_server()
                #set_width += step_size
                set_width = release
                goal = MoveGoal(width = set_width, speed = 0.04)
                client.send_goal(goal) 
            bend = index.data
        elif (float(data.data) > 1.2) & (self.flag == False):
            print('grasp')
            self.flag = True
            if (set_width >= min_open):
                client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
                client.wait_for_server()
                #set_width -= step_size
                set_width = grasp
                action = GraspGoal(width = 0.05, speed = 0.04, force = 1) 
                client.send_goal(action)
        

def init_gripper():
    rospy.init_node('Franka_gripper_move_action')
    client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
    client.wait_for_server()
    goal = MoveGoal(width = 0.1, speed = 0.04)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))   
       


if __name__ == '__main__':

    init_gripper()

    loop = True
    
    index = joint()
    rospy.spin()
    
           

        