#!/usr/bin/env python2

import rospy
import actionlib
from franka_gripper.msg import MoveGoal, MoveAction, GraspAction, StopAction, GraspGoal, StopGoal
import curses

if __name__ == '__main__':
    rospy.init_node('Franka_gripper_move_action')
    client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
    client.wait_for_server()
    goal = MoveGoal(width = 0.1, speed = 0.04)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))

    max_open = 0.1
    min_open = 0
    set_width = 0.1
    #step_size = 0.01
    grasp = 0.07
    release = 0.09 
 
    # get the curses screen window
    screen = curses.initscr()
    
    # turn off input echoing
    curses.noecho()
    
    # respond to keys immediately (don't wait for enter)
    curses.cbreak()
    
    # map arrow keys to special values
    screen.keypad(True)

    loop = True
    

    while loop:
        char = screen.getch()
        if char == ord('q'):
            break
        elif char == curses.KEY_RIGHT:
            # print doesn't work with curses, use addstr instead
            screen.addstr(0, 0, 'open gripper (right)')
            if (set_width <= max_open):
                client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
                client.wait_for_server()
                #set_width += step_size
                set_width = release
                goal = MoveGoal(width = set_width, speed = 0.04)
                client.send_goal(goal) 
        elif char == curses.KEY_LEFT:
            screen.addstr(0, 0, 'close gripper (left)')   
            if (set_width >= min_open):
                client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
                client.wait_for_server()
                #set_width -= step_size
                set_width = grasp
                action = GraspGoal(width = set_width, speed = 0.04, force = 1) 
                client.send_goal(action) 
        elif char == curses.KEY_UP:
            curses.nocbreak(); screen.keypad(0); curses.echo()
            curses.endwin()  
            loop = False   

        