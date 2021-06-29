import rospy
import actionlib
from franka_gripper.msg import HomingAction, HomingGoal

if __name__ == '__main__':
    rospy.init_node('Franka_gripper_homing_action')
    client = actionlib.SimpleActionClient('/franka_gripper/homing', HomingAction)
    client.wait_for_server()
    goal = HomingGoal()
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))