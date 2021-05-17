#include "qr24.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv)
{

  ros::init(argc, argv, "calibration_node");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Define the calibration class
  QR24 qr24;

  // Define the planning group, thus the set of joints to be steered
  static const std::string PLANNING_GROUP = "panda_arm";

  // Setup the movegroup
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Plan to joint states
  // qmax	2.8973	1.7628	2.8973	-0.0698	 2.8973	 3.7525	 2.8973	rad
  // qmin  -2.8973 -1.7628 -2.8973	-3.0718	-2.8973	-0.0175	-2.8973 rad
  moveit::core::RobotStatePtr current_state;
  std::vector<double> joint_group_positions;
  for (int i=0; i<3; i++){
    
    // Set the pose
    current_state = move_group.getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    joint_group_positions[i] = -1.0;
    move_group.setJointValueTarget(joint_group_positions);

    // Now, we call the planner to compute the plan
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // Move the robot
    move_group.move();

    // Get the current end effektor pose
    geometry_msgs::PoseStamped pose = move_group.getCurrentPose();
    ROS_INFO("%f, %f, %f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);

    // Store pose data in the calibration class
    qr24.storeMeasurement(pose, pose);

  } 

  // Calculate calibration
  qr24.calculateCalibration();

  ros::shutdown();
  return 0;
}