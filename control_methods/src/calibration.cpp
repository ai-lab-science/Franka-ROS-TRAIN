#include "ros/ros.h"

#include "qr24.h"
#include<vector>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <geometry_msgs/PoseStamped.h>

const double PI = 3.1415;

// qmax	2.8973	1.7628	2.8973	-0.0698	 2.8973	 3.7525	 2.8973	rad
// qmin  -2.8973 -1.7628 -2.8973	-3.0718	-2.8973	-0.0175	-2.8973 rad
double calibration_poses[4][7] = { 
      {-0.39, -0.79, 0.34, -2.18, -1.17 ,2.39, 0.14}, 
      {-0.20, -1.0, 0.34, -1.50, -1.50, 2.39, 0.9},
      // {-1.0, -1.0, 0.34, -1.50, -1.50, 2.39, 0.9},
      {-1.0 , 0.5, -0.3, -1.0 , -2.5, 2, -0.9},
      {0.6 , 0.5, -0.3, -1.0 , -2.5, 2, -0.9} 
};

class HandEyeCalibration 
{
  public:
    HandEyeCalibration(ros::NodeHandle nh, ros::NodeHandle nhp);
    void callback(const geometry_msgs::PoseStamped::ConstPtr& msg_in);
    geometry_msgs::PoseStamped getOptiTrackPose();
  private:
    geometry_msgs::PoseStamped optiTrackPose;
    ros::Subscriber sub;
};

HandEyeCalibration::HandEyeCalibration(ros::NodeHandle nh, ros::NodeHandle nhp) {
  sub = nh.subscribe("/vrpn_client_node/RigidBody1/pose", 10, &HandEyeCalibration::callback, this);
}

void HandEyeCalibration::callback(const geometry_msgs::PoseStamped::ConstPtr& msg_in) {
  optiTrackPose = *msg_in;
} 

geometry_msgs::PoseStamped HandEyeCalibration::getOptiTrackPose() {
  return optiTrackPose;
}  

int main(int argc, char** argv)
{

  ros::init(argc, argv, "calibration_node");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Start the callback for the odometry data
  HandEyeCalibration handEyeCalibration(nh, nhp);

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
  moveit::core::RobotStatePtr current_state;
  std::vector<double> joint_group_positions;
  current_state = move_group.getCurrentState();
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  for (int i=0; i<4; i++){

    current_state = move_group.getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    for (int j=0; j<7; j++){
      // Set the pose
      joint_group_positions[j] = calibration_poses[i][j];
    } 
    move_group.setJointValueTarget(joint_group_positions);

    // Now, we call the planner to compute the plan
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    if (success) {
      
      // Move the robot
      move_group.move();

      // Wait to get correct position from OptiTrack
      sleep(5);

      // Get the current end effektor pose
      geometry_msgs::PoseStamped poseEndeffector = move_group.getCurrentPose();
      ROS_INFO("%f, %f, %f", poseEndeffector.pose.position.x, poseEndeffector.pose.position.y, poseEndeffector.pose.position.z);

      // Store pose data in the calibration class
      qr24.storeMeasurement(poseEndeffector, handEyeCalibration.getOptiTrackPose());
    
    }

  } 

  // Go back to initial position
  joint_group_positions[0] = 0;
  joint_group_positions[1] = -PI/4;
  joint_group_positions[2] = 0;
  joint_group_positions[3] = -3*PI/4;
  joint_group_positions[4] = 0;
  joint_group_positions[5] = PI/2;
  joint_group_positions[6] = PI/4;
  move_group.setJointValueTarget(joint_group_positions);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  if (success) move_group.move();

  // Calculate calibration
  qr24.calculateCalibration(); 

  ros::shutdown();
  return 0;
}