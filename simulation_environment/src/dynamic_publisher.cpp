// ROS includes
#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/Float32.h"

// Other libraries
#include <string.h>

// Define String properties
#define NUMBER_OF_STRING 12
#define MAX_STRING_SIZE 40

int main(int argc, char **argv)
{
  char arr[NUMBER_OF_STRING][MAX_STRING_SIZE] =
  { "index_finger_joint_1",
    "index_finger_joint_2",
    "index_finger_joint_3",
    "middle_finger_joint_1",
    "middle_finger_joint_2",
    "middle_finger_joint_3",
    "pinkie_finger_joint_1",
    "pinkie_finger_joint_2",
    "pinkie_finger_joint_3",
    "ring_finger_joint_1",
    "ring_finger_joint_2",
    "ring_finger_joint_3"
  };

  // Set up ROS.
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;

  // Create a publisher and name the topic.
  ros::Publisher pub_message[NUMBER_OF_STRING];
  for (int i=0; i<NUMBER_OF_STRING; i++) {
	char str_tmp[MAX_STRING_SIZE + 19];
	strcpy(str_tmp, "/force_joints/hand/");
	strcat(str_tmp, arr[i]);
	pub_message[i] = n.advertise<std_msgs::Float32>(str_tmp, 10);
  }

  // Tell ROS how fast to run this node.
  ros::Rate r(20);

  // Main loop.
  while (n.ok())
  {
    // Publish the message.
    for (int i=0; i<NUMBER_OF_STRING; i++) {
		std_msgs::Float32 msg;
		msg.data = 0.5;
		pub_message[i].publish(msg);
	}

    ros::spinOnce();
    r.sleep();
  }

  return 0;
} // end main()
