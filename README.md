# Franka Emika Remote Control

This repository contains all the information required to start remote controlling the Franka Emika 7-dof robot arm. 



## Table of Contents

1. [Prerequisites](#prerequisites) 
2. [Install the ROS package](#installpackage)
3. [Calibration](#calibration)
4. [Remote Control](#remotecontrol)



## Prerequisites <a name="prerequisites"></a>

Prerequisites are 

- [Ubuntu 18.04](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview)
- [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)

Also, you require to install libfranka as well as franka_ros. Check out the [wiki](../../wiki/ROS-Franka-Installation-instructions) for further information. For tracking your movements for remote control you need a tracking system, e.g. [Optitrack](https://optitrack.com/). Check out the [wiki](wiki/Setup-of-the-Optitrack-System) for further information.



## Install the ROS Package <a name="installpackage"></a>

For installing the ROS package for remote control of the Franka Emika robot, generate a catkin workspace and go into the source folder

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
```

Clone the repository to the source folder and build it.

````bash
git clone https://github.com/NRottmann/ROS_PANDA
cd ~/catkin_ws
catkin_make
````



## Calibration <a name="calibration"></a>

We need to find a transformation from the Optitrack coordinate system to the robot coordinate system. Therefore, we use hand-eye calibration as proposed [here](https://onlinelibrary.wiley.com/doi/full/10.1002/rcs.1427). Before running the calibration, make sure that the Optitrack system is set properly and is connected within the same network as the panda robot. Configure in the `calibration.launch` file the IP addresses

````xml
...

<include file="$(find vrpn_client_ros)/launch/sample.launch">
	<arg name="server" value="OPTITRACK_IP_ADDRESS"/>
</include>

...

<include file="$(find franka_control)/launch/franka_control.launch">
	<arg name="robot_ip" value="ROBOT_IP_ADDRESS"/>
    <arg name="load_gripper" value="$(arg load_gripper)"/>
</include>

...
````

and connect the [tracking object](../tree/master/files/cad) to the robot's flange. Now you can start the calibration by sourcing the environment and starting the `calibration.launch` file.

````bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch franka_ros_train_controllers calibration.launch
````

 After the calibration finished, there should appear a calibration file `base2tracking.txt` in the config folder. This file contains now the transformation information between the robot's base and the Optitrack system.



## Remote Control <a name="remotecontrol"></a>

