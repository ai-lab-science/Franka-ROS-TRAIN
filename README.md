# Franka Emika Remote Control

This repository contains all the information required to start remote controlling the Franka Emika 7-dof robot arm. 



## Table of Contents

1. [Prerequisites](#prerequisites) 
2. [Install the ROS package](#installpackage)
3. [Calibration](#calibration)
4. [Remote Control](#remotecontrol)



## Prerequisites <a name="prerequisites"></a>

Prerequisites are 

- [Ubuntu 20.04](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview)
- [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)

Also, you require to install libfranka as well as franka_ros. Check out the offcial [webpage](https://frankaemika.github.io/docs/installation_linux.html) for further information. You will notice that the instructions require you to install a [realtime kernel](https://mirrors.edge.kernel.org/pub/linux/kernel/projects/rt/). Therefore checkout your current kernel version with
```bash
uname -r
```
and choose a realtime kernel closest to your current one. The kernel version and major revision number should be the same. Also, since kernel version 5, you require to set 
```
CONFIG_SYSTEM_TRUSTED_KEYS=""
```
within the hidden .config file within your realtime kernel folder (ctrl+H to show the hidden files). In order to be able to control the robot you have to enable the Franka Control Interface (FCI) on your robot. Therefore, go to the [Franka World](https://www.franka.de/franka-world) website and log in. Then check the FCI addon for your robot and download it onto the robot (https://yourRobotsIP/admin/world). For tracking your movements for remote control you need a tracking system, e.g. [Optitrack](https://optitrack.com/). In order to send the data into the ROS system, the OptiTrack system has to be within the same network and the [vrpn_client_ros package](http://wiki.ros.org/vrpn_client_ros) is required. Also make sure that vrpn data streaming is enabled within [Motive](https://v22.wiki.optitrack.com/index.php?title=Data_Streaming). Finally, we need to install moveit by following the instructions stated [here](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html).


## Install the ROS Package <a name="installpackage"></a>

For installing the ROS package for remote control of the Franka Emika robot, generate a catkin workspace and go into the source folder

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
```

Clone the repository to the source folder and build it.

````bash
git clone https://github.com/ai-lab-science/Franka-ROS-TRAIN
cd ~/catkin_ws
catkin build
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

and connect the [tracking object](../../tree/master/files/cad) to the robot's flange. Make sure that you defined the tracking object as `RigidBody1` object in the Optitrack system. Now you can start the calibration by sourcing the environment and starting the `calibration.launch` file.

````bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch franka_ros_train_controllers calibration.launch
````

 After the calibration finished, there should appear a calibration file `base2tracking.txt` in the config folder. This file contains now the transformation information between the robot's base and the Optitrack system.



## Remote Control <a name="remotecontrol"></a>

For remote control of the Franka Emika robot, just disconnect the [tracking object](../../tree/master/files/cad) from the robot's flange and take it into your hand. Now reconfigure the `remoteControl.launch` such that the correct IP addresses are set (similar as for the `calibration.launch` file). Start the remote control.

````bash
roslaunch franka_ros_train_controllers remoteControl.launch
````

