### Under current development
Donwload the git repository into a ROS workspace
```
cd ~
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/NRottmann/ROS_PANDA
```
Build the workspace without the 

Start the environment using
```
roslaunch simulation_environment hand.launch
```
Make sure you built and sourced the simulation environment beforehand. You can then type in
```
rostopic list
```
to see all available topics. You should see many /force_joints/... topics. They set the specific joints to a certain angle (in radian). You can test this by typing
```
rostopic pub -1 /force_joints/hand/middle_finger_joint_1 std_msgs/Float32 '0.5'
```
 which sets the first joint of the middle finger to 0.5 radians.
