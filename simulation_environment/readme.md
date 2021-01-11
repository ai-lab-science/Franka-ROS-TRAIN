### Under current development

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
