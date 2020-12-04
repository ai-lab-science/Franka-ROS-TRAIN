cmake_minimum_required(VERSION 3.4)
project(control_methods)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  controller_manager
  franka_hw
  franka_msgs
  geometry_msgs
  message_generation
  pluginlib
  realtime_tools
  roscpp
  sensor_msgs
  tf
  tf2_msgs
)

catkin_package(
  LIBRARIES franka_state_controller
  CATKIN_DEPENDS
    controller_interface
    franka_hw
    franka_msgs
    geometry_msgs
    pluginlib
    realtime_tools
    roscpp
    sensor_msgs
    tf2_msgs
  DEPENDS Franka
)







