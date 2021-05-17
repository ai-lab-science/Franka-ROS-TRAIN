cmake_minimum_required(VERSION 3.4)
project(control_methods)

find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning_interface
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

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES franka_state_controller
  CATKIN_DEPENDS
    moveit_ros_planning_interface
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

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${src}
  include
  ${EIGEN3_INCLUDE_DIR}
  ${Franka_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/qr24.cpp
)

add_executable(calibration src/calibration.cpp)
target_link_libraries(calibration ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PROJECT_NAME})











