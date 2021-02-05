cmake_minimum_required(VERSION 2.8.3)
project(simulation_environment)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  gazebo_ros
)

# Depending on system install of Gazebo
find_package(gazebo REQUIRED)

catkin_package(
   CATKIN_DEPENDS
   roscpp
   rospy
)

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${src}
  ${GAZEBO_INCLUDE_DIRS}
)

# Gazebo set flags
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(force_joint SHARED src/force_joint.cpp)
target_link_libraries(force_joint ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})

add_library(set_imu_data SHARED src/set_imu_data.cpp)
target_link_libraries(set_imu_data ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})

add_executable(dynamic_publisher src/dynamic_publisher.cpp)
target_link_libraries(dynamic_publisher ${catkin_LIBRARIES})
