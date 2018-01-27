cmake_minimum_required(VERSION 2.8.3)
project(tracking)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-std=c++14)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  sensor_msgs
  rospy
  cv_bridge
  roscpp
  std_msgs
  tf2
  tf2_ros
)

catkin_package(
  LIBRARIES offboard
  CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp std_msgs tf2 tf2_ros rospy sensor_msgs cv_bridge
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

