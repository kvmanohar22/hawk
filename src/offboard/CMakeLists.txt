cmake_minimum_required(VERSION 2.8.3)
project(offboard)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-std=c++14)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES offboard
  CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/offboard.cpp
)

