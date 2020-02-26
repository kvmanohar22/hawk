cmake_minimum_required(VERSION 2.8.3)
project(offboard)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-std=c++14)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  std_msgs
  tf2
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES offboard
  CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp std_msgs tf2 tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/offboard.cpp
)

add_executable(test_offboard_basic test/test_offboard_basic.cpp)
target_link_libraries(test_offboard_basic ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(test_offboard_trajectory test/test_offboard_trajectory.cpp)
target_link_libraries(test_offboard_trajectory ${catkin_LIBRARIES} ${PROJECT_NAME})
