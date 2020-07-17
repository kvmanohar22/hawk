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

add_executable(test_offboard_basic_00 test/test_offboard_basic_00.cpp)
target_link_libraries(test_offboard_basic_00 ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(test_offboard_basic_01 test/test_offboard_basic_01.cpp)
target_link_libraries(test_offboard_basic_01 ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(test_offboard_basic_02 test/test_offboard_basic_02.cpp)
target_link_libraries(test_offboard_basic_02 ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(test_offboard_basic_04 test/test_offboard_basic_04.cpp)
target_link_libraries(test_offboard_basic_04 ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(test_offboard_vio_00 test/test_offboard_vio_00.cpp)
target_link_libraries(test_offboard_vio_00 ${catkin_LIBRARIES} ${PROJECT_NAME})
