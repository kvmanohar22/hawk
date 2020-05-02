cmake_minimum_required(VERSION 2.8.3)
project(tracking)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  image_transport
  cv_bridge
  rosbag
  message_filters
)

# add launch files to catkin for checks
if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()

find_package(dlib REQUIRED)
find_package(OpenCV 3.2 EXACT REQUIRED)
FIND_PACKAGE(Boost REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES tracking
 CATKIN_DEPENDS roscpp sensor_msgs image_transport cv_bridge rosbag message_filters
 DEPENDS OpenCV Boost
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/tracker.cpp
  src/utils.cpp
 )
target_include_directories(${PROJECT_NAME} PUBLIC include)


add_executable(${PROJECT_NAME}_node src/tracking_node.cpp)
add_executable(image_pub_node src/image_pub_node.cpp)
add_executable(tracking_bag_node src/tracking_bag.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS} dlib::dlib)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
target_link_libraries(image_pub_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(tracking_bag_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_node dlib::dlib)

add_subdirectory(bbox)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_tracking.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
# Mark variables as used so cmake doesn't complain about them
