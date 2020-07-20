cmake_minimum_required(VERSION 2.8.3)
project(tracking)

add_subdirectory(bbox)
# cv_bridge patch for opencv4, credit: https://github.com/fizyr-forks/vision_opencv.git (opencv4 branch)
add_subdirectory(cv_bridge)

add_subdirectory(tracking_msg)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  image_transport
  rosbag
  message_filters
)

# add launch files to catkin for checks
if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()

find_package(dlib REQUIRED)
find_package(OpenCV 4.0.0 CONFIG REQUIRED)
find_package(Boost REQUIRED)
find_package(benchmark REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tracking
 CATKIN_DEPENDS roscpp sensor_msgs image_transport rosbag message_filters
#  DEPENDS OpenCV Boost
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
add_executable(bbox_test_node src/bbox_test.cpp)
add_executable(perf_node perf/perf.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} dlib::dlib bbox cv_bridge)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
target_link_libraries(image_pub_node ${catkin_LIBRARIES} ${OpenCV_LIBS} cv_bridge)
target_link_libraries(tracking_bag_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} cv_bridge)
target_link_libraries(bbox_test_node bbox ${catkin_LIBRARIES}  ${OpenCV_LIBS} cv_bridge)

# Perf Node #
target_link_libraries(perf_node dlib::dlib benchmark::benchmark opencv_core opencv_dnn opencv_imgcodecs opencv_videoio opencv_highgui)


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
