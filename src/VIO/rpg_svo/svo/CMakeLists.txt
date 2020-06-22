################################################################################
# user build settings

SET(TRACE TRUE)
SET(USE_EDGELETS FALSE) # Set TRUE if you want to use edgelet features
SET(USE_ROS TRUE) # Set FALSE if you want to build this package without Catkin
SET(DEBUG_OUTPUT FALSE) # Only relevant if build without ROS
SET(SVO_BUILD_TESTS TRUE) # Enable this to build tests
################################################################################

SET(PROJECT_NAME svo)
PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED (VERSION 2.8.3)
SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
SET(CMAKE_VERBOSE_MAKEFILE OFF)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")
SET(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set definitions
IF(TRACE)
  ADD_DEFINITIONS(-DSVO_TRACE)
ENDIF()
IF(USE_ROS)
  ADD_DEFINITIONS(-DSVO_USE_ROS)
ENDIF()
IF(DEBUG_OUTPUT)
  ADD_DEFINITIONS(-DSVO_DEBUG_OUTPUT)
ENDIF()
IF(USE_EDGELETS)
  ADD_DEFINITIONS(-DSVO_USE_EDGELETS)
ENDIF()

# Set build flags, set ARM_ARCHITECTURE environment variable on Odroid
# Set build flags. Set IS_ARM on odroid board as environment variable
SET(CMAKE_CXX_FLAGS "-Wall -D_LINUX -D_REENTRANT -march=native -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")
IF(DEFINED ENV{ARM_ARCHITECTURE})
  # SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon -march=armv7-a")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse -msse2 -msse3 -mssse3")
ENDIF()
IF(CMAKE_COMPILER_IS_GNUCC)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -g")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g")
ENDIF()
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

# Add plain cmake packages 
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(Sophus REQUIRED)
FIND_PACKAGE(fast REQUIRED)
FIND_PACKAGE(Boost REQUIRED COMPONENTS thread system filesystem serialization regex timer)
FIND_PACKAGE(GTSAM REQUIRED)

IF(USE_ROS)
  FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp roslib cmake_modules vikit_common vikit_ros)
  catkin_package(
      DEPENDS Eigen OpenCV Sophus Boost fast
      CATKIN_DEPENDS roscpp roslib vikit_common vikit_ros
      INCLUDE_DIRS include
      LIBRARIES svo
  )
ELSE()
  FIND_PACKAGE(vikit_common REQUIRED)
  SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
ENDIF()

# Include dirs
INCLUDE_DIRECTORIES(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${fast_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIRS}
)

# Set link libraries
LIST(APPEND LINK_LIBS 
  ${OpenCV_LIBS}
  ${Sophus_LIBRARIES}
  ${Boost_LIBRARIES}
  ${fast_LIBRARIES}
  gtsam
)

IF(USE_ROS)
  INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})
  LIST(APPEND LINK_LIBS ${catkin_LIBRARIES})
ELSE()
  INCLUDE_DIRECTORIES(${vikit_common_INCLUDE_DIRS})
  LIST(APPEND LINK_LIBS ${vikit_common_LIBRARIES})
ENDIF()

# Set sourcefiles
LIST(APPEND SOURCEFILES
  src/frame_handler_mono.cpp
  src/frame_handler_base.cpp
  src/frame.cpp
  src/point.cpp
  src/map.cpp
  src/pose_optimizer.cpp
  src/initialization.cpp
  src/matcher.cpp
  src/reprojector.cpp
  src/feature_alignment.cpp
  src/feature_detection.cpp
  src/depth_filter.cpp
  src/config.cpp
  src/sparse_img_align.cpp
  src/imu.cpp
  src/visual_inertial_estimator.cpp
  src/inertial_initialization.cpp
  src/bundle_adjustment.cpp)

# Create svo library
ADD_LIBRARY(svo SHARED ${SOURCEFILES})
TARGET_LINK_LIBRARIES(svo ${LINK_LIBS})

################################################################################
# TESTS
IF(SVO_BUILD_TESTS)
  ADD_EXECUTABLE(test_feature_align test/test_feature_alignment.cpp)
  TARGET_LINK_LIBRARIES(test_feature_align svo)

  ADD_EXECUTABLE(test_matcher test/test_matcher.cpp)
  TARGET_LINK_LIBRARIES(test_matcher svo)

  ADD_EXECUTABLE(test_feature_detection test/test_feature_detection.cpp)
  TARGET_LINK_LIBRARIES(test_feature_detection svo)

  ADD_EXECUTABLE(test_depth_filter test/test_depth_filter.cpp)
  TARGET_LINK_LIBRARIES(test_depth_filter svo)

  ADD_EXECUTABLE(test_sparse_img_align test/test_sparse_img_align.cpp)
  TARGET_LINK_LIBRARIES(test_sparse_img_align svo)

  ADD_EXECUTABLE(test_pose_optimizer test/test_pose_optimizer.cpp)
  TARGET_LINK_LIBRARIES(test_pose_optimizer svo)

  ADD_EXECUTABLE(test_imu_container test/test_imu_container.cpp)
  TARGET_LINK_LIBRARIES(test_imu_container svo)
ENDIF(SVO_BUILD_TESTS)
