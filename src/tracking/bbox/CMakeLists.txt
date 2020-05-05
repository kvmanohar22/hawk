cmake_minimum_required(VERSION 2.8.3)
project(bbox)

add_compile_options(-std=c++11)

# SET(OpenCV_DIR /usr/local/lib/cmake/opencv4)

find_package(OpenCV 4.2.0 CONFIG REQUIRED)

## Declare a C++ library
add_library(bbox SHARED
  src/bbox.cpp
)
target_include_directories(bbox PUBLIC include)

# message(STATUS "    config: ${OpenCV_DIR}")
# message(STATUS "    version: ${OpenCV_VERSION}")
# message(STATUS "    libraries: ${OpenCV_LIBS}")
# message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

target_link_libraries(bbox PUBLIC
      opencv_core opencv_dnn opencv_imgcodecs opencv_videoio opencv_highgui)

#############
## Testing ##
#############
