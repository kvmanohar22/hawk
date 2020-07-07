cmake_minimum_required(VERSION 2.8.3)
project(bbox)

add_compile_options(-std=c++11)

# SET(OpenCV_DIR /usr/local/lib/cmake/opencv4)

find_package(OpenCV 4.2.0 CONFIG REQUIRED)

## Declare a C++ library
add_library(bbox SHARED
  src/bbox.cpp
)

# target_include_directories(bbox PUBLIC include)

# message(STATUS "    config: ${OpenCV_DIR}")
# message(STATUS "    version: ${OpenCV_VERSION}")
# message(STATUS "    libraries: ${OpenCV_LIBS}")
# message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

target_include_directories(bbox 
  PUBLIC 
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(bbox PUBLIC
     opencv_core opencv_dnn opencv_imgcodecs opencv_videoio opencv_highgui)

#############
## Testing ##
#############

# Export
install(TARGETS bbox
    EXPORT bbox-targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(EXPORT bbox-targets
  FILE
    bboxTargets.cmake
  NAMESPACE
    bbox::
  DESTINATION
    lib/cmake/bbox
)

install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/include/bbox/bbox.h
  DESTINATION "include/bbox")

install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/bboxConfig.cmake
  DESTINATION "lib/cmake/bbox" )

set(CMAKE_MODULE_PATH
    ${CMAKE_MODULE_PATH}
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake
)