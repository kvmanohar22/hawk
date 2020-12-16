get_filename_component(bbox_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
include(CMakeFindDependencyMacro)

find_dependency(OpenCV 4.2.0 CONFIG REQUIRED)

if(NOT TARGET bbox::bbox)
    include("${bbox_CMAKE_DIR}/bboxTargets.cmake")
endif()
