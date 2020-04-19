cmake_minimum_required(VERSION 3.14)

project(tracking)

include(FetchContent)

FetchContent_Declare(dlib
    GIT_REPOSITORY https://github.com/davisking/dlib.git
    GIT_TAG        v19.18
)
FetchContent_MakeAvailable(dlib)

add_executable(tracker_main tracker_main.cpp)
target_link_libraries(tracker_main dlib::dlib)

