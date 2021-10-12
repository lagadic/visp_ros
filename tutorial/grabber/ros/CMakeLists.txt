project(tutorial-ros-grabber)

cmake_minimum_required(VERSION 3.0)

find_package(visp_ros REQUIRED)

include_directories(${visp_ros_INCLUDE_DIRS})

add_executable(tutorial-ros-grabber tutorial-ros-grabber.cpp)
target_link_libraries(tutorial-ros-grabber ${visp_ros_LIBRARIES})

