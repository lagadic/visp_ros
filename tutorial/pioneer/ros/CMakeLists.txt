project(tutorial-ros-pioneer)

cmake_minimum_required(VERSION 2.6)

find_package(visp_ros REQUIRED)

include_directories(${visp_ros_INCLUDE_DIRS})

add_executable(servo-pioneer-ros servo-pioneer-ros.cpp)
target_link_libraries(servo-pioneer-ros ${visp_ros_LIBRARIES})

add_executable(tutorial-ros-pioneer tutorial-ros-pioneer.cpp)
target_link_libraries(tutorial-ros-pioneer ${visp_ros_LIBRARIES})

