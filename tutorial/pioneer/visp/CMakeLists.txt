project(tutorial-visp-grabber)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED)

include_directories(${VISP_INCLUDE_DIRS})

add_executable(servo-pioneer-visp servo-pioneer-visp.cpp)
target_link_libraries(servo-pioneer-visp ${VISP_LIBRARIES})

