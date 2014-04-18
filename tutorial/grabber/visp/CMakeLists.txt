project(tutorial-visp-grabber)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED)

include_directories(${VISP_INCLUDE_DIRS})

add_executable(tutorial-grabber-1394 tutorial-grabber-1394.cpp)
target_link_libraries(tutorial-grabber-1394 ${VISP_LIBRARIES})

