project(tutorial-visp-grabber)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED)

include_directories(${VISP_INCLUDE_DIRS})

add_executable(tutorial-visp-grabber-1394 tutorial-visp-grabber-1394.cpp)
target_link_libraries(tutorial-visp-grabber-1394 ${VISP_LIBRARIES})

add_executable(tutorial-visp-grabber-v4l2 tutorial-visp-grabber-v4l2.cpp)
target_link_libraries(tutorial-visp-grabber-v4l2 ${VISP_LIBRARIES})

