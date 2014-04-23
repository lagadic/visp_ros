project(tutorial-pioneer-visp)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED)

include_directories(${VISP_INCLUDE_DIRS})

add_executable(tutorial-visp-pioneer-visual-servo tutorial-visp-pioneer-visual-servo.cpp)
target_link_libraries(tutorial-visp-pioneer-visual-servo ${VISP_LIBRARIES})

