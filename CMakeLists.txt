cmake_minimum_required(VERSION 2.8.3)
project(visp_ros)

# By default set RelWithDebInfo configuration
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE String "Choose the type of build, options are: None Debug Release" FORCE)
endif()

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  visp_bridge
  cv_bridge
  image_geometry
)

find_package(VISP REQUIRED)
# Add package definitions
add_definitions(${VISP_DEFINITIONS})

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
    include

  LIBRARIES
    ${PROJECT_NAME}
 
  CATKIN_DEPENDS
    geometry_msgs
    roscpp
    sensor_msgs
    std_msgs
    visp_bridge
    cv_bridge
    image_geometry

  DEPENDS
    VISP
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${VISP_INCLUDE_DIRS}
)


## Declare a cpp library
add_library(visp_ros
  src/device/framegrabber/vpROSGrabber.cpp
  src/robot/vpROSRobot.cpp
  src/robot/real-robot/pioneer/vpROSRobotPioneer.cpp
)

add_dependencies(visp_ros ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
