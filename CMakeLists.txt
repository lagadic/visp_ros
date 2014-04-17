cmake_minimum_required(VERSION 2.8.3)
project(visp_ros)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  visp_bridge
  cv_bridge
  image_geometry
  rospy
  tf
)

find_package(VISP REQUIRED)
# Add package definitions
#add_definitions(${VISP_DEFINITIONS})

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

###################
## Build library ##
###################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${VISP_INCLUDE_DIRS}
)


## Declare a cpp library
add_library(visp_ros
  include/visp_ros/vpROSGrabber.h
  include/visp_ros/vpROSRobot.h
  include/visp_ros/vpROSRobotPioneer.h
  src/device/framegrabber/vpROSGrabber.cpp
  src/robot/vpROSRobot.cpp
  src/robot/real-robot/pioneer/vpROSRobotPioneer.cpp
)

add_dependencies(visp_ros ${catkin_EXPORTED_TARGETS})

#################
## Build nodes ##
#################
## Declare a cpp executable
add_executable(visp_ros_biclops_node nodes/biclops.cpp)
add_executable(visp_ros_afma6_node nodes/afma6.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(visp_ros_biclops_node ${catkin_LIBRARIES})
target_link_libraries(visp_ros_afma6_node ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(
  TARGETS 
    visp_ros
    visp_ros_biclops_node
    visp_ros_afma6_node
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

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

