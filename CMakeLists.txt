cmake_minimum_required(VERSION 2.8.3)
project(visp_ros)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_geometry
  image_transport
  nodelet
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  visp_bridge
)

## Declare ROS messages and services
## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY
    msg
  FILES
    PoseStampedStatus.msg
    BlobTracker.msg
    ImagePoint.msg
    ProjectedPoint.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    std_msgs
    sensor_msgs
    geometry_msgs
)

find_package(VISP REQUIRED)
# Add package definitions
#add_definitions(${VISP_DEFINITIONS})

###################################
## catkin specific configuration ##
###################################
set(PROJECT_LIBRARIES ${PROJECT_NAME})
if(${VISP_VERSION} VERSION_GREATER "3.2.0")
  list(APPEND PROJECT_LIBRARIES "visp_ros_bebop2_visual_servo_nodelet")
endif()

catkin_package(
  INCLUDE_DIRS
    include

  LIBRARIES ${PROJECT_LIBRARIES}
 
  CATKIN_DEPENDS
    cv_bridge
    geometry_msgs
    image_geometry
    message_runtime
    nodelet
    roscpp
    sensor_msgs
    std_msgs
    visp_bridge

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
  src/robot/real-robot/pioneer/vpROSRobotPioneer.cpp
  src/robot/vpROSRobot.cpp
)

if(${VISP_VERSION} VERSION_GREATER "3.2.0")
  add_library(visp_ros_bebop2_visual_servo_nodelet
    tutorial/bebop2/visual_servo_nodelet.cpp
  )
endif()

target_link_libraries(visp_ros ${catkin_LIBRARIES})
add_dependencies(visp_ros ${catkin_EXPORTED_TARGETS})

#################
## Build nodes ##
#################
## Declare a cpp executable
add_executable(visp_ros_biclops_node nodes/biclops.cpp)
add_executable(visp_ros_afma6_node nodes/afma6.cpp)
add_executable(visp_ros_viper650_node nodes/viper650.cpp)
add_executable(visp_ros_viper850_node nodes/viper850.cpp)
add_executable(visp_ros_blob_tracker_node nodes/blob_tracker.cpp)
add_executable(visp_ros_pbvs_blob_node nodes/pbvs_blob.cpp)
if(${VISP_VERSION} VERSION_GREATER "3.2.0")
  add_executable(visp_ros_bebop2_visual_servo_node tutorial/bebop2/visual_servo_node.cpp)
endif()

## Specify libraries to link a library or executable target against
target_link_libraries(visp_ros_biclops_node ${catkin_LIBRARIES})
target_link_libraries(visp_ros_afma6_node ${catkin_LIBRARIES})
target_link_libraries(visp_ros_viper650_node ${catkin_LIBRARIES})
target_link_libraries(visp_ros_viper850_node ${catkin_LIBRARIES})
target_link_libraries(visp_ros_blob_tracker_node ${catkin_LIBRARIES})
target_link_libraries(visp_ros_pbvs_blob_node ${catkin_LIBRARIES})
if(${VISP_VERSION} VERSION_GREATER "3.2.0")
  target_link_libraries(visp_ros_bebop2_visual_servo_node ${catkin_LIBRARIES})
  target_link_libraries(visp_ros_bebop2_visual_servo_nodelet ${catkin_LIBRARIES})
endif()

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(
  TARGETS
    visp_ros
    visp_ros_afma6_node
    visp_ros_biclops_node
    visp_ros_viper650_node
    visp_ros_viper850_node
    visp_ros_blob_tracker_node
    visp_ros_pbvs_blob_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(${VISP_VERSION} VERSION_GREATER "3.2.0")
install(
  TARGETS
    visp_ros_bebop2_visual_servo_node
    visp_ros_bebop2_visual_servo_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
endif()

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

if(${VISP_VERSION} VERSION_GREATER "3.2.0")
install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endif()

