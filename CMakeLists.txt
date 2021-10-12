cmake_minimum_required(VERSION 3.0)
project(visp_ros)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_geometry
  image_transport
  kdl_parser
  message_generation
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
    BlobTracker.msg
    ImagePoint.msg
    PoseStampedStatus.msg
    ProjectedPoint.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    geometry_msgs
    sensor_msgs
    std_msgs
)

find_package(VISP REQUIRED)

# Specific packages for franka simulator
find_package(orocos_kdl QUIET)
find_package(iir QUIET)

if(orocos_kdl_FOUND)
  add_definitions("-DVISP_HAVE_OROCOS_KDL")
  message("-- orocos kdl found: We can build franka simulator")
  if(iir_FOUND)
    message("-- iir found")
    add_definitions("-DVISP_HAVE_IIR")
  endif()
else()
  message("-- orocos kdl not found: Unable to build franka simulator")
endif()


###################################
## catkin specific configuration ##
###################################
set(PROJECT_LIBRARIES ${PROJECT_NAME})
if("${VISP_VERSION}" VERSION_GREATER "3.2.0")
  list(APPEND PROJECT_LIBRARIES "${PROJECT_NAME}_bebop2_visual_servo_nodelet")
endif()

catkin_package(
  INCLUDE_DIRS
    include

  LIBRARIES
    ${PROJECT_LIBRARIES}
 
  CATKIN_DEPENDS
    cv_bridge
    geometry_msgs
    image_geometry
    message_runtime
    nodelet
    roscpp
    sensor_msgs
    std_msgs
    tf
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

set(srcs
  include/${PROJECT_NAME}/vpROSGrabber.h
  include/${PROJECT_NAME}/vpROSRobot.h
  include/${PROJECT_NAME}/vpROSRobotPioneer.h
  src/device/framegrabber/vpROSGrabber.cpp
  src/robot/real-robot/pioneer/vpROSRobotPioneer.cpp
  src/robot/vpROSRobot.cpp
  )

if(orocos_kdl_FOUND)
  # With Franka simulator
  set(srcs
    include/${PROJECT_NAME}/vpROSGrabber.h
    include/${PROJECT_NAME}/vpROSRobot.h
    include/${PROJECT_NAME}/vpRobotFrankaSim.h
    include/${PROJECT_NAME}/vpROSRobotFrankaCoppeliasim.h
    include/${PROJECT_NAME}/vpROSRobotPioneer.h
    src/device/framegrabber/vpROSGrabber.cpp
    src/robot/real-robot/pioneer/vpROSRobotPioneer.cpp
    src/robot/sim-robot/franka/vpRobotFrankaSim.cpp
    src/robot/sim-robot/franka/vpROSRobotFrankaCoppeliasim.cpp
    src/robot/sim-robot/franka/model/CoriolisMatrix.cpp
    src/robot/sim-robot/franka/model/franka_model.h
    src/robot/sim-robot/franka/model/FrictionTorque.cpp
    src/robot/sim-robot/franka/model/GravityVector.cpp
    src/robot/sim-robot/franka/model/MassMatrix.cpp
    src/robot/vpROSRobot.cpp
    )
else()
  # Without Franka simulator
  set(srcs
    include/${PROJECT_NAME}/vpROSGrabber.h
    include/${PROJECT_NAME}/vpROSRobot.h
    include/${PROJECT_NAME}/vpROSRobotPioneer.h
    src/device/framegrabber/vpROSGrabber.cpp
    src/robot/real-robot/pioneer/vpROSRobotPioneer.cpp
    src/robot/vpROSRobot.cpp
    )
endif()

## Declare library
add_library(${PROJECT_NAME} ${srcs})

if("${VISP_VERSION}" VERSION_GREATER "3.2.0")
  add_library(${PROJECT_NAME}_bebop2_visual_servo_nodelet
    tutorial/bebop2/visual_servo_nodelet.cpp
  )
endif()

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages)

#################
## Build nodes ##
#################
## Declare a cpp executable
add_executable(${PROJECT_NAME}_biclops_node nodes/biclops.cpp)
add_executable(${PROJECT_NAME}_afma6_node nodes/afma6.cpp)
add_executable(${PROJECT_NAME}_viper650_node nodes/viper650.cpp)
add_executable(${PROJECT_NAME}_viper850_node nodes/viper850.cpp)
add_executable(${PROJECT_NAME}_blob_tracker_node nodes/blob_tracker.cpp)
add_executable(${PROJECT_NAME}_pbvs_blob_node nodes/pbvs_blob.cpp)
if("${VISP_VERSION}" VERSION_GREATER "3.2.0")
  add_executable(${PROJECT_NAME}_bebop2_visual_servo_node tutorial/bebop2/visual_servo_node.cpp)
endif()

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_biclops_node ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_afma6_node ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_viper650_node ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_viper850_node ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_blob_tracker_node ${catkin_LIBRARIES} ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}_pbvs_blob_node ${catkin_LIBRARIES} ${PROJECT_NAME})
if("${VISP_VERSION}" VERSION_GREATER "3.2.0")
  target_link_libraries(${PROJECT_NAME}_bebop2_visual_servo_node ${catkin_LIBRARIES})
  target_link_libraries(${PROJECT_NAME}_bebop2_visual_servo_nodelet ${catkin_LIBRARIES})
endif()

######################################################################
## Build tutorial for real franka robot to compare with coppeliasim ##
######################################################################
add_executable(tutorial-franka-real-ibvs-apriltag tutorial/franka/real-robot/tutorial-franka-real-ibvs-apriltag.cpp)
add_executable(tutorial-franka-real-pbvs-apriltag tutorial/franka/real-robot/tutorial-franka-real-pbvs-apriltag.cpp)
add_executable(tutorial-franka-real-joint-impedance-control tutorial/franka/real-robot/tutorial-franka-real-joint-impedance-control.cpp)
target_link_libraries(tutorial-franka-real-ibvs-apriltag ${catkin_LIBRARIES})
target_link_libraries(tutorial-franka-real-pbvs-apriltag ${catkin_LIBRARIES})
target_link_libraries(tutorial-franka-real-joint-impedance-control ${catkin_LIBRARIES})

##############################################################
## Build tutorial to simulate franka robot with coppeliasim ##
##############################################################
if(orocos_kdl_FOUND)
  add_executable(tutorial-franka-coppeliasim-ibvs-apriltag tutorial/franka/coppeliasim/tutorial-franka-coppeliasim-ibvs-apriltag.cpp)
  add_executable(tutorial-franka-coppeliasim-pbvs-apriltag tutorial/franka/coppeliasim/tutorial-franka-coppeliasim-pbvs-apriltag.cpp)
  add_executable(tutorial-franka-coppeliasim-dual-arm tutorial/franka/coppeliasim/tutorial-franka-coppeliasim-dual-arm.cpp)
  add_executable(tutorial-franka-coppeliasim-joint-impedance-control tutorial/franka/coppeliasim/tutorial-franka-coppeliasim-joint-impedance-control.cpp)
  add_executable(tutorial-franka-coppeliasim-cartesian-impedance-control tutorial/franka/coppeliasim/tutorial-franka-coppeliasim-cartesian-impedance-control.cpp)
  add_executable(test-vel tutorial/franka/coppeliasim/test-vel.cpp)
  set(franka_sim_LIBRARIES ${catkin_LIBRARIES} ${PROJECT_NAME} ${orocos_kdl_LIBRARIES})
  if(iir_FOUND)
    list(APPEND franka_sim_LIBRARIES iir::iir)
  endif()
  target_link_libraries(tutorial-franka-coppeliasim-ibvs-apriltag ${franka_sim_LIBRARIES})
  target_link_libraries(tutorial-franka-coppeliasim-pbvs-apriltag ${franka_sim_LIBRARIES})
  target_link_libraries(tutorial-franka-coppeliasim-dual-arm ${franka_sim_LIBRARIES})
  target_link_libraries(tutorial-franka-coppeliasim-joint-impedance-control ${franka_sim_LIBRARIES})
  target_link_libraries(tutorial-franka-coppeliasim-cartesian-impedance-control ${franka_sim_LIBRARIES})
  target_link_libraries(test-vel ${franka_sim_LIBRARIES})

  install(
    TARGETS
    tutorial-franka-coppeliasim-ibvs-apriltag
    tutorial-franka-coppeliasim-pbvs-apriltag
    tutorial-franka-coppeliasim-joint-impedance-control
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(
  TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_afma6_node
    ${PROJECT_NAME}_biclops_node
    ${PROJECT_NAME}_blob_tracker_node
    ${PROJECT_NAME}_pbvs_blob_node
    ${PROJECT_NAME}_viper650_node
    ${PROJECT_NAME}_viper850_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if("${VISP_VERSION}" VERSION_GREATER "3.2.0")
  install(
    TARGETS
      ${PROJECT_NAME}_bebop2_visual_servo_node
      ${PROJECT_NAME}_bebop2_visual_servo_nodelet
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install(FILES nodelet_plugins.xml
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
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
