/*
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2021 by INRIA. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 *
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * Franka robot ROS simulator coupled with Coppeliasim.
 *
 * Authors:
 * Alexander Oliva
 * Fabien Spindler
 */

#ifndef vpROSRobotFrankaCoppeliasim_h
#define vpROSRobotFrankaCoppeliasim_h

#include <ros/ros.h>

#include <geometry_msgs/Inertia.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

#include <visp3/core/vpConfig.h>

#include <visp_ros/vpRobotFrankaSim.h>

/*!
 * This class does the connexion using ROS 1 between Franka robot simulator implemented in
 * vpRobotFrankaSim and Coppeliasim simulator.
 *
 * It subscribes to default topics emited and updated on Coppeliasim side:
 * - `/coppeliasim/franka/joint_state` topic that contains the robot joint state as
 *   sensor_msgs::JointState message. To change this topic name use setTopicJointState().
 * - `/coppeliasim/franka/eMc` that contains the camera extrinsic homogeneous transformation from
 *   end-effector to camera frame as a geometry_msgs::Pose message.
 *   To change this topic name use setTopic_eMc().
 *
 * Given the type of command to apply to the simulated robot, it published default
 * topics that are taken into account by Coppeliasim to update the robot state:
 * - `/fakeFCI/joint_state` topic that contains the command to apply to the robot as a
 *   sensor_msgs::JointState message. To change this topic name use setTopicJointStateCmd().
 * - `/fakeFCI/robot_state` topic that contains the kind of robot control as a
 *   std_msgs::Int32 message. The values of this message should match the one in
 *   vpRobot::vpRobotStateType. To change this topic name use setTopicRobotStateCmd().
 *
 * Note that changing topic names should be achieved before calling connect().
 * \code
 * vpROSRobotFrankaCoppeliasim robot;
 * robot.setTopicJointState("/coppeliasim/franka/joint_state");
 * robot.setTopic_eMc("/coppeliasim/franka/eMc");
 * robot.setTopicJointStateCmd("/fakeFCI/joint_state");
 * robot.setTopicRobotStateCmd("/fakeFCI/robot_state");
 * robot.connect();
 * \endcode
 *
 * If you decide to change the topic names, be aware that you should also change their names
 * in Coppeliasim scene lua script.
 */
class VISP_EXPORT vpROSRobotFrankaCoppeliasim : public vpRobotFrankaSim
{
public:
  vpROSRobotFrankaCoppeliasim();
  virtual ~vpROSRobotFrankaCoppeliasim();

  void connect();

  void coppeliasimPauseSimulation( double sleep_ms = 1000. );
  void coppeliasimStartSimulation( double sleep_ms = 1000. );
  void coppeliasimStopSimulation( double sleep_ms = 1000. );
  void coppeliasimTriggerNextStep();

  double getCoppeliasimSimulationTime();
  bool getCoppeliasimSimulationStepDone();
  int getCoppeliasimSimulationState();

  /*!
   * Get Coppeliasim synchronous mode.
   * \return true when synchronous mode is enable, false otherwise.
   *
   * \sa setCoppeliasimSyncMode()
   */
  inline bool getCoppeliasimSyncMode() { return m_syncModeEnabled; }

  /*!
   * Return true when ROS connexion with Coppeliasim is established.
   */
  inline bool isConnected() const { return m_connected; }

  void setCoppeliasimSyncMode( bool enable, double sleep_ms = 1000. );

  void setPosition( const vpRobot::vpControlFrameType frame, const vpColVector &position );

  vpRobot::vpRobotStateType setRobotState( vpRobot::vpRobotStateType newState );

  /*!
   * Set topic name that contains the robot state including position,
   * velocity and effort for all the joints.
   *
   * \param topic_jointState : Topic name.
   */
  inline void setTopicJointState( const std::string &topic_jointState ) { m_topic_jointState = topic_jointState; }

  /*!
   * Name of the topic used to public joint state command that has to be applied to the robot.
   *
   * \param topic_jointStateCmd : Topic name.
   */
  inline void setTopicJointStateCmd( const std::string &topic_jointStateCmd )
  {
    m_topic_jointStateCmd = topic_jointStateCmd;
  }

  /*!
   * Set topic name that contains the robot controller state
   * (stopped, position, velocity, force/torque).
   *
   * \param topic_robotState : Topic name.
   */
  inline void setTopicRobotStateCmd( const std::string &topic_robotState ) { m_topic_robotStateCmd = topic_robotState; }

  /*!
   * Set topic name that contains `g0` corresponding to the absolute acceleration
   * vector at the robot's base.
   *
   * \param topic_g0 : Topic name.
   */
  inline void setTopic_g0( const std::string &topic_g0 ) { m_topic_g0 = topic_g0; }

  /*!
   * Set topic name that contains `eMc` corresponding to the end-effector
   * to camera transformation.
   *
   * \param topic_eMc : Topic name.
   */
  inline void setTopic_eMc( const std::string &topic_eMc ) { m_topic_eMc = topic_eMc; }

  /*!
   * Set topic name that contains `flMe` corresponding to the flange
   * to end-effector transformation.
   *
   * \param topic_flMe : Topic name.
   */
  inline void setTopic_flMe( const std::string &topic_flMe ) { m_topic_flMe = topic_flMe; }

  /*!
   * Set topic name that contains `flMcom` corresponding to the flange
   * to Center-of-Mass transformation.
   *
   * \param topic_flMcom : Topic name.
   */
  inline void setTopic_flMcom( const std::string &topic_flMcom ) { m_topic_flMcom = topic_flMcom; }

  /*!
   * Set topic name that contains the `Inertia` parameters of the tool.
   *
   * \param topic_toolInertia : Topic name.
   */
  inline void setTopic_toolInertia( const std::string &topic_toolInertia ) { m_topic_toolInertia = topic_toolInertia; }

  void setCoppeliasimSimulationStepDone( bool simulationStepDone );

  void wait( double timestamp_second, double duration_second );

protected:
  void callbackJointState( const sensor_msgs::JointState &joint_state );
  void callback_g0( const geometry_msgs::Vector3 &g0_msg );
  void callback_eMc( const geometry_msgs::Pose &pose_msg );
  void callback_flMe( const geometry_msgs::Pose &pose_msg );
  void callback_flMcom( const geometry_msgs::Pose &pose_msg );
  void callback_toolInertia( const geometry_msgs::Inertia &inertia_msg );
  void callbackSimulationStepDone( const std_msgs::Bool &msg );
  void callbackSimulationTime( const std_msgs::Float32 &msg );
  void callbackSimulationState( const std_msgs::Int32 &msg );

  void readingLoop();

  void positionControlLoop();
  void torqueControlLoop();
  void velocityControlLoop();

  std::thread m_controlThread;
  std::thread m_acquisitionThread;

  // Subscribed topics updated by Coppeliasim
  std::string m_topic_jointState;
  std::string m_topic_g0;
  std::string m_topic_eMc;
  std::string m_topic_flMe;
  std::string m_topic_flMcom;
  std::string m_topic_toolInertia;
  // Published topics to update Coppeliasim
  std::string m_topic_jointStateCmd;
  std::string m_topic_robotStateCmd;

  bool m_connected;

  // Position controller
  std::atomic_bool m_posControlThreadIsRunning;
  std::atomic_bool m_posControlThreadStopAsked;
  std::atomic_bool m_posControlLock;
  std::atomic_bool m_posControlNewCmd;

  // Velocity controller
  std::atomic_bool m_velControlThreadIsRunning;
  std::atomic_bool m_velControlThreadStopAsked;

  // Force/torque controller
  std::atomic_bool m_ftControlThreadIsRunning;
  std::atomic_bool m_ftControlThreadStopAsked;

  // Publisher
  ros::Publisher m_pub_jointStateCmd;
  ros::Publisher m_pub_robotStateCmd;
  ros::Publisher m_pub_startSimulation;
  ros::Publisher m_pub_pauseSimulation;
  ros::Publisher m_pub_stopSimulation;
  ros::Publisher m_pub_triggerNextStep;
  ros::Publisher m_pub_enableSyncMode;

  // Subscriber
  ros::Subscriber m_sub_coppeliasim_jointState;
  ros::Subscriber m_sub_coppeliasim_g0;
  ros::Subscriber m_sub_coppeliasim_eMc;
  ros::Subscriber m_sub_coppeliasim_flMe;
  ros::Subscriber m_sub_coppeliasim_flMcom;
  ros::Subscriber m_sub_coppeliasim_toolInertia;
  ros::Subscriber m_sub_coppeliasim_simulationStepDone;
  ros::Subscriber m_sub_coppeliasim_simulationTime;
  ros::Subscriber m_sub_coppeliasim_simulationState;

  // Simulation
  bool m_simulationStepDone;
  float m_simulationTime;
  bool m_syncModeEnabled;
  int m_simulationState;

  bool m_overwrite_toolInertia; // Flag to indicate that the inertia parameters of the tool should no more be updated
                                // from topic
  bool m_overwrite_flMe;        // Flag to indicate that flMe should no more be updated from topic
  bool m_overwrite_flMcom;      // Flag to indicate that flMcom should no more be updated from topic
};

#endif
