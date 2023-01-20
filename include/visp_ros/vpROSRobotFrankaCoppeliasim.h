/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2022 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See https://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
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

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/inertia.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>

#include <visp3/core/vpConfig.h>

#include <visp_ros/vpRobotFrankaSim.h>

/*!
 * This class does the connection using ROS 2 between Franka robot simulator
 * implemented in vpRobotFrankaSim and CoppeliaSim simulator.
 *
 * It subscribes to default topics emitted and updated on Coppeliasim side:
 * - `/coppeliasim/franka/joint_state` topic that contains the robot joint state
 *   as sensor_msgs::JointState message. To change this topic name use
 *   setTopicJointState().
 * - `/coppeliasim/franka/eMc` that contains the camera extrinsic homogeneous
 *   transformation from end-effector to camera frame as a geometry_msgs::Pose
 *   message.To change this topic name use setTopic_eMc().
 *
 * Given the type of command to apply to the simulated robot, it published default
 * topics that are taken into account by CoppeliaSim to update the robot state:
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
 * If you decide to change the topic names, be aware that you should also change
 * their names in Coppeliasim scene lua script.
 */
class VISP_EXPORT vpROSRobotFrankaCoppeliasim : public vpRobotFrankaSim, rclcpp::Node
{
public:
  vpROSRobotFrankaCoppeliasim(const std::string &node_name = "frankasim");
  virtual ~vpROSRobotFrankaCoppeliasim();

  void connect( const std::string &robot_ID = "" );

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
   * Return true when ROS connection with Coppeliasim is established.
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
   * Name of the topic used to public joint state command that has to be applied
   * to the robot.
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
  void callbackJointState( const sensor_msgs::msg::JointState::ConstSharedPtr &joint_state );
  void callback_g0( const geometry_msgs::msg::Vector3::ConstSharedPtr &g0_msg );
  void callback_eMc( const geometry_msgs::msg::Pose::ConstSharedPtr &pose_msg );
  void callback_flMe( const geometry_msgs::msg::Pose::ConstSharedPtr &pose_msg );
  void callback_toolInertia( const geometry_msgs::msg::Inertia::ConstSharedPtr &inertia_msg );
  void callbackSimulationStepDone( const std_msgs::msg::Bool::ConstSharedPtr &msg );
  void callbackSimulationTime( const std_msgs::msg::Float32::ConstSharedPtr &msg );
  void callbackSimulationState( const std_msgs::msg::Int32::ConstSharedPtr &msg );

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
  rclcpp::Publisher< sensor_msgs::msg::JointState >::SharedPtr m_pub_jointStateCmd;
  rclcpp::Publisher< std_msgs::msg::Int32 >::SharedPtr m_pub_robotStateCmd;
  rclcpp::Publisher< std_msgs::msg::Bool >::SharedPtr m_pub_startSimulation;
  rclcpp::Publisher< std_msgs::msg::Bool >::SharedPtr m_pub_pauseSimulation;
  rclcpp::Publisher< std_msgs::msg::Bool >::SharedPtr m_pub_stopSimulation;
  rclcpp::Publisher< std_msgs::msg::Bool >::SharedPtr m_pub_triggerNextStep;
  rclcpp::Publisher< std_msgs::msg::Bool >::SharedPtr m_pub_enableSyncMode;

  // Subscriber
  rclcpp::Subscription< sensor_msgs::msg::JointState >::SharedPtr m_sub_coppeliasim_jointState;
  rclcpp::Subscription< geometry_msgs::msg::Vector3 >::SharedPtr m_sub_coppeliasim_g0;
  rclcpp::Subscription< geometry_msgs::msg::Pose >::SharedPtr m_sub_coppeliasim_eMc;
  rclcpp::Subscription< geometry_msgs::msg::Pose >::SharedPtr m_sub_coppeliasim_flMe;
  rclcpp::Subscription< geometry_msgs::msg::Inertia >::SharedPtr m_sub_coppeliasim_toolInertia;
  rclcpp::Subscription< std_msgs::msg::Bool >::SharedPtr m_sub_coppeliasim_simulationStepDone;
  rclcpp::Subscription< std_msgs::msg::Float32 >::SharedPtr m_sub_coppeliasim_simulationTime;
  rclcpp::Subscription< std_msgs::msg::Int32 >::SharedPtr m_sub_coppeliasim_simulationState;

  // Simulation
  bool m_simulationStepDone;
  float m_simulationTime;
  bool m_syncModeEnabled;
  int m_simulationState;

  bool m_overwrite_toolInertia; // Flag to indicate that the inertia parameters of the tool should no more be updated
                                // from topic
  bool m_overwrite_flMe;        // Flag to indicate that flMe should no more be updated from topic
};

#endif
