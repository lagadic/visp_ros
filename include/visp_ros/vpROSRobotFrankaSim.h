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
#include <thread>
#include <mutex>
#include <atomic>

#include <visp3/core/vpConfig.h>
#include <visp3/robot/vpRobot.h>
#include <visp3/core/vpPoseVector.h>
#include <visp3/core/vpThetaUVector.h>

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/solveri.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>

class vpROSRobotFrankaSim {
public:
  vpROSRobotFrankaSim();
  virtual ~vpROSRobotFrankaSim();

  void connect();

  vpRobot::vpRobotStateType getRobotState(void);

  void get_fJe(vpMatrix &fJe);
  void get_fJe(const vpColVector &q, vpMatrix &fJe);
  void get_eJe(vpMatrix &eJe_);
  void get_eJe(const vpColVector &q, vpMatrix &fJe);
  vpHomogeneousMatrix get_eMc() const;
  vpHomogeneousMatrix get_fMe(const vpColVector &q);
  vpHomogeneousMatrix get_fMe();

  void getCoriolis(vpColVector &coriolis);

  void getForceTorque(const vpRobot::vpControlFrameType frame, vpColVector &force);
  void getGravity(vpColVector &gravity);
  void getMass(vpMatrix &mass);
  void getPosition(const vpRobot::vpControlFrameType frame, vpColVector &position);
  void getPosition(const vpRobot::vpControlFrameType frame, vpPoseVector &position);
  void getVelocity(const vpRobot::vpControlFrameType frame, vpColVector &d_position);

  void set_eMc(const vpHomogeneousMatrix &eMc); // TODO FABIEN: to remove or when used, disable eMc from topic update

  void setForceTorque(const vpRobot::vpControlFrameType frame, const vpColVector &force);

  inline void seteMcTopic(const std::string &eMc_topic)
  {
    m_eMc_topic = eMc_topic;
  }

  inline void setJointStateTopic(const std::string &jointState_topic)
  {
    m_jointState_topic = jointState_topic;
  }

  // this function should control the robot in order to bring it to the desired configuration "position"
  // be aware that by now it directly assigns the given position as current position!
  void setPosition(const vpRobot::vpControlFrameType frame, const vpColVector &position);

  vpRobot::vpRobotStateType setRobotState(vpRobot::vpRobotStateType newState);

  void setVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &vel);

  /*!
   * Enable/disable verbose mode to print additional info.
   * \param verbose : true to enable verbose mode, false otherwise.
   */
  inline void setVerbose(bool verbose)
  {
    m_verbose = verbose;
  }

  inline bool is_connected() const
  {
    return m_connected;
  }

protected:
  vpColVector m_q;      // Joint Positions
  vpColVector m_dq;     // Joint Velocities
  vpColVector m_tau_J;  // Joint efforts

  bool m_connected;
  std::thread m_acquisitionThread;
  std::mutex m_mutex;

  vpColVector solveIK(const vpHomogeneousMatrix &edMw);
  vpColVector getVelDes();

  void readingLoop();
  void jointState_callback(const sensor_msgs::JointState& J_state);
  void eMc_callback(const geometry_msgs::Pose& pose_msg);

  KDL::JntArray m_q_kdl;
  KDL::JntArray m_dq_des_kdl;
  KDL::Chain m_chain_kdl;
  KDL::JntArray m_q_min_kdl;
  KDL::JntArray m_q_max_kdl;
  KDL::ChainFkSolverPos_recursive* m_fksolver_kdl;
  KDL::ChainJntToJacSolver* m_jacobianSolver_kdl;
  KDL::ChainIkSolverVel_pinv* m_diffIkSolver_kdl;
  KDL::ChainIkSolverPos_NR_JL* m_iksolver_JL_kdl;

  vpRobot::vpRobotStateType m_stateRobot;
  std::thread m_controlThread;

  // Position controller
  std::atomic_bool m_posControlThreadIsRunning;
  std::atomic_bool m_posControlThreadStopAsked;
  std::atomic_bool m_posControlLock;
  std::atomic_bool m_posControlNewCmd;
  vpColVector m_q_des;             // Desired joint position.
  void positionControlLoop();

  // Velocity controller
  std::atomic_bool m_velControlThreadIsRunning;
  std::atomic_bool m_velControlThreadStopAsked;
  vpColVector m_dq_des;            // Desired joint velocity.
  vpColVector m_dq_des_filt;       // Desired joint velocity filtered.
  vpColVector m_v_cart_des;        // Desired Cartesian velocity in end-effector frame.
  void velocityControlLoop();

  // Force/torque controller
  std::atomic_bool m_ftControlThreadIsRunning;
  std::atomic_bool m_ftControlThreadStopAsked;
  vpColVector m_tau_J_des;         // Desired joint torques.
  vpColVector m_tau_J_des_filt;    // Desired joint torques filtered.
  void torqueControlLoop();

  vpHomogeneousMatrix m_eMc;
  vpVelocityTwistMatrix m_eVc;
  bool m_overwrite_eMc; // Flag to indicate that eMc should no more be updated from topic

  std::string m_jointState_topic;
  std::string m_eMc_topic;
  bool m_verbose;
};
