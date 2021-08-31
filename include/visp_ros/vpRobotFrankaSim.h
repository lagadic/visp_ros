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
 * Franka robot simulator.
 *
 * Authors:
 * Alexander Oliva
 * Fabien Spindler
 */

#ifndef vpRobotFrankaSim_h
#define vpRobotFrankaSim_h

#include <atomic>
#include <mutex>
#include <thread>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpPoseVector.h>
#include <visp3/core/vpThetaUVector.h>
#include <visp3/robot/vpRobot.h>
#define VISP_HAVE_OROCOS_KDL
#if defined( VISP_HAVE_OROCOS_KDL )

#ifdef VISP_HAVE_OROCOS_KDL
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/solveri.hpp>
#endif

/*!
 * Franka robot simulator.
 */
class VISP_EXPORT vpRobotFrankaSim
{
public:
  vpRobotFrankaSim();
  virtual ~vpRobotFrankaSim();

  vpRobot::vpRobotStateType getRobotState( void );

  void get_fJe( vpMatrix &fJe );
  void get_fJe( const vpColVector &q, vpMatrix &fJe );
  void get_eJe( vpMatrix &eJe_ );
  void get_eJe( const vpColVector &q, vpMatrix &fJe );
  vpHomogeneousMatrix get_eMc() const;
  vpHomogeneousMatrix get_fMe( const vpColVector &q );
  vpHomogeneousMatrix get_fMe();
  vpHomogeneousMatrix get_flMe() const;
  vpHomogeneousMatrix get_flMcom() const;
  double get_tool_mass() const;

  void getGravity( vpColVector &gravity );
  void getMass( vpMatrix &mass );
  void getCoriolis( vpColVector &coriolis );
  void getCoriolisMatrix(vpMatrix &coriolis);
  void getFriction( vpColVector &friction );

  virtual void getForceTorque( const vpRobot::vpControlFrameType frame, vpColVector &force );
  virtual void getPosition( const vpRobot::vpControlFrameType frame, vpColVector &position );
  virtual void getPosition( const vpRobot::vpControlFrameType frame, vpPoseVector &position );
  virtual void getVelocity( const vpRobot::vpControlFrameType frame, vpColVector &d_position );

  virtual void set_eMc( const vpHomogeneousMatrix &eMc );
  virtual void setForceTorque( const vpRobot::vpControlFrameType frame, const vpColVector &force );
  virtual void setPosition( const vpRobot::vpControlFrameType frame, const vpColVector &position );
  virtual void setVelocity( const vpRobot::vpControlFrameType frame, const vpColVector &vel );

  virtual void add_tool( const vpHomogeneousMatrix &flMe, const double mL, const vpHomogeneousMatrix &flMcom,
                         const vpMatrix &I_L );
  virtual void set_flMe( const vpHomogeneousMatrix &flMe );
  virtual void set_g0( const vpColVector &g0 );


  /*!
   * Enable/disable verbose mode to print additional info.
   * \param verbose : true to enable verbose mode, false otherwise.
   */
  inline void setVerbose( bool verbose ) { m_verbose = verbose; }

protected:
  vpColVector m_q;     // Joint Positions
  vpColVector m_dq;    // Joint Velocities
  vpColVector m_tau_J; // Joint efforts

  double m_mL;                 // payload mass
  vpHomogeneousMatrix m_flMcom;// payload Center of Mass pose in flange frame
  vpMatrix m_Il;               // payload inertia tensor
  vpHomogeneousMatrix m_flMe;  // End-effector pose in flange frame
  bool m_toolMounted;          // flag to indicate the presence of a tool
  bool m_camMounted;           // flag to indicate the presence of a camera

  vpColVector m_g0; // Absolute gravitational acceleration vector in base frame

  std::mutex m_mutex;

  vpColVector solveIK( const vpHomogeneousMatrix &edMw );
  vpColVector getVelDes();

#ifdef VISP_HAVE_OROCOS_KDL
  KDL::JntArray m_q_kdl;
  KDL::JntArray m_dq_des_kdl;
  KDL::Chain m_chain_kdl;
  KDL::JntArray m_q_min_kdl;
  KDL::JntArray m_q_max_kdl;
  KDL::ChainFkSolverPos_recursive *m_fksolver_kdl;
  KDL::ChainJntToJacSolver *m_jacobianSolver_kdl;
  KDL::ChainIkSolverVel_pinv *m_diffIkSolver_kdl;
  KDL::ChainIkSolverPos_NR_JL *m_iksolver_JL_kdl;
#endif

  vpRobot::vpRobotStateType m_stateRobot;

  vpColVector m_q_des; // Desired joint position.

  vpColVector m_dq_des;      // Desired joint velocity.
  vpColVector m_dq_des_filt; // Desired joint velocity filtered.
  vpColVector m_v_cart_des;  // Desired Cartesian velocity in end-effector frame.

  vpColVector m_tau_J_des;      // Desired joint torques.
  vpColVector m_tau_J_des_filt; // Desired joint torques filtered.

  vpHomogeneousMatrix m_eMc; // Transformation of the camera w.r.t. End-Effector frame
  vpVelocityTwistMatrix m_eVc;

  bool m_verbose;
};

#endif
#endif
