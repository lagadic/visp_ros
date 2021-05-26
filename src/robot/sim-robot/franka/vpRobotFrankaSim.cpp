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

#include <visp_ros/vpRobotFrankaSim.h>

#include "model/franka_model.h"

#if defined(VISP_HAVE_OROCOS_KDL)

/*!
 * Default constructor.
 */
vpRobotFrankaSim::vpRobotFrankaSim() :
  m_q(7,0), m_dq(7,0), m_tau_J(7,0),
  m_mutex(),
  m_q_kdl(7), m_dq_des_kdl(7), m_chain_kdl(), m_q_min_kdl(7), m_q_max_kdl(7),
  m_stateRobot(vpRobot::STATE_STOP),
  m_q_des(7,0),
  m_dq_des(7,0), m_dq_des_filt(7,0), m_v_cart_des(6,0),
  m_tau_J_des(7,0), m_tau_J_des_filt(7,0),
  m_eMc(), m_eVc(), m_overwrite_eMc(false),
  m_verbose(false)
{
#ifdef VISP_HAVE_OROCOS_KDL
  m_chain_kdl.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame::DH_Craig1989( 0.0   ,  0.0  ,0.333,0.0)));
  m_chain_kdl.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH_Craig1989( 0.0   ,-M_PI_2,0.0  ,0.0)));
  m_chain_kdl.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH_Craig1989( 0.0   , M_PI_2,0.316,0.0)));
  m_chain_kdl.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH_Craig1989( 0.0825, M_PI_2,0.0  ,0.0)));
  m_chain_kdl.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH_Craig1989(-0.0825,-M_PI_2,0.384,0.0)));
  m_chain_kdl.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH_Craig1989( 0.0   , M_PI_2,0.0  ,0.0)));
  m_chain_kdl.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH_Craig1989( 0.088 , M_PI_2,0.0  ,0.0)));
  m_chain_kdl.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH_Craig1989( 0.0   ,  0.0  ,0.107,0.0)));

  m_q_min_kdl(0) = -2.8973; m_q_min_kdl(1) = -1.7628; m_q_min_kdl(2) = -2.8973;
  m_q_min_kdl(3) = -3.0718; m_q_min_kdl(4) = -2.8973; m_q_min_kdl(5) = -0.0175;
  m_q_min_kdl(6) = -2.8973;
  m_q_max_kdl(0) =  2.8973; m_q_max_kdl(1) =  1.7628; m_q_max_kdl(2) =  2.8973;
  m_q_max_kdl(3) = -0.0698; m_q_max_kdl(4) =  2.8973; m_q_max_kdl(5) =  3.7525;
  m_q_max_kdl(6) =  2.8973;

  m_fksolver_kdl = new KDL::ChainFkSolverPos_recursive(m_chain_kdl);
  m_jacobianSolver_kdl = new KDL::ChainJntToJacSolver(m_chain_kdl);
  m_diffIkSolver_kdl = new KDL::ChainIkSolverVel_pinv (m_chain_kdl);
  m_iksolver_JL_kdl = new KDL::ChainIkSolverPos_NR_JL(m_chain_kdl, m_q_min_kdl, m_q_max_kdl, *(m_fksolver_kdl), *(m_diffIkSolver_kdl), 100, 1e-6); //Maximum 100 iterations, stop at accuracy 1e-6
#endif
}

/*!
 * Destructor that frees allocated memory.
 */
vpRobotFrankaSim::~vpRobotFrankaSim()
{
#ifdef VISP_HAVE_OROCOS_KDL
  delete m_fksolver_kdl;
  delete m_jacobianSolver_kdl;
  delete m_diffIkSolver_kdl;
  delete m_iksolver_JL_kdl;
#endif
}

/*!
 * Return desired joint velocities applied to the robot.
 */
vpColVector vpRobotFrankaSim::getVelDes()
{
  return m_dq_des_filt;
}

/*!
 * Set camera extrinsics as the homogeneous transformation between the end-effector
 *  and the camera frame.
 * \param[in] eMc : Homogeneous transformation between the end-effector
 * and the camera frame.
 */
void vpRobotFrankaSim::set_eMc(const vpHomogeneousMatrix &eMc)
{
  std::lock_guard<std::mutex> lock(m_mutex);
  m_overwrite_eMc = true;
  m_eMc = eMc;
  m_eVc.buildFrom(m_eMc);
}

/*!
 * Get current robot control state.
 * \return Control state.
 */
vpRobot::vpRobotStateType vpRobotFrankaSim::getRobotState(void)
{
  return m_stateRobot;
}


/*!
 * Get robot Jacobian expressed in the robot base frame considering the current joint position.
 * \param[out] fJe : Corresponding 6-by-7 Jacobian matrix expressed in the robot base frame.
 */
void vpRobotFrankaSim::get_fJe(vpMatrix &fJe)
{
  fJe.reshape(6,7);

#ifdef VISP_HAVE_OROCOS_KDL
  KDL::Jacobian Jac(7);

  m_mutex.lock();
  m_jacobianSolver_kdl->JntToJac(m_q_kdl, Jac);
  m_mutex.unlock();

  for(unsigned int i = 0; i < 6; i++) {
    for(unsigned int j = 0; j < 7; j++) {
      fJe[i][j] = Jac.data(i,j);
    }
  }
#endif
}

/*!
 * Get robot Jacobian expressed in the robot base frame.
 * \param[in] q : Joint position as a 7-dim vector with values in [rad].
 * \param[out] fJe : Corresponding 6-by-7 Jacobian matrix expressed in the robot base frame.
 */
void vpRobotFrankaSim::get_fJe(const vpColVector &q, vpMatrix &fJe)
{
  if (q.size() != 7) {
    throw(vpException(vpException::dimensionError, "Joint position vector is not a 7-dim vector (%d)", q.size()));
  }

  fJe.reshape(6, 7);

#ifdef VISP_HAVE_OROCOS_KDL
  KDL::JntArray jnts = KDL::JntArray(7);
  KDL::Jacobian Jac(7);

  for(unsigned int i=0;i<7;i++) {
    jnts(i)=q[i];
  }
  m_jacobianSolver_kdl->JntToJac(jnts, Jac);

  for(unsigned int i=0;i<6;i++) {
    for(unsigned int j=0;j<7;j++) {
      fJe[i][j] = Jac.data(i,j);
    }
  }
#endif
}

/*!
 * Get robot Jacobian in the end-effector frame.
 * \param[out] eJe : 6-by-7 Jacobian matrix corresponding to the robot current joint
 * position expressed in the end-effector frame.
 */
void vpRobotFrankaSim::get_eJe(vpMatrix &eJe)
{
  vpMatrix fJe(6,7);
  get_fJe(fJe);
  vpHomogeneousMatrix fMe(get_fMe());
  vpMatrix eVf(6,6);
  eVf.insert(fMe.getRotationMatrix().inverse(), 0, 0);
  eVf.insert(fMe.getRotationMatrix().inverse(), 3, 3);

  eJe = eVf * fJe;
}

/*!
 * Get robot Jacobian in the end-effector frame.
 * \param[in] q : Joint position as a 7-dim vector with values in [rad].
 * \param[out] eJe : Corresponding 6-by-7 Jacobian matrix expressed in the end-effector frame.
 */
void vpRobotFrankaSim::get_eJe(const vpColVector &q, vpMatrix &eJe)
{
  if (q.size() != 7) {
    throw(vpException(vpException::dimensionError, "Joint position vector is not a 7-dim vector (%d)", q.size()));
  }

  vpMatrix fJe(6,7);
  get_fJe(q, fJe);
  vpHomogeneousMatrix fMe(get_fMe(q));
  vpMatrix eVf(6, 6);
  eVf.insert(fMe.getRotationMatrix().inverse(), 0, 0);
  eVf.insert(fMe.getRotationMatrix().inverse(), 3, 3);

  eJe = eVf * fJe;
}

/*!
 * Get end-effector to camera constant homogeneous transformation.
 * \return End-effector to camera homogeneous transformation.
 */
vpHomogeneousMatrix vpRobotFrankaSim::get_eMc() const
{
  return m_eMc;
}

/*!
 * Get robot position.
 * \param[in] frame : Control frame to consider.
 * \param[out] position : Robot position.
 * - When \e frame is set to vpRobot::JOINT_STATE,
 * `position` is a 7-dim vector that contains the joint positions in [rad].
 * - When `frame` is set to vpRobot::END_EFFECTOR_FRAME, `position` is a 6-dim vector that
 * contains the pose of the end-effector in the robot base frame. This pose vector
 * contains the 3 translations values [tx, ty, tz] followed by the 3 rotations using the
 * axis-angle representation [tux, tuy, tyz] with values in [rad].
 * - When `frame` is set to vpRobot::CAMERA_FRAME, `position` is a 6-dim vector that
 * contains the pose of the camera in the robot base frame. This pose vector
 * contains the 3 translations values [tx, ty, tz] followed by the 3 rotations using the
 * axis-angle representation [tux, tuy, tyz] with values in [rad].
 */
void vpRobotFrankaSim::getPosition(const vpRobot::vpControlFrameType frame, vpColVector &position)
{
  switch(frame) {
  case vpRobot::JOINT_STATE: { // Same as ARTICULAR_FRAME
    position.resize(7);
    std::lock_guard<std::mutex> lock(m_mutex);
    position = m_q;
    break;
  }
  case vpRobot::END_EFFECTOR_FRAME: {
    position.resize(6);

    vpPoseVector fPe(get_fMe());
    for (unsigned int i = 0; i < 6; i++) {
      position[i] = fPe[i];
    }

    break;
  }
  case vpRobot::CAMERA_FRAME: { // same as TOOL_FRAME
    position.resize(6);
    vpPoseVector fPc(get_fMe() * m_eMc);
    for (unsigned int i = 0; i < 6; i++) {
      position[i] = fPc[i];
    }
    break;
  }
  default: {
    throw(vpException(vpException::fatalError, "Cannot get Franka cartesian position: wrong method"));
  }
  }
}

/*!
 * Get robot cartesian position.
 * \param[in] frame : Control frame to consider. Admissible values are vpRobot::END_EFFECTOR_FRAME
 * and vpRobot::CAMERA_FRAME.
 * \param[out] position : Robot cartesian position.
 * - When `frame` is set to vpRobot::END_EFFECTOR_FRAME, `position` is a 6-dim pose vector that
 * contains the pose of the end-effector in the robot base frame. This pose vector
 * contains the 3 translations values [tx, ty, tz] followed by the 3 rotations using the
 * axis-angle representation [tux, tuy, tyz] with values in [rad].
 * - When `frame` is set to vpRobot::CAMERA_FRAME, `position` is a 6-dim pose vector that
 * contains the pose of the camera in the robot base frame. This pose vector
 * contains the 3 translations values [tx, ty, tz] followed by the 3 rotations using the
 * axis-angle representation [tux, tuy, tyz] with values in [rad].
 */
void vpRobotFrankaSim::getPosition(const vpRobot::vpControlFrameType frame, vpPoseVector &position)
{
  vpColVector pose(6,0);
  if(frame == vpRobot::END_EFFECTOR_FRAME || frame == vpRobot::CAMERA_FRAME){
    getPosition(frame, pose);
    for (unsigned int i = 0; i < 6; i++) {
      position[i] = pose[i];
    }
  }
  else{
    throw(vpException(vpException::fatalError, "Cannot get a cartesian position for the specified frame"));
  }
}

/*!
 * Set robot position.
 * \param[in] frame : Control frame to consider.
 * \param[in] position : Position to reach.
 * - When `frame` is set to vpRobot::JOINT_STATE, `position` is a 7-dim
 * vector that contains joint positions in [rad].
 * - When `frame` is set to vpRobot::END_EFFECTOR_FRAME, `position` is a 6-dim
 * vector containing the pose of the end-effector in the robot base frame. This pose
 * contains the 3 translations values [tx, ty, tz] followed by the 3 rotations using
 * the axis-angle representation [tux, tuy, tyz] with values in [rad].
 * - When `frame` is set to vpRobot::CAMERA_FRAME, `position` is a 6-dim
 * vector containing the pose of the camera in the robot base frame. This pose
 * contains the 3 translations values [tx, ty, tz] followed by the 3 rotations using
 * the axis-angle representation [tux, tuy, tyz] with values in [rad].
 */
void vpRobotFrankaSim::setPosition(const vpRobot::vpControlFrameType frame, const vpColVector &position)
{
  switch(frame) {
  case vpRobot::JOINT_STATE: {
    if (position.size() != 7) {
      throw(vpException(vpException::dimensionError, "Joint position vector is not a 7-dim vector (%d)", position.size()));
    }
    std::lock_guard<std::mutex> lock(m_mutex);
    m_q_des = position;

    break;
  }
  case vpRobot::END_EFFECTOR_FRAME: {
    if (position.size() != 6) {
      throw(vpException(vpException::dimensionError, "Cartesian position vector is not a 6-dim vector (%d)", position.size()));
    }
    vpHomogeneousMatrix wMe;
    wMe.buildFrom(position[0], position[1], position[2], position[3], position[4], position[5]);
    vpColVector q_des = solveIK(wMe);
    std::lock_guard<std::mutex> lock(m_mutex);
    m_q_des = q_des;

    break;
  }
  case vpRobot::CAMERA_FRAME: {
    if (position.size() != 6) {
      throw(vpException(vpException::dimensionError, "Cartesian position vector is not a 6-dim vector (%d)", position.size()));
    }
    vpHomogeneousMatrix wMc;
    wMc.buildFrom(position[0], position[1], position[2], position[3], position[4], position[5]);

    vpHomogeneousMatrix wMe = wMc * m_eMc.inverse();
    vpColVector q_des = solveIK(wMe);
    std::lock_guard<std::mutex> lock(m_mutex);
    m_q_des = q_des;

    break;
  }
  default: {
    throw(vpException(vpException::fatalError, "Franka positioning frame is not implemented"));
  }
  }
}

/*!
 * Solve inverse kinematics.
 * \param wMe : Robot base frame to end-effector homogeneous transformation.
 * \return Corresponding joint position as a 7-dim vector with values in [rad].
 */
vpColVector vpRobotFrankaSim::solveIK(const vpHomogeneousMatrix &wMe)
{
  vpColVector q_solved(7,0);
#ifdef VISP_HAVE_OROCOS_KDL
  KDL::JntArray q_out(7);
  KDL::Rotation wRe(wMe[0][0], wMe[0][1], wMe[0][2],
      wMe[1][0], wMe[1][1], wMe[0][2],
      wMe[2][0], wMe[2][1], wMe[2][2]);
  KDL::Vector wte(wMe[0][3], wMe[1][3], wMe[2][3]);
  KDL::Frame wMe_kdl(wRe, wte);
  m_mutex.lock();
  int ret = m_iksolver_JL_kdl->CartToJnt(m_q_kdl, wMe_kdl, q_out);
  m_mutex.unlock();
  switch(ret){
  case KDL::SolverI::E_NOERROR:{
    std::cout << "solveIK: E_NOERROR" << std::endl;
    break;
  }
  case KDL::SolverI::E_MAX_ITERATIONS_EXCEEDED:{
    std::cout << "solveIK: E_MAX_ITERATIONS_EXCEEDED" << std::endl;
    break;
  }
  case KDL::SolverI::E_NOT_IMPLEMENTED:{
    std::cout << "solveIK: E_NOT_IMPLEMENTED" << std::endl;
    break;
  }
  default: {
    throw(vpException(vpException::fatalError, "Error: unable to solve ik\n"));
  }
  }

  for (unsigned int i = 0; i < 7; i++) {
    q_solved[i] = q_out(i);
  }
#endif

  return q_solved;
}

/*!
 * Get robot velocity.
 * \param[in] frame : Control frame to consider.
 * \param[out] velocity : Robot velocity.
 * - When `frame` is set to vpRobot::JOINT_STATE, `velocity` is a 7-dim vector
 * corresponding to the joint velocities in [rad/s].
 * - When `frame` is set to vpRobot::END_EFFECTOR_FRAME, `velocity` is a 6-dim vector
 * corresponding to the cartesian velocities of the end-effector expressed in the end-effector.
 * This vector contains the 3 translational velocities in [m/s] followed by the 3 rotational
 * velocities in [rad/s].
 * - When `frame` is set to vpRobot::REFERENCE_FRAME, `velocity` is a 6-dim vector
 * corresponding to the cartesian velocities of the end-effector expressed in the robot base frame.
 * This vector contains the 3 translational velocities in [m/s] followed by the 3 rotational
 * velocities in [rad/s].
 */
void vpRobotFrankaSim::getVelocity(const vpRobot::vpControlFrameType frame, vpColVector &velocity)
{
  switch(frame) {
  case vpRobot::JOINT_STATE: {
    velocity.resize(7);
    std::lock_guard<std::mutex> lock(m_mutex);
    velocity = m_dq;
    break;
  }
  case vpRobot::END_EFFECTOR_FRAME: {
    velocity.resize(6);
    vpMatrix eJe(6,7);
    this->get_eJe(eJe);
    std::lock_guard<std::mutex> lock(m_mutex);
    velocity = eJe * m_dq;
    break;
  }
  case vpRobot::REFERENCE_FRAME: {
    velocity.resize(6);
    vpMatrix fJe(6,7);
    this->get_fJe(fJe);
    std::lock_guard<std::mutex> lock(m_mutex);
    velocity = fJe * m_dq;
    break;
  }
  default: {
    throw(vpException(vpException::fatalError, "Cannot get Franka velocity in the specified frame"));
  }
  }
}

/*!
 * Set robot velocity.
 * \param[in] frame : Control frame to consider.
 * \param[in] velocity : Velocity to apply to the robot.
 * - When `frame` is set to vpRobot::JOINT_STATE, `velocity` is a 7-dim vector
 * corresponding to the joint velocities in [rad/s].
 * - When `frame` is set to vpRobot::REFERENCE_FRAME, `velocity` is a 6-dim vector
 * corresponding to the cartesian velocities of the end-effector expressed in the robot base frame.
 * This vector contains the 3 translational velocities in [m/s] followed by the 3 rotational
 * velocities in [rad/s].
 * - When `frame` is set to vpRobot::END_EFFECTOR_FRAME, `velocity` is a 6-dim vector
 * corresponding to the cartesian velocities of the end-effector expressed in the  end-effector frame.
 * This vector contains the 3 translational velocities in [m/s] followed by the 3 rotational
 * velocities in [rad/s].
 * - When `frame` is set to vpRobot::CAMERA_FRAME, `velocity` is a 6-dim vector
 * corresponding to the cartesian velocities of the camera expressed in the camera frame.
 * This vector contains the 3 translational velocities in [m/s] followed by the 3 rotational
 * velocities in [rad/s].
 */
void vpRobotFrankaSim::setVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &velocity)
{
  if (vpRobot::STATE_VELOCITY_CONTROL != getRobotState()) {
    std::cout <<  "Cannot send a velocity to the robot. "
                  "Use setRobotState(vpRobot::STATE_VELOCITY_CONTROL) first. \n";
  }

  switch (frame) {
  case vpRobot::JOINT_STATE: {
    if (velocity.size() != 7) {
      std::cout <<  "Joint velocity vector "<< velocity.size() <<" is not of size 7 \n";
    }
    std::lock_guard<std::mutex> lock(m_mutex);
    m_dq_des = velocity;
    break;
  }

  case vpRobot::REFERENCE_FRAME: {
    if (velocity.size() != 6) {
      std::cout <<  "Cartesian velocity vector " << velocity.size() << " is not of size 6 \n";
    }
    vpColVector vel_max(6);

    for (unsigned int i = 0; i < 3; i++) {
      vel_max[i] = 1.7;
      vel_max[3 + i] = 2.5;
    }
    // velocities are expressed in Base frame
    m_v_cart_des = vpRobot::saturateVelocities(velocity, vel_max, true);

#ifdef VISP_HAVE_OROCOS_KDL
    KDL::Twist v_cart;
    for(unsigned int i = 0; i < 3; i++) {
      v_cart.vel.data[i] = m_v_cart_des[i];
      v_cart.rot.data[i] = m_v_cart_des[i + 3];
    }
    std::lock_guard<std::mutex> lock(m_mutex);
    m_diffIkSolver_kdl->CartToJnt(m_q_kdl, v_cart, m_dq_des_kdl);
    for(unsigned int i = 0; i < 7; i++) {
      m_dq_des[i] = m_dq_des_kdl.data(i);
    }
#endif
    break;
  }

  case vpRobot::END_EFFECTOR_FRAME: {
    if (velocity.size() != 6) {
      std::cout <<  "Cartesian velocity vector " << velocity.size() << " is not of size 6 \n";
    }
    // Apply Cartesian velocity limits according to the specifications
    vpColVector vel_max(6);
    for (unsigned int i = 0; i < 3; i++) {
      vel_max[i] = 1.7;
      vel_max[3 + i] = 2.5;
    }
    // Refer End-Effector velocities in Base frame
    vpHomogeneousMatrix fMe = this->get_fMe();
    vpVelocityTwistMatrix fVe(fMe, false);
    m_v_cart_des = fVe * vpRobot::saturateVelocities(velocity, vel_max, true);

#ifdef VISP_HAVE_OROCOS_KDL
    KDL::Twist v_cart;
    for (unsigned int i = 0; i < 3; i++) {
      v_cart.vel.data[i] = m_v_cart_des[i];
      v_cart.rot.data[i] = m_v_cart_des[i + 3];
    }

    std::lock_guard<std::mutex> lock(m_mutex);
    m_diffIkSolver_kdl->CartToJnt(m_q_kdl, v_cart, m_dq_des_kdl);
    for(unsigned int i = 0; i < 7; i++) {
      m_dq_des[i] = m_dq_des_kdl.data(i);
    }
#endif
    break;
  }
  case vpRobot::CAMERA_FRAME: {
    if (velocity.size() != 6) {
      std::cout <<  "Cartesian velocity vector " << velocity.size() << " is not of size 6 \n";
    }
    // Apply Cartesian velocity limits according to the specifications
    vpColVector vel_max(6);
    for (unsigned int i = 0; i < 3; i++) {
      vel_max[i] = 1.7;
      vel_max[3 + i] = 2.5;
    }
    // Refer End-Effector velocities in Base frame
    vpHomogeneousMatrix fMe = this->get_fMe();
    vpVelocityTwistMatrix fWe(fMe, false);
    std::lock_guard<std::mutex> lock(m_mutex);
    m_v_cart_des = vpRobot::saturateVelocities(fWe * m_eVc * velocity, vel_max, true);

#ifdef VISP_HAVE_OROCOS_KDL
    KDL::Twist v_cart;
    for(unsigned int i = 0; i < 3; i++) {
      v_cart.vel.data[i] = m_v_cart_des[i];
      v_cart.rot.data[i] = m_v_cart_des[i + 3];
    }

    m_diffIkSolver_kdl->CartToJnt(m_q_kdl, v_cart, m_dq_des_kdl);
    for(unsigned int i = 0; i < 7; i++) {
      m_dq_des[i] = m_dq_des_kdl.data(i);
    }
#endif
    break;
  }
  case vpRobot::MIXT_FRAME:
    throw(vpException(vpException::functionNotImplementedError, "MIXT_FRAME is not implemented"));
  }
}

/*!
 * Get robot force/torque.
 * \param[in] frame : Control frame to consider. Only vpRobot::JOINT_STATE is implemented yet.
 * \param[out] force : 7-dim vector corresponding to the joint torques in [Nm].
 */
void vpRobotFrankaSim::getForceTorque(const vpRobot::vpControlFrameType frame, vpColVector &force)
{
  switch(frame) {
  case vpRobot::JOINT_STATE: {
    force.resize(7);
    std::lock_guard<std::mutex> lock(m_mutex);
    force = m_tau_J;
    break;
  }
    //  case vpRobot::END_EFFECTOR_FRAME: {//TODO
    //    force.resize(6);
    //    vpMatrix eJe;
    //    this->get_eJe(eJe);
    //    std::lock_guard<std::mutex> lock(m_mutex);
    //    force = eJe * m_dq;
    //    break;
    //  }
    //  case vpRobot::REFERENCE_FRAME: {//TODO
    //    force.resize(6);
    //    vpMatrix fJe;
    //    this->get_fJe(fJe);
    //    std::lock_guard<std::mutex> lock(m_mutex);
    //    force = fJe * m_dq;
    //    break;
    //  }
  default: {
    throw(vpException(vpException::fatalError, "Cannot get Franka position for the specified frame "));
  }
  }
}

/*!
 * Set robot force/torque.
 * \param[in] frame : Control frame to consider.
 * \param[in] force : Force/torque applied to the robot.
 * - When `frame` is set to vpRobot::JOINT_STATE, `force` is a 7-dim vector that contains torques in [Nm].
 * - When `frame` is set to vpRobot::REFERENCE_FRAME, `force` is a 6-dim vector that contains cartesian
 * end-effector forces/torques expressed in the reference frame. This vector contains 3 forces [Fx, Fy, Fz]
 * in [N] followed by 3 torques [Tx, Ty, Tz] in [Nm].
 * - When `frame` is set to vpRobot::END_EFFECTOR_FRAME, `force` is a 6-dim vector that contains cartesian
 * end-effector forces/torques expressed in the end-effector frame. This vector contains 3 forces [Fx, Fy, Fz]
 * in [N] followed by 3 torques [Tx, Ty, Tz] in [Nm].
 */
void vpRobotFrankaSim::setForceTorque(const vpRobot::vpControlFrameType frame, const vpColVector &force)
{
  if (vpRobot::STATE_FORCE_TORQUE_CONTROL != getRobotState()) {
    std::cout <<  "Cannot send a torque command to the robot. "
                  "Use setRobotState(vpRobot::STATE_FORCE_TORQUE_CONTROL) first. \n";
  }

  switch (frame) {
  // Saturation in joint space
  case vpRobot::JOINT_STATE: {
    if (force.size() != 7) {
      std::cout <<  "Joint velocity vector "<< force.size() <<" is not of size 7 \n";
    }
    std::lock_guard<std::mutex> lock(m_mutex);
    m_tau_J_des = force;

    break;
  }

  case vpRobot::REFERENCE_FRAME: {
    if (force.size() != 6) {
      std::cout <<  "Cartesian velocity vector " << force.size() << " is not of size 6 \n";
    }
    vpMatrix fJe(6,7);
    this->get_fJe(fJe);
    std::lock_guard<std::mutex> lock(m_mutex);
    m_tau_J_des = fJe.t()*force;
    break;
  }

  case vpRobot::END_EFFECTOR_FRAME: {
    if (force.size() != 6) {
      std::cout <<  "Cartesian velocity vector " << force.size() << " is not of size 6 \n";
    }
    vpMatrix eJe(6,7);
    this->get_eJe(eJe);
    std::lock_guard<std::mutex> lock(m_mutex);
    m_tau_J_des = eJe.t()*force;
    break;
  }
  case vpRobot::CAMERA_FRAME: {
    throw(vpException(vpException::functionNotImplementedError, "force/torque control in camera frame is not implemented"));
  }
  case vpRobot::MIXT_FRAME: {
    throw(vpException(vpException::functionNotImplementedError, "force/torque control in mixt frame is not implemented"));
  }
  }
}

/*!
 * Get robot direct kinematics corresponding to the homogeneous transformation between the
 * robot base frame and the end-effector for the current joint position.
 * \return Homogeneous transformation between the robot base frame and the end-effector.
 */
vpHomogeneousMatrix vpRobotFrankaSim::get_fMe()
{
  vpHomogeneousMatrix fMe;

#ifdef VISP_HAVE_OROCOS_KDL
  KDL::Frame cartpos;
  // Calculate forward kinematics
  int kinematics_status;
  vpRotationMatrix R;
  vpTranslationVector t;
  std::lock_guard<std::mutex> lock(m_mutex);
  kinematics_status = m_fksolver_kdl->JntToCart(m_q_kdl, cartpos);
  if (kinematics_status >= 0){
    for (unsigned int i = 0; i < 3; i++) {
      for (unsigned int j = 0; j < 3; j++) {
        R[i][j] = cartpos.M.data[3 * i + j];
      }
      t[i] = cartpos.p.data[i];
    }
    fMe.buildFrom(t, R);
  }
#endif

  return fMe;
}

/*!
 * Get robot direct kinematics corresponding to the homogeneous transformation between the
 * robot base frame and the end-effector for a given joint position.
 * \param[in] q : Joint position to consider as a 7-dim vector with values in [rad].
 * \return Homogeneous transformation between the robot base frame and the end-effector.
 */
vpHomogeneousMatrix vpRobotFrankaSim::get_fMe(const vpColVector &q)
{
  if (q.size() != 7) {
    throw(vpException(vpException::dimensionError, "Joint position vector is not a 7-dim vector (%d)", q.size()));
  }
  vpRotationMatrix R;
  vpTranslationVector t;

#ifdef VISP_HAVE_OROCOS_KDL
  KDL::Frame cartpos;
  KDL::JntArray qq(7);
  for (unsigned int i = 0; i < 7; i++) {
    qq(i) = q[i];
  }
  // Calculate forward kinematics
  int kinematics_status;
  kinematics_status = m_fksolver_kdl->JntToCart(qq, cartpos);
  if(kinematics_status>=0){
    for(unsigned int i = 0; i < 3; i++) {
      for(unsigned int j = 0; j < 3; j++) {
        R[i][j] = cartpos.M.data[3 * i + j];
      }
      t[i] = cartpos.p.data[i];
    }
  }
#endif

  vpHomogeneousMatrix fMe(t, R);
  return fMe;
}

/*!
 * Get the 7x7 mass matrix. Unit: \f$[kg \times m^2]\f$.
 * \param[out] mass : 7x7 mass matrix, row-major.
 */
void vpRobotFrankaSim::getMass(vpMatrix &mass){
  std::lock_guard<std::mutex> lock(m_mutex);
  mass = franka_model::massMatrix(m_q);
}

/*!
 * Get the gravity vector calculated from the current robot state. Unit: \f$[Nm]\f$.
 * \param[out] gravity : Gravity 7-dim vector.
 */
void vpRobotFrankaSim::getGravity(vpColVector &gravity){
  std::lock_guard<std::mutex> lock(m_mutex);
  gravity = franka_model::gravityVector(m_q);
}

/*!
 * Get the Coriolis force vector (state-space equation) calculated from the current
 * robot state: \f$ c= C \times dq\f$, in \f$[Nm]\f$.
 * \param[out] coriolis : Coriolis 7-dim force vector.
 */
void vpRobotFrankaSim::getCoriolis(vpColVector &coriolis){
  std::lock_guard<std::mutex> lock(m_mutex);
  vpMatrix C(7,7);
  C = franka_model::coriolisMatrix(m_q, m_dq);
  coriolis = C * m_dq;
}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: lib*.a(vpRobotFrankaSim.cpp.o) has
// no symbols
void dummy_vpRobotFrankaSim(){};
#endif
