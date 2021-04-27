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

#include <ros/ros.h>
#include <std_msgs/Int32.h>

#include <visp_bridge/3dpose.h>
#include <visp_ros/vpROSRobotFrankaSim.h>

#ifdef HAVE_IIR
#  include <Iir.h>
#endif

#include "model/franka_model.h"

vpROSRobotFrankaSim::vpROSRobotFrankaSim(): m_q(7,0), m_dq(7,0), m_tau_J(7,0),
  m_connected(false), m_acquisitionThread(), m_mutex(),
  m_q_kdl(7), m_dq_des_kdl(7), m_chain_kdl(), m_q_min_kdl(7), m_q_max_kdl(7),
  m_stateRobot(vpRobot::STATE_STOP), m_controlThread(),
  m_posControlThreadIsRunning(false), m_posControlThreadStopAsked(false),
  m_posControlLock(false), m_posControlNewCmd(false),
  m_q_des(7,0),
  m_velControlThreadIsRunning(false), m_velControlThreadStopAsked(false),
  m_dq_des(7,0), m_dq_des_filt(7,0), m_v_cart_des(6,0),
  m_ftControlThreadIsRunning(false), m_ftControlThreadStopAsked(false),
  m_tau_J_des(7,0), m_tau_J_des_filt(7,0),
  m_eMc(), m_eVc(), m_overwrite_eMc(false),
  m_jointState_topic(), m_eMc_topic(),
  m_verbose(false)
{
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
}

vpROSRobotFrankaSim::~vpROSRobotFrankaSim()
{
  m_mutex.lock();
  m_connected = false;
  m_posControlThreadStopAsked = true;
  m_velControlThreadStopAsked = true;
  m_ftControlThreadStopAsked = true;
  m_mutex.unlock();
  if (m_acquisitionThread.joinable()) {
    m_acquisitionThread.join();
  }
  if (m_controlThread.joinable()) {
    m_controlThread.join();
    m_posControlThreadStopAsked = false;
    m_posControlThreadIsRunning = false;
    m_velControlThreadStopAsked = false;
    m_velControlThreadIsRunning = false;
    m_ftControlThreadStopAsked = false;
    m_ftControlThreadIsRunning = false;
  }

  delete m_fksolver_kdl;
  delete m_jacobianSolver_kdl;
  delete m_diffIkSolver_kdl;
  delete m_iksolver_JL_kdl;
}

vpColVector vpROSRobotFrankaSim::getVelDes()
{
  return m_dq_des_filt;
}

void vpROSRobotFrankaSim::connect()
{
  std::lock_guard<std::mutex> lock(m_mutex);
  m_connected = true;
  m_acquisitionThread = std::thread([this] { readingLoop(); });
  std::cout << "ROS is initialized ? " << (ros::isInitialized() ? "yes" : "no") << std::endl;
}

void vpROSRobotFrankaSim::set_eMc(const vpHomogeneousMatrix &eMc)
{
  std::lock_guard<std::mutex> lock(m_mutex);
  m_overwrite_eMc = true;
  m_eMc = eMc;
  m_eVc.buildFrom(m_eMc);
}

vpRobot::vpRobotStateType vpROSRobotFrankaSim::getRobotState(void)
{
  return m_stateRobot;
}

/*!
 * Change robot state.
 * \param newState : New robot state.
 * \return The new robot state.
 */
vpRobot::vpRobotStateType vpROSRobotFrankaSim::setRobotState(vpRobot::vpRobotStateType newState)
{
  switch (newState) {
  case vpRobot::STATE_STOP: {
    // Start primitive STOP only if the current state is velocity or force/torque
    if (vpRobot::STATE_POSITION_CONTROL == getRobotState()) {
      std::cout << "Change the control mode from position control to stop.\n";
      m_posControlThreadStopAsked = true;
    }
    else if (vpRobot::STATE_VELOCITY_CONTROL == getRobotState()) {
      std::cout << "Change the control mode from velocity control to stop.\n";
      m_velControlThreadStopAsked = true;
    }
    else if (vpRobot::STATE_FORCE_TORQUE_CONTROL == getRobotState()) {
      std::cout << "Change the control mode from force/torque control to stop.\n";
      m_ftControlThreadStopAsked = true;
    }

    if(m_controlThread.joinable()) {
      m_controlThread.join();
      m_posControlThreadStopAsked = false;
      m_posControlThreadIsRunning = false;
      m_velControlThreadStopAsked = false;
      m_velControlThreadIsRunning = false;
      m_ftControlThreadStopAsked = false;
      m_ftControlThreadIsRunning = false;
    }
    this->m_stateRobot = newState;
    break;
  }
  case vpRobot::STATE_POSITION_CONTROL: {
    if (vpRobot::STATE_STOP == getRobotState()) {
      std::cout << "Change the control mode from stop to position control.\n";
    }
    else if (vpRobot::STATE_VELOCITY_CONTROL == getRobotState()) {
      std::cout << "Change the control mode from velocity to position control.\n";
      // Stop the velocity control loop
      m_velControlThreadStopAsked = true;
    }
    else if (vpRobot::STATE_FORCE_TORQUE_CONTROL == getRobotState()) {
      std::cout << "Change the control mode from force/torque to position control.\n";
      // Stop the force control loop
      m_ftControlThreadStopAsked = true;
    }
    if(m_controlThread.joinable()) {
      m_controlThread.join();
      m_velControlThreadStopAsked = false;
      m_velControlThreadIsRunning = false;
      m_ftControlThreadStopAsked = false;
      m_ftControlThreadIsRunning = false;
    }
    //    m_controlThread = std::thread([this] {this->positionControlLoop();});
    this->m_stateRobot = newState;
    break;
  }
  case vpRobot::STATE_VELOCITY_CONTROL: {
    if (vpRobot::STATE_STOP == getRobotState()) {
      std::cout << "Change the control mode from stop to velocity control.\n";
    }
    else if (vpRobot::STATE_POSITION_CONTROL == getRobotState()) {
      std::cout << "Change the control mode from position to velocity control.\n";
      m_posControlThreadStopAsked = true;
    }
    else if (vpRobot::STATE_FORCE_TORQUE_CONTROL == getRobotState()) {
      std::cout << "Change the control mode from force/torque to velocity control.\n";
      // Stop the force control loop
      m_ftControlThreadStopAsked = true;
    }

    if (getRobotState() != vpRobot::STATE_VELOCITY_CONTROL) {
      if (m_controlThread.joinable()) {
        m_controlThread.join();
        m_posControlThreadStopAsked = false;
        m_posControlThreadIsRunning = false;
        m_ftControlThreadStopAsked = false;
        m_ftControlThreadIsRunning = false;
      }
    }
    if (! m_velControlThreadIsRunning) {
      m_controlThread = std::thread([this] {this->velocityControlLoop();});
    }
    this->m_stateRobot = newState;
    break;
  }
  case vpRobot::STATE_FORCE_TORQUE_CONTROL: {
    if (vpRobot::STATE_STOP == getRobotState()) {
      std::cout << "Change the control mode from stop to force/torque control.\n";
    }
    else if (vpRobot::STATE_POSITION_CONTROL == getRobotState()) {
      std::cout << "Change the control mode from position to force/torque control.\n";
      m_posControlThreadStopAsked = true;
    }
    else if (vpRobot::STATE_VELOCITY_CONTROL == getRobotState()) {
      std::cout << "Change the control mode from velocity to force/torque control.\n";
      m_velControlThreadStopAsked = true;
    }
    if (getRobotState() != vpRobot::STATE_FORCE_TORQUE_CONTROL) {
      if(m_controlThread.joinable()) {
        m_controlThread.join();
        m_posControlThreadStopAsked = false;
        m_posControlThreadIsRunning = false;
        m_velControlThreadStopAsked = false;
        m_velControlThreadIsRunning = false;
      }
    }
    if (! m_ftControlThreadIsRunning) {
      m_controlThread = std::thread([this] {this->torqueControlLoop();});
    }
    this->m_stateRobot = newState;
    break;
  }

  default:
    break;
  }

  return newState;
}

void vpROSRobotFrankaSim::readingLoop()
{
  ros::NodeHandlePtr n = boost::make_shared<ros::NodeHandle>();
  ros::Rate loop_rate(500); // Hz

  if (m_jointState_topic.empty()) {
    throw(vpException(vpException::fatalError, "Error: Joint state topic is not set"));
  }
  if (m_eMc_topic.empty()) {
    throw(vpException(vpException::fatalError, "Error: eMc transform topic is not set"));
  }
  std::cout << "Subscribe to " << m_jointState_topic << std::endl;
  ros::Subscriber sub_franka_jointState = n->subscribe(m_jointState_topic, 1, &vpROSRobotFrankaSim::jointState_callback,this);
  std::cout << "Subscribe to " << m_eMc_topic << std::endl;
  ros::Subscriber sub_franka_eMc = n->subscribe(m_eMc_topic, 1, &vpROSRobotFrankaSim::eMc_callback,this);

  while (ros::ok() && m_connected)
  {
    loop_rate.sleep();
    ros::spinOnce();
  }

  std::lock_guard<std::mutex> lock(m_mutex);
  m_connected = false;
}

void vpROSRobotFrankaSim::positionControlLoop()
{
  if (m_verbose) {
    std::cout << "Position controller thread launched" << std::endl;
  }
  ros::NodeHandlePtr n = boost::make_shared<ros::NodeHandle>();
  ros::Rate pos_loop_rate(500); // Hz
  ros::Publisher jointState_pub = n->advertise<sensor_msgs::JointState>("/fakeFCI/joint_state", 1);
  ros::Publisher robotCtrlType_pub = n->advertise<std_msgs::Int32>("/fakeFCI/robot_ctrl_type", 1);
  sensor_msgs::JointState joint_state_msg;
  joint_state_msg.velocity.resize(7);
  joint_state_msg.name.resize(7);
  joint_state_msg.header.frame_id = "Joint_velocity_cmd";
  for (unsigned int i = 0; i < 7; i ++) {
    joint_state_msg.name[i] = "J" + std::to_string(i);
  }
  vpColVector vel_max(7, 0), dq_sat(7,0), gains(7,0);
  vel_max = {2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100};

  vpMatrix Kp(7,7), Kd(7,7);

  gains = {6.0, 6.0, 6.0, 6.0, 3.5, 2.5, 2.0};
  Kp.diag(gains);
  gains = {0.5, 0.5, 0.5, 0.5, 0.3, 0.25, 0.2};
  Kd.diag(gains);

  // Publish robot state to the corresponding ROS topic
  std_msgs::Int32 robot_ctrl_type_msg;
  robot_ctrl_type_msg.data = static_cast<int>(m_stateRobot);

  m_posControlThreadIsRunning = true;
  bool robot_ctrl_type_published = false;
  while (ros::ok() && !m_posControlThreadStopAsked && m_posControlNewCmd) {
    pos_loop_rate.sleep();
    if (! robot_ctrl_type_published) {
      std::cout << "Publish /fakeFCI/robot_ctrl_type: " << m_stateRobot << std::endl;
      robot_ctrl_type_published = true;
    }
    robotCtrlType_pub.publish(robot_ctrl_type_msg); // Should be published more than one time to be received by CoppeliaSim !
    m_mutex.lock();
    m_dq_des = Kp * 10. * (m_q_des - m_q) - Kd * m_dq;
    dq_sat = vpRobot::saturateVelocities(m_dq_des, vel_max, false);
    if(std::sqrt(((180./M_PI)*(m_q_des - m_q)).sumSquare()) > 0.1 ){
      for (unsigned int i = 0; i < 7; i ++) {
        joint_state_msg.velocity[i] = dq_sat[i];
      }
    }
    else {
      for (unsigned int i = 0; i < 7; i ++) {
        joint_state_msg.velocity[i] = 0;
      }
      m_posControlNewCmd = false;
    }

    m_mutex.unlock();
    jointState_pub.publish(joint_state_msg);
  }

  for (unsigned int i = 0; i < 7; i++) {
    joint_state_msg.velocity[i] = 0.;
  }
  jointState_pub.publish(joint_state_msg);
  m_posControlThreadIsRunning = false;

  if (m_verbose) {
    std::cout << "Position controller thread finished" << std::endl;
  }
}

void vpROSRobotFrankaSim::velocityControlLoop()
{
  if (m_verbose) {
    std::cout << "Velocity controller thread launched" << std::endl;
  }
  ros::NodeHandlePtr n = boost::make_shared<ros::NodeHandle>();
  ros::Rate vel_loop_rate(500); // Hz
  ros::Publisher jointState_pub = n->advertise<sensor_msgs::JointState>("/fakeFCI/joint_state", 1);
  ros::Publisher robotCtrlType_pub = n->advertise<std_msgs::Int32>("/fakeFCI/robot_ctrl_type", 1);

  sensor_msgs::JointState joint_state_msg;
  joint_state_msg.velocity.resize(7);
  joint_state_msg.name.resize(7);
  joint_state_msg.header.frame_id = "Joint_velocity_cmd";
  for (unsigned int i = 0; i < 7; i++) {
    joint_state_msg.name[i] = "J" + std::to_string(i);
  }
  // Publish robot state to the corresponding ROS topic
  std_msgs::Int32 robot_ctrl_type_msg;
  robot_ctrl_type_msg.data = static_cast<int>(m_stateRobot);

  vpColVector vel_max(7, 0), dq_sat(7,0);
  vel_max = {2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100};

  m_velControlThreadIsRunning = true;
  bool robot_ctrl_type_published = false;
  while (ros::ok() && ! m_velControlThreadStopAsked) {
    vel_loop_rate.sleep();
    if (! robot_ctrl_type_published) {
      std::cout << "Publish /fakeFCI/robot_ctrl_type: " << m_stateRobot << std::endl;
      robot_ctrl_type_published = true;
    }
    robotCtrlType_pub.publish(robot_ctrl_type_msg); // Should be published more than one time to be received by CoppeliaSim !
    m_mutex.lock();
    dq_sat = vpRobot::saturateVelocities(m_dq_des, vel_max, true);
    for (unsigned int i = 0; i < 7; i ++) {
      joint_state_msg.velocity[i] = dq_sat[i];
    }
    m_mutex.unlock();
    jointState_pub.publish(joint_state_msg);
  }

  for (unsigned int i = 0; i < 7; i ++) {
    joint_state_msg.velocity[i] = 0;
  }
  jointState_pub.publish(joint_state_msg);
  m_velControlThreadIsRunning = false;

  if (m_verbose) {
    std::cout << "Velocity controller thread finished" << std::endl;
  }
};

void vpROSRobotFrankaSim::torqueControlLoop()
{
  if (m_verbose) {
    std::cout << "Torque controller thread launched" << std::endl;
  }
  ros::NodeHandlePtr n = boost::make_shared<ros::NodeHandle>();
  ros::Rate ft_loop_rate(500); // Hz
  ros::Publisher jointState_pub = n->advertise<sensor_msgs::JointState>("/fakeFCI/joint_state", 1);
  sensor_msgs::JointState joint_state_msg;
  joint_state_msg.effort.resize(7);
  joint_state_msg.name.resize(7);
  joint_state_msg.header.frame_id = "Joint_torque_cmd";
  for (unsigned int i = 0; i < 7; i ++) {
    joint_state_msg.name[i] = "J" + std::to_string(i);
  }
  vpColVector g(7,0), tau_sat(7,0), tau_max(7,0);
  tau_max = {87, 87,87, 87, 12, 12, 12};
#ifdef HAVE_IIR
  const int order = 1; // ex: 4th order (=2 biquads)
  const double samplingrate = 1000.; // Hz
  const double cutoff_frequency = 10.; // Hz
  std::array<Iir::Butterworth::LowPass<order>, 7> TorqueFilter;
  for (unsigned int i = 0; i < 7; i++) {
    TorqueFilter[i].setup(samplingrate, cutoff_frequency);
  }
#endif

  m_ftControlThreadIsRunning = true;
  while (ros::ok() && ! m_ftControlThreadStopAsked) {
    ft_loop_rate.sleep();
    this->getGravity(g);

    m_mutex.lock();
    for (unsigned int i = 0; i < 7; i++) {
      if(std::abs(tau_sat[i]) >= 0.0){ // this simulates the static friction
#ifdef HAVE_IIR
        m_tau_J_des_filt[i] = TorqueFilter[i].filter(m_tau_J_des[i]);
        joint_state_msg.effort[i] = m_tau_J_des_filt[i] + g[i];
#else
        joint_state_msg.effort[i] = m_tau_J_des[i] + g[i];
#endif
      }
      else {
        joint_state_msg.effort[i] = g[i];
      }
    }
    m_mutex.unlock();

    jointState_pub.publish(joint_state_msg);
  }

  for (unsigned int i = 0; i < 7; i ++) {
    joint_state_msg.effort[i] = 0;
  }
  jointState_pub.publish(joint_state_msg);
  m_ftControlThreadIsRunning = false;

  if (m_verbose) {
    std::cout << "Torque controller thread finished" << std::endl;
  }
}

void vpROSRobotFrankaSim::jointState_callback(const sensor_msgs::JointState& J_state)
{
  std::lock_guard<std::mutex> lock(m_mutex);
  for (unsigned int i = 0; i < 7; i ++) {
    m_q_kdl(i) = m_q[i] = J_state.position[i];
    m_dq[i] = J_state.velocity[i];
    m_tau_J[i] = J_state.effort[i];
  }
}


void vpROSRobotFrankaSim::eMc_callback(const geometry_msgs::Pose& pose_msg)
{
  std::lock_guard<std::mutex> lock(m_mutex);
  if (! m_overwrite_eMc) {
    m_eMc = visp_bridge::toVispHomogeneousMatrix(pose_msg);
    m_eVc.buildFrom(m_eMc);
  }
}

void vpROSRobotFrankaSim::get_fJe(vpMatrix &fJe)
{
  fJe.reshape(6,7);
  KDL::Jacobian Jac(7);

  m_mutex.lock();
  m_jacobianSolver_kdl->JntToJac(m_q_kdl, Jac);
  m_mutex.unlock();

  for(unsigned int i = 0; i < 6; i++) {
    for(unsigned int j = 0; j < 7; j++) {
      fJe[i][j] = Jac.data(i,j);
    }
  }
}

void vpROSRobotFrankaSim::get_fJe(const vpColVector &q, vpMatrix &fJe)
{
  fJe.reshape(6, 7);
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
}

void vpROSRobotFrankaSim::get_eJe(vpMatrix &eJe)
{
  vpMatrix fJe(6,7);
  get_fJe(fJe);
  vpHomogeneousMatrix fMe(get_fMe());
  vpMatrix eVf(6,6);
  eVf.insert(fMe.getRotationMatrix().t(), 0, 0);
  eVf.insert(fMe.getRotationMatrix().t(), 3, 3);

  eJe = eVf * fJe;
}

void vpROSRobotFrankaSim::get_eJe(const vpColVector &q, vpMatrix &eJe)
{
  vpMatrix fJe(6,7);
  get_fJe(q, fJe);
  vpHomogeneousMatrix fMe(get_fMe(q));
  vpMatrix eVf(6, 6);
  eVf.insert(fMe.getRotationMatrix().t(), 0, 0);
  eVf.insert(fMe.getRotationMatrix().t(), 3, 3);

  eJe = eVf * fJe;
}

vpHomogeneousMatrix vpROSRobotFrankaSim::get_eMc() const
{
  return m_eMc;
}

void vpROSRobotFrankaSim::getPosition(const vpRobot::vpControlFrameType frame, vpColVector &position)
{
  switch(frame) {
  case vpRobot::JOINT_STATE: {
    position.resize(7);
    std::lock_guard<std::mutex> lock(m_mutex);
    position = m_q;
    break;
  }
  case vpRobot::END_EFFECTOR_FRAME: {
    position.resize(6);

    vpPoseVector fPc(get_fMe());
    for (unsigned int i = 0; i < 6; i++) {
      position[i] = fPc[i];
    }

    break;
  }
  case vpRobot::CAMERA_FRAME: { // same as CAMERA_FRAME
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

void vpROSRobotFrankaSim::getPosition(const vpRobot::vpControlFrameType frame, vpPoseVector &position)
{
  vpColVector pos(6,0);
  if(frame == vpRobot::END_EFFECTOR_FRAME || frame == vpRobot::CAMERA_FRAME){
    getPosition(frame, pos);
    for (unsigned int i = 0; i < 6; i++) {
      position[i] = pos[i];
    }
  }else{
    throw(vpException(vpException::fatalError, "Cannot get a cartesian position for the specified frame"));
  }
}

void vpROSRobotFrankaSim::setPosition(const vpRobot::vpControlFrameType frame, const vpColVector &position)
{
  switch(frame) {
  case vpRobot::JOINT_STATE: {
    std::lock_guard<std::mutex> lock(m_mutex);
    for (unsigned int i = 0; i < 7; i++) {
      m_q_des[i] = position[i];
    }
    m_posControlNewCmd = true;

    break;
  }
  case vpRobot::END_EFFECTOR_FRAME: {
    vpHomogeneousMatrix H;
    H.buildFrom(position[0],position[1],position[2],position[3],position[4],position[5]);
    std::lock_guard<std::mutex> lock(m_mutex);
    m_q_des = solveIK(H);
    m_posControlNewCmd = true;

    break;
  }
  default: {
    throw(vpException(vpException::fatalError, "Cannot set Franka position for the specified frame \n"));
  }
  }

  if(vpRobot::STATE_POSITION_CONTROL == getRobotState()){
    m_controlThread = std::thread([this] {this->positionControlLoop();});

    if(m_controlThread.joinable()){
      m_controlThread.join();
    }
  } else {
    std::cout << "Robot is not in STATE_POSITION_CONTROL, you should change to position control before. \n" ;
  }
}

vpColVector vpROSRobotFrankaSim::solveIK(const vpHomogeneousMatrix &edMw)
{
  vpColVector q_solved(7,0);
  KDL::JntArray q_out(7);
  KDL::Rotation R(edMw[0][0],edMw[0][1],edMw[0][2],
      edMw[1][0],edMw[1][1],edMw[0][2],
      edMw[2][0],edMw[2][1],edMw[2][2]);
  KDL::Vector v(edMw[0][3],edMw[1][3],edMw[2][3]);
  KDL::Frame dest(R, v);
  m_mutex.lock();
  int ret = m_iksolver_JL_kdl->CartToJnt(m_q_kdl, dest, q_out);
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
  return q_solved;
}

void vpROSRobotFrankaSim::getVelocity(const vpRobot::vpControlFrameType frame, vpColVector &d_velocity)
{
  switch(frame) {
  case vpRobot::JOINT_STATE: {
    d_velocity.resize(7);
    std::lock_guard<std::mutex> lock(m_mutex);
    d_velocity = m_dq;
    break;
  }
  case vpRobot::END_EFFECTOR_FRAME: {
    d_velocity.resize(6);
    vpMatrix eJe(6,7);
    this->get_eJe(eJe);
    std::lock_guard<std::mutex> lock(m_mutex);
    d_velocity = eJe * m_dq;
    break;
  }
  case vpRobot::REFERENCE_FRAME: {
    d_velocity.resize(6);
    vpMatrix fJe(6,7);
    this->get_fJe(fJe);
    std::lock_guard<std::mutex> lock(m_mutex);
    d_velocity = fJe * m_dq;
    break;
  }
  default: {
    throw(vpException(vpException::fatalError, "Cannot get Franka position for the specified frame "));
  }
  }
}

void vpROSRobotFrankaSim::setVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &vel)
{
  if (vpRobot::STATE_VELOCITY_CONTROL != getRobotState()) {
    std::cout <<  "Cannot send a velocity to the robot. "
                  "Use setRobotState(vpRobot::STATE_VELOCITY_CONTROL) first. \n";
  }

  switch (frame) {
  case vpRobot::JOINT_STATE: {
    if (vel.size() != 7) {
      std::cout <<  "Joint velocity vector "<< vel.size() <<" is not of size 7 \n";
    }
    std::lock_guard<std::mutex> lock(m_mutex);
    m_dq_des = vel;
    break;
  }

  case vpRobot::REFERENCE_FRAME: {
    if (vel.size() != 6) {
      std::cout <<  "Cartesian velocity vector " << vel.size() << " is not of size 6 \n";
    }
    vpColVector vel_max(6);

    for (unsigned int i = 0; i < 3; i++) {
      vel_max[i] = 1.7;
      vel_max[3 + i] = 2.5;
    }
    // velocities are expressed in Base frame
    m_v_cart_des = vpRobot::saturateVelocities(vel, vel_max, true);

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
    break;
  }

  case vpRobot::END_EFFECTOR_FRAME: {
    if (vel.size() != 6) {
      std::cout <<  "Cartesian velocity vector " << vel.size() << " is not of size 6 \n";
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
    m_v_cart_des = fWe * vpRobot::saturateVelocities(vel, vel_max, true);

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

    break;
  }
  case vpRobot::CAMERA_FRAME: {
    if (vel.size() != 6) {
      std::cout <<  "Cartesian velocity vector " << vel.size() << " is not of size 6 \n";
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
    m_v_cart_des = vpRobot::saturateVelocities(fWe * m_eVc * vel, vel_max, true);

    KDL::Twist v_cart;
    for(unsigned int i = 0; i < 3; i++) {
      v_cart.vel.data[i] = m_v_cart_des[i];
      v_cart.rot.data[i] = m_v_cart_des[i + 3];
    }

    m_diffIkSolver_kdl->CartToJnt(m_q_kdl, v_cart, m_dq_des_kdl);
    for(unsigned int i = 0; i < 7; i++) {
      m_dq_des[i] = m_dq_des_kdl.data(i);
    }

    break;
  }
  case vpRobot::MIXT_FRAME:
    throw(vpException(vpException::functionNotImplementedError, "MIXT_FRAME is not implemented"));
  }
}

void vpROSRobotFrankaSim::getForceTorque(const vpRobot::vpControlFrameType frame, vpColVector &force)
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

void vpROSRobotFrankaSim::setForceTorque(const vpRobot::vpControlFrameType frame, const vpColVector &force)
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

vpHomogeneousMatrix vpROSRobotFrankaSim::get_fMe()
{
  vpHomogeneousMatrix fMe;
  KDL::Frame cartpos;
  // Calculate forward kinematics
  int kinematics_status;
  vpRotationMatrix R;
  vpTranslationVector t;
  std::lock_guard<std::mutex> lock(m_mutex);
  kinematics_status = m_fksolver_kdl->JntToCart(m_q_kdl, cartpos);
  if(kinematics_status >= 0){
    for(unsigned int i = 0; i < 3; i++) {
      for(unsigned int j = 0; j < 3; j++) {
        R[i][j] = cartpos.M.data[3 * i + j];
      }
      t[i] = cartpos.p.data[i];
    }
    fMe.buildFrom(t, R);
  }

  return fMe;
}

vpHomogeneousMatrix vpROSRobotFrankaSim::get_fMe(const vpColVector &q)
{
  KDL::Frame cartpos;
  KDL::JntArray qq(7);
  for (unsigned int i = 0; i < 7; i++) {
    qq(i) = q[i];
  }
  // Calculate forward kinematics
  int kinematics_status;
  vpRotationMatrix R;
  vpTranslationVector t;
  kinematics_status = m_fksolver_kdl->JntToCart(qq,cartpos);
  if(kinematics_status>=0){
    for(unsigned int i = 0; i < 3; i++) {
      for(unsigned int j = 0; j < 3; j++) {
        R[i][j] = cartpos.M.data[3 * i + j];
      }
      t[i] = cartpos.p.data[i];
    }
  }
  vpHomogeneousMatrix fMe(t, R);
  return fMe;
}

void vpROSRobotFrankaSim::getMass(vpMatrix &mass){
  std::lock_guard<std::mutex> lock(m_mutex);
  mass = franka_model::massMatrix(m_q);
}

void vpROSRobotFrankaSim::getGravity(vpColVector &gravity){
  std::lock_guard<std::mutex> lock(m_mutex);
  gravity = franka_model::gravityVector(m_q);
}

void vpROSRobotFrankaSim::getCoriolis(vpColVector &coriolis){
  std::lock_guard<std::mutex> lock(m_mutex);
  vpMatrix C(7,7);
  C = franka_model::coriolisMatrix(m_q, m_dq);
  coriolis = C * m_dq;
}
