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

#include <visp_ros/vpROSRobotFrankaCoppeliasim.h>

#ifdef VISP_HAVE_IIR
#include <Iir.h>
#endif

/*!
 * Default constructor.
 */
vpROSRobotFrankaCoppeliasim::vpROSRobotFrankaCoppeliasim()
  : m_controlThread()
  , m_acquisitionThread()
  , m_topic_jointState( "/coppeliasim/franka/joint_state" )
  , m_topic_g0( "/coppeliasim/franka/g0" )
  , m_topic_eMc( "/coppeliasim/franka/eMc" )
  , m_topic_flMe( "/coppeliasim/franka/flMe" )
  , m_topic_flMcom( "/coppeliasim/franka/flMcom" )
  , m_topic_toolInertia( "/coppeliasim/franka/tool/inertia" )
  , m_topic_jointStateCmd( "/fakeFCI/joint_state" )
  , m_topic_robotStateCmd( "/fakeFCI/robot_state" )
  , m_connected( false )
  , m_posControlThreadIsRunning( false )
  , m_posControlThreadStopAsked( false )
  , m_posControlLock( false )
  , m_posControlNewCmd( false )
  , m_velControlThreadIsRunning( false )
  , m_velControlThreadStopAsked( false )
  , m_ftControlThreadIsRunning( false )
  , m_ftControlThreadStopAsked( false )
  , m_pub_jointStateCmd()
  , m_pub_robotStateCmd()
  , m_pub_startSimulation()
  , m_pub_pauseSimulation()
  , m_pub_stopSimulation()
  , m_pub_triggerNextStep()
  , m_pub_enableSyncMode()
  , m_sub_coppeliasim_jointState()
  , m_sub_coppeliasim_eMc()
  , m_sub_coppeliasim_simulationStepDone()
  , m_sub_coppeliasim_simulationTime()
  , m_sub_coppeliasim_simulationState()
  , m_sub_coppeliasim_flMe()
  , m_sub_coppeliasim_toolInertia()
  , m_simulationStepDone( false )
  , m_simulationTime( 0. )
  , m_syncModeEnabled( false )
  , m_simulationState( 0 )
  , m_overwrite_flMe( true )
  , m_overwrite_toolInertia( true )
{
}

/*!
 * Destructor.
 */
vpROSRobotFrankaCoppeliasim::~vpROSRobotFrankaCoppeliasim()
{
  m_mutex.lock();
  m_connected                 = false;
  m_posControlThreadStopAsked = true;
  m_velControlThreadStopAsked = true;
  m_ftControlThreadStopAsked  = true;
  m_mutex.unlock();

  if ( m_controlThread.joinable() )
  {
    m_controlThread.join();
    m_posControlThreadStopAsked = false;
    m_posControlThreadIsRunning = false;
    m_velControlThreadStopAsked = false;
    m_velControlThreadIsRunning = false;
    m_ftControlThreadStopAsked  = false;
    m_ftControlThreadIsRunning  = false;
  }

  if ( m_acquisitionThread.joinable() )
  {
    m_acquisitionThread.join();
  }

}

/*!
 * Connect to Coppeliasim simulator by subscribing to the requested topics.
 *
 * It is possible to change the default topic names, before calling this function.
 *
 * \sa setTopicJointState(), setTopic_eMc(), setTopicJointStateCmd(), setTopicRobotStateCmdCmd()
 */
void
vpROSRobotFrankaCoppeliasim::connect( const std::string &robot_ID )
{
  ros::NodeHandlePtr n = boost::make_shared< ros::NodeHandle >();

  if(robot_ID != "")// TODO: regex to check name is valid for ROS topic
  {
    m_topic_jointState = "/coppeliasim/franka/" + robot_ID + "/joint_state";
    m_topic_g0 = "/coppeliasim/franka/" + robot_ID + "/g0";
    m_topic_eMc = "/coppeliasim/franka/" + robot_ID + "/eMc";
    m_topic_flMe = "/coppeliasim/franka/" + robot_ID + "/flMe";
    m_topic_flMcom = "/coppeliasim/franka/" + robot_ID + "/flMcom";
    m_topic_toolInertia = "/coppeliasim/franka/" + robot_ID + "/tool/inertia";
    m_topic_jointStateCmd = "/fakeFCI/" + robot_ID + "/joint_state";
    m_topic_robotStateCmd = "/fakeFCI/" + robot_ID + "/robot_state";
  }

  if ( m_verbose )
  {
    std::cout << "Subscribe " << m_topic_jointState << std::endl;
    std::cout << "Subscribe " << m_topic_g0 << std::endl;
    std::cout << "Subscribe " << m_topic_eMc << std::endl;
    std::cout << "Subscribe " << m_topic_flMe << std::endl;
    std::cout << "Subscribe " << m_topic_flMcom << std::endl;
    std::cout << "Subscribe " << m_topic_toolInertia << std::endl;
  }
  m_sub_coppeliasim_jointState =
      n->subscribe( m_topic_jointState, 1, &vpROSRobotFrankaCoppeliasim::callbackJointState, this );
  m_sub_coppeliasim_g0     = n->subscribe( m_topic_g0, 1, &vpROSRobotFrankaCoppeliasim::callback_g0, this );
  m_sub_coppeliasim_eMc    = n->subscribe( m_topic_eMc, 1, &vpROSRobotFrankaCoppeliasim::callback_eMc, this );
  m_sub_coppeliasim_flMe   = n->subscribe( m_topic_flMe, 1, &vpROSRobotFrankaCoppeliasim::callback_flMe, this );
  m_sub_coppeliasim_toolInertia =
      n->subscribe( m_topic_toolInertia, 1, &vpROSRobotFrankaCoppeliasim::callback_toolInertia, this );

  if ( m_verbose )
  {
    std::cout << "Advertise " << m_topic_jointStateCmd << std::endl;
    std::cout << "Advertise " << m_topic_robotStateCmd << std::endl;
  }
  m_pub_jointStateCmd = n->advertise< sensor_msgs::JointState >( m_topic_jointStateCmd, 1 );
  m_pub_robotStateCmd = n->advertise< std_msgs::Int32 >( m_topic_robotStateCmd, 1 );

  // Coppeliasim specific topic names
  m_sub_coppeliasim_simulationStepDone =
      n->subscribe( "/simulationStepDone", 1, &vpROSRobotFrankaCoppeliasim::callbackSimulationStepDone, this );
  m_sub_coppeliasim_simulationTime =
      n->subscribe( "/simulationTime", 1, &vpROSRobotFrankaCoppeliasim::callbackSimulationTime, this );
  m_sub_coppeliasim_simulationState =
      n->subscribe( "/simulationState", 1, &vpROSRobotFrankaCoppeliasim::callbackSimulationState, this );

  m_pub_startSimulation = n->advertise< std_msgs::Bool >( "/startSimulation", 1 );
  m_pub_pauseSimulation = n->advertise< std_msgs::Bool >( "/pauseSimulation", 1 );
  m_pub_stopSimulation  = n->advertise< std_msgs::Bool >( "/stopSimulation", 1 );
  m_pub_enableSyncMode  = n->advertise< std_msgs::Bool >( "/enableSyncMode", 1 );
  m_pub_triggerNextStep = n->advertise< std_msgs::Bool >( "/triggerNextStep", 1 );

  std::lock_guard< std::mutex > lock( m_mutex );
  m_connected         = true;
  m_acquisitionThread = std::thread( [this] { readingLoop(); } );

  // Sleep a couple of ms to ensure thread is launched and topics are created
  vpTime::sleepMs( 3000 );
  std::cout << "ROS is initialized ? " << ( ros::isInitialized() ? "yes" : "no" ) << std::endl;
}

/*!
 * Enable/disable Coppeliasim synchronous simulation mode by publishing a std_msgs::Bool message
 * on `/enableSyncMode` topic.
 * \param[in] enable : Set true to enable synchronous simulation mode, false otherwise.
 * When synchronous simulation mode is enable, you need to call coppeliasimTriggerNextStep()
 * to run a new simulation step.
 *
 * \param[in] sleep_ms : Sleeping time in [ms] added after publishing the message to ensure
 * that it will be taken into account by Coppeliasim.
 *
 * \sa coppeliasimTriggerNextStep(), getCoppeliasimSybcMode()
 */
void
vpROSRobotFrankaCoppeliasim::setCoppeliasimSyncMode( bool enable, double sleep_ms )
{
  std_msgs::Bool msg_enableSyncMode;
  msg_enableSyncMode.data = enable;
  m_pub_enableSyncMode.publish( msg_enableSyncMode );
  m_syncModeEnabled = enable;
  ros::Rate loop_rate( 1000. / sleep_ms );
  loop_rate.sleep();
}

/*!
 * Pause Coppeliasim simulation by publishing a std_msgs::Bool message
 * on `/pauseSimulation` topic.
 *
 * \param[in] sleep_ms : Sleeping time in [ms] added after publishing the message to ensure
 * that it will be taken into account by Coppeliasim.
 *
 * \sa coppeliasimStartSimulation(), coppeliasimStopSimulation(), getCoppeliasimSimulationState()
 */
void
vpROSRobotFrankaCoppeliasim::coppeliasimPauseSimulation( double sleep_ms )
{
  std_msgs::Bool msg_pauseSimulation;
  msg_pauseSimulation.data = true;
  m_pub_pauseSimulation.publish( msg_pauseSimulation );
  ros::Rate loop_rate( 1000. / sleep_ms );
  loop_rate.sleep();
}

/*!
 * Start Coppeliasim simulation by publishing a a std_msgs::Bool message
 * on `/startSimulation` topic.
 *
 * \param[in] sleep_ms : Sleeping time in [ms] added after publishing the message to ensure
 * that it will be taken into account by Coppeliasim.
 *
 * \sa coppeliasimPauseSimulation(), coppeliasimStopSimulation(), getCoppeliasimSimulationState()
 */
void
vpROSRobotFrankaCoppeliasim::coppeliasimStartSimulation( double sleep_ms )
{
  std_msgs::Bool msg_startSimulation;
  msg_startSimulation.data = true;
  m_pub_startSimulation.publish( msg_startSimulation );
  ros::Rate loop_rate( 1000. / sleep_ms );
  loop_rate.sleep();
  coppeliasimTriggerNextStep();
}

/*!
 * Stop Coppeliasim simulation by publishing a std_msgs::Bool message
 * on `/stopSimulation` topic.
 *
 * \param[in] sleep_ms : Sleeping time in [ms] added after publishing the message to ensure
 * that it will be taken into account by Coppeliasim.
 *
 * \sa coppeliasimStartSimulation(), coppeliasimPauseSimulation(), getCoppeliasimSimulationState()
 */
void
vpROSRobotFrankaCoppeliasim::coppeliasimStopSimulation( double sleep_ms )
{
  std_msgs::Bool msg_stopSimulation;
  msg_stopSimulation.data = true;
  m_pub_stopSimulation.publish( msg_stopSimulation );
  ros::Rate loop_rate( 1000. / sleep_ms );
  loop_rate.sleep();
}

/*!
 * Trigger Coppeliasim next simulation step by publishing a std_msgs::Bool
 * message on `/triggerNextStep` topic, while in the synchronous simulation mode.
 *
 * \note It is not useful to call this method when Coppeliasim synchronous mode is not set
 * using setCoppeliasimSyncMode().
 *
 * \sa setCoppeliasimSyncMode()
 */
void
vpROSRobotFrankaCoppeliasim::coppeliasimTriggerNextStep()
{
  std_msgs::Bool msg_triggerNextStep;
  msg_triggerNextStep.data = true;
  m_pub_triggerNextStep.publish( msg_triggerNextStep );
}

/*!
 * Callback that allows to know when a simulation step is done on Coppeliasim side.
 * \param[in] msg : Simulation step done message send by Coppeliasim
 */
void
vpROSRobotFrankaCoppeliasim::callbackSimulationStepDone( const std_msgs::Bool &msg )
{
  std::lock_guard< std::mutex > lock( m_mutex );
  m_simulationStepDone = msg.data;
}

/*!
 * Return true when a simulation step is done on Coppeliasim side, false otherwise.
 */
bool
vpROSRobotFrankaCoppeliasim::getCoppeliasimSimulationStepDone()
{
  std::lock_guard< std::mutex > lock( m_mutex );
  return m_simulationStepDone;
}

/*!
 * Set Coppeliasim simulation step done flag.
 * \param[in] simulationStepDone : True to indicate that the simmulation step is done,
 * false otherwise.
 */
void
vpROSRobotFrankaCoppeliasim::setCoppeliasimSimulationStepDone( bool simulationStepDone )
{
  std::lock_guard< std::mutex > lock( m_mutex );
  m_simulationStepDone = simulationStepDone;
}

/*!
 * Callback that allows to update the Coppeliasim simulation time.
 * \param[in] msg : Simulation time message.
 */
void
vpROSRobotFrankaCoppeliasim::callbackSimulationTime( const std_msgs::Float32 &msg )
{
  std::lock_guard< std::mutex > lock( m_mutex );
  m_simulationTime = msg.data;
}

/*!
 * Return Coppeliasim simulation time.
 */
double
vpROSRobotFrankaCoppeliasim::getCoppeliasimSimulationTime()
{
  std::lock_guard< std::mutex > lock( m_mutex );
  return static_cast< double >( m_simulationTime );
}

/*!
 * Callback that allows to update the Coppeliasim simulation state.
 * \param[in] msg : Simulation state message.
 */
void
vpROSRobotFrankaCoppeliasim::callbackSimulationState( const std_msgs::Int32 &msg )
{
  std::lock_guard< std::mutex > lock( m_mutex );
  m_simulationState = msg.data;
}

/*!
 * Return Coppeliasim simulation state.
 * 0 indicates that the simulation is stopped, 1 that it is running and 2 that it is paused.
 *
 * \sa coppeliasimStartSimulation(), coppeliasimPauseSimulation(), coppeliasimStopSimulation()
 *
 */
int
vpROSRobotFrankaCoppeliasim::getCoppeliasimSimulationState()
{
  std::lock_guard< std::mutex > lock( m_mutex );
  return m_simulationState;
}

/*!
 * Subscribes to topics updated on Coppeliasim side and runs an infinite loop
 * that updates the robot state.
 *
 * \sa setTopicJointState(), setTopic_eMc()
 */
void
vpROSRobotFrankaCoppeliasim::readingLoop()
{
  ros::Rate loop_rate( 500 ); // Hz

  while ( ros::ok() && m_connected )
  {
    loop_rate.sleep();
    ros::spinOnce();
  }

  std::lock_guard< std::mutex > lock( m_mutex );
  m_connected = false;
}

/*!
 * Callback that updates robot joint state (joint position, joint velocity, joint torque)
 * with the values updated on Coppeliasim side.
 * \param[in] joint_state : Joint state message associated to the topic set by setTopicJointState().
 */
void
vpROSRobotFrankaCoppeliasim::callbackJointState( const sensor_msgs::JointState &joint_state )
{
  std::lock_guard< std::mutex > lock( m_mutex );
  for ( unsigned int i = 0; i < 7; i++ )
  {
    m_q_kdl( i ) = m_q[i] = joint_state.position[i];
    m_dq[i]               = joint_state.velocity[i];
    m_tau_J[i]            = joint_state.effort[i];
  }
}

/*!
 * Callback that updates the gravitational acceleration vector as perceived at
 * the robot's base considered on Coppeliasim side.
 * \param[in] g0_msg : Message associated to the topic set by setTopic_g0() the
 * absolute acceleration vector in base frame.
 */
void
vpROSRobotFrankaCoppeliasim::callback_g0( const geometry_msgs::Vector3 &g0_msg )
{
  vpColVector g0( 3, 0 );
  g0[0] = g0_msg.x;
  g0[1] = g0_msg.y;
  g0[2] = g0_msg.z;
  this->set_g0( g0 );
}

/*!
 * Callback that updates camera extrinsics
 * with the transformation considered on Coppeliasim side.
 * \param[in] pose_msg : Message associated to the topic set by setTopic_eMc() that contains
 * the homogeneous transformation between end-effector and camera frame.
 */
void
vpROSRobotFrankaCoppeliasim::callback_eMc( const geometry_msgs::Pose &pose_msg )
{
  if ( !m_camMounted )
  {
    this->set_eMc( visp_bridge::toVispHomogeneousMatrix( pose_msg ) );
  }
}

/*!
 * Callback that updates end-effector extrinsics w.r.t. the robot flange
 * with the transformation considered on Coppeliasim side.
 * \param[in] pose_msg : Message associated to the topic set by setTopic_flMe() that contains
 * the homogeneous transformation between end-effector and flange frame.
 * This callback works jointly with the `callback_toolInertia` to automatically
 * retrieve the kinematic and dynamic parameters of the tool.
 */
void
vpROSRobotFrankaCoppeliasim::callback_flMe( const geometry_msgs::Pose &pose_msg )
{
  if ( !m_toolMounted && m_overwrite_flMe )
  {
    this->set_flMe( visp_bridge::toVispHomogeneousMatrix( pose_msg ) );
    m_overwrite_flMe = false;

    if ( !m_overwrite_flMe && !m_overwrite_toolInertia )
    {
      m_toolMounted = true;
      if ( m_verbose )
      {
        std::cout << "A tool has been mounted on the robot.\n";
        std::cout << "Mass: " << m_mL << " [kg]\n";
        std::cout << "Inertia Tensor in flange frame:\n" << m_Il << " [kg*m^2]\n";
        std::cout << "CoM position in flange frame: " << m_flMcom.getTranslationVector().t() << " [m]\n";
        std::cout << "CoM orientation in flange frame: \n" << m_flMcom.getRotationMatrix() << std::endl;
      }
    }
  }
}


/*!
 * Callback that updates the tool inertial parameters.
 * \param[in] inertia_msg : Message associated to the topic set by setTopic_toolInertia()
 * that contains the inertial parameters of the mounted tool as considered on
 * Coppeliasim side. The inertia tensor must be specified in flange frame as well
 * as the center of mass.
 * This callback works jointly with the `callback_flMe` to automatically
 * retrieve the kinematic and dynamic parameters of the tool.
 */
void
vpROSRobotFrankaCoppeliasim::callback_toolInertia( const geometry_msgs::Inertia &inertia_msg )
{
  std::lock_guard< std::mutex > lock( m_mutex );
  if ( !m_toolMounted && m_overwrite_toolInertia )
  {
    m_mL       = inertia_msg.m;
    m_Il[0][0] = inertia_msg.ixx;
    m_Il[0][1] = m_Il[1][0] = inertia_msg.ixy;
    m_Il[0][2] = m_Il[2][0] = inertia_msg.ixz;
    m_Il[1][1]              = inertia_msg.iyy;
    m_Il[1][2] = m_Il[2][1] = inertia_msg.iyz;
    m_Il[2][2]              = inertia_msg.izz;

    vpMatrix R(3,3);

    vpColVector eig;
    m_Il.eigenValues( eig, R );

    m_Il = R.t() * m_Il * R;

    m_flMcom = vpHomogeneousMatrix( vpTranslationVector ( inertia_msg.com.x,
    		                                              inertia_msg.com.y,
													      inertia_msg.com.z ),
    		                       ( vpRotationMatrix ) R.t() );


    m_overwrite_toolInertia = false;

    if ( !m_overwrite_flMe && !m_overwrite_toolInertia )
    {
      m_toolMounted = true;
      if ( m_verbose )
      {
        std::cout << "A tool has been mounted on the robot.\n";
        std::cout << "Mass: " << m_mL << " [kg]\n";
        std::cout << "Inertia Tensor in flange frame:\n" << m_Il << " [kg*m^2]\n";
        std::cout << "CoM position in flange frame: " << m_flMcom.getTranslationVector().t() << " [m]\n";
        std::cout << "CoM orientation in flange frame: \n" << m_flMcom.getRotationMatrix() << std::endl;
      }
    }
  }
}

/*!
 * Infinite loop launched when the control is done in position.
 */
void
vpROSRobotFrankaCoppeliasim::positionControlLoop()
{
  if ( m_verbose )
  {
    std::cout << "Position controller thread launched" << std::endl;
  }
  ros::NodeHandlePtr n = boost::make_shared< ros::NodeHandle >();
  ros::Rate loop_rate( 500 ); // Hz
  sensor_msgs::JointState joint_state_cmd_msg;
  joint_state_cmd_msg.velocity.resize( 7 );
  joint_state_cmd_msg.name.resize( 7 );
  joint_state_cmd_msg.header.frame_id = "Joint_velocity_cmd";
  for ( unsigned int i = 0; i < 7; i++ )
  {
    joint_state_cmd_msg.name[i] = "J" + std::to_string( i );
  }
  vpColVector vel_max( 7, 0 ), dq_sat( 7, 0 ), gains( 7, 0 );
  vel_max = { 2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100 };

  vpMatrix Kp( 7, 7 ), Kd( 7, 7 );

  gains = { 6.0, 6.0, 6.0, 6.0, 3.5, 2.5, 2.0 };
  Kp.diag( gains );
  gains = { 0.5, 0.5, 0.5, 0.5, 0.3, 0.25, 0.2 };
  Kd.diag( gains );

  // Publish robot state to the corresponding ROS topic
  std_msgs::Int32 robot_ctrl_type_msg;
  robot_ctrl_type_msg.data = static_cast< int >( m_stateRobot );
  m_pub_robotStateCmd.publish(
      robot_ctrl_type_msg ); // Should be published more than one time to be received by CoppeliaSim !

  m_posControlThreadIsRunning = true;

  bool backup_sync_mode = m_syncModeEnabled;

  if ( backup_sync_mode )
  {
    setCoppeliasimSyncMode( false );
  }

  while ( ros::ok() && !m_posControlThreadStopAsked && m_posControlNewCmd )
  {
    loop_rate.sleep();
    m_pub_robotStateCmd.publish(
        robot_ctrl_type_msg ); // Should be published more than one time to be received by CoppeliaSim !

    m_mutex.lock();
    m_dq_des = Kp * 10. * ( m_q_des - m_q ) - Kd * m_dq;
    dq_sat   = vpRobot::saturateVelocities( m_dq_des, vel_max, false );
    if ( std::sqrt( ( ( 180. / M_PI ) * ( m_q_des - m_q ) ).sumSquare() ) > 0.1 )
    {
      for ( unsigned int i = 0; i < 7; i++ )
      {
        joint_state_cmd_msg.velocity[i] = dq_sat[i];
      }
    }
    else
    {
      for ( unsigned int i = 0; i < 7; i++ )
      {
        joint_state_cmd_msg.velocity[i] = 0;
      }
      m_posControlNewCmd = false;
    }

    m_mutex.unlock();
    m_pub_jointStateCmd.publish( joint_state_cmd_msg );
  }

  for ( unsigned int i = 0; i < 7; i++ )
  {
    joint_state_cmd_msg.velocity[i] = 0.;
  }
  m_pub_jointStateCmd.publish( joint_state_cmd_msg );
  m_posControlThreadIsRunning = false;

  if ( backup_sync_mode )
  {
    setCoppeliasimSyncMode( true );
  }

  if ( m_verbose )
  {
    std::cout << "Position controller thread finished" << std::endl;
  }
}

/*!
 * Infinite loop launched when the control is done in joint velocity.
 */
void
vpROSRobotFrankaCoppeliasim::velocityControlLoop()
{
  if ( m_verbose )
  {
    std::cout << "Velocity controller thread launched" << std::endl;
  }
  ros::NodeHandlePtr n = boost::make_shared< ros::NodeHandle >();
  ros::Rate loop_rate( 500 ); // Hz

  sensor_msgs::JointState joint_state_cmd_msg;
  joint_state_cmd_msg.velocity.resize( 7 );
  joint_state_cmd_msg.name.resize( 7 );
  joint_state_cmd_msg.header.frame_id = "Joint_velocity_cmd";
  for ( unsigned int i = 0; i < 7; i++ )
  {
    joint_state_cmd_msg.name[i] = "J" + std::to_string( i );
  }

  // Publish robot state to the corresponding ROS topic
  std_msgs::Int32 robot_ctrl_type_msg;
  robot_ctrl_type_msg.data = static_cast< int >( m_stateRobot );
  m_pub_robotStateCmd.publish(
      robot_ctrl_type_msg ); // Should be published more than one time to be received by CoppeliaSim !

  vpColVector vel_max( 7, 0 ), dq_sat( 7, 0 );
  vel_max = { 2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100 };

  m_velControlThreadIsRunning = true;

  while ( ros::ok() && !m_velControlThreadStopAsked )
  {
    loop_rate.sleep();
    m_pub_robotStateCmd.publish(
        robot_ctrl_type_msg ); // Should be published more than one time to be received by CoppeliaSim !
    m_mutex.lock();
    dq_sat = vpRobot::saturateVelocities( m_dq_des, vel_max, true );
    for ( unsigned int i = 0; i < 7; i++ )
    {
      joint_state_cmd_msg.velocity[i] = dq_sat[i];
    }
    m_mutex.unlock();
    m_pub_jointStateCmd.publish( joint_state_cmd_msg );
  }

  for ( unsigned int i = 0; i < 7; i++ )
  {
    joint_state_cmd_msg.velocity[i] = 0;
  }
  m_pub_jointStateCmd.publish( joint_state_cmd_msg );
  m_velControlThreadIsRunning = false;

  if ( m_verbose )
  {
    std::cout << "Velocity controller thread finished" << std::endl;
  }
}

/*!
 * Infinite loop launched when the control is done in joint torque.
 */
void
vpROSRobotFrankaCoppeliasim::torqueControlLoop()
{
  if ( m_verbose )
  {
    std::cout << "Torque controller thread launched" << std::endl;
  }
  ros::NodeHandlePtr n = boost::make_shared< ros::NodeHandle >();
  ros::Rate loop_rate( 500 ); // Hz

  sensor_msgs::JointState joint_state_cmd_msg;
  joint_state_cmd_msg.effort.resize( 7 );
  joint_state_cmd_msg.name.resize( 7 );
  joint_state_cmd_msg.header.frame_id = "Joint_torque_cmd";
  for ( unsigned int i = 0; i < 7; i++ )
  {
    joint_state_cmd_msg.name[i] = "J" + std::to_string( i );
  }

  // Publish robot state to the corresponding ROS topic
  std_msgs::Int32 robot_ctrl_type_msg;
  robot_ctrl_type_msg.data = static_cast< int >( m_stateRobot );
  m_pub_robotStateCmd.publish(
      robot_ctrl_type_msg ); // Should be published more than one time to be received by CoppeliaSim !

  vpColVector g( 7, 0 ), tau_max( 7, 0 ), tau_sat( 7, 0 ), f( 7, 0 );
  tau_max = { 87, 87, 87, 87, 12, 12, 12 };
#ifdef VISP_HAVE_IIR
  const int order               = 1;     // ex: 4th order (=2 biquads)
  const double samplingrate     = 1000.; // Hz
  const double cutoff_frequency = 10.;   // Hz
  std::array< Iir::Butterworth::LowPass< order >, 7 > TorqueFilter;
  for ( unsigned int i = 0; i < 7; i++ )
  {
    TorqueFilter[i].setup( samplingrate, cutoff_frequency );
  }
#endif

  m_ftControlThreadIsRunning = true;

  while ( ros::ok() && !m_ftControlThreadStopAsked )
  {
    loop_rate.sleep();
    m_pub_robotStateCmd.publish(
        robot_ctrl_type_msg ); // Should be published more than one time to be received by CoppeliaSim !
    this->getGravity( g );     // as for the real robot, we compensate for the gravity
    this->getFriction( f );    // this is to simulate the dynamic friction

    m_mutex.lock();
    for ( unsigned int i = 0; i < 7; i++ )
    {
#ifdef VISP_HAVE_IIR
      m_tau_J_des_filt[i]           = TorqueFilter[i].filter( m_tau_J_des[i] );
      joint_state_cmd_msg.effort[i] = m_tau_J_des_filt[i] + g[i] - f[i];
#else
      joint_state_cmd_msg.effort[i] = m_tau_J_des[i] + g[i] - f[i];
#endif
      if ( m_verbose && std::abs( joint_state_cmd_msg.effort[i] ) > tau_max[i] )
      {
        std::cout << "Excess torque " << joint_state_cmd_msg.effort[i] << " axis nr. " << i << std::endl;
      }
    }
    m_mutex.unlock();
    m_pub_jointStateCmd.publish( joint_state_cmd_msg );
  }

  for ( unsigned int i = 0; i < 7; i++ )
  {
    joint_state_cmd_msg.effort[i] = 0;
  }
  m_pub_jointStateCmd.publish( joint_state_cmd_msg );
  m_ftControlThreadIsRunning = false;

  if ( m_verbose )
  {
    std::cout << "Torque controller thread finished" << std::endl;
  }
}

/*!
 * Change robot state.
 * \param newState : New robot state.
 * \return The new robot state.
 */
vpRobot::vpRobotStateType
vpROSRobotFrankaCoppeliasim::setRobotState( vpRobot::vpRobotStateType newState )
{
  switch ( newState )
  {
  case vpRobot::STATE_STOP:
  {
    // Start primitive STOP only if the current state is velocity or force/torque
    if ( vpRobot::STATE_POSITION_CONTROL == getRobotState() )
    {
      std::cout << "Change the control mode from position control to stop.\n";
      m_mutex.lock();
      m_posControlThreadStopAsked = true;
      m_mutex.unlock();
    }
    else if ( vpRobot::STATE_VELOCITY_CONTROL == getRobotState() )
    {
      std::cout << "Change the control mode from velocity control to stop.\n";
      m_mutex.lock();
      m_velControlThreadStopAsked = true;
      m_mutex.unlock();
    }
    else if ( vpRobot::STATE_FORCE_TORQUE_CONTROL == getRobotState() )
    {
      std::cout << "Change the control mode from force/torque control to stop.\n";
      m_ftControlThreadStopAsked = true;
    }

    if ( m_controlThread.joinable() )
    {
      m_controlThread.join();
      m_mutex.lock();
      m_posControlThreadStopAsked = false;
      m_posControlThreadIsRunning = false;
      m_velControlThreadStopAsked = false;
      m_velControlThreadIsRunning = false;
      m_ftControlThreadStopAsked  = false;
      m_ftControlThreadIsRunning  = false;
      m_mutex.unlock();
    }
    this->m_stateRobot = newState;
    break;
  }
  case vpRobot::STATE_POSITION_CONTROL:
  {
    if ( vpRobot::STATE_STOP == getRobotState() )
    {
      std::cout << "Change the control mode from stop to position control.\n";
    }
    else if ( vpRobot::STATE_VELOCITY_CONTROL == getRobotState() )
    {
      std::cout << "Change the control mode from velocity to position control.\n";
      // Stop the velocity control loop
      m_mutex.lock();
      m_velControlThreadStopAsked = true;
      m_mutex.unlock();
    }
    else if ( vpRobot::STATE_FORCE_TORQUE_CONTROL == getRobotState() )
    {
      std::cout << "Change the control mode from force/torque to position control.\n";
      // Stop the force control loop
      m_mutex.lock();
      m_ftControlThreadStopAsked = true;
      m_mutex.unlock();
    }
    if ( m_controlThread.joinable() )
    {
      m_controlThread.join();
      m_mutex.lock();
      m_velControlThreadStopAsked = false;
      m_velControlThreadIsRunning = false;
      m_ftControlThreadStopAsked  = false;
      m_ftControlThreadIsRunning  = false;
      m_mutex.unlock();
    }
    //    m_controlThread = std::thread([this] {this->positionControlLoop();});
    this->m_stateRobot = newState;
    break;
  }
  case vpRobot::STATE_VELOCITY_CONTROL:
  {
    if ( vpRobot::STATE_STOP == getRobotState() )
    {
      std::cout << "Change the control mode from stop to velocity control.\n";
    }
    else if ( vpRobot::STATE_POSITION_CONTROL == getRobotState() )
    {
      std::cout << "Change the control mode from position to velocity control.\n";
      m_mutex.lock();
      m_posControlThreadStopAsked = true;
      m_mutex.unlock();
    }
    else if ( vpRobot::STATE_FORCE_TORQUE_CONTROL == getRobotState() )
    {
      std::cout << "Change the control mode from force/torque to velocity control.\n";
      // Stop the force control loop
      m_mutex.lock();
      m_ftControlThreadStopAsked = true;
      m_mutex.unlock();
    }

    if ( getRobotState() != vpRobot::STATE_VELOCITY_CONTROL )
    {
      if ( m_controlThread.joinable() )
      {
        m_controlThread.join();
        m_mutex.lock();
        m_posControlThreadStopAsked = false;
        m_posControlThreadIsRunning = false;
        m_ftControlThreadStopAsked  = false;
        m_ftControlThreadIsRunning  = false;
        m_mutex.unlock();
      }
    }
    if ( !m_velControlThreadIsRunning )
    {
      m_controlThread = std::thread( [this] { this->velocityControlLoop(); } );
    }
    this->m_stateRobot = newState;
    break;
  }
  case vpRobot::STATE_FORCE_TORQUE_CONTROL:
  {
    if ( vpRobot::STATE_STOP == getRobotState() )
    {
      std::cout << "Change the control mode from stop to force/torque control.\n";
    }
    else if ( vpRobot::STATE_POSITION_CONTROL == getRobotState() )
    {
      std::cout << "Change the control mode from position to force/torque control.\n";
      m_posControlThreadStopAsked = true;
    }
    else if ( vpRobot::STATE_VELOCITY_CONTROL == getRobotState() )
    {
      std::cout << "Change the control mode from velocity to force/torque control.\n";
      m_mutex.lock();
      m_velControlThreadStopAsked = true;
      m_mutex.unlock();
    }
    if ( getRobotState() != vpRobot::STATE_FORCE_TORQUE_CONTROL )
    {
      if ( m_controlThread.joinable() )
      {
        m_controlThread.join();
        m_mutex.lock();
        m_posControlThreadStopAsked = false;
        m_posControlThreadIsRunning = false;
        m_velControlThreadStopAsked = false;
        m_velControlThreadIsRunning = false;
        m_mutex.unlock();
      }
    }
    if ( !m_ftControlThreadIsRunning )
    {
      m_controlThread = std::thread( [this] { this->torqueControlLoop(); } );
    }
    this->m_stateRobot = newState;
    break;
  }

  default:
    break;
  }

  return newState;
}

/*!
 * Set robot position. This function is blocking and returns only when the desired
 * position is reached.
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
void
vpROSRobotFrankaCoppeliasim::setPosition( const vpRobot::vpControlFrameType frame, const vpColVector &position )
{
  vpRobotFrankaSim::setPosition( frame, position );

  m_posControlNewCmd = true;

  if ( vpRobot::STATE_POSITION_CONTROL == getRobotState() )
  {
    m_controlThread = std::thread( [this] { this->positionControlLoop(); } );

    if ( m_controlThread.joinable() )
    {
      m_controlThread.join();
    }
  }
  else
  {
    std::cout << "Robot is not in position state control. "
              << "You should call vpROSRobotFrankaCoppeliasim::setRobotState(vpRobot::STATE_POSITION_CONTROL)"
              << std::endl;
  }
}

/*!
 * Wait a given time in [s].
 * \param timestamp_second : Simulation time in [s] used to start the chrono.
 * \param duration_second : Wait the given duration time in [s] since chrono is started
 */
void
vpROSRobotFrankaCoppeliasim::wait( double timestamp_second, double duration_second )
{
  ros::Rate loop_rate( 1000 ); // Hz
  if ( m_syncModeEnabled )
  {
    setCoppeliasimSimulationStepDone( false );

    double epsilon = 0.0001; // 100 ns
    do
    {
      coppeliasimTriggerNextStep();
      while ( !getCoppeliasimSimulationStepDone() )
      {
        loop_rate.sleep();
      }
      setCoppeliasimSimulationStepDone( false );

    } while ( getCoppeliasimSimulationTime() < timestamp_second + duration_second - epsilon );
  }
  else
  {
    while ( getCoppeliasimSimulationTime() < timestamp_second + duration_second )
    {
      loop_rate.sleep();
    }
  }
}
