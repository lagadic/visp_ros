/****************************************************************************
 *
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
 *****************************************************************************/

//! \example test-franka-coppeliasim-controller.cpp

#include <iostream>
#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include <visp_ros/vpROSRobotFrankaCoppeliasim.h>

using namespace std::chrono_literals;

static bool s_simStepDone = true;
static std::mutex s_mutex_ros;
static float s_simTime = 0;

void
simStepDone_callback( const std_msgs::msg::Bool &msg )
{
  std::lock_guard< std::mutex > lock( s_mutex_ros );
  s_simStepDone = msg.data;
}

void
simTime_callback( const std_msgs::msg::Float32 &msg )
{
  std::lock_guard< std::mutex > lock( s_mutex_ros );
  s_simTime = msg.data;
}

int
main( int argc, char **argv )
{
  try
  {
    rclcpp::init( argc, argv );
    auto node = std::make_shared< rclcpp::Node >( "visp_ros" );
    rclcpp::WallRate loop_rate( 100ms );
    rclcpp::spin_some( node );

    vpROSRobotFrankaCoppeliasim robot;
    robot.setVerbose( true );
    robot.setTopicJointState( "/coppeliasim/franka/joint_state" );
    robot.setTopic_eMc( "/coppeliasim/franka/eMc" );
    robot.connect();

    const auto enableSyncMode_pub  = node->create_publisher< std_msgs::msg::Bool >( "/enableSyncMode", 1 );
    const auto startSimTrigger_pub = node->create_publisher< std_msgs::msg::Bool >( "/startSimulation", 1 );
    const auto stopSimTrigger_pub  = node->create_publisher< std_msgs::msg::Bool >( "/stopSimulation", 1 );

    std_msgs::msg::Bool trigger;
    std_msgs::msg::Bool syncMode;
    std_msgs::msg::Bool startStopSim;

    std::string simulationStepDone_topic_name = "/simulationStepDone";
    std::cout << "Subscribe to " << simulationStepDone_topic_name << std::endl;

    // auto sub_simStepDone = node->create_subscription< std_msgs::msg::Bool >(
    //     simulationStepDone_topic_name, 1, std::bind( &simStepDone_callback, node, std::placeholders::_1 ) );
    auto sub_simStepDone =
        node->create_subscription< std_msgs::msg::Bool >( simulationStepDone_topic_name, 1, simStepDone_callback );

    std::string simulationTime_topic_name = "/simulationTime";
    std::cout << "Subscribe to " << simulationTime_topic_name << std::endl;
    // auto sub_simulationTime = node->create_subscription< std_msgs::msg::Float32 >(
    //     simulationTime_topic_name, 1, std::bind( &simTime_callback, node, std::placeholders::_1 ) );
    auto sub_simulationTime =
        node->create_subscription< std_msgs::msg::Float32 >( simulationTime_topic_name, 1, simTime_callback );

    startStopSim.data = true;
    startSimTrigger_pub->publish( startStopSim );
    vpTime::wait( 1000 );
    stopSimTrigger_pub->publish( startStopSim );
    vpTime::wait( 1000 );
    syncMode.data = true;
    enableSyncMode_pub->publish( trigger );
    startSimTrigger_pub->publish( startStopSim );
    vpTime::wait( 1000 );

    robot.setRobotState( vpRobot::STATE_VELOCITY_CONTROL );

    double t_start     = vpTime::measureTimeSecond();
    double t_init      = vpTime::measureTimeMs();
    double t_init_prev = t_init;

    float t_simTime_start = 0, t_simTime = 0, t_simTime_prev = 0;

    s_mutex_ros.lock();
    t_simTime_start = t_simTime_prev = s_simTime;
    s_mutex_ros.unlock();
    t_init_prev = vpTime::measureTimeMs();

    vpColVector q_init, q_final;

    robot.getPosition( vpRobot::JOINT_STATE, q_init );
    std::cout << "q initial: " << 180. / M_PI * q_init.t() << " deg" << std::endl;

    vpColVector qdot( 7, 0 );
    qdot[0] = vpMath::rad( 10 );

    std::cout << "Apply joint velocity to FrankaSim (deg/s): " << ( qdot * 180. / M_PI ).t() << std::endl;

    while ( vpTime::measureTimeSecond() - t_start < 10 )
    {
      rclcpp::spin_some( node );
      t_init = vpTime::measureTimeMs();
      s_mutex_ros.lock();
      t_simTime = s_simTime;
      s_mutex_ros.unlock();

      // Apply 10 deg/s vel on joint 1
      robot.setVelocity( vpRobot::JOINT_STATE, qdot );

      vpTime::sleepMs( 50 );

      std::stringstream ss;
      ss << "Loop time: " << t_init - t_init_prev << " - " << t_simTime - t_simTime_prev << " ms";
      //      std::cout << ss.str() << std::endl;
      t_init_prev    = t_init;
      t_simTime_prev = t_simTime;
    }
    std::cout << "Elapsed time: " << vpTime::measureTimeSecond() - t_start << " seconds - "
              << t_simTime - t_simTime_start << std::endl;

    robot.getPosition( vpRobot::JOINT_STATE, q_final );
    std::cout << "q final: " << 180 / M_PI * q_final.t() << " deg" << std::endl;
    std::cout << "Joint displacement: " << 180. / M_PI * ( q_final - q_init ).t() << " deg" << std::endl;

    stopSimTrigger_pub->publish( startStopSim );
  }
  catch ( const vpException &e )
  {
    std::cout << "ViSP exception: " << e.what() << std::endl;
    std::cout << "Stop the robot " << std::endl;
    return EXIT_FAILURE;
  }

  return 0;
}
