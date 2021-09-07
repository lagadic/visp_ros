/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2021 by Inria. All rights reserved.
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

//! \example tutorial-franka-real-joint-impedance-control.cpp

#include <iostream>

#include <visp3/gui/vpPlot.h>
#include <visp3/robot/vpRobotFranka.h>

vpColVector
sign( const vpColVector &v )
{

  vpColVector s( v.size(), 0 );
  for ( size_t i = 0; i < v.size(); i++ )
  {
    if ( v[i] >= 0 )
    {
      s[i] = 1;
    }
    else
    {
      s[i] = -1;
    }
  }
  return s;
}

int
main( int argc, char **argv )
{
  std::string opt_robot_ip = "192.168.1.1";

  for ( int i = 1; i < argc; i++ )
  {
    if ( std::string( argv[i] ) == "--ip" && i + 1 < argc )
    {
      opt_robot_ip = std::string( argv[i + 1] );
    }
    else if ( std::string( argv[i] ) == "--help" || std::string( argv[i] ) == "-h" )
    {
      std::cout << argv[0] << " [--ip <default " << opt_robot_ip << ">]"
                << " [--help] [-h] " << std::endl;
      return EXIT_SUCCESS;
    }
  }

  vpRobotFranka robot;
  std::cout << "ip: " << opt_robot_ip << std::endl;
  try
  {
    robot.connect( opt_robot_ip );

    vpColVector q_init( { 0, vpMath::rad( -45 ), 0, vpMath::rad( -135 ), 0, vpMath::rad( 90 ), vpMath::rad( 45 ) } );

    robot.setRobotState( vpRobot::STATE_POSITION_CONTROL );
    robot.setPosition( vpRobot::JOINT_STATE, q_init );
    vpTime::wait( 500 );

    vpPlot *plotter = nullptr;

    plotter = new vpPlot( 4, 800, 800, 10, 10, "Real time curves plotter" );
    plotter->setTitle( 0, "Joint Positions [rad]" );
    plotter->initGraph( 0, 7 );
    plotter->setLegend( 0, 0, "q1" );
    plotter->setLegend( 0, 1, "q2" );
    plotter->setLegend( 0, 2, "q3" );
    plotter->setLegend( 0, 3, "q4" );
    plotter->setLegend( 0, 4, "q5" );
    plotter->setLegend( 0, 5, "q6" );
    plotter->setLegend( 0, 6, "q7" );

    plotter->setTitle( 1, "Joint position error [rad]" );
    plotter->initGraph( 1, 7 );
    plotter->setLegend( 1, 0, "e_q1" );
    plotter->setLegend( 1, 1, "e_q2" );
    plotter->setLegend( 1, 2, "e_q3" );
    plotter->setLegend( 1, 3, "e_q4" );
    plotter->setLegend( 1, 4, "e_q5" );
    plotter->setLegend( 1, 5, "e_q6" );
    plotter->setLegend( 1, 6, "e_q7" );

    plotter->setTitle( 2, "Joint torque command [Nm]" );
    plotter->initGraph( 2, 7 );
    plotter->setLegend( 2, 0, "Tau1" );
    plotter->setLegend( 2, 1, "Tau2" );
    plotter->setLegend( 2, 2, "Tau3" );
    plotter->setLegend( 2, 3, "Tau4" );
    plotter->setLegend( 2, 4, "Tau5" );
    plotter->setLegend( 2, 5, "Tau6" );
    plotter->setLegend( 2, 6, "Tau7" );

    plotter->setTitle( 3, "Joint error norm [deg]" );
    plotter->initGraph( 3, 1 );
    plotter->setLegend( 3, 0, "||qd - d||" );

    // Create joint array
    vpColVector q( 7, 0 ), qd( 7, 0 ), dq( 7, 0 ), dqd( 7, 0 ), ddqd( 7, 0 ), tau_d( 7, 0 ), C( 7, 0 ), q0( 7, 0 ),
        F( 7, 0 ), tau_d0( 7, 0 ), tau_cmd( 7, 0 );
    vpMatrix B( 7, 7 );

    std::cout << "Reading current joint position" << std::endl;
    robot.getPosition( vpRobot::JOINT_STATE, q0 );
    std::cout << "Initial joint position: " << q0.t() << std::endl;

    robot.setRobotState( vpRobot::STATE_FORCE_TORQUE_CONTROL );
    qd = q0;

    bool final_quit = false;
    bool send_cmd   = true;
    bool restart    = false;
    bool first_time = true;

    vpMatrix K( 7, 7 ), D( 7, 7 ), I( 7, 7 );
    K.diag( { 400.0, 400.0, 400.0, 400.0, 400.0, 400.0, 900.0 } );
    D.diag( { 20.0, 45.0, 45.0, 45.0, 45.0, 45.0, 60.0 } );
    I.diag( { 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 60.0 } );

    vpColVector integral( 7, 0 );
    vpColVector eps( 7, 0 ), P( 7, 0.1 ), Kt( 7, 0 ), delt( 7, 0.5 ), G( 7, 0 ), tau_J( 7, 0 ), sig( 7, 0 );

    double time       = vpTime::measureTimeSecond();
    double time_start = time;
    double time_prev  = time;

    double c_time = 0.0;
    double mu     = 0.01;
    double dt     = 0;

    while ( !final_quit )
    {
      time = vpTime::measureTimeSecond();

      robot.getPosition( vpRobot::JOINT_STATE, q );
      robot.getVelocity( vpRobot::JOINT_STATE, dq );
      robot.getMass( B );
      robot.getCoriolis( C );
      //      robot.getFriction( F );

      if ( first_time )
      {
        time_start = time;
        first_time = false;
      }
      // Compute joint trajectories
      qd[0]   = q0[0] + std::sin( 2 * M_PI * 0.1 * ( time - time_start ) );
      dqd[0]  = 2 * M_PI * 0.1 * std::cos( 2 * M_PI * 0.1 * ( time - time_start ) );
      ddqd[0] = -std::pow( 2 * 0.1 * M_PI, 2 ) * std::sin( 2 * M_PI * 0.1 * ( time - time_start ) );

      qd[2]   = q0[2] + ( M_PI / 16 ) * std::sin( 2 * M_PI * 0.2 * ( time - time_start ) );
      dqd[2]  = M_PI * ( M_PI / 8 ) * 0.2 * std::cos( 2 * M_PI * 0.2 * ( time - time_start ) );
      ddqd[2] = -M_PI * M_PI * 0.2 * ( M_PI / 4 ) * std::sin( 2 * M_PI * 0.2 * ( time - time_start ) );

      qd[3]   = q0[3] + 0.25 * std::sin( 2 * M_PI * 0.05 * ( time - time_start ) );
      dqd[3]  = 2 * M_PI * 0.05 * 0.25 * std::cos( 2 * M_PI * 0.05 * ( time - time_start ) );
      ddqd[3] = -0.25 * std::pow( 2 * 0.05 * M_PI, 2 ) * std::sin( 2 * M_PI * 0.05 * ( time - time_start ) );

      //      qd[6]   = q0[6] + std::sin( 2 * M_PI * 0.1 * ( time - time_start ) );
      //      dqd[6]  = 2 * M_PI * 0.1 * std::cos( 2 * M_PI * 0.1 * ( time - time_start ) );
      //      ddqd[6] = -std::pow( 2 * 0.1 * M_PI, 2 ) * std::sin( 2 * M_PI * 0.1 * ( time - time_start ) );

      integral += ( qd - q ) * dt;

      // Compute the control law
      tau_d = B * ( K * ( qd - q ) + D * ( dqd - dq ) + I * ( integral ) + ddqd ) + C + F;

      if ( !send_cmd )
      {
        tau_cmd = 0; // Stop the robot
        restart = true;
      }
      else
      {
        if ( restart )
        {
          c_time  = time;
          tau_d0  = tau_d;
          restart = false;
        }
        tau_cmd = tau_d - tau_d0 * std::exp( -mu * ( time - c_time ) );
      }

      // Filter to compensate for static friction
      {
        robot.getForceTorque( vpRobot::JOINT_STATE, tau_J );
        robot.getGravity( G );
        vpColVector aux( 7, 0 );
        sig = sign( dq );
        for ( size_t i = 0; i < 7; i++ )
        {
          eps[i] = ( tau_cmd[i] + G[i] - tau_J[i] ) - sig[i] * delt[i];
          Kt[i]  = ( P[i] * sig[i] ) / ( 1 + P[i] );
          delt[i] += Kt[i] * eps[i];
          P[i] += -Kt[i] * P[i] * sig[i];

          aux[i] = sig[i] * delt[i];
          tau_cmd[i] += aux[i];
        }
      }

      // Send command to the torque robot
      robot.setForceTorque( vpRobot::JOINT_STATE, tau_cmd );

      vpColVector norm( 1, vpMath::deg( std::sqrt( ( qd - q ).sumSquare() ) ) );
      plotter->plot( 0, time - time_start, q );
      plotter->plot( 1, time - time_start, qd - q );
      plotter->plot( 2, time - time_start, tau_cmd );
      plotter->plot( 3, time - time_start, norm );

      vpMouseButton::vpMouseButtonType button;
      if ( vpDisplay::getClick( plotter->I, button, false ) )
      {
        if ( button == vpMouseButton::button3 )
        {
          final_quit = true;
          tau_cmd    = 0;
          std::cout << "Stop the robot " << std::endl;
          robot.setRobotState( vpRobot::STATE_STOP );
        }
        if ( button == vpMouseButton::button1 )
        {
          send_cmd = !send_cmd;
        }
      }

      dt        = time - time_prev;
      time_prev = time;
      vpTime::wait( time / 1000., 1 ); // Sync loop at 1000 Hz (1 ms)
    }

    if ( plotter != nullptr )
    {
      delete plotter;
      plotter = nullptr;
    }
  }
  catch ( const vpException &e )
  {
    std::cout << "ViSP exception: " << e.what() << std::endl;
    std::cout << "Stop the robot " << std::endl;
    robot.setRobotState( vpRobot::STATE_STOP );
    return EXIT_FAILURE;
  }
  catch ( const franka::NetworkException &e )
  {
    std::cout << "Franka network exception: " << e.what() << std::endl;
    std::cout << "Check if you are connected to the Franka robot"
              << " or if you specified the right IP using --ip command line option set by default to 192.168.1.1. "
              << std::endl;
    return EXIT_FAILURE;
  }
  catch ( const std::exception &e )
  {
    std::cout << "Franka exception: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  return 0;
}
