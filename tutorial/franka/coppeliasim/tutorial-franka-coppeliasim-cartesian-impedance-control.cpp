/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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

//! \example tutorial-franka-coppeliasim-cartesian-impedance-control.cpp

#include <iostream>

#include <visp3/gui/vpPlot.h>
#include <visp_ros/vpROSRobotFrankaCoppeliasim.h>

using namespace std::chrono_literals;

vpMatrix
Ta( const vpHomogeneousMatrix &edMe )
{
  vpMatrix Lx( 6, 6 ), Lw( 3, 3 ), skew_u( 3, 3 );

  Lx.eye();
  vpThetaUVector tu( edMe );

  vpColVector u;
  double theta;

  tu.extract( theta, u );
  skew_u = vpColVector::skew( u );
  Lw.eye();
  if ( theta != 0.0 )
  {
    Lw -= 0.5 * theta * skew_u;
    Lw += ( 1 - ( ( vpMath::sinc( theta ) ) / ( vpMath::sqr( vpMath::sinc( theta * 0.5 ) ) ) ) ) * skew_u * skew_u;
  }

  Lx.insert( Lw, 3, 3 );

  return Lx;
}

int
main( int argc, char **argv )
{
  bool opt_coppeliasim_sync_mode = false;
  bool opt_verbose               = false;
  bool opt_save_data             = false;

  for ( int i = 1; i < argc; i++ )
  {
    if ( std::string( argv[i] ) == "--enable-coppeliasim-sync-mode" )
    {
      opt_coppeliasim_sync_mode = true;
    }
    else if ( std::string( argv[i] ) == "--verbose" || std::string( argv[i] ) == "-v" )
    {
      opt_verbose = true;
    }
    else if ( std::string( argv[i] ) == "--save" )
    {
      opt_save_data = true;
    }
    else if ( std::string( argv[i] ) == "--help" || std::string( argv[i] ) == "-h" )
    {
      std::cout << argv[0] << "[--enable-coppeliasim-sync-mode]"
                << " [--save]"
                << " [--verbose] [-v] "
                << " [--help] [-h]" << std::endl;
      return EXIT_SUCCESS;
    }
  }

  rclcpp::init( argc, argv );
  auto node = std::make_shared< rclcpp::Node >( "frankasim_cart_impedance_ctrl" );
  rclcpp::WallRate loop_rate( 100ms );
  rclcpp::spin_some( node );

  vpROSRobotFrankaCoppeliasim robot;

  try
  {
    robot.setVerbose( opt_verbose );
    robot.connect();

    std::cout << "Coppeliasim sync mode enabled: " << ( opt_coppeliasim_sync_mode ? "yes" : "no" ) << std::endl;
    robot.coppeliasimStopSimulation(); // Allows to reset simulation, moving the robot to initial position
    robot.setCoppeliasimSyncMode( false );
    robot.coppeliasimStartSimulation();

    // Move to a secure initial position
    vpColVector q_init( { 0, vpMath::rad( -45 ), 0, vpMath::rad( -135 ), 0, vpMath::rad( 90 ), vpMath::rad( 45 ) } );

    robot.setRobotState( vpRobot::STATE_POSITION_CONTROL );
    robot.setPosition( vpRobot::JOINT_STATE, q_init );
    vpTime::wait( 500 );

    vpPlot *plotter = nullptr;

    plotter = new vpPlot( 4, 800, 800, 10, 10, "Real time curves plotter" );
    plotter->setTitle( 0, "Joint positions [rad]" );
    plotter->initGraph( 0, 7 );
    plotter->setLegend( 0, 0, "q1" );
    plotter->setLegend( 0, 1, "q2" );
    plotter->setLegend( 0, 2, "q3" );
    plotter->setLegend( 0, 3, "q4" );
    plotter->setLegend( 0, 4, "q5" );
    plotter->setLegend( 0, 5, "q6" );
    plotter->setLegend( 0, 6, "q7" );

    plotter->setTitle( 1, "Joint torques measure [Nm]" );
    plotter->initGraph( 1, 7 );
    plotter->setLegend( 1, 0, "Tau1" );
    plotter->setLegend( 1, 1, "Tau2" );
    plotter->setLegend( 1, 2, "Tau3" );
    plotter->setLegend( 1, 3, "Tau4" );
    plotter->setLegend( 1, 4, "Tau5" );
    plotter->setLegend( 1, 5, "Tau6" );
    plotter->setLegend( 1, 6, "Tau7" );

    plotter->setTitle( 2, "Cartesian EE pose error [m] - [rad]" );
    plotter->initGraph( 2, 6 );
    plotter->setLegend( 2, 0, "e_x" );
    plotter->setLegend( 2, 1, "e_y" );
    plotter->setLegend( 2, 2, "e_z" );
    plotter->setLegend( 2, 3, "e_tu_x" );
    plotter->setLegend( 2, 4, "e_tu_y" );
    plotter->setLegend( 2, 5, "e_tu_z" );

    plotter->setTitle( 3, "Pose error norm [m] - [rad]" );
    plotter->initGraph( 3, 2 );
    plotter->setLegend( 3, 0, "||e_p||" );
    plotter->setLegend( 3, 1, "||e_o||" );

    vpColVector q( 7, 0 ), dq( 7, 0 ), tau_d( 7, 0 ), C( 7, 0 ), F( 7, 0 ), tau_d0( 7, 0 ), tau_cmd( 7, 0 ),
        x_e( 6, 0 ), dx_e( 6, 0 ), dx_ed( 6, 0 ), ddx_ed( 6, 0 );
    vpMatrix fJe( 6, 7 ), Ja( 6, 7 ), dJa( 6, 7 ), Ja_old( 6, 7 ), B( 7, 7 ), I7( 7, 7 ), Ja_pinv_B_t( 6, 7 );
    vpColVector pose_err_norm( 2, 0 ), tau( 7, 0 );

    std::cout << "Reading current joint position" << std::endl;
    robot.getPosition( vpRobot::JOINT_STATE, q );
    std::cout << "Initial joint position: " << q.t() << std::endl;

    robot.setRobotState( vpRobot::STATE_FORCE_TORQUE_CONTROL );
    robot.setCoppeliasimSyncMode( opt_coppeliasim_sync_mode );
    vpTime::wait( 500 );

    vpHomogeneousMatrix fMed, fMed0;
    fMed0 = robot.get_fMe();
    fMed  = fMed0;

    bool final_quit       = false;
    bool first_time       = false;
    bool start_trajectory = false;

    vpMatrix K( 6, 6 ), D( 6, 6 ), edVf( 6, 6 );

    double wp = 50;
    double wo = 20;
    K.diag( { wp * wp, wp * wp, wp * wp, wo * wo, wo * wo, wo * wo } );
    D.diag( { 2 * wp, 2 * wp, 2 * wp, 2 * wo, 2 * wo, 2 * wo } );
    I7.eye();

    double mu = 4;
    double dt = 0;

    double time_cur;
    double time_prev = robot.getCoppeliasimSimulationTime();
    double time_start_trajectory = time_prev;
    double delay_before_trajectory = 0.5; // Start sinusoidal joint trajectory after this delay in [s]

    // Control loop
    while ( !final_quit )
    {
      time_cur = robot.getCoppeliasimSimulationTime();

      robot.getPosition( vpRobot::JOINT_STATE, q );
      robot.getVelocity( vpRobot::JOINT_STATE, dq );
      robot.getMass( B );
      robot.getCoriolis( C );
      robot.getFriction( F );
      robot.get_fJe( fJe );
      robot.getForceTorque( vpRobot::JOINT_STATE, tau );

      if ( time_cur < delay_before_trajectory )
      {
        time_start_trajectory = time_cur; // To ensure exp() = 1
        first_time            = true;
      }
      else if ( !start_trajectory ) // After the delay we start joint trajectory
      {
        time_start_trajectory = time_cur;
        start_trajectory      = true;
      }

      // Compute Cartesian trajectories
      // clang-format off
      fMed[1][3] = fMed0[1][3] + ( start_trajectory ? ( 0.1 * sin( 2 * M_PI * 0.3 * ( time_cur - time_start_trajectory  ) ) ) : 0 );
      fMed[2][3] = fMed0[2][3] - ( start_trajectory ? ( 0.05 * sin( 2 * M_PI * 0.6 * ( time_cur - time_start_trajectory  ) ) ) : 0) ;

      dx_ed[1] = ( start_trajectory ? (2 * M_PI * 0.3 * 0.1 * cos( 2 * M_PI * 0.3 * ( time_cur - time_start_trajectory  ) ) ) : 0) ;
      dx_ed[2] = ( start_trajectory ? (-2 * M_PI * 0.6 * 0.05 * cos( 2 * M_PI * 0.6 * ( time_cur - time_start_trajectory  ) ) ) : 0) ;

      ddx_ed[1] = ( start_trajectory ? (-2 * M_PI * 0.3 * 2 * M_PI * 0.3 * 0.1 * sin( 2 * M_PI * 0.3 * ( time_cur - time_start_trajectory  ) ) ) : 0) ;
      ddx_ed[2] = ( start_trajectory ? (2 * M_PI * 0.6 * 2 * M_PI * 0.6 * 0.05 * sin( 2 * M_PI * 0.6 * ( time_cur - time_start_trajectory  ) ) ) : 0) ;
      // clang-format on

      edVf.insert( fMed.getRotationMatrix().t(), 0, 0 );
      edVf.insert( fMed.getRotationMatrix().t(), 3, 3 );

      x_e = (vpColVector)vpPoseVector( fMed.inverse() * robot.get_fMe() ); // edMe
      Ja  = Ta( fMed.inverse() * robot.get_fMe() ) * edVf * fJe;

      dx_e = Ta( fMed.inverse() * robot.get_fMe() ) * edVf * ( dx_ed - fJe * dq );

      dt = time_cur - time_prev;

      if ( dt != 0 )
      {
        dJa = ( Ja - Ja_old ) / dt;
      }
      else
      {
        dJa = 0;
      }
      Ja_old = Ja;

      Ja_pinv_B_t = ( Ja * B.inverseByCholesky() * Ja.t() ).inverseByCholesky() * Ja * B.inverseByCholesky();

      // Compute the control law
      tau_d = B * Ja.pseudoInverse() * ( -K * ( x_e ) + D * (dx_e)-dJa * dq + ddx_ed ) + C + F -
              ( I7 - Ja.t() * Ja_pinv_B_t ) * B * dq * 100;

      if ( first_time )
      {
        tau_d0 = tau_d;
      }

      tau_cmd = tau_d - tau_d0 * std::exp( -mu * ( time_cur - time_start_trajectory ) );

      robot.setForceTorque( vpRobot::JOINT_STATE, tau_cmd );

      plotter->plot( 0, time_cur, q );
      plotter->plot( 1, time_cur, tau );
      plotter->plot( 2, time_cur, x_e );
      pose_err_norm[0] = sqrt( x_e.extract( 0, 3 ).sumSquare() );
      pose_err_norm[1] = sqrt( x_e.extract( 3, 3 ).sumSquare() );
      plotter->plot( 3, time_cur, pose_err_norm );

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
      }

      if ( opt_verbose )
      {
        std::cout << "dt: " << dt << std::endl;
      }

      time_prev = time_cur;
      robot.wait( time_cur, 0.001 ); // Sync loop at 1000 Hz (1 ms)
    }                                // end while

    if ( opt_save_data )
    {
      plotter->saveData( 0, "sim-cart-joint-position.txt", "# " );
      plotter->saveData( 1, "sim-cart-joint-torques.txt", "# " );
      plotter->saveData( 2, "sim-cart-pose-error.txt", "# " );
      plotter->saveData( 3, "sim-cart-pose-error-norm.txt", "# " );
    }

    if ( plotter != nullptr )
    {
      delete plotter;
      plotter = nullptr;
    }
    robot.coppeliasimStopSimulation();
  }
  catch ( const vpException &e )
  {
    std::cout << "ViSP exception: " << e.what() << std::endl;
    std::cout << "Stop the robot " << std::endl;
    robot.setRobotState( vpRobot::STATE_STOP );
    return EXIT_FAILURE;
  }

  return 0;
}
