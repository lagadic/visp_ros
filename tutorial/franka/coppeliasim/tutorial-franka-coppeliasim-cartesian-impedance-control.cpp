/*
 * tutorial-franka-coppeliasim-cartesian-impedance-control.cpp
 *
 *  Created on: Aug 31, 2021
 *      Author: oliva
 */

#include <iostream>

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpPlot.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/visual_features/vpFeatureThetaU.h>
#include <visp3/visual_features/vpFeatureTranslation.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>

#include <visp_ros/vpROSGrabber.h>
#include <visp_ros/vpROSRobotFrankaCoppeliasim.h>

#include "geometry_msgs/WrenchStamped.h"

void
display_point_trajectory( const vpImage< unsigned char > &I, const std::vector< vpImagePoint > &vip,
                          std::vector< vpImagePoint > *traj_vip )
{
  for ( size_t i = 0; i < vip.size(); i++ )
  {
    if ( traj_vip[i].size() )
    {
      // Add the point only if distance with the previous > 1 pixel
      if ( vpImagePoint::distance( vip[i], traj_vip[i].back() ) > 1. )
      {
        traj_vip[i].push_back( vip[i] );
      }
    }
    else
    {
      traj_vip[i].push_back( vip[i] );
    }
  }
  for ( size_t i = 0; i < vip.size(); i++ )
  {
    for ( size_t j = 1; j < traj_vip[i].size(); j++ )
    {
      vpDisplay::displayLine( I, traj_vip[i][j - 1], traj_vip[i][j], vpColor::green, 2 );
    }
  }
}

vpMatrix Ta(const vpHomogeneousMatrix &edMe){
  vpMatrix Lx(6,6), Lw(3,3), skew_u(3,3);

  Lx.eye();
  vpThetaUVector tu(edMe);

  vpColVector u;
  double theta;

  tu.extract(theta, u);
  skew_u = vpColVector::skew(u);
  Lw.eye();
  if(theta != 0.0){
    Lw -= 0.5*theta*skew_u;
    Lw += (1-((vpMath::sinc(theta))/(vpMath::sqr(vpMath::sinc(theta*0.5)))))*skew_u*skew_u;
  }


  Lx.insert(Lw, 3, 3);

  return Lx;
}


vpMatrix compute_interaction_matrix_3D(const vpHomogeneousMatrix &cdMc){
  vpMatrix Lx(6,6), Lw(3,3), skew_u(3,3);

  vpRotationMatrix cdRc(cdMc);
  vpThetaUVector tu(cdMc);

  vpColVector u;
  double theta;

  tu.extract(theta, u);
  skew_u = vpColVector::skew(u);
  Lw.eye();
  Lw += 0.5*theta*skew_u;
  Lw += (1-((vpMath::sinc(theta))/(vpMath::sqr(vpMath::sinc(theta*0.5)))))*skew_u*skew_u;

  Lx.insert(cdRc, 0, 0);
  Lx.insert(Lw, 3, 3);

  return Lx;
}

int
main( int argc, char **argv )
{

  bool opt_verbose               = false;
  bool opt_coppeliasim_sync_mode = false;

  for ( int i = 1; i < argc; i++ )
  {
    if ( std::string( argv[i] ) == "--verbose" || std::string( argv[i] ) == "-v" )
    {
      opt_verbose = true;
    }
    else if ( std::string( argv[i] ) == "--enable-coppeliasim-sync-mode" )
    {
      opt_coppeliasim_sync_mode = true;
    }
    else if ( std::string( argv[i] ) == "--help" || std::string( argv[i] ) == "-h" )
    {
      std::cout << argv[0] << "[--enable-coppeliasim-sync-mode] "
                << "[--verbose] [-v] "
                << "[--help] [-h]" << std::endl;
      ;
      return EXIT_SUCCESS;
    }
  }

  try
  {
    //------------------------------------------------------------------------//
    //------------------------------------------------------------------------//
    // ROS node
    ros::init( argc, argv, "visp_ros" );
    ros::NodeHandlePtr n = boost::make_shared< ros::NodeHandle >();
    ros::Rate loop_rate( 1000 );
    ros::spinOnce();

    vpROSRobotFrankaCoppeliasim robot;
    robot.setVerbose( opt_verbose );
    robot.connect();

    std::cout << "Coppeliasim sync mode enabled: " << ( opt_coppeliasim_sync_mode ? "yes" : "no" ) << std::endl;
    robot.coppeliasimStopSimulation(); // Allows to reset simulation, moving the robot to initial position
    robot.setCoppeliasimSyncMode( false );
    robot.coppeliasimStartSimulation();


    vpPlot *plotter = nullptr;

    plotter = new vpPlot( 4, 800, 800, 10, 10, "Real time curves plotter" );
    plotter->setTitle( 0, "EE Pose [m] - [rad]" );
    plotter->initGraph( 0, 6 );
    plotter->setLegend( 0, 0, "x" );
    plotter->setLegend( 0, 1, "y" );
    plotter->setLegend( 0, 2, "z" );
    plotter->setLegend( 0, 3, "tu_x" );
    plotter->setLegend( 0, 4, "tu_y" );
    plotter->setLegend( 0, 5, "tu_z" );

    plotter->setTitle( 1, "EE pose error [m] - [rad]" );
    plotter->initGraph( 1, 6 );
    plotter->setLegend( 1, 0, "e_x" );
    plotter->setLegend( 1, 1, "e_y" );
    plotter->setLegend( 1, 2, "e_z" );
    plotter->setLegend( 1, 3, "e_tu_x" );
    plotter->setLegend( 1, 4, "e_tu_y" );
    plotter->setLegend( 1, 5, "e_tu_z" );

    plotter->setTitle( 2, "Joint torque command [Nm]" );
    plotter->initGraph( 2, 7 );
    plotter->setLegend( 2, 0, "Tau1" );
    plotter->setLegend( 2, 1, "Tau2" );
    plotter->setLegend( 2, 2, "Tau3" );
    plotter->setLegend( 2, 3, "Tau4" );
    plotter->setLegend( 2, 4, "Tau5" );
    plotter->setLegend( 2, 5, "Tau6" );
    plotter->setLegend( 2, 6, "Tau7" );

    plotter->setTitle( 3, "Pose error norm [m] - [rad]" );
    plotter->initGraph( 3, 2 );
    plotter->setLegend( 3, 0, "||e_p||" );
    plotter->setLegend( 3, 1, "||e_o||" );


    bool final_quit                           = false;
    bool send_cmd                             = false;
    bool restart                              = false;
    std::vector< vpImagePoint > *traj_corners = nullptr; // To memorize point trajectory

    double sim_time             = robot.getCoppeliasimSimulationTime();
    double sim_time_prev        = sim_time;
    double sim_time_start       = sim_time;
    double dt                   = 0.0;


    vpMatrix K( 6, 6 ), D( 6, 6 );
    double wp = 50;
    double wo = 20;
    K.diag({wp*wp,wp*wp,wp*wp,wo*wo,wo*wo,wo*wo});
    D.diag({2*wp,2*wp,2*wp,2*wo,2*wo,2*wo});

    double c_time = 0.0;
    double mu     = 2.0;


    vpColVector dq( 7, 0 ), tau_d( 7, 0 ), C( 7, 0 ),F( 7, 0 ), tau_d0( 7, 0 ),
    		    tau_cmd( 7, 0 ), x_e( 6, 0 ), dx_e( 6, 0 ), dx_ed( 6, 0 ), ddx_ed( 6, 0 );
    vpMatrix J( 6, 7 ), Ja( 6, 7 ), dJa( 6, 7 ), Ja_old( 6, 7 ), B( 7, 7 ),
    		 I7( 7, 7 ), Ja_pinv_B_t( 6, 7 );

    I7.eye();

    robot.setRobotState( vpRobot::STATE_FORCE_TORQUE_CONTROL);
    robot.setCoppeliasimSyncMode( opt_coppeliasim_sync_mode );


    vpHomogeneousMatrix wMed, wMed0;
    wMed0 = robot.get_fMe();
    wMed = wMed0;


    // control loop
    while ( !final_quit )
    {
		sim_time = robot.getCoppeliasimSimulationTime();
		dt = sim_time - sim_time_prev;
		sim_time_prev = sim_time;
		if(opt_verbose){
			std::cout << "dt: " << dt<< std::endl;
		}


		robot.getVelocity( vpRobot::JOINT_STATE, dq );
		robot.getMass( B );
		robot.getCoriolis( C );
		robot.getFriction( F );
		robot.get_fJe(J);


	    wMed[1][3] = wMed0[1][3] + 0.1*sin(2*M_PI*0.3*(sim_time - sim_time_start)) ;
	    wMed[2][3] = wMed0[2][3] - 0.05*sin(2*M_PI*0.6*(sim_time - sim_time_start)) ;

	    dx_ed[1] =  2*M_PI*0.3*0.1*cos(2*M_PI*0.3*(sim_time - sim_time_start));
	    dx_ed[2] = -2*M_PI*0.6*0.05*cos(2*M_PI*0.6*(sim_time - sim_time_start));

	    ddx_ed[1] = -2*M_PI*0.3*2*M_PI*0.3*0.1*sin(2*M_PI*0.3*(sim_time - sim_time_start));
	    ddx_ed[2] =  2*M_PI*0.6*2*M_PI*0.6*0.05*sin(2*M_PI*0.6*(sim_time - sim_time_start));


        vpMatrix RR(6,6);
        RR.insert(wMed.getRotationMatrix().t(), 0, 0);
        RR.insert(wMed.getRotationMatrix().t(), 3, 3);

        x_e = (vpColVector)vpPoseVector(wMed.inverse() * robot.get_fMe());
        Ja = Ta(wMed.inverse() * robot.get_fMe()) * RR * J ;

        dx_e = Ta(wMed.inverse()*robot.get_fMe()) * RR * (dx_ed-J*dq);

        if(dt != 0){
      	  dJa = (Ja - Ja_old)/dt;
        }else{
      	  dJa = 0;
        }
        Ja_old = Ja;

        Ja_pinv_B_t = (Ja*B.inverseByCholesky()*Ja.t()).inverseByCholesky()*Ja*B.inverseByCholesky();

        // Compute the control law
        tau_d = B*Ja.pseudoInverse()* ( - K * ( x_e ) + D * ( dx_e ) - dJa*dq + ddx_ed)
      		  + C + F - (I7 - Ja.t()*Ja_pinv_B_t)*B*dq*100;


        if (!send_cmd) {
      	  tau_cmd = 0;
      	  restart = true;
        }else{
      	  if(restart){
      		  c_time = sim_time;
      		  tau_d0 = tau_d;
      		  restart = false;
      	  }
      	tau_cmd = tau_d - tau_d0*std::exp(-mu*(sim_time - c_time));
        }

        robot.setForceTorque( vpRobot::JOINT_STATE, tau_cmd );

        plotter->plot( 0, sim_time, (vpColVector)vpPoseVector(robot.get_fMe()));
        plotter->plot( 1, sim_time, x_e );
        plotter->plot( 2, sim_time, tau_cmd );
        vpColVector aux(2,0);
        aux[0] = sqrt( x_e.extract(0, 3).sumSquare() ) ;
        aux[1] = sqrt( x_e.extract(3, 3).sumSquare() );
        plotter->plot( 3, sim_time,  aux );

        vpMouseButton::vpMouseButtonType button;
        if ( vpDisplay::getClick( plotter->I, button, false ) )
        {
        	switch ( button )
        	{
        	case vpMouseButton::button1:
        		send_cmd = !send_cmd;
        		break;
        	case vpMouseButton::button3:
        		final_quit = true;
        		tau_cmd = 0;
        		break;
        	default:
        		break;
        	}
        }

      robot.wait( sim_time, 0.002 );

    }// end while

    if ( plotter != nullptr )
    {
      delete plotter;
      plotter = nullptr;
    }
    robot.coppeliasimStopSimulation();
    robot.setRobotState( vpRobot::STATE_STOP);


  }
  catch ( const vpException &e )
  {
    std::cout << "ViSP exception: " << e.what() << std::endl;
    std::cout << "Stop the robot " << std::endl;
    return EXIT_FAILURE;
  }

  return 0;
}





