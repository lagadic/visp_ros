/*
 * \example tutorial-franka-coppeliasim-duar-arm.cpp
 *
 *  Created on: Aug 1, 2021
 *      Author: oliva
 */

//! \example tutorial-franka-coppeliasim-duar-arm.cpp

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

vpColVector We(6,0);
std::mutex shared_data;

void Wrench_callback(const geometry_msgs::WrenchStamped& sensor_wrench)
{
  std::lock_guard<std::mutex> lock(shared_data);
  We[0]=sensor_wrench.wrench.force.x;
  We[1]=sensor_wrench.wrench.force.y;
  We[2]=sensor_wrench.wrench.force.z;
  We[3]=sensor_wrench.wrench.torque.x;
  We[4]=sensor_wrench.wrench.torque.y;
  We[5]=sensor_wrench.wrench.torque.z;
  return;
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
  double opt_tagSize             = 0.08;
  bool display_tag               = true;
  int opt_quad_decimate          = 2;
  bool opt_verbose               = false;
  bool opt_plot                  = false;
  bool opt_adaptive_gain         = false;
  double convergence_threshold_t = 0.005, convergence_threshold_tu = 0.5;
  bool opt_coppeliasim_sync_mode = false;

  for ( int i = 1; i < argc; i++ )
  {
    if ( std::string( argv[i] ) == "--tag_size" && i + 1 < argc )
    {
      opt_tagSize = std::stod( argv[i + 1] );
    }
    else if ( std::string( argv[i] ) == "--verbose" || std::string( argv[i] ) == "-v" )
    {
      opt_verbose = true;
    }
    else if ( std::string( argv[i] ) == "--plot" )
    {
      opt_plot = true;
    }
    else if ( std::string( argv[i] ) == "--adaptive_gain" )
    {
      opt_adaptive_gain = true;
    }
    else if ( std::string( argv[i] ) == "--quad_decimate" && i + 1 < argc )
    {
      opt_quad_decimate = std::stoi( argv[i + 1] );
    }
    else if ( std::string( argv[i] ) == "--no-convergence-threshold" )
    {
      convergence_threshold_t  = 0.;
      convergence_threshold_tu = 0.;
    }
    else if ( std::string( argv[i] ) == "--enable-coppeliasim-sync-mode" )
    {
      opt_coppeliasim_sync_mode = true;
    }
    else if ( std::string( argv[i] ) == "--help" || std::string( argv[i] ) == "-h" )
    {
      std::cout << argv[0] << "[--tag_size <marker size in meter; default " << opt_tagSize << ">] "
                << "[--quad_decimate <decimation; default " << opt_quad_decimate << ">] "
                << "[--adaptive_gain] "
                << "[--plot] "
                << "[--task_sequencing] "
                << "[--no-convergence-threshold] "
                << "[--enable-coppeliasim-sync-mode] "
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

    ros::Subscriber sub_wrench = n->subscribe("/coppeliasim/franka/right_arm/ft_sensor", 1, Wrench_callback);

    vpROSRobotFrankaCoppeliasim right_arm, left_arm;
    right_arm.setVerbose( opt_verbose );
    right_arm.connect("right_arm");
    left_arm.setVerbose( opt_verbose );
    left_arm.connect("left_arm");

    std::cout << "Coppeliasim sync mode enabled: " << ( opt_coppeliasim_sync_mode ? "yes" : "no" ) << std::endl;
    right_arm.coppeliasimStopSimulation(); // Allows to reset simulation, moving the robot to initial position
    right_arm.setCoppeliasimSyncMode( false );
    left_arm.setCoppeliasimSyncMode( false );
    right_arm.coppeliasimStartSimulation();

    vpHomogeneousMatrix bMfra( vpTranslationVector( 0.040, -0.03, 0.17 ),
            vpRotationMatrix( { 1, 0, 0, 0, 0, -1, 0, 1, 0 } ) );
    vpHomogeneousMatrix bMfla( vpTranslationVector( 0.040, 0.03, 0.17 ),
            vpRotationMatrix( { 1, 0, 0, 0, 0, 1, 0, -1, 0 } ) );

    vpImage< unsigned char > I;
    vpROSGrabber g;
    g.setImageTopic( "/coppeliasim/franka/camera/image" );
    g.setCameraInfoTopic( "/coppeliasim/franka/camera/camera_info" );
    g.open( argc, argv );

    g.acquire( I );

    std::cout << "Image size: " << I.getWidth() << " x " << I.getHeight() << std::endl;
    vpCameraParameters cam;

    g.getCameraInfo( cam );
    std::cout << cam << std::endl;
    vpDisplayOpenCV dc( I, 10, 10, "Color image" );

    vpDetectorAprilTag::vpAprilTagFamily tagFamily                  = vpDetectorAprilTag::TAG_36h11;
    vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;
    vpDetectorAprilTag detector( tagFamily );
    detector.setAprilTagPoseEstimationMethod( poseEstimationMethod );
    detector.setDisplayTag( display_tag );
    detector.setAprilTagQuadDecimate( opt_quad_decimate );

    // Servo
    vpHomogeneousMatrix ccMo, cMo, oMo, ccMc, cdMcc, wMo, wMt;
    // Desired pose used to compute the desired features
    vpHomogeneousMatrix cdMo( vpTranslationVector( 0.0, 0.0, 0.2 ),
                              vpRotationMatrix( { 1, 0, 0, 0, -1, 0, 0, 0, -1 } ) );

    // Create visual features
    vpFeatureTranslation t( vpFeatureTranslation::cdMc );
    vpFeatureThetaU tu( vpFeatureThetaU::cdRc );
    t.buildFrom( ccMc );
    tu.buildFrom( ccMc );

    vpFeatureTranslation td( vpFeatureTranslation::cdMc );
    vpFeatureThetaU tud( vpFeatureThetaU::cdRc );

    vpServo task;
    // Add the visual features
    task.addFeature( t, td );
    task.addFeature( tu, tud );

    task.setServo( vpServo::EYEINHAND_CAMERA );
    task.setInteractionMatrixType( vpServo::CURRENT );

    if ( opt_adaptive_gain )
    {
      std::cout << "Enable adptive gain" << std::endl;
      vpAdaptiveGain lambda( 4, 1.2, 25 ); // lambda(0)=4, lambda(oo)=1.2 and lambda'(0)=25
      task.setLambda( lambda );
    }
    else
    {
      task.setLambda( 1.2 );
    }

    vpPlot *plotter = nullptr;
    vpPlot *plotter_left = nullptr;

    if ( opt_plot )
    {
      plotter = new vpPlot( 3, static_cast< int >( 250 * 2 ), 600, static_cast< int >( I.getWidth() ) + 80, 10,
                            "Real time curves plotter" );
      plotter->setTitle( 0, "Visual features error" );
      plotter->setTitle( 1, "Camera velocities" );
      plotter->setTitle( 2, "Measured wrench" );
      plotter->initGraph( 0, 6 );
      plotter->initGraph( 1, 6 );
      plotter->initGraph( 2, 6 );
      plotter->setLegend( 0, 0, "error_feat_tx" );
      plotter->setLegend( 0, 1, "error_feat_ty" );
      plotter->setLegend( 0, 2, "error_feat_tz" );
      plotter->setLegend( 0, 3, "error_feat_theta_ux" );
      plotter->setLegend( 0, 4, "error_feat_theta_uy" );
      plotter->setLegend( 0, 5, "error_feat_theta_uz" );
      plotter->setLegend( 1, 0, "vc_x" );
      plotter->setLegend( 1, 1, "vc_y" );
      plotter->setLegend( 1, 2, "vc_z" );
      plotter->setLegend( 1, 3, "wc_x" );
      plotter->setLegend( 1, 4, "wc_y" );
      plotter->setLegend( 1, 5, "wc_z" );
      plotter->setLegend( 2, 0, "F_x" );
      plotter->setLegend( 2, 1, "F_y" );
      plotter->setLegend( 2, 2, "F_z" );
      plotter->setLegend( 2, 3, "Tau_x" );
      plotter->setLegend( 2, 4, "Tau_y" );
      plotter->setLegend( 2, 5, "Tau_z" );

      plotter_left = new vpPlot( 4, 500, 600, 10, 10, "Real time curves plotter" );
      plotter_left->setTitle( 0, "EE Pose [m] - [rad]" );
      plotter_left->initGraph( 0, 6 );
      plotter_left->setLegend( 0, 0, "x" );
      plotter_left->setLegend( 0, 1, "y" );
      plotter_left->setLegend( 0, 2, "z" );
      plotter_left->setLegend( 0, 3, "tu_x" );
      plotter_left->setLegend( 0, 4, "tu_y" );
      plotter_left->setLegend( 0, 5, "tu_z" );

      plotter_left->setTitle( 1, "EE pose error [m] - [rad]" );
      plotter_left->initGraph( 1, 6 );
      plotter_left->setLegend( 1, 0, "e_x" );
      plotter_left->setLegend( 1, 1, "e_y" );
      plotter_left->setLegend( 1, 2, "e_z" );
      plotter_left->setLegend( 1, 3, "e_tu_x" );
      plotter_left->setLegend( 1, 4, "e_tu_y" );
      plotter_left->setLegend( 1, 5, "e_tu_z" );

      plotter_left->setTitle( 2, "Joint torque command [Nm]" );
      plotter_left->initGraph( 2, 7 );
      plotter_left->setLegend( 2, 0, "Tau1" );
      plotter_left->setLegend( 2, 1, "Tau2" );
      plotter_left->setLegend( 2, 2, "Tau3" );
      plotter_left->setLegend( 2, 3, "Tau4" );
      plotter_left->setLegend( 2, 4, "Tau5" );
      plotter_left->setLegend( 2, 5, "Tau6" );
      plotter_left->setLegend( 2, 6, "Tau7" );

      plotter_left->setTitle( 3, "Pose error norm [rad]" );
      plotter_left->initGraph( 3, 1 );
      plotter_left->setLegend( 3, 0, "||e||" );// change this
    }

    bool final_quit                           = false;
    bool has_converged                        = false;
    bool send_cmd                             = false;
    bool restart                              = false;
    std::vector< vpImagePoint > *traj_corners = nullptr; // To memorize point trajectory

    double sim_time             = right_arm.getCoppeliasimSimulationTime();
    double sim_time_prev        = sim_time;
    double sim_time_init_servo  = sim_time;
    double sim_time_img         = sim_time;
    double sim_time_img_prev    = sim_time;
    double sim_time_start       = sim_time;
    double sim_time_convergence = sim_time;
    double dt                   = 0.0;


    vpMatrix K( 6, 6 ), D( 6, 6 );
    double wp = 50;
    double wo = 20;
    K.diag({wp*wp,wp*wp,wp*wp,wo*wo,wo*wo,wo*wo});
    D.diag({2*wp,2*wp,2*wp,2*wo,2*wo,2*wo});

    double c_time = 0.0;
    double mu     = 2.0;

    std::cout << "eMc:\n" << right_arm.get_eMc() << std::endl;

    vpColVector v_c( 6, 0 ), q( 7, 0 ), dq( 7, 0 ), tau_d( 7, 0 ), C( 7, 0 ),
    		    pos( 6, 0 ), F( 7, 0 ), tau_d0( 7, 0 ), tau_cmd( 7, 0 ),
				x_e( 6, 0 ), dx_e( 6, 0 ), dx_ed( 6, 0 ), ddx_ed( 6, 0 );
    vpMatrix J( 6, 7 ), Ja( 6, 7 ), dJa( 6, 7 ), Ja_old( 6, 7 ), B( 7, 7 ),
    		 I7( 7, 7 ), Ja_pinv_B_t( 6, 7 ), Ls( 6, 6 );

    I7.eye();

    right_arm.setRobotState( vpRobot::STATE_VELOCITY_CONTROL );
    left_arm.setRobotState( vpRobot::STATE_FORCE_TORQUE_CONTROL);

    right_arm.setCoppeliasimSyncMode( opt_coppeliasim_sync_mode );
    left_arm.setCoppeliasimSyncMode( opt_coppeliasim_sync_mode );

    vpRotationMatrix wRed0({1,0,0,0,-1,0,0,0,-1});
    vpHomogeneousMatrix wMed0(vpTranslationVector(0.5, 0.0, 0.0),wRed0);
    vpHomogeneousMatrix wMed;
    wMed = wMed0;

    std::atomic_bool admittance_thread_running{true};
    bool init_done = false;

    vpHomogeneousMatrix ftTee;
    // the FT sensor frame is coincident with the flange frame in Coppeliasim model
    // then the transformation between flange and end-effector is the same than
    // the transformation between the FT sensor and the end-effector
    ftTee = right_arm.get_flMe();
    std::cout << "ftTee:\n" << ftTee << "\n";

    vpVelocityTwistMatrix cTe;
    cTe.buildFrom(right_arm.get_eMc().inverse());

    vpForceTwistMatrix cFee, eeFft, eeFc;
    cFee.buildFrom(right_arm.get_eMc().inverse());
    eeFc.buildFrom(right_arm.get_eMc());
    eeFft.buildFrom(ftTee.inverse());


    // Admittance parameters
    vpColVector dde_s(6,0), de_s(6,0), e_s(6,0), d_err(6,0), err(6,0), old_err(6,0);
    vpMatrix Ks(6,6), Ds(6,6), Be( 6, 6 ), Bc_inv( 6, 6 ), I3( 3, 3 );
    Ks.diag(100);
    Ds.diag(100);

    I3.eye();
    Be.insert(1*I3, 0, 0);// mass
    Be.insert(0.1*I3, 3, 3);   // inertia
    // in this way we achieve Isotropic behavior on the ee frame
    Bc_inv = cTe*Be.inverseByCholesky()*((vpMatrix)cTe).t();

    vpColVector r( 3, 0 ), wFcog( 3, 0 ), ft_load( 6, 0 ), ft_meas( 6, 0 ),
    		    cfc_meas( 6, 0 ), Fs( 6, 0 );
    double m = right_arm.get_tool_mass();// [kg]
    wFcog[2] = -9.81*m;
    r = right_arm.get_flMcom().getTranslationVector();


    std::thread fast_thread([&]() {
      std::cout << "fast_thread: started... \n" << std::endl;
      try{
        while (admittance_thread_running) {
          if(init_done){

		    sim_time = right_arm.getCoppeliasimSimulationTime();
		    dt = sim_time - sim_time_prev;
		    sim_time_prev = sim_time;
//              std::cout << "dt: " << dt<< std::endl;


            vpColVector aux(3,0);
            aux = ((bMfra*right_arm.get_fMe()*right_arm.get_flMe().inverse()).inverse()).getRotationMatrix()*wFcog;
            ft_load[0] = aux[0]; ft_load[1] = aux[1];ft_load[2] = aux[2];
            aux = vpColVector::skew(r)*aux;
            ft_load[3] = aux[0]; ft_load[4] = aux[1];ft_load[5] = aux[2];
            ft_meas = We - ft_load;

            shared_data.lock();
            cfc_meas = cFee*eeFft*ft_meas;
            task.computeControlLaw();
            Ls = compute_interaction_matrix_3D(cdMcc);

            Fs = Ls*Bc_inv*cfc_meas;

            // admittance law
            dde_s = -Ks*e_s - Ds*de_s  + Fs;
            de_s += dde_s*dt;
            e_s += de_s*dt;

            cdMcc.insert((vpTranslationVector)e_s.extract(0, 3));
            if(sqrt(e_s.extract(3, 3).sumSquare()) != 0.0){
                vpRotationMatrix R_aux;
                R_aux.buildFrom((vpThetaUVector)e_s.extract(3, 3));
                cdMcc.insert(R_aux);
            }else{
                cdMcc.insert(vpRotationMatrix());
            }
            ccMo = cdMcc.inverse()*cdMo;
            ccMc = ccMo * oMo * cMo.inverse();
            t.buildFrom( ccMc );
            tu.buildFrom( ccMc );

            v_c = task.computeControlLaw();

            left_arm.getPosition( vpRobot::JOINT_STATE, q );
            left_arm.getVelocity( vpRobot::JOINT_STATE, dq );
            left_arm.getMass( B );
            left_arm.getCoriolis( C );
            left_arm.getFriction( F );
            left_arm.get_fJe(J);

            if(has_converged){
          	  wMed[0][3] = wMed0[0][3] + 0.05*sin(2*M_PI*0.3*(sim_time - sim_time_convergence));
          	  wMed[1][3] = wMed0[1][3] + 0.05*(1 - cos(2*M_PI*0.3*(sim_time - sim_time_convergence)) );
//          	  wMed[2][3] = wMed0[2][3] + 0.05*sin(2*M_PI*0.5*(sim_time - sim_time_convergence));

          	  dx_ed[0] = 2*M_PI*0.3*0.05*cos(2*M_PI*0.3*(sim_time - sim_time_convergence));
          	  dx_ed[1] = 2*M_PI*0.3*0.05*sin(2*M_PI*0.3*(sim_time - sim_time_convergence));
//          	  dx_ed[2] = 2*M_PI*0.5*0.05*cos(2*M_PI*0.5*(sim_time - sim_time_convergence));

          	  ddx_ed[0] = -2*M_PI*0.3*2*M_PI*0.3*0.05*sin(2*M_PI*0.3*(sim_time - sim_time_convergence));
          	  ddx_ed[1] =  2*M_PI*0.3*2*M_PI*0.3*0.05*cos(2*M_PI*0.3*(sim_time - sim_time_convergence));
//          	  ddx_ed[2] = -2*M_PI*0.5*2*M_PI*0.5*0.05*sin(2*M_PI*0.5*(sim_time - sim_time_convergence));
            }

            vpMatrix RR(6,6);
            RR.insert(wMed.getRotationMatrix().t(), 0, 0);
            RR.insert(wMed.getRotationMatrix().t(), 3, 3);

            x_e = (vpColVector)vpPoseVector(wMed.inverse() * left_arm.get_fMe());
            Ja = Ta(wMed.inverse() * left_arm.get_fMe()) * RR * J ;

            dx_e = Ta(wMed.inverse()*left_arm.get_fMe())*RR*(dx_ed-J*dq);

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
          	  v_c = 0; // Stop the robot
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

            right_arm.setVelocity( vpRobot::CAMERA_FRAME, v_c );
            left_arm.setForceTorque( vpRobot::JOINT_STATE, tau_cmd );

            shared_data.unlock();

          }// end init_done
          right_arm.wait( sim_time, 0.002 );
        }//end while
      }catch(const vpException &e) {
        std::cout << "ViSP exception: " << e.what() << std::endl;
        admittance_thread_running = false;
      }catch(... ) {
        std::cout << "Something wrong happens: "  << std::endl;
        admittance_thread_running = false;
      }
      std::cout << "fast_thread: exiting..." << std::endl;
      final_quit = true;
    });

    // control loop
    while ( !final_quit )
    {
      g.acquire( I, sim_time_img );
      vpDisplay::display( I );

      std::vector< vpHomogeneousMatrix > cMo_vec;
      detector.detect( I, opt_tagSize, cam, cMo_vec );

      {
        std::stringstream ss;
        ss << "Left click to " << ( send_cmd ? "stop the robot" : "servo the robot" )
           << ", right click to quit.";
        vpDisplay::displayText( I, 20, 20, ss.str(), vpColor::red );
      }

      // Only one tag is detected
      if ( cMo_vec.size() == 1 )
      {
        shared_data.lock();
        cMo = cMo_vec[0];
        if ( !init_done )
        {
          // Introduce security wrt tag positionning in order to avoid PI rotation
          std::vector< vpHomogeneousMatrix > v_oMo( 2 ), v_cdMc( 2 );
          v_oMo[1].buildFrom( 0, 0, 0, 0, 0, M_PI );
          for ( size_t i = 0; i < 2; i++ )
          {
            v_cdMc[i] = cdMo * v_oMo[i] * cMo.inverse();
          }
          if ( std::fabs( v_cdMc[0].getThetaUVector().getTheta() ) <
               std::fabs( v_cdMc[1].getThetaUVector().getTheta() ) )
          {
            oMo = v_oMo[0];
          }
          else
          {
            std::cout << "Desired frame modified to avoid PI rotation of the camera" << std::endl;
            oMo = v_oMo[1]; // Introduce PI rotation
          }
          sim_time_start = sim_time;
    	  ccMo = cdMo;
        } // end first_time

        // Update visual features
        ccMc = ccMo * oMo * cMo.inverse();
        t.buildFrom( ccMc );
        tu.buildFrom( ccMc );

        shared_data.unlock();

        // Display the current and desired feature points in the image display
        // Display desired and current pose features
        vpDisplay::displayFrame( I, cdMo * oMo, cam, opt_tagSize / 1.5, vpColor::yellow, 2 );
        vpDisplay::displayFrame( I, ccMo * oMo, cam, opt_tagSize / 2, vpColor::none, 3 );
        vpDisplay::displayFrame( I, cMo, cam, opt_tagSize / 2, vpColor::none, 3 );


        // Get tag corners
        std::vector< vpImagePoint > corners = detector.getPolygon( 0 );

        // Get the tag cog corresponding to the projection of the tag frame in the image
        corners.push_back( detector.getCog( 0 ) );
        // Display the trajectory of the points
        if ( !init_done )
        {
          traj_corners = new std::vector< vpImagePoint >[corners.size()];
        }

        // Display the trajectory of the points used as features
//        display_point_trajectory( I, corners, traj_corners );


        vpTranslationVector cd_t_c = ccMc.getTranslationVector();
        vpThetaUVector cd_tu_c     = ccMc.getThetaUVector();
        double error_tr            = sqrt( cd_t_c.sumSquare() );
        double error_tu            = vpMath::deg( sqrt( cd_tu_c.sumSquare() ) );

        std::stringstream ss;
        ss << "error_t [m]: " << error_tr;
        vpDisplay::displayText( I, 20, static_cast< int >( I.getWidth() ) - 160, ss.str(), vpColor::red );
        ss.str( "" );
        ss << "error_tu [deg]: " << error_tu;
        vpDisplay::displayText( I, 40, static_cast< int >( I.getWidth() ) - 160, ss.str(), vpColor::red );


        if ( !has_converged && error_tr < convergence_threshold_t && error_tu < convergence_threshold_tu )
        {
          has_converged = true;
          sim_time_convergence = sim_time;
          std::cout << "Servo task has converged \n";
          vpDisplay::displayText( I, 100, 20, "Servo task has converged", vpColor::red );
        }

        if ( !init_done )
        {
        	init_done = true;
        }
      } // end if (cMo_vec.size() == 1)
      else
      {
        v_c = 0; // Stop the robot
        tau_cmd = 0;
      }

      if ( opt_plot )
      {
        plotter->plot( 0, static_cast< double >( sim_time ), task.getError() );
        plotter->plot( 1, static_cast< double >( sim_time ), v_c );
        plotter->plot( 2, static_cast< double >( sim_time ), cfc_meas );

        plotter_left->plot( 0, sim_time, (vpColVector)vpPoseVector(bMfla*left_arm.get_fMe()));
        plotter_left->plot( 1, sim_time, x_e );
        plotter_left->plot( 2, sim_time, tau_cmd );
        plotter_left->plot( 3, sim_time, vpColVector(1, sqrt(x_e.sumSquare())));
      }

      std::stringstream ss;
      ss << "Loop time [s]: " << std::round((sim_time_img - sim_time_img_prev)*1000.)/1000.;
      ss << " Simulation time [s]: " << sim_time_img;
      sim_time_img_prev = sim_time_img;
      vpDisplay::displayText( I, 40, 20, ss.str(), vpColor::red );

      vpMouseButton::vpMouseButtonType button;
      if ( vpDisplay::getClick( I, button, false ) )
      {
        switch ( button )
        {
        case vpMouseButton::button1:
          send_cmd = !send_cmd;
          break;

        case vpMouseButton::button3:
          final_quit = true;
          v_c        = 0;
          tau_cmd = 0;
          admittance_thread_running = false;
          break;

        default:
          break;
        }
      }

      vpDisplay::flush( I );
    }                                // end while

    if ( opt_plot && plotter != nullptr && plotter_left != nullptr )
    {
      delete plotter;
      plotter = nullptr;
      delete plotter_left;
      plotter_left = nullptr;
    }
    right_arm.coppeliasimStopSimulation();
    right_arm.setRobotState( vpRobot::STATE_STOP );
    left_arm.setRobotState( vpRobot::STATE_STOP);

    if ( !final_quit )
    {
      while ( !final_quit )
      {
        g.acquire( I );
        vpDisplay::display( I );

        vpDisplay::displayText( I, 20, 20, "Click to quit the program.", vpColor::red );
        vpDisplay::displayText( I, 40, 20, "Visual servo converged.", vpColor::red );

        if ( vpDisplay::getClick( I, false ) )
        {
          final_quit = true;
        }

        vpDisplay::flush( I );
      }
    }
    if (fast_thread.joinable()) {
    	fast_thread.join();
      std::cout << "fast thread joined.. \n ";
    }
    if ( traj_corners )
    {
      delete[] traj_corners;
    }

  }
  catch ( const vpException &e )
  {
    std::cout << "ViSP exception: " << e.what() << std::endl;
    std::cout << "Stop the robot " << std::endl;
    return EXIT_FAILURE;
  }

  return 0;
}



