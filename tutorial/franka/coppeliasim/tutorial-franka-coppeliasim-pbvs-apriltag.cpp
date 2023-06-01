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

//! \example tutorial-franka-coppeliasim-pbvs-apriltag.cpp

#include <iostream>

#include <visp3/core/vpCameraParameters.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpPlot.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/visual_features/vpFeatureThetaU.h>
#include <visp3/visual_features/vpFeatureTranslation.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>
#include <geometry_msgs/WrenchStamped.h>

#include <visp_ros/vpROSGrabber.h>
#include <visp_ros/vpROSRobotFrankaCoppeliasim.h>

void display_point_trajectory( const vpImage< unsigned char > &I, const std::vector< vpImagePoint > &vip,
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

vpHomogeneousMatrix getMatrixPush0(const std::vector<double>& Box1, const std::vector<double>& Box2, const std::vector<double>& Box3, double x, double y, double z) {//prepare for translation push 1
    std::cout << "Go to target position above by 0.02 meters" << std::endl;
    
    //delta_Z
    double delta = 0.02;
    
    // target position
    double X = x-0.5*delta;	// use "-0.5*delta" due to the offset of the orientation
    double Y = y + 0.5*Box1[1] + 0.5*Box3[1] + delta;
    double Z = Box3[2] + Box1[2] + delta;

    vpHomogeneousMatrix fM_Push0(vpTranslationVector(X, Y, Z),
                                 vpRotationMatrix( {1,0,0,0,1,0,0,0,1} ) );
    //print HomogeneousMatrix
    std::cout << "fM_Push0:\n" << fM_Push0 << std::endl;
    
    return fM_Push0;
}
vpHomogeneousMatrix getMatrixPush1(const std::vector<double>& Box1, const std::vector<double>& Box2, const std::vector<double>& Box3, double x, double y, double z) {//prepare for translation push 2
    std::cout << "Go to suitable height, preparing for translation push" << std::endl;
	
    //delta_Z=delta_Y
    double delta = 0.02;
    
    // target position
    double X = x-0.8*delta;
    double Y = y + 0.5*Box1[1] + 0.5*Box3[1] + delta;
    double Z = Box3[2] + 0.5*delta;

    vpHomogeneousMatrix fM_Push1(vpTranslationVector(X, Y, Z),
                                 vpRotationMatrix( {1,0,0,0,1,0,0,0,1} ) );
    
    //print HomogeneousMatrix
    std::cout << "fM_Push1:\n" << fM_Push1 << std::endl;
    
    return fM_Push1;
}

vpHomogeneousMatrix getMatrixPush2(const std::vector<double>& Box1, const std::vector<double>& Box2, const std::vector<double>& Box3, double x, double y, double z) {//translating
    std::cout << "translation push" << std::endl;
    
    //delta_Z=delta_Y
    double delta = 0.02;
    
    // target position
    double X = x-0.8*delta;
    double Y = Box1[1] + 0.5*Box3[1] - 0.1*delta;
    double Z = Box3[2] + delta;
    
    // translation motion Push, step 2
    vpHomogeneousMatrix fM_Push2(vpTranslationVector(X, Y, Z),
                                 vpRotationMatrix( {1,0,0,0,1,0,0,0,1} ) );
                                 
    //print HomogeneousMatrix
    std::cout << "fM_Push2:\n" << fM_Push2 << std::endl;
    
    return fM_Push2;
}

vpHomogeneousMatrix getMatrixPush3(const std::vector<double>& Box1, const std::vector<double>& Box2, const std::vector<double>& Box3, double x, double y, double z) {//placing
    std::cout << "go to side then place the parcel" << std::endl;
    
    //delta_Z
    double delta = 0.02;
    
    // target position
    double X = 0.37 - 0.5*Box3[0] - 0.1*delta;
    double Y = Box1[1] + 0.5*Box3[1] - 0.1*delta;
    double Z = Box3[2] + delta;
    
    // translation motion Push, step 3
    vpHomogeneousMatrix fM_Push3(vpTranslationVector(X, Y, Z),
                                 vpRotationMatrix( {1,0,0,0,1,0,0,0,1} ) );
    
    //print HomogeneousMatrix
    std::cout << "fM_Push3:\n" << fM_Push3 << std::endl;
    
    return fM_Push3;
}


vpHomogeneousMatrix getMatrixY0(const std::vector<double>& Box1, const std::vector<double>& Box2, double x, double y, double z) {//prepare for rotating
    std::cout << "Go to target position above by 0.45 meters without any tilt angle." << std::endl;
    double factor1 = 0.45;
    
    // delta_X
    double delta = 0.015;
    double len = x - 0.5*Box1[0] - delta;
    
    // Rotation angle
    double theta = std::acos(len / Box2[0]);
    double c = cos(theta);
    double s = sin(theta);
    
    // target position
    double X = Box2[2]*s + len - 0.5*Box2[0]*c;
    double Y = y;	// middle of target parcel
    double Z = Box2[2]*c + 0.5*Box2[1]*s + Box1[2];

    vpHomogeneousMatrix fM_Y0_rotate(vpTranslationVector(X, Y, Z+factor1),
                                       vpRotationMatrix( {1,0,0,0,1,0,0,0,1} ) );
                                       
    //print Rotation angle and HomogeneousMatrix
    std::cout << "Rotation angle: " << theta << std::endl;
    std::cout << "fM_Y0_rotate:\n" << fM_Y0_rotate << std::endl;
    
    return fM_Y0_rotate;
}

vpHomogeneousMatrix getMatrixY1(const std::vector<double>& Box1, const std::vector<double>& Box2, double x, double y, double z) {//rotating and go down to push
    std::cout << "Going down with rotating" << std::endl;

    // factor1 is used for deeper push due to the limitation of physical engine and heavy parcel, you may change it according to the parcel you want
    double factor1 = 1.25;
    
    // delta_X
    double delta = 0.015;
    double len = x - 0.5*Box1[0] - delta;
    
    // Rotation angle
    double theta = std::acos(len / Box2[0]);
    double c = cos(theta);
    double s = sin(theta);
    double tan_theta = std::tan(theta);
    
    //dz is how much you push downwards
    double dz = (Box2[1]*(1-c))/tan_theta;
    
    // target position
    double X = Box2[2]*s + len - 0.5*Box2[0]*c;
    double Y = y;	// middle of target parcel
    double Z = Box2[2]*c + 0.5*Box2[1]*s + Box1[2] - factor1*dz;

    vpHomogeneousMatrix fM_Y1_rotate(vpTranslationVector(X, Y, Z),
                                     vpRotationMatrix( {c,0,s,0,1,0,-s,0,c} ) );
                                     
    //print Rotation angle and HomogeneousMatrix
    std::cout << "Rotation angle: " << theta << std::endl;
    std::cout << "fM_Y1_rotate:\n" << fM_Y1_rotate << std::endl;
    
    return fM_Y1_rotate;
}

vpHomogeneousMatrix getMatrixY2(const std::vector<double>& Box1, const std::vector<double>& Box2, double x, double y, double z) {//go up without rotating due, if you rotate instead of going up first, it will crash the side of tote
    std::cout << "go up without rotating" << std::endl;
    
    // delta_X
    double delta = 0.015;
    double len = x - 0.5*Box1[0] - delta;
    
    // Rotation angle
    double theta = std::acos(len / Box2[0]);
    double c = cos(theta);
    double s = sin(theta);
    double tan_theta = std::tan(theta);
    
    //dz is how much you push downwards
    double dz = (Box2[1]*(1-c))/tan_theta;
    
    // target position
    double X = Box2[2]*s + len - 0.5*Box2[0]*c;
    double Y = y;
    double Z = Box2[2] + Box2[1] + 10*delta;

    vpHomogeneousMatrix fM_Y2_rotate(vpTranslationVector(X, Y, Z),
                                     vpRotationMatrix( {c,0,s,0,1,0,-s,0,c} ) );
                                     
    //print Rotation angle and HomogeneousMatrix
    std::cout << "Rotation angle: " << theta << std::endl;
    std::cout << "fM_Y2_rotate:\n" << fM_Y2_rotate << std::endl;
    
    return fM_Y2_rotate;
}

vpHomogeneousMatrix getMatrixY3(const std::vector<double>& Box1, const std::vector<double>& Box2, double x, double y, double z) {//rorate back
    std::cout << "Rorating back" << std::endl;
    double factor3 = 1.1;
    
    // delta_X
    double delta = 0.015;
    double len = x - 0.5*Box1[0] - delta;
    
    // Rotation angle
    double theta = std::acos(len / Box2[0]);
    double c = cos(theta);
    double s = sin(theta);
    
    // target position
    double X = 0.5*Box2[0]*1.02;
    double Y = 0.5*Box2[1]*1.15; 	// multiplied by some values otherwise it crash the side of tote
    double Z = Box2[2] + Box1[2] + 10*delta;
    
    // rotating motion Y3, step 3
    vpHomogeneousMatrix fM_Y3_rotate(vpTranslationVector(X, Y, factor3*Z),//0.1,0.2,0.1
                                     vpRotationMatrix( {1,0,0,0,1,0,0,0,1} ) );
    
    //print Rotation angle and HomogeneousMatrix
    std::cout << "Rotation angle: " << theta << std::endl;
    std::cout << "fM_Y3_rotate:\n" << fM_Y3_rotate << std::endl;
    
    return fM_Y3_rotate;
}

vpHomogeneousMatrix getMatrixY4(const std::vector<double>& Box1, const std::vector<double>& Box2, double x, double y, double z) {//after having enough width, go down to place box
    std::cout << "after having enough width, go down" << std::endl;
    
    // delta_X
    double delta = 0.015;
    
    // target position
    double X = 0.5*Box2[0]*1.02;
    double Y = 0.5*Box2[1]*1.15;	// multiplied by some values otherwise it crash the side of tote
    double Z = Box2[2] + 2*delta;	// added by some values otherwise it crash the bottom of tote

    vpHomogeneousMatrix fM_Y4_rotate(vpTranslationVector(X, Y, Z),
                                     vpRotationMatrix( {1,0,0,0,1,0,0,0,1} ) );
                                     
    //print HomogeneousMatrix
    std::cout << "fM_Y4_rotate:\n" << fM_Y4_rotate << std::endl;
    
    return fM_Y4_rotate;
}

vpHomogeneousMatrix getMatrixY5(const std::vector<double>& Box1, const std::vector<double>& Box2, double x, double y, double z) {// do translation motion again then place the parcel
    std::cout << "after going down, do translation motion and place the parcel" << std::endl;
    
    // delta_X
    double delta = 0.015;
    
    // target position
    double X = 0.37 - (Box1[0] + 0.5*Box2[0]); //0.37 is the width of tote
    double Y = 0.5*Box2[1]*1.10;	// multiplied by some values otherwise it crash the side of tote
    double Z = Box2[2] + 2*delta;	// added by some values otherwise it crash the bottom of tote

    vpHomogeneousMatrix fM_Y5_push(vpTranslationVector(X, Y, Z),
                                     vpRotationMatrix( {1,0,0,0,1,0,0,0,1} ) );
                                     
    //print HomogeneousMatrix
    std::cout << "fM_Y5_push:\n" << fM_Y5_push << std::endl;
    
    return fM_Y5_push;
}




// set force and torque to 0
double force_x = 0.0;
double force_y = 0.0;
double force_z = 0.0;
double torque_x = 0.0;
double torque_y = 0.0;
double torque_z = 0.0;

vpColVector ft_sensor(6,0);

void ftSensorCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
    // Retrieve force and torque values
    ft_sensor[0] = msg->wrench.force.x;
    ft_sensor[1] = msg->wrench.force.y;
    ft_sensor[2] = msg->wrench.force.z;
    ft_sensor[3] = msg->wrench.torque.x;
    ft_sensor[4] = msg->wrench.torque.y;
    ft_sensor[5] = msg->wrench.torque.z;
}


int main( int argc, char **argv )
{
    double opt_tagSize             = 0.08;
    bool display_tag               = true;
    int opt_quad_decimate          = 2;
    bool opt_verbose               = false;
    bool opt_plot                  = false;
    bool opt_adaptive_gain         = false;
    bool opt_task_sequencing       = true;
    double convergence_threshold_t = 0.0005, convergence_threshold_tu = vpMath::rad( 0.5 );
    bool opt_coppeliasim_sync_mode = true;

    // change mode based on inputs of interface
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
        else if ( std::string( argv[i] ) == "--task_sequencing" )
        {
            opt_task_sequencing = true;
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
	
	//suction grabber
        ros::Publisher pub_suctionpad = n->advertise<std_msgs::Int32>("/suctionpad_activate", 1);
        std_msgs::Int32 activate;
        activate.data = 0;	// 0 - no force
        
        //torque and force sensor subscriber
        ros::Subscriber sub = n->subscribe("/coppeliasim/franka/ft_sensor", 1, ftSensorCallback);

        vpROSRobotFrankaCoppeliasim robot;
        robot.setVerbose( opt_verbose );
        robot.connect();

        std::cout << "Coppeliasim sync mode enabled: " << ( opt_coppeliasim_sync_mode ? "yes" : "no" ) << std::endl;
        robot.coppeliasimStopSimulation(); // Allows to reset simulation, moving the robot to initial position
        robot.setCoppeliasimSyncMode( false );
        robot.coppeliasimStartSimulation();

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
        vpHomogeneousMatrix cMo, oMo, active_cdMc;

        // Desired pose used to compute the desired features
        vpHomogeneousMatrix cdMo( vpTranslationVector( 0, 0.0, opt_tagSize * 3 ),
                                  vpRotationMatrix( { 1, 0, 0, 0, -1, 0, 0, 0, -1 } ) );
        vpHomogeneousMatrix eedMo( vpTranslationVector( 0.1, 0.0, 0.01 ),
                                   vpRotationMatrix( { 1, 0, 0, 0, -1, 0, 0, 0, -1 } ) );
        // pick the bin
        vpHomogeneousMatrix edMo(vpTranslationVector(0.0, 0.0, 0.001),
                                 vpRotationMatrix( {0, 1, 0, 1, 0, 0, 0, 0, -1} ) );

        // pick the bin on conveyor
        vpHomogeneousMatrix fM_eed_up_conveyor(vpTranslationVector(-0.43, 0.32, 0.15),
                                               vpRotationMatrix( {0, 1, 0, 1, 0, 0, 0, 0, -1} ) );

        // home picking position
        vpHomogeneousMatrix fM_eed_home(vpTranslationVector(-0.2, 0, 0.2), 
                                        vpRotationMatrix( {1, 0, 0, 0, -1, 0, 0, 0, -1} ) );
                                        
        // top of left tote
        vpHomogeneousMatrix fM_eed_l_tote(vpTranslationVector(0.45, -0.4, 0.15),
                                          vpRotationMatrix( {0, 1, 0, 1, 0, 0, 0, 0, -1} ) );
                                          
        // top of conveyor
        vpHomogeneousMatrix fM_eed_ConveyorTop(vpTranslationVector(0.2, 0.4, 0.1),
                                          vpRotationMatrix( {0, 1, 0, 1, 0, 0, 0, 0, -1} ) );

        // origin of left tote in robot base frame
        vpHomogeneousMatrix fM_origin_l_tote(vpTranslationVector(-0.046235, -0.23424, -0.4225),
                                      vpRotationMatrix( {0, 1, 0, -1, 0, 0, 0, 0, 1} ) );

        // Box placement in tote origin coordinates (remember that the box's frame is on its top at the center)
        // vpHomogeneousMatrix r_toteM_tag(vpTranslationVector(0.27, -0.07, 0.11),
                                        // vpRotationMatrix( {1, 0, 0, 0, 1, 0, 0, 0, 1} ) );


        cdMo = robot.get_eMc().inverse()*eedMo;
        active_cdMc = cdMo * cMo.inverse();

        // Create visual features
        vpFeatureTranslation t( vpFeatureTranslation::cdMc );
        vpFeatureThetaU tu( vpFeatureThetaU::cdRc );
        t.buildFrom( active_cdMc );
        tu.buildFrom( active_cdMc );

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
            std::cout << "Enable adaptive gain" << std::endl;
            vpAdaptiveGain lambda( 4, 1.2, 25 ); 
            task.setLambda( lambda );
        }
        else
        {
            task.setLambda( 1.2 );
        }

        vpPlot *plotter = nullptr;

        if ( opt_plot )
        {
            plotter = new vpPlot( 2, static_cast< int >( 250 * 2 ), 500, static_cast< int >( I.getWidth() ) + 80, 10,
                                  "Real time curves plotter" );
            plotter->setTitle( 0, "Visual features error" );
            plotter->setTitle( 1, "Camera velocities" );
            plotter->initGraph( 0, 6 );
            plotter->initGraph( 1, 6 );
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
        }

        bool final_quit                           = false;
        bool has_converged                        = false;
        bool send_velocities                      = false;
        bool servo_started                        = false;
        std::vector< vpImagePoint > *traj_corners = nullptr; // To memorize point trajectory
        double sim_time            = robot.getCoppeliasimSimulationTime();
        double sim_time_prev       = sim_time;
        double sim_time_init_servo = sim_time;
        double sim_time_img        = sim_time;

	// pre-define variables
        vpHomogeneousMatrix eMc, tem, fM_Y1_rotate, fM_Y0_rotate, fM_Y3_rotate;
    	double x, y, z, x0, y0, z0, x1, y1, z1;
    	
        eMc = robot.get_eMc() ;

        robot.setRobotState( vpRobot::STATE_VELOCITY_CONTROL );
        robot.setCoppeliasimSyncMode( opt_coppeliasim_sync_mode );

        int TagID = 1;
        int State = 0;
        int c_idx = 0;
        typedef struct{
            int ID;
            vpHomogeneousMatrix wMo;
            vpHomogeneousMatrix oMo;
        } scene_obj;

	// vector of all objects in the scene w.r.t. the world frame
        std::vector<scene_obj> obj_vec;  
        
        // predefine size of Boxes and the tote
        std::vector<std::vector<double>> Box = {
        {0.2, 0.2, 0.1},
        {0.15, 0.15, 0.1},
        {0.15, 0.15, 0.1}
    	};
    	std::vector<double> Tote = {0.37, 0.58, 0.249};

        vpColVector  v_0( 6 ), v_c( 6 );

        while ( !final_quit )
        {
            sim_time = robot.getCoppeliasimSimulationTime();

            g.acquire( I, sim_time_img );
            vpDisplay::display( I );

            std::vector< vpHomogeneousMatrix > cMo_vec;
            std::vector<int> TagsID;
            
            //position and pose of all detected apriltags are saved in cMo_vec, cMo_vec is based on camera frame
            detector.detect( I, opt_tagSize, cam, cMo_vec );
            TagsID = detector.getTagsId();

            {
                std::stringstream ss;
                ss << "Left click to " << ( send_velocities ? "stop the robot" : "servo the robot" )
                   << ", right click to quit.";
                vpDisplay::displayText( I, 20, 20, ss.str(), vpColor::red );
            }

            // One or more targets are detected
            if ( cMo_vec.size() >= 1 )
            {
                static bool first_time = true;
                if ( first_time )
                {
                    for(int idx=0;idx<cMo_vec.size();idx++)
                    { // scan all the objects in the FoV and add them to the object vector
                        scene_obj ob;
                        
                        // Introduce security wrt tag positionning in order to avoid PI rotation
                        std::vector<vpHomogeneousMatrix> v_oMo(2), v_ccMc(2);
                        v_oMo[1].buildFrom(0, 0, 0, 0, 0, M_PI);
                        for (size_t i = 0; i < 2; i++) {
                            v_ccMc[i] = active_cdMc * v_oMo[i] * cMo.inverse();
                        }
                        if (std::fabs(v_ccMc[0].getThetaUVector().getTheta()) < std::fabs(v_ccMc[1].getThetaUVector().getTheta())) {
                            ob.oMo = v_oMo[0];
                        }
                        else {
                            std::cout << "Desired frame modified to avoid PI rotation of the camera" << std::endl;
                            ob.oMo = v_oMo[1];   // Introduce PI rotation of frame
                        }

                        ob.ID = TagsID[idx];
                        ob.wMo = robot.get_fMe()*eMc*cMo_vec[idx];
                        obj_vec.push_back(ob);
                        vpHomogeneousMatrix tem000=fM_origin_l_tote.inverse()*obj_vec[0].wMo;
                         tem = tem000;
            		
                    }
                    State = 0;
                    active_cdMc = (fM_eed_home*eMc).inverse()*robot.get_fMe()*eMc ;//cam_home(cd) to camera (c)
                    t.buildFrom(active_cdMc);
                    tu.buildFrom(active_cdMc);
                    v_0 = task.computeControlLaw();
                    double x = (fM_origin_l_tote.inverse()*obj_vec[0].wMo)[0][3],
            	y = (fM_origin_l_tote.inverse()*obj_vec[0].wMo)[1][3],
            	z = (fM_origin_l_tote.inverse()*obj_vec[0].wMo)[2][3];
            	x0=x;
            	y0=y;
            	z0=z;
                }
                
            else{ // scan all the visible objects in the FoV and update those already in the vector and add the new objects
                    for(int idx=0;idx<cMo_vec.size();idx++){ // scan all the objects in the FoV and ...
                        bool found = false;
                        for(int j=0;j<obj_vec.size();j++){             //if one of them is already in the object vector, update it, or
                            if(obj_vec[j].ID == TagsID[idx]){obj_vec[j].wMo = robot.get_fMe()*eMc*cMo_vec[idx]; // if you assume the objects are static, you can comment this line
                                found = true;
                                break;
                            }
                        }
                        if(!found){ // if is a new element in the scene... add it in the vector
                            scene_obj ob;
                            ob.ID = TagsID[idx];
                            ob.wMo = robot.get_fMe()*eMc*cMo_vec[idx];
                            obj_vec.push_back(ob);
                        }
                    }
                }
               
                if ( first_time )
                {
                    first_time = false;
                }
            } // end if (cMo_vec.size() >= 1)
            else
            {
                // if no object is detected, do something.

            }
            
            
            // -------------------------------------------------------------------------------------------------------------
            // -------------------------------------------------------------------------------------------------------------
            // Update active pose to track <- this must be done by the FSM
            
            // Finite State Machine (FSM) States
            
            // go to top of the conveyor
            if( State == 0){
                active_cdMc = (fM_eed_ConveyorTop*eMc).inverse()*robot.get_fMe()*eMc ;
                t.buildFrom( active_cdMc );
                tu.buildFrom( active_cdMc );

                if ( !servo_started )
                {
                    if ( send_velocities )
                    {
                        servo_started = true;
                    }
                    v_0 = task.computeControlLaw();
                    sim_time_init_servo = robot.getCoppeliasimSimulationTime();
                }
		
            }
            
            // go to catch the parcel using suction cup
            else if( State == 11 ){
                cdMo = eMc.inverse()*edMo;
                cMo = (robot.get_fMe()*eMc).inverse() * obj_vec[c_idx].wMo;
                active_cdMc = cdMo * obj_vec[c_idx].oMo * cMo.inverse();
                t.buildFrom(active_cdMc);
                tu.buildFrom(active_cdMc);
                if ( !servo_started )
                {
                    if ( send_velocities )
                    {
                        servo_started = true;
                    }
                    v_0 = task.computeControlLaw();
                    sim_time_init_servo = robot.getCoppeliasimSimulationTime();
                }
            }
            
            // back to top of the conveyor, similar to interpolation
            else if( State == 12 ){
                active_cdMc = (fM_eed_ConveyorTop*eMc).inverse()*robot.get_fMe()*eMc ;
                t.buildFrom(active_cdMc);
                tu.buildFrom(active_cdMc);
                if ( !servo_started )
                {
                    if ( send_velocities )
                    {
                        servo_started = true;
                    }
                    v_0 = task.computeControlLaw();
                    sim_time_init_servo = robot.getCoppeliasimSimulationTime();
                }
            }
            
            // go to top of the left of the tote
            else if( State == 13 ){
                active_cdMc = (fM_eed_l_tote*eMc).inverse()*robot.get_fMe()*eMc ;
                t.buildFrom(active_cdMc);
                tu.buildFrom(active_cdMc);
                if ( !servo_started )
                {
                    if ( send_velocities )
                    {
                        servo_started = true;
                    }
                    v_0 = task.computeControlLaw();
                    sim_time_init_servo = robot.getCoppeliasimSimulationTime();
                }
            }
            
            // Go to target position above by 0.02 meters, prepare for translation push (1)
            else if( State == 14 ){
        	vpHomogeneousMatrix fM_Push0 = getMatrixPush0(Box[0], Box[1], Box[2], x0, y0, z0);
                active_cdMc = (fM_origin_l_tote* fM_Push0*edMo.inverse()*eMc).inverse()*robot.get_fMe()*eMc;
                t.buildFrom(active_cdMc);
                tu.buildFrom(active_cdMc);
                if ( !servo_started )
                {
                    if ( send_velocities )
                    {
                        servo_started = true;
                    }
                    v_0 = task.computeControlLaw();
                    sim_time_init_servo = robot.getCoppeliasimSimulationTime();
                }
            }
            
            // Go to suitable height, preparing for translation push, prepare for translation push (2)
            else if( State == 15 ){
        	vpHomogeneousMatrix fM_Push1 = getMatrixPush1(Box[0], Box[1], Box[2], x0, y0, z0);
		active_cdMc = (fM_origin_l_tote* fM_Push1*edMo.inverse()*eMc).inverse()*robot.get_fMe()*eMc ; 
                t.buildFrom(active_cdMc);
                tu.buildFrom(active_cdMc);
                if ( !servo_started )
                {
                    if ( send_velocities )
                    {
                        servo_started = true;
                    }
                    v_0 = task.computeControlLaw();
                    sim_time_init_servo = robot.getCoppeliasimSimulationTime();
                }
            }
            
            // translation motion 
            else if( State == 16 ){
        	vpHomogeneousMatrix fM_Push1 = getMatrixPush2(Box[0], Box[1], Box[2], x0, y0, z0);
		active_cdMc = (fM_origin_l_tote* fM_Push1*edMo.inverse()*eMc).inverse()*robot.get_fMe()*eMc ;
                t.buildFrom(active_cdMc);
                tu.buildFrom(active_cdMc);
                if ( !servo_started )
                {
                    if ( send_velocities )
                    {
                        servo_started = true;
                    }
                    v_0 = task.computeControlLaw();
                    sim_time_init_servo = robot.getCoppeliasimSimulationTime();
                }
            }
            
            // placing to side
            else if( State == 17 ){
        	vpHomogeneousMatrix fM_Push1 = getMatrixPush3(Box[0], Box[1], Box[2], x0, y0, z0);
		active_cdMc = (fM_origin_l_tote* fM_Push1*edMo.inverse()*eMc).inverse()*robot.get_fMe()*eMc ;
                t.buildFrom(active_cdMc);
                tu.buildFrom(active_cdMc);
                if ( !servo_started )
                {
                    if ( send_velocities )
                    {
                        servo_started = true;
                    }
                    v_0 = task.computeControlLaw();
                    sim_time_init_servo = robot.getCoppeliasimSimulationTime();
                }
            }
            
            // back to top of the left tote
            else if( State == 18 ){
        	active_cdMc = (fM_eed_l_tote*eMc).inverse()*robot.get_fMe()*eMc ;
                t.buildFrom(active_cdMc);
                tu.buildFrom(active_cdMc);
                if ( !servo_started )
                {
                    if ( send_velocities )
                    {
                        servo_started = true;
                    }
                    v_0 = task.computeControlLaw();
                    sim_time_init_servo = robot.getCoppeliasimSimulationTime();
                }
            }
            
            // back to top of the conveyor, prepare for capturing next parcel using camera
            else if( State == 10){
                active_cdMc = (fM_eed_ConveyorTop*eMc).inverse()*robot.get_fMe()*eMc ;
                t.buildFrom(active_cdMc);
                tu.buildFrom(active_cdMc);
                if ( !servo_started )
                {
                    if ( send_velocities )
                    {
                        servo_started = true;
                    }
                    v_0 = task.computeControlLaw();
                    sim_time_init_servo = robot.getCoppeliasimSimulationTime();
                }
           }
           
           
            // pick next parcel
            else if( State == 1 ){
                cdMo = eMc.inverse()*edMo;
                cMo = (robot.get_fMe()*eMc).inverse() * obj_vec[c_idx].wMo;
                active_cdMc = cdMo * obj_vec[c_idx].oMo * cMo.inverse();
                t.buildFrom(active_cdMc);
                tu.buildFrom(active_cdMc);
                if ( !servo_started )
                {
                    if ( send_velocities )
                    {
                        servo_started = true;
                    }
                    v_0 = task.computeControlLaw();
                    sim_time_init_servo = robot.getCoppeliasimSimulationTime();
                }
            }
            
            // back to top of the conveyor
            else if( State == 2 ){
                active_cdMc = (fM_eed_ConveyorTop*eMc).inverse()*robot.get_fMe()*eMc ;
                t.buildFrom(active_cdMc);
                tu.buildFrom(active_cdMc);
                if ( !servo_started )
                {
                    if ( send_velocities )
                    {
                        servo_started = true;
                    }
                    v_0 = task.computeControlLaw();
                    sim_time_init_servo = robot.getCoppeliasimSimulationTime();
                }
            }
            
            // go to the top of the left tote
            else if( State == 3 ){
                active_cdMc = (fM_eed_l_tote*eMc).inverse()*robot.get_fMe()*eMc ;
                t.buildFrom(active_cdMc);
                tu.buildFrom(active_cdMc);
                if ( !servo_started )
                {
                    if ( send_velocities )
                    {
                        servo_started = true;
                    }
                    v_0 = task.computeControlLaw();
                    sim_time_init_servo = robot.getCoppeliasimSimulationTime();
                }
            }
            
            // Go to target position above by 0.45 meters without any tilt angle
            else if( State == 4 ){//fM_origin_l_tote.inverse()*obj_vec[0].wMo)[0]: new position after translation motion
            	vpHomogeneousMatrix fM_Y0_rotate = getMatrixY0(Box[0], Box[1], (fM_origin_l_tote.inverse()*obj_vec[0].wMo)[0][3], (fM_origin_l_tote.inverse()*obj_vec[0].wMo)[1][3], (fM_origin_l_tote.inverse()*obj_vec[0].wMo)[2][3]);
                active_cdMc = (fM_origin_l_tote* fM_Y0_rotate*edMo.inverse()*eMc).inverse()*robot.get_fMe()*eMc ; //translation (prepare for rotating)
                t.buildFrom(active_cdMc);
                tu.buildFrom(active_cdMc);
                if ( !servo_started )
                {
                    if ( send_velocities )
                    {
                        servo_started = true;
                    }
                    v_0 = task.computeControlLaw();
                    sim_time_init_servo = robot.getCoppeliasimSimulationTime();
                }
            }
            
            // Going down with rotating
            else if( State == 5 ){
                vpHomogeneousMatrix fM_Y1_rotate = getMatrixY1(Box[0], Box[1], (fM_origin_l_tote.inverse()*obj_vec[0].wMo)[0][3], (fM_origin_l_tote.inverse()*obj_vec[0].wMo)[1][3], (fM_origin_l_tote.inverse()*obj_vec[0].wMo)[2][3]);
                active_cdMc = (fM_origin_l_tote*fM_Y1_rotate*edMo.inverse()*eMc).inverse()*robot.get_fMe()*eMc ; //rotating and go down
                t.buildFrom(active_cdMc);
                tu.buildFrom(active_cdMc);
                if ( !servo_started )
                {
                    if ( send_velocities )
                    {
                        servo_started = true;
                    }
                    v_0 = task.computeControlLaw();
                    sim_time_init_servo = robot.getCoppeliasimSimulationTime();
                }
            }
            
            //go up without rotating
            else if( State == 6 ){
                vpHomogeneousMatrix fM_Y2_rotate = getMatrixY2(Box[0], Box[1], (fM_origin_l_tote.inverse()*obj_vec[0].wMo)[0][3], (fM_origin_l_tote.inverse()*obj_vec[0].wMo)[1][3], (fM_origin_l_tote.inverse()*obj_vec[0].wMo)[2][3]);
                active_cdMc = (fM_origin_l_tote* fM_Y2_rotate*edMo.inverse()*eMc).inverse()*robot.get_fMe()*eMc ; //go up without rotating
                t.buildFrom(active_cdMc);
                tu.buildFrom(active_cdMc);
                if ( !servo_started )
                {
                    if ( send_velocities )
                    {
                        servo_started = true;
                    }
                    v_0 = task.computeControlLaw();
                    sim_time_init_servo = robot.getCoppeliasimSimulationTime();
                }
            }
            
            //Rorating back
            else if( State == 7 ){
        	vpHomogeneousMatrix fM_Y3_rotate = getMatrixY3(Box[0], Box[1], (fM_origin_l_tote.inverse()*obj_vec[0].wMo)[0][3], (fM_origin_l_tote.inverse()*obj_vec[0].wMo)[1][3], (fM_origin_l_tote.inverse()*obj_vec[0].wMo)[2][3]);
                active_cdMc = (fM_origin_l_tote* fM_Y3_rotate*edMo.inverse()*eMc).inverse()*robot.get_fMe()*eMc ; //rorate back
                t.buildFrom(active_cdMc);
                tu.buildFrom(active_cdMc);
                if ( !servo_started )
                {
                    if ( send_velocities )
                    {
                        servo_started = true;
                    }
                    v_0 = task.computeControlLaw();
                    sim_time_init_servo = robot.getCoppeliasimSimulationTime();
                }
            }
            
            //after having enough width, go down
            else if( State == 8 ){
        	vpHomogeneousMatrix fM_Y4_rotate = getMatrixY4(Box[0], Box[1], (fM_origin_l_tote.inverse()*obj_vec[0].wMo)[0][3], (fM_origin_l_tote.inverse()*obj_vec[0].wMo)[1][3], (fM_origin_l_tote.inverse()*obj_vec[0].wMo)[2][3]);
                active_cdMc = (fM_origin_l_tote* fM_Y4_rotate*edMo.inverse()*eMc).inverse()*robot.get_fMe()*eMc ; //go down to place box
                t.buildFrom(active_cdMc);
                tu.buildFrom(active_cdMc);
                if ( !servo_started )
                {
                    if ( send_velocities )
                    {
                        servo_started = true;
                    }
                    v_0 = task.computeControlLaw();
                    sim_time_init_servo = robot.getCoppeliasimSimulationTime();
                }
            }
            
            // do translation motion again and place the parcel
            else if( State == 19 ){
        	vpHomogeneousMatrix fM_Y5_push = getMatrixY5(Box[0], Box[1], (fM_origin_l_tote.inverse()*obj_vec[0].wMo)[0][3], (fM_origin_l_tote.inverse()*obj_vec[0].wMo)[1][3], (fM_origin_l_tote.inverse()*obj_vec[0].wMo)[2][3]);
                active_cdMc = (fM_origin_l_tote* fM_Y5_push*edMo.inverse()*eMc).inverse()*robot.get_fMe()*eMc ; //go down to place box
                t.buildFrom(active_cdMc);
                tu.buildFrom(active_cdMc);
                if ( !servo_started )
                {
                    if ( send_velocities )
                    {
                        servo_started = true;
                    }
                    v_0 = task.computeControlLaw();
                    sim_time_init_servo = robot.getCoppeliasimSimulationTime();
                }
            }
            
            // back to top of the left tote
            else if( State == 20 ){
        	active_cdMc = (fM_eed_l_tote*eMc).inverse()*robot.get_fMe()*eMc ;
                t.buildFrom(active_cdMc);
                tu.buildFrom(active_cdMc);
                if ( !servo_started )
                {
                    if ( send_velocities )
                    {
                        servo_started = true;
                    }
                    v_0 = task.computeControlLaw();
                    sim_time_init_servo = robot.getCoppeliasimSimulationTime();
                }
            }
            
            // back to top of the conveyor
            else if( State == 9){
                active_cdMc = (fM_eed_ConveyorTop*eMc).inverse()*robot.get_fMe()*eMc ;
                t.buildFrom(active_cdMc);
                tu.buildFrom(active_cdMc);
                if ( !servo_started )
                {
                    if ( send_velocities )
                    {
                        servo_started = true;
                    }
                    v_0 = task.computeControlLaw();
                    sim_time_init_servo = robot.getCoppeliasimSimulationTime();
                }
            }else if( State == 100 ){ // idle state
                if( !has_converged ){
                    std::cout << "State: Idle \n";
                    has_converged = true;
                }
                active_cdMc.eye();//do nothing
                t.buildFrom(active_cdMc);
                tu.buildFrom(active_cdMc);
                v_c = 0;
            }
            
            double error_tr  = sqrt( active_cdMc.getTranslationVector().sumSquare() );           // translation error
            double error_tu  = vpMath::deg( sqrt( active_cdMc.getThetaUVector().sumSquare() ) ); // orientation error

            // FSM transition conditions
            if(error_tr <= 1 && error_tu <= 5 && State == 0)// once reached the top of the right tote, go to pick the parcel on the conveyor
            {
                std::cout << "Left tote reached... moving to the next state \n";

                bool found = false;
                for(int idx=0;idx<obj_vec.size();idx++){  // search in the object vector if the next object already exist
                    if(obj_vec[idx].ID == TagID){
                        std::cout << "The parcel is visible and I know where to go to pick it! \n";
                        State = 11; // New state
                        TagID = 1; // box with tag ID = 1 that have to be picked
                        c_idx = idx;
                        found = true;
                        servo_started = false;
                        break;
                    }
                }
                if ( !found ){
                    std::cout << "No box was found to pick, look somewhere else... \n";
                    // here you can implement an alternative motion to look for the packages around.
                    // for the moment, we suppose the box is always there
                    State = 100;
                    v_c = 0;
                }


            }else if(error_tr <= 0.001 && error_tu <= 1 && State == 11){ // once you reach the box to pick, activate vacuum and move up

                std::cout << "On top of the parcel to pick \n";
                std::cout << "Activating suction pad \n";
                std::cout << "Parcel was picked, going to lift it \n";
                activate.data = 1;
                pub_suctionpad.publish(activate);
                State = 12;
                servo_started = false;

            }else if(error_tr <= 0.1 && error_tu <= 1 && State == 12){ // once reached the position above the right tote, move to the left tote

                std::cout << "Moving the parcel to the left tote \n";
                State = 13;
                servo_started = false;

            }else if(error_tr <= 0.1 && error_tu <= 1 && State == 13){ // once reached the position above the left tote, prepare for translation push

                std::cout << "prepare for translation push 1\n";
                State = 14;
                servo_started = false;

            }else if(error_tr <= 0.001 && error_tu <= 0.5 && State == 14){ 
                std::cout << "prepare for translation push 2\n";
                State = 15;
                servo_started = false;
                
            }
            else if(error_tr <= 0.001 && error_tu <= 0.1 && State == 15){ 

                std::cout << "translating\n";
                State = 16;
                servo_started = false;

            }
            else if(((error_tr <= 0.001 && error_tu <= 0.1) || abs(ft_sensor[1]) >= 1.5 || abs(ft_sensor[0]) >= 1.5) && State == 16){ // stop based on force sensor

                std::cout << "placing\n";
                State = 17;
                servo_started = false;

            }else if(((error_tr <= 0.001 && error_tu <= 0.1) || abs(ft_sensor[1]) >= 1.5 || abs(ft_sensor[0]) >= 1.5) && State == 17){ // once you have placed the box, go to top of left tote

                std::cout << "go to top of left tote\n";
                activate.data = 0;
                pub_suctionpad.publish(activate);
                State = 18;
                servo_started = false;

            }else if(error_tr <= 0.2 && error_tu <= 5 && State == 18){ // once you have placed the box, go to top of conveyor

                std::cout << "go to top of conveyor\n";
                State = 10;
                servo_started = false;

            }else if(error_tr <= 0.01 && error_tu <= 5 && State == 10){ //once top of conveyor is reached, go to pick next box on the conveyor
            
                std::cout << "picking next box \n";
                TagID =2;
                bool found = false;
                for(int idx=0;idx<obj_vec.size();idx++){  // search in the object vector if the next object already exist
                    if ( obj_vec[idx].ID == TagID )
                    {
                        std::cout << "Next Box is visible and I know where to go to pick it! \n";
                        State         = 1;     // New state
                        TagID         = 2; // box with tag ID have to be picked in order
                        c_idx         = idx;
                        std::cout << "c_idx:\n"<< c_idx << std::endl;
                        found         = true;
                        servo_started = false;
                        break;
                    }

                }
                if ( !found ){
                    std::cout << "No box was found to pick, look somewhere else... \n";
                    // here you can implement an alternative motion to look for the packages around.
                    // for the moment, we suppose the box is always there
                    State = 100;
                    v_c = 0;
                }
            }
            
// picking next box and do rotation
            else if(error_tr <= 0.001 && error_tu <= 1 && State == 1){ // once you reach the box to pick, activate vacuum and move up

                std::cout << "On top of the parcel to pick \n";
                std::cout << "Activating suction pad \n";
                std::cout << "Parcel was picked, going to lift it \n";
                activate.data = 1;
                pub_suctionpad.publish(activate);
                State = 2;
                servo_started = false;

            }else if(error_tr <= 0.2 && error_tu <= 5 && State == 2){ // once reached the position above the right tote, move to the left tote

                std::cout << "Moving the parcel to the left tote \n";
                State = 3;
                servo_started = false;

            }else if(error_tr <= 0.2 && error_tu <= 5 && State == 3){ // once reached the position above the left tote, prepare for rotating

                std::cout << "prepare for rotating \n";
                State = 4;
                servo_started = false;

            }else if(error_tr <= 0.001 && error_tu <= 0.1 && State == 4){ // once you finished preparation, rotate and go down
                //activate.data = 0;
                std::cout << "rotating and go down\n";
                State = 5;
                servo_started = false;

            }
            else if(error_tr <= 0.001 && error_tu <= 0.1 && State == 5){ //push another box using roration, go up without rotating

                std::cout << "go up without rotating\n";
                State = 6;
                servo_started = false;

            }
            else if(error_tr <= 0.001 && error_tu <= 1 && State == 6){ // once you have gone up, rotate back

                std::cout << "rotate back\n";
                State = 7;
                servo_started = false;
                
            }
            else if(error_tr <= 0.002 && error_tu <= 1 && State == 7){ // once you have rotated back, go down

                std::cout << "go down\n";
                State = 8;
                servo_started = false;
                
            }
            else if(error_tr <= 0.002 && error_tu <= 1 && State == 8){ // once you have gone down, do translation motion again

                std::cout << "do translation motion again\n";
                State = 19;
                servo_started = false;
                
            }
            else if((error_tr <= 0.001 && error_tu <= 0.5|| abs(ft_sensor[1]) >= 2 || abs(ft_sensor[0]) >= 2) && State == 19){ // once you have reached the side, go to top of left tote

                std::cout << "go to top of left tote\n";
                activate.data = 0;
                pub_suctionpad.publish(activate);
                State = 20;
                servo_started = false;

            }else if(error_tr <= 0.2 && error_tu <= 5 && State == 20){ // once you have reached top of left tote, go to top of conveyor for searching next parcel

                std::cout << "go to top of conveyor for searching next parcel\n";
                State = 9;
                servo_started = false;
                
            }else if(error_tr <= 0.1 && error_tu <= 5 && State == 9){ // once you have placed the box, got o home position
                std::cout << "Parcel placed... Deactivating vacuum\n";
                std::cout << "Home position reached, going Idle \n";
                State = 100; //

            }
            
            
            v_c = task.computeControlLaw() - v_0*exp(-8.0*(robot.getCoppeliasimSimulationTime() - sim_time_init_servo));	// -10.0 for slower

            // Display the current and desired feature points in the image display
            // Display desired and current pose features
            vpDisplay::displayFrame( I, cdMo * oMo, cam, opt_tagSize / 1.5, vpColor::yellow, 2 );
            vpDisplay::displayFrame( I, cMo, cam, opt_tagSize / 2, vpColor::none, 3 );

            if ( opt_plot )
            {
                plotter->plot( 0, static_cast< double >( sim_time ), task.getError() );
                plotter->plot( 1, static_cast< double >( sim_time ), v_c );
            }



            std::stringstream ss;
            ss << "error_t: " << error_tr;
            vpDisplay::displayText( I, 20, static_cast< int >( I.getWidth() ) - 150, ss.str(), vpColor::red );
            ss.str( "" );
            ss << "error_tu: " << error_tu;
            vpDisplay::displayText( I, 40, static_cast< int >( I.getWidth() ) - 150, ss.str(), vpColor::red );
            ss.str( "" );
            ss << "force x: " << ft_sensor[0];
            vpDisplay::displayText( I, 60, static_cast< int >( I.getWidth() ) - 150, ss.str(), vpColor::red );
            ss.str( "" );
            ss << "force y: " << ft_sensor[1];
            vpDisplay::displayText( I, 80, static_cast< int >( I.getWidth() ) - 150, ss.str(), vpColor::red );
            ss.str( "" );
            ss << "force z: " << ft_sensor[2];
            vpDisplay::displayText( I, 100, static_cast< int >( I.getWidth() ) - 150, ss.str(), vpColor::red );
            ss.str( "" );
            ss << "current state: " << State;
            vpDisplay::displayText( I, 120, static_cast< int >( I.getWidth() ) - 150, ss.str(), vpColor::red );


            // -------------------------------------------------------------------------------------------------------------
            // -------------------------------------------------------------------------------------------------------------

            if ( !send_velocities )
            {
                v_c = 0; // Stop the robot
                servo_started = false;
            }

            robot.setVelocity( vpRobot::CAMERA_FRAME, v_c );

//      std::stringstream ss;
            ss << "Loop time [s]: " << std::round( ( sim_time - sim_time_prev ) * 1000. ) / 1000.;
            ss << " Simulation time [s]: " << sim_time;
            sim_time_prev = sim_time;
            vpDisplay::displayText( I, 40, 20, ss.str(), vpColor::red );

            vpMouseButton::vpMouseButtonType button;
            if ( vpDisplay::getClick( I, button, false ) )
            {
                switch ( button )
                {
                    case vpMouseButton::button1:
                        send_velocities = !send_velocities;
                        break;

                    case vpMouseButton::button3:
                        final_quit = true;
                        v_c        = 0;
                        break;

                    default:
                        break;
                }
            }

            vpDisplay::flush( I );
            robot.wait( sim_time, 0.020 ); // Slow down the loop to simulate a camera at 50 Hz
        }                                // end while

        if ( opt_plot && plotter != nullptr )
        {
            delete plotter;
            plotter = nullptr;
        }
        robot.coppeliasimStopSimulation();

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
