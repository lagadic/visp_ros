
#include <iostream>

#include <visp3/blob/vpDot2.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpVelocityTwistMatrix.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeatureDepth.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/vs/vpServo.h>

#include <visp_ros/vpROSGrabber.h>
#include <visp_ros/vpROSRobotPioneer.h>

#if defined( VISP_HAVE_DC1394_2 ) && defined( VISP_HAVE_X11 )
#define TEST_COULD_BE_ACHIEVED
#endif

/*!
  \example tutorial_ros_node_pioneer_visual_servo.cpp

  Example that shows how to create a ROS node able to control the Pioneer mobile robot by IBVS visual servoing with
  respect to a blob. The current visual features that are used are s = (x, log(Z/Z*)). The desired one are s* = (x*, 0),
  with:
  - x the abscisse of the point corresponding to the blob center of gravity measured at each iteration,
  - x* the desired abscisse position of the point (x* = 0)
  - Z the depth of the point measured at each iteration
  - Z* the desired depth of the point equal to the initial one.

  The degrees of freedom that are controlled are (vx, wz), where wz is the rotational velocity
  and vx the translational velocity of the mobile platform at point M located at the middle
  between the two wheels.

  The feature x allows to control wy, while log(Z/Z*) allows to control vz.
  The value of x is measured thanks to a blob tracker.
  The value of Z is estimated from the surface of the blob that is proportional to the depth Z.

  */
#ifdef TEST_COULD_BE_ACHIEVED
int
main( int argc, char **argv )
{
  try
  {
    vpImage< unsigned char > I; // Create a gray level image container
    double depth  = 1.;
    double lambda = 0.6;
    double coef   = 1. / 6.77; // Scale parameter used to estimate the depth Z of the blob from its surface

    vpROSRobotPioneer robot;
    robot.setCmdVelTopic( "/RosAria/cmd_vel" );
    robot.init( argc, argv );

    // Wait 3 sec to be sure that the low level Aria thread used to control
    // the robot is started. Without this delay we experienced a delay (arround 2.2 sec)
    // between the velocity send to the robot and the velocity that is really applied
    // to the wheels.
    vpTime::sleepMs( 3000 );

    std::cout << "Robot connected" << std::endl;

    // Camera parameters. In this experiment we don't need a precise calibration of the camera
    vpCameraParameters cam;

    // Create a grabber based on libdc1394-2.x third party lib (for firewire cameras under Linux)
    vpROSGrabber g;
    g.setCameraInfoTopic( "/camera/camera_info" );
    g.setImageTopic( "/camera/image_raw" );
    g.setRectify( true );
    g.open( argc, argv );
    // Get camera parameters from /camera/camera_info topic
    if ( g.getCameraInfo( cam ) == false )
      cam.initPersProjWithoutDistortion( 600, 600, I.getWidth() / 2, I.getHeight() / 2 );

    g.acquire( I );

    // Create an image viewer
    vpDisplayX d( I, 10, 10, "Current frame" );
    vpDisplay::display( I );
    vpDisplay::flush( I );

    // Create a blob tracker
    vpDot2 dot;
    dot.setGraphics( true );
    dot.setComputeMoments( true );
    dot.setEllipsoidShapePrecision( 0. );       // to track a blob without any constraint on the shape
    dot.setGrayLevelPrecision( 0.9 );           // to set the blob gray level bounds for binarisation
    dot.setEllipsoidBadPointsPercentage( 0.5 ); // to be accept 50% of bad inner and outside points with bad gray level
    dot.initTracking( I );
    vpDisplay::flush( I );

    vpServo task;
    task.setServo( vpServo::EYEINHAND_L_cVe_eJe );
    task.setInteractionMatrixType( vpServo::DESIRED, vpServo::PSEUDO_INVERSE );
    task.setLambda( lambda );
    vpVelocityTwistMatrix cVe;
    cVe = robot.get_cVe();
    task.set_cVe( cVe );

    std::cout << "cVe: \n" << cVe << std::endl;

    vpMatrix eJe;
    robot.get_eJe( eJe );
    task.set_eJe( eJe );
    std::cout << "eJe: \n" << eJe << std::endl;

    // Current and desired visual feature associated to the x coordinate of the point
    vpFeaturePoint s_x, s_xd;

    // Create the current x visual feature
    vpFeatureBuilder::create( s_x, cam, dot );

    // Create the desired x* visual feature
    s_xd.buildFrom( 0, 0, depth );

    // Add the feature
    task.addFeature( s_x, s_xd );

    // Create the current log(Z/Z*) visual feature
    vpFeatureDepth s_Z, s_Zd;
    // Surface of the blob estimated from the image moment m00 and converted in meters
    double surface = 1. / sqrt( dot.m00 / ( cam.get_px() * cam.get_py() ) );
    double Z, Zd;
    // Initial depth of the blob in from of the camera
    Z = coef * surface;
    // Desired depth Z* of the blob. This depth is learned and equal to the initial depth
    Zd = Z;

    std::cout << "Z " << Z << std::endl;
    s_Z.buildFrom( s_x.get_x(), s_x.get_y(), Z, 0 );   // log(Z/Z*) = 0 that's why the last parameter is 0
    s_Zd.buildFrom( s_x.get_x(), s_x.get_y(), Zd, 0 ); // log(Z/Z*) = 0 that's why the last parameter is 0

    // Add the feature
    task.addFeature( s_Z, s_Zd );

    vpColVector v; // vz, wx

    while ( 1 )
    {
      // Acquire a new image
      g.acquire( I );

      // Set the image as background of the viewer
      vpDisplay::display( I );

      // Does the blob tracking
      dot.track( I );
      // Update the current x feature
      vpFeatureBuilder::create( s_x, cam, dot );

      // Update log(Z/Z*) feature. Since the depth Z change, we need to update the intection matrix
      surface = 1. / sqrt( dot.m00 / ( cam.get_px() * cam.get_py() ) );
      Z       = coef * surface;
      s_Z.buildFrom( s_x.get_x(), s_x.get_y(), Z, log( Z / Zd ) );

      robot.get_cVe( cVe );
      task.set_cVe( cVe );

      robot.get_eJe( eJe );
      task.set_eJe( eJe );

      // Compute the control law. Velocities are computed in the mobile robot reference frame
      v = task.computeControlLaw();

      std::cout << "Send velocity to the pionner: " << v[0] << " m/s " << vpMath::deg( v[1] ) << " deg/s" << std::endl;

      // Send the velocity to the robot
      robot.setVelocity( vpRobot::REFERENCE_FRAME, v );

      // Draw a vertical line which corresponds to the desired x coordinate of the dot cog
      vpDisplay::displayLine( I, 0, 320, 479, 320, vpColor::red );
      vpDisplay::flush( I );

      // A click in the viewer to exit
      if ( vpDisplay::getClick( I, false ) )
        break;
    }

    std::cout << "Ending robot thread..." << std::endl;

    // Kill the servo task
    task.print();
    task.kill();
  }
  catch ( vpException e )
  {
    std::cout << "Catch an exception: " << e << std::endl;
    return 1;
  }
}
#else
int
main()
{
  std::cout << "You don't have the right 3rd party libraries to run this example..." << std::endl;
}
#endif
