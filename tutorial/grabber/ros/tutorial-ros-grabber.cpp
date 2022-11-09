//! \example tutorial-ros-grabber.cpp
#include <visp3/core/vpIoTools.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>

#include <visp_ros/vpROSGrabber.h>

int
main( int argc, char **argv )
{
  try
  {
    std::string opt_image_raw_topic      = "/image_raw";
    bool opt_subscribe_camera_info_topic = false;
    bool opt_rectify                     = false;
    bool opt_flip                        = false;
    for ( int i = 0; i < argc; i++ )
    {
      if ( std::string( argv[i] ) == "--rectify" )
      {
        opt_rectify = true;
      }
      else if ( std::string( argv[i] ) == "--flip" )
      {
        opt_flip = true;
      }
      else if ( std::string( argv[i] ) == "--subscribe-camera-info" )
      {
        opt_subscribe_camera_info_topic = true;
      }
      else if ( std::string( argv[i] ) == "--camera-topic" )
      {
        opt_image_raw_topic = std::string( argv[i + 1] );
        i++;
      }
      else if ( std::string( argv[i] ) == "--help" || std::string( argv[i] ) == "-h" )
      {
        std::cout << "SYNOPSIS " << std::endl
                  << "  " << argv[0]
                  << " [--camera-topic <topic>] [--subscribe-camera-info] [--rectify] [--flip] [--help] [-h]"
                  << std::endl
                  << std::endl;
        std::cout << "DESCRIPTION" << std::endl
                  << "  Subscribe to a ROS image topic and display the image." << std::endl
                  << std::endl;
        std::cout << "  --camera-topic <topic>" << std::endl
                  << "\tROS image raw topic name to subscribe." << std::endl
                  << "\tDefault: " << opt_image_raw_topic << std::endl
                  << std::endl;
        std::cout << "  --subscribe-camera-info" << std::endl
                  << "\tSubscribe to camera_info topic." << std::endl
                  << std::endl;
        std::cout << "  --rectify" << std::endl
                  << "\tSubscribe to camera_info topic and rectify the image." << std::endl
                  << std::endl;
        std::cout << "  --flip" << std::endl << "\tFlip image." << std::endl << std::endl;
        std::cout << "  --help, -h" << std::endl << "\tDisplay this helper message" << std::endl;
        return EXIT_SUCCESS;
      }
    }
    std::cout << "Use cam info : " << ( ( opt_subscribe_camera_info_topic == true ) ? "yes" : "no" ) << std::endl;
    std::cout << "Rectify image: " << ( ( opt_rectify == true ) ? "yes (using camera_info topic)" : "no" ) << std::endl;
    std::cout << "Flip image   : " << ( ( opt_flip == true ) ? "yes" : "no" ) << std::endl;

    // vpImage< unsigned char > I; // Create a gray level image container
    vpImage< vpRGBa > I; // Create a color image container

    //! [Construction]
    vpROSGrabber g;
    //! [Construction]
    //! [Setting camera topic]
    g.subscribeImageTopic( opt_image_raw_topic );
    g.subscribeCameraInfoTopic( opt_subscribe_camera_info_topic );
    //! [Setting camera topic]
    if ( opt_rectify )
    {
      if ( opt_subscribe_camera_info_topic )
      {
        g.setRectify( true );
      }
      else
      {
        std::cout << "Cannot rectify images without enabling camera info topic subscription" << std::endl;
        std::cout << "Use --subscribe-camera-info command line option to overcome this error!" << std::endl;
        return EXIT_FAILURE;
      }
    }
    if ( opt_flip )
    {
      g.setFlip( true );
    }
    //! [Opening]
    g.open( argc, argv );
    //! [Opening]

    g.acquire( I );
    std::cout << "Image size: " << I.getWidth() << " " << I.getHeight() << std::endl;

    vpCameraParameters cam;
    if ( g.getCameraInfo( cam ) )
    {
      std::cout << "Camera parameters: \n" << cam << std::endl;
    }
    else
    {
      std::cout << "Camera parameters are not available. Are you publishing them on cam_info topic?" << std::endl;
    }

#if defined( VISP_HAVE_X11 )
    vpDisplayX d( I );
#elif defined( VISP_HAVE_GDI )
    vpDisplayGDI d( I );
#elif defined( VISP_HAVE_OPENCV )
    vpDisplayOpenCV d( I );
#else
    std::cout << "No image viewer is available..." << std::endl;
#endif

    vpDisplay::setTitle( I, opt_image_raw_topic );

    while ( 1 )
    {
      //! [Acquisition]
      g.acquire( I );
      //! [Acquisition]
      vpDisplay::display( I );
      vpDisplay::displayText( I, 20, 20, "A click to quit...", vpColor::red );
      vpDisplay::flush( I );
      if ( vpDisplay::getClick( I, false ) )
        break;
    }
  }
  catch ( const vpException &e )
  {
    std::cout << "Catch an exception: " << e.what() << std::endl;
  }
  catch ( const rclcpp::exceptions::RCLError &e )
  {
    std::cout << "Catch an rclcpp exception: " << e.what() << std::endl;
  }
  catch ( const rclcpp::exceptions::RCLInvalidArgument &e )
  {
    std::cout << "Catch an rclcpp exception: " << e.what() << std::endl;
  }
  catch ( std::runtime_error &e )
  {
    std::cout << "Catch runtime error: " << e.what() << std::endl;
  }
}
