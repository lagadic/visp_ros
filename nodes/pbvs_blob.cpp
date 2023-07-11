#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros/console.h>
#include <std_msgs/msg/int8.hpp>

#include <cv_bridge/cv_bridge.h>

#include <visp_bridge/3dpose.h>
#include <visp_bridge/camera.h>

#include <visp_ros/BlobTracker.h>

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/visual_features/vpFeatureThetaU.h>
#include <visp3/visual_features/vpFeatureTranslation.h>
#include <visp3/vs/vpAdaptiveGain.h>
#include <visp3/vs/vpServo.h>

class VS
{
private:
  ros::NodeHandle m_nh;
  ros::Publisher m_pubTwist;   // cmd_vel
  ros::Subscriber m_subData;   // pose_stamped
  ros::Subscriber m_subStatus; // tracker status (1 OK, 0 NOK)

  vpServo m_task;
  vpFeatureTranslation m_s_t, m_s_t_d;
  vpFeatureThetaU m_s_tu, m_s_tu_d;

  double m_lambda;
  vpAdaptiveGain m_lambda_adapt;

  vpColVector m_v;

  vpHomogeneousMatrix m_cMo, m_cdMc, m_cdMo;
  bool m_init;
  bool m_enable_servo;

  double m_t_x_d, m_t_y_d, m_t_z_d;    // Desired translation as a vpTranslationVector in [m]
  double m_tu_x_d, m_tu_y_d, m_tu_z_d; // Desired rotation as a vpThetaUVector in [deg]

  double m_square_size;

  double m_cam_px;
  double m_cam_py;
  double m_cam_u0;
  double m_cam_v0;
  double m_cam_kud;
  double m_cam_kdu;

  vpCameraParameters m_cam;
  vpImage< unsigned char > m_I_grayscale;
#ifdef VISP_HAVE_X11
  vpDisplayX *m_display;
#elif defined( VISP_HAVE_GDI )
  vpDisplayGDI *m_display;
#elif defined( VISP_HAVE_OPENCV )
  vpDisplayOpenCV *m_display;
#endif

  unsigned int m_thickness;
  std::vector< std::vector< vpImagePoint > > ips_trajectory;

  bool m_quit;

public:
  void init_vs();
  void init_display();
  void display( const visp_ros::BlobTracker::ConstPtr &msg );
  void data_callback( const visp_ros::BlobTracker::ConstPtr &msg );
  void status_callback( const std_msgs::Int8ConstPtr &msg );
  void spin();
  VS( int argc, char **argv );
  virtual ~VS() { m_task.kill(); };
};

VS::VS( int argc, char **argv )
  : m_nh()
  , m_pubTwist()
  , m_subData()
  , m_subStatus()
  , m_s_t()
  , m_s_t_d()
  , m_s_tu()
  , m_s_tu_d()
  , m_lambda( 0.5 )
  , m_v( 6, 0 )
  , m_cMo()
  , m_cdMc()
  , m_cdMo()
  , m_init( false )
  , m_enable_servo( 0 )
  , m_t_x_d( 0 )
  , m_t_y_d( 0 )
  , m_t_z_d( 0 )
  , m_tu_x_d( 0 )
  , m_tu_y_d( 0 )
  , m_tu_z_d( 0 )
  , m_square_size( 0.12 )
  , m_cam_px( -1 )
  , m_cam_py( -1 )
  , m_cam_u0( -1 )
  , m_cam_v0( -1 )
  , m_cam_kud( -1 )
  , m_cam_kdu( -1 )
  , m_cam()
  , m_I_grayscale()
  , m_display( NULL )
  , m_thickness( 2 )
  , ips_trajectory()
  , m_quit( false )
{
  m_subData   = m_nh.subscribe( "/blob_tracker/data", 1000, &VS::data_callback, this );
  m_subStatus = m_nh.subscribe( "/blob_tracker/status", 1000, &VS::status_callback, this );
  m_pubTwist  = m_nh.advertise< geometry_msgs::TwistStamped >( "pbvs/cmd_camvel", 1000 );

  // Load desired pose from parameters
  m_nh.getParam( "tx_d", m_t_x_d );
  m_nh.getParam( "ty_d", m_t_y_d );
  m_nh.getParam( "tz_d", m_t_z_d );
  m_nh.getParam( "tux_d", m_tu_x_d );
  m_nh.getParam( "tuy_d", m_tu_y_d );
  m_nh.getParam( "tuy_d", m_tu_z_d );

  // Load object size
  m_nh.getParam( "square_size", m_square_size );

  // Load camera parameters
  m_nh.getParam( "cam_px", m_cam_px );
  m_nh.getParam( "cam_py", m_cam_py );
  m_nh.getParam( "cam_u0", m_cam_u0 );
  m_nh.getParam( "cam_v0", m_cam_v0 );
  m_nh.getParam( "cam_kud", m_cam_kud );
  m_nh.getParam( "cam_kdu", m_cam_kdu );

  if ( m_cam_px < 0 || m_cam_py < 0 || m_cam_u0 < 0 || m_cam_v0 < 0 )
  {
    ROS_ERROR( "Camera parameters are not set" );
  }

  if ( m_cam_kud == -1 || m_cam_kdu == -1 )
  {
    m_cam.initPersProjWithoutDistortion( m_cam_px, m_cam_py, m_cam_u0, m_cam_v0 );
  }
  else
  {
    m_cam.initPersProjWithDistortion( m_cam_px, m_cam_py, m_cam_u0, m_cam_v0, m_cam_kud, m_cam_kdu );
  }

  m_cdMo.buildFrom( m_t_x_d, m_t_y_d, m_t_z_d, vpMath::rad( m_tu_x_d ), vpMath::rad( m_tu_y_d ),
                    vpMath::rad( m_tu_z_d ) );
  std::cout << "Desired pose: " << m_cdMo << std::endl;
}

void
VS::init_vs()
{
  m_lambda_adapt.initStandard( 3, 0.2, 30 );

  m_task.setServo( vpServo::EYEINHAND_CAMERA );
  m_task.setInteractionMatrixType( vpServo::CURRENT );
  m_task.setLambda( m_lambda );
  //  m_task.setLambda(m_lambda_adapt);

  m_task.addFeature( m_s_t, m_s_t_d );
  m_task.addFeature( m_s_tu, m_s_tu_d );
}

void
VS::init_display()
{
#ifdef VISP_HAVE_X11
  m_display = new vpDisplayX;
#elif VISP_HAVE_GDI
  m_display = new vpDisplayGDI;
#elif VISP_HAVE_OPENCV
  m_display = new vpDisplayOpenCV;
#endif
  if ( m_display )
  {
    std::cout << "Image size: " << m_I_grayscale.getWidth() << " x " << m_I_grayscale.getHeight() << std::endl;
    std::cout << "Camera parameters:\n" << m_cam << std::endl;
    m_display->init( m_I_grayscale, m_I_grayscale.getWidth() + 120, m_I_grayscale.getHeight() + 20, "PBVS controller" );
  }
}

void
VS::display( const visp_ros::BlobTracker::ConstPtr &msg )
{
  try
  {
    cv_bridge::CvImagePtr cv_image_ptr = cv_bridge::toCvCopy( msg->image );
    cv::Mat cv_frame                   = cv_image_ptr->image;
    vpImageConvert::convert( cv_frame, m_I_grayscale );

    if ( m_display == NULL )
    {
      init_display();
    }

    vpDisplay::display( m_I_grayscale );

    std::vector< vpImagePoint > ips;
    for ( unsigned int i = 0; i < msg->blob_cogs.size(); i++ )
    {
      vpImagePoint ip( msg->blob_cogs[i].i, msg->blob_cogs[i].j );
      ips.push_back( ip );
      std::stringstream ss;
      ss << i;
      vpDisplay::displayText( m_I_grayscale, ip + vpImagePoint( -20, 20 ), ss.str(), vpColor::red );
      vpDisplay::displayCross( m_I_grayscale, ip, 15, vpColor::green, m_thickness );
    }
    for ( unsigned int i = 1; i < ips_trajectory.size(); i++ )
    {
      for ( unsigned int j = 0; j < ips_trajectory[i].size(); j++ )
      {
        vpDisplay::displayLine( m_I_grayscale, ips_trajectory[i][j], ips_trajectory[i - 1][j], vpColor::green,
                                m_thickness );
      }
    }

    ips_trajectory.push_back( ips );

    vpDisplay::displayFrame( m_I_grayscale, m_cMo, m_cam, m_square_size * 2. / 3., vpColor::none, m_thickness );
    vpDisplay::displayFrame( m_I_grayscale, m_cdMo, m_cam, m_square_size / 2., vpColor::none, m_thickness );

    vpDisplay::flush( m_I_grayscale );
  }
  catch ( cv_bridge::Exception &e )
  {
    ROS_ERROR( "cv_bridge exception: %s", e.what() );
    return;
  }
}

void
VS::data_callback( const visp_ros::BlobTracker::ConstPtr &msg )
{
  geometry_msgs::TwistStamped camvel_msg;
  try
  {
    if ( !m_init )
    {
      init_vs();
      m_init = true;
    }
    std::ostringstream strs;
    strs << "Receive a new pose" << std::endl;
    std::string str;
    str = strs.str();
    ROS_DEBUG( "%s", str.c_str() );

    m_cMo = visp_bridge::toVispHomogeneousMatrix( msg->pose_stamped.pose );

    std::cout << "pbvs cMo:\n" << m_cMo << std::endl;
    std::cout << "pbvs cdMo:\n" << m_cdMo << std::endl;
    // Update visual features
    m_cdMc = m_cdMo * m_cMo.inverse();
    std::cout << "m_cdMc:\n" << m_cdMc << std::endl;
    m_s_t.buildFrom( m_cdMc );
    m_s_tu.buildFrom( m_cdMc );

    m_v = m_task.computeControlLaw();
    std::cout << "v: " << m_v.t() << std::endl;

    display( msg );

    camvel_msg.header.stamp    = ros::Time::now();
    camvel_msg.twist.linear.x  = m_v[0];
    camvel_msg.twist.linear.y  = m_v[1];
    camvel_msg.twist.linear.z  = m_v[2];
    camvel_msg.twist.angular.x = m_v[3];
    camvel_msg.twist.angular.y = m_v[4];
    camvel_msg.twist.angular.z = m_v[5];

    m_pubTwist.publish( camvel_msg );
  }
  catch ( ... )
  {
    ROS_INFO( "Catch an exception: set vel to 0" );
    camvel_msg.header.stamp    = ros::Time::now();
    camvel_msg.twist.linear.x  = 0;
    camvel_msg.twist.linear.y  = 0;
    camvel_msg.twist.linear.z  = 0;
    camvel_msg.twist.angular.x = 0;
    camvel_msg.twist.angular.y = 0;
    camvel_msg.twist.angular.z = 0;
    m_pubTwist.publish( camvel_msg );
  }
}

void
VS::status_callback( const std_msgs::Int8ConstPtr &msg )
{
  if ( msg->data == 0 )
  {
    m_enable_servo = false;

    // Stop the robot
    geometry_msgs::TwistStamped camvel_msg;
    camvel_msg.header.stamp    = ros::Time::now();
    camvel_msg.twist.linear.x  = 0;
    camvel_msg.twist.linear.y  = 0;
    camvel_msg.twist.linear.z  = 0;
    camvel_msg.twist.angular.x = 0;
    camvel_msg.twist.angular.y = 0;
    camvel_msg.twist.angular.z = 0;
    m_pubTwist.publish( camvel_msg );
  }
  else if ( msg->data == 1 )
  {
    m_enable_servo = true;
  }
  else
  {
    m_quit = true;
  }
}

void
VS::spin()
{
  ros::Rate loop_rate( 60 );
  while ( m_nh.ok() && !m_quit )
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  if ( m_display )
  {
    delete m_display;
    m_display = NULL;
  }
  std::cout << "PBVS controller stopped..." << std::endl;
}

int
main( int argc, char **argv )
{
  ros::init( argc, argv, "PBVS" );

  VS vs( argc, argv );

  vs.spin();
}
