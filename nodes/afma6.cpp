#include <math.h>
#include <sstream>
#include <stdio.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <visp/vpRobotAfma6.h> // visp

#include <visp_bridge/3dpose.h> // visp_bridge

#ifdef VISP_HAVE_AFMA6

class RosAfma6Node
{
public:
  RosAfma6Node( ros::NodeHandle n );
  virtual ~RosAfma6Node();

public:
  int setup();
  void setCameraVel( const geometry_msgs::TwistStampedConstPtr & );
  void spin();
  void publish();

protected:
  ros::NodeHandle m_nh;
  ros::Publisher m_pose_pub;
  ros::Publisher m_vel_pub;
  ros::Subscriber m_cmd_camvel_sub;

  ros::Time m_veltime;

  std::string m_serial_port;

  vpRobotAfma6 *m_robot;
  geometry_msgs::PoseStamped m_position;

  // for odom->base_link transform
  tf::TransformBroadcaster m_odom_broadcaster;
  geometry_msgs::TransformStamped m_odom_trans;
  // for resolving tf names.
  std::string m_tf_prefix;
  std::string m_frame_id_odom;
  std::string m_frame_id_base_link;

  vpHomogeneousMatrix m_wMc; // world to camera transformation
  vpColVector m_q;           // measured joint position

  int m_tool_type = 0; // See https://visp-doc.inria.fr/doxygen/visp-daily/classvpAfma6.html structure vpAfma6ToolType
};

RosAfma6Node::RosAfma6Node( ros::NodeHandle nh )
{
  // read in config options
  m_n = nh;

  ROS_INFO( "Using Afma6 robot" );

  m_robot = NULL;
  /*
   * Figure out what frame_id's to use. if a tf_prefix param is specified,
   * it will be added to the beginning of the frame_ids.
   *
   * e.g. rosrun ... _tf_prefix:=MyRobot (or equivalently using <param>s in
   * roslaunch files)
   * will result in the frame_ids being set to /MyRobot/odom etc,
   * rather than /odom. This is useful for Multi Robot Systems.
   * See ROS Wiki for further details.
   */
  m_tf_prefix = tf::getPrefixParam( m_nh );
  //  m_frame_id_odom = tf::resolve(tf_prefix, "odom");
  //  m_frame_id_base_link = tf::resolve(tf_prefix, "base_link");

  m_nh.getParam( "tool_type", m_tool_type );

  // advertise services
  m_pose_pub = n.advertise< geometry_msgs::PoseStamped >( "pose", 1000 );
  m_vel_pub  = n.advertise< geometry_msgs::TwistStamped >( "velocity", 1000 );

  // subscribe to services
  m_cmd_camvel_sub =
      m_nh.subscribe( "cmd_camvel", 1,
                      (boost::function< void( const geometry_msgs::TwistStampedConstPtr & ) >)boost::bind(
                          &RosAfma6Node::setCameraVel, this, _1 ) );
}

RosAfma6Node::~RosAfma6Node()
{
  if ( robot )
  {
    m_robot->stopMotion();
    delete m_robot;
    m_robot = NULL;
  }
}

int
RosAfma6Node::setup()
{
  m_robot = new vpRobotAfma6;

  m_robot->init( static_cast< vpAfma6ToolType >( m_tool_type ), vpCameraParameters::perspectiveProjWithDistortion );
  vpCameraParameters cam;
  m_robot->getCameraParameters( cam, 640, 480 );
  std::cout << "Camera parameters (640 x 480):\n" << cam << std::endl;

  m_robot->setRobotState( vpRobot::STATE_VELOCITY_CONTROL );

  return 0;
}

void
RosAfma6Node::spin()
{
  ros::Rate loop_rate( 100 );
  while ( ros::ok() )
  {
    this->publish();
    ros::spinOnce();
    loop_rate.sleep();
  }
  //  ros::spin();
}

void
RosAfma6Node::publish()
{
  double timestamp;
  m_robot->getPosition( vpRobot::ARTICULAR_FRAME, m_q, timestamp );
  m_wMc                   = robot->get_fMc( m_q );
  m_position.pose         = visp_bridge::toGeometryMsgsPose( m_wMc );
  m_position.header.stamp = ros::Time( timestamp ); // to improve: should be the timestamp returned by getPosition()

  //  ROS_INFO( "Afma6 publish pose at %f s: [%0.2f %0.2f %0.2f] - [%0.2f %0.2f %0.2f %0.2f]",
  //            position.header.stamp.toSec(),
  //            position.pose.position.x, position.pose.position.y, position.pose.position.z,
  //            position.pose.orientation.w, position.pose.orientation.x, position.pose.orientation.y,
  //            position.pose.orientation.z);
  m_pose_pub.publish( m_position );

  vpColVector vel( 6 );
  m_robot->getVelocity( vpRobot::CAMERA_FRAME, vel, timestamp );
  geometry_msgs::TwistStamped vel_msg;
  vel_msg.header.stamp    = ros::Time( timestamp );
  vel_msg.twist.linear.x  = vel[0];
  vel_msg.twist.linear.y  = vel[1];
  vel_msg.twist.linear.z  = vel[2];
  vel_msg.twist.angular.x = vel[3];
  vel_msg.twist.angular.y = vel[4];
  vel_msg.twist.angular.z = vel[5];
  m_vel_pub.publish( vel_msg );

  //	ros::Duration(1e-3).sleep();
}

void
RosAfma6Node::setCameraVel( const geometry_msgs::TwistStampedConstPtr &msg )
{
  veltime = ros::Time::now();

  vpColVector vc( 6 ); // Vel in m/s and rad/s

  vc[0] = msg->twist.linear.x;
  vc[1] = msg->twist.linear.y;
  vc[2] = msg->twist.linear.z;

  vc[3] = msg->twist.angular.x;
  vc[4] = msg->twist.angular.y;
  vc[5] = msg->twist.angular.z;

  //  ROS_INFO( "Afma6 new camera vel at %f s: [%0.2f %0.2f %0.2f] m/s [%0.2f %0.2f %0.2f] rad/s",
  //            veltime.toSec(),
  //            vc[0], vc[1], vc[2], vc[3], vc[4], vc[5]);
  m_robot->setVelocity( vpRobot::CAMERA_FRAME, vc );

  //  this->publish();
}

#endif // #ifdef VISP_HAVE_AFMA6

int
main( int argc, char **argv )
{
#ifdef VISP_HAVE_AFMA6
  ros::init( argc, argv, "RosAfma6" );
  ros::NodeHandle n( std::string( "~" ) );

  RosAfma6Node *node = new RosAfma6Node( n );

  if ( node->setup() != 0 )
  {
    printf( "Afma6 setup failed... \n" );
    return -1;
  }

  node->spin();

  delete node;

  printf( "\nQuitting... \n" );
#else
  printf( "This node is node available since ViSP was \nnot build with Afma6 robot support...\n" );
#endif
  return 0;
}
