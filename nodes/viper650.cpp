#include <stdio.h>
#include <math.h>
#include <sstream>

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <visp/vpRobotViper650.h> // visp

#include <visp_bridge/3dpose.h> // visp_bridge


#ifdef VISP_HAVE_VIPER650

class RosViper650Node
{
  public:
    RosViper650Node(ros::NodeHandle n);
    virtual ~RosViper650Node();
    
  public:
    int setup();
    void setCameraVel( const geometry_msgs::TwistStampedConstPtr &);
    void spin();
    void publish();
 
  protected:
    ros::NodeHandle n;
    ros::Publisher pose_pub;
    ros::Publisher vel_pub;
    ros::Subscriber cmd_camvel_sub;

    ros::Time veltime;

    std::string serial_port;

    vpRobotViper650 *robot;
    geometry_msgs::PoseStamped position;
    		
    //for odom->base_link transform
    tf::TransformBroadcaster odom_broadcaster;
    geometry_msgs::TransformStamped odom_trans;
    //for resolving tf names.
    std::string tf_prefix;
    std::string frame_id_odom;
    std::string frame_id_base_link;

    vpHomogeneousMatrix wMc; // world to camera transformation
    vpColVector q; // measured joint position
 };


RosViper650Node::RosViper650Node(ros::NodeHandle nh)
{
  // read in config options
  n = nh;

  ROS_INFO( "using Viper650 robot" );

  robot = NULL;
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
  tf_prefix = tf::getPrefixParam(n);
//  frame_id_odom = tf::resolve(tf_prefix, "odom");
//  frame_id_base_link = tf::resolve(tf_prefix, "base_link");

  // advertise services
  pose_pub = n.advertise<geometry_msgs::PoseStamped>("pose", 1000);
  vel_pub = n.advertise<geometry_msgs::TwistStamped>("velocity", 1000);
  
  // subscribe to services
  cmd_camvel_sub = n.subscribe( "cmd_camvel", 1, (boost::function < void(const geometry_msgs::TwistStampedConstPtr&)>) boost::bind( &RosViper650Node::setCameraVel, this, _1 ));
}

RosViper650Node::~RosViper650Node()
{
  if (robot) {
    robot->stopMotion();
    delete robot;
    robot = NULL;
  }
}

int RosViper650Node::setup()
{
  robot = new vpRobotViper650;

  robot->init(vpViper650::TOOL_GENERIC_CAMERA, vpCameraParameters::perspectiveProjWithDistortion);

  robot->setRobotState(vpRobot::STATE_VELOCITY_CONTROL);

  return 0;
}

void RosViper650Node::spin()
{
	ros::Rate loop_rate(100);
	while(ros::ok()){
		this->publish();
		ros::spinOnce();
		loop_rate.sleep();
	}
//  ros::spin();
}

void RosViper650Node::publish()
{
	double timestamp;
	robot->getPosition(vpRobot::ARTICULAR_FRAME, q, timestamp);
	wMc = robot->get_fMc(q);
	position.pose = visp_bridge::toGeometryMsgsPose(wMc);
	position.header.stamp = ros::Time(timestamp); // to improve: should be the timestamp returned by getPosition()

	//  ROS_INFO( "Viper650 publish pose at %f s: [%0.2f %0.2f %0.2f] - [%0.2f %0.2f %0.2f %0.2f]",
	//            position.header.stamp.toSec(),
	//            position.pose.position.x, position.pose.position.y, position.pose.position.z,
	//            position.pose.orientation.w, position.pose.orientation.x, position.pose.orientation.y, position.pose.orientation.z);
	pose_pub.publish(position);

	vpColVector vel(6);
	robot->getVelocity(vpRobot::CAMERA_FRAME, vel, timestamp);
	geometry_msgs::TwistStamped vel_msg;
	vel_msg.header.stamp = ros::Time(timestamp);
	vel_msg.twist.linear.x = vel[0];
	vel_msg.twist.linear.y = vel[1];
	vel_msg.twist.linear.z = vel[2];
	vel_msg.twist.angular.x = vel[3];
	vel_msg.twist.angular.y = vel[4];
	vel_msg.twist.angular.z = vel[5];
	vel_pub.publish(vel_msg);

//	ros::Duration(1e-3).sleep();
}

void
RosViper650Node::setCameraVel( const geometry_msgs::TwistStampedConstPtr &msg)
{
  veltime = ros::Time::now();

  vpColVector vc(6); // Vel in m/s and rad/s

  vc[0] = msg->twist.linear.x;
  vc[1] = msg->twist.linear.y;
  vc[2] = msg->twist.linear.z;

  vc[3] = msg->twist.angular.x;
  vc[4] = msg->twist.angular.y;
  vc[5] = msg->twist.angular.z;

  ROS_INFO( "Viper650 new camera vel at %f s: [%0.2f %0.2f %0.2f] m/s [%0.2f %0.2f %0.2f] rad/s",
            veltime.toSec(),
            vc[0], vc[1], vc[2], vc[3], vc[4], vc[5]);
  robot->setVelocity(vpRobot::CAMERA_FRAME, vc);

//  this->publish();
}

#endif // #ifdef VISP_HAVE_Viper650

int main( int argc, char** argv )
{
#ifdef VISP_HAVE_VIPER650
  ros::init(argc,argv, "RosViper650");
  ros::NodeHandle n(std::string("~"));

  RosViper650Node *node = new RosViper650Node(n);

  if( node->setup() != 0 )
  {
    printf( "Viper650 setup failed... \n" );
    return -1;
  }

  node->spin();

  delete node;

  printf( "\nQuitting... \n" );
#else
  printf("The Viper650 robot is not supported by ViSP...\n");
#endif
  return 0;
}

