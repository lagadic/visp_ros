#include <stdio.h>
#include <math.h>
#include <visp/vpRobotBiclops.h> // visp

#include <visp_bridge/3dpose.h> // visp_bridge

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>	
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>


#include <sstream>

#ifdef VISP_HAVE_BICLOPS

class RosBiclopsNode
{
  public:
    RosBiclopsNode(ros::NodeHandle n);
    virtual ~RosBiclopsNode();
    
  public:
    int setup();
    void setJointVel( const geometry_msgs::TwistConstPtr &);
    void setJointPos( const geometry_msgs::PoseConstPtr &);
    void spin();
    void publish();
 
  protected:
    ros::NodeHandle n;
    ros::Publisher pose_pub;
    ros::Publisher vel_pub;
    ros::Subscriber cmd_jointvel_sub;
    ros::Subscriber cmd_jointpos_sub;

    ros::Time veltime;

    std::string serial_port;

    vpRobotBiclops *robot;
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


RosBiclopsNode::RosBiclopsNode(ros::NodeHandle nh)
{
  // read in config options
  n = nh;

  ROS_INFO( "Using Biclops robot" );

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
  pose_pub = n.advertise<geometry_msgs::PoseStamped>("biclops/odom", 1000);
  
  // subscribe to services
  cmd_jointvel_sub = n.subscribe( "cmd_vel", 1, (boost::function < void(const geometry_msgs::TwistConstPtr&)>) boost::bind( &RosBiclopsNode::setJointVel, this, _1 ));
  cmd_jointpos_sub = n.subscribe( "pose", 1, (boost::function < void(const geometry_msgs::PoseConstPtr&)>) boost::bind( &RosBiclopsNode::setJointPos, this, _1 ));
}

RosBiclopsNode::~RosBiclopsNode()
{
  if (robot) {
    robot->stopMotion();
    delete robot;
    robot = NULL;
  }
}

int RosBiclopsNode::setup()
{
  robot = new vpRobotBiclops("/usr/share/BiclopsDefault.cfg");
  robot->setDenavitHartenbergModel(vpBiclops::DH2);

  vpColVector qinit(2);
  qinit = 0;
  robot->setRobotState(vpRobot::STATE_POSITION_CONTROL) ;
  robot->setPositioningVelocity(40);
  robot->setPosition(vpRobot::ARTICULAR_FRAME, qinit);

  robot->setRobotState(vpRobot::STATE_VELOCITY_CONTROL);

  return 0;
}

void RosBiclopsNode::spin()
{
  ros::Rate loop_rate(15);
	while(ros::ok()){
		this->publish();
		ros::spinOnce();
		loop_rate.sleep();
	}
//  ros::spin();
}

void RosBiclopsNode::publish()
{
  robot->getPosition(vpRobot::ARTICULAR_FRAME, q);

  position.pose.position.x = 0;
  position.pose.position.y = 0;
  position.pose.position.z = 0;
  position.pose.orientation.x = q[1];
  position.pose.orientation.y = q[0];
  position.pose.orientation.z = 0;
  position.pose.orientation.w = 0;

  position.header.stamp = ros::Time::now();

	//  ROS_INFO( "Biclops publish pose at %f s: [%0.2f %0.2f %0.2f] - [%0.2f %0.2f %0.2f %0.2f]",
	//            position.header.stamp.toSec(),
	//            position.pose.position.x, position.pose.position.y, position.pose.position.z,
	//            position.pose.orientation.w, position.pose.orientation.x, position.pose.orientation.y, position.pose.orientation.z);
	pose_pub.publish(position);
}

void
RosBiclopsNode::setJointVel( const geometry_msgs::TwistConstPtr &msg)
{
  veltime = ros::Time::now();

  vpColVector qdot(2); // Vel in rad/s for pan and tilt

  qdot[1] = msg->angular.x;
  qdot[0] = msg->angular.y;


  ROS_INFO( "Biclops new joint vel at %f s: [%0.2f %0.2f] rad/s",
            veltime.toSec(),
            qdot[0], qdot[1]);
  robot->setRobotState(vpRobot::STATE_VELOCITY_CONTROL);
  robot->setVelocity(vpRobot::ARTICULAR_FRAME, qdot);
}
void
RosBiclopsNode::setJointPos( const geometry_msgs::PoseConstPtr &msg)
{
  veltime = ros::Time::now();

  vpColVector qdes(2); // Vel in rad/s for pan and tilt

  qdes[0] = msg->orientation.x;
  qdes[1] = msg->orientation.y;

  ROS_INFO( "Biclops new joint pos at %f s: [%0.2f %0.2f] rad/s",
            veltime.toSec(),
            qdes[0], qdes[1]);
  robot->setRobotState(vpRobot::STATE_POSITION_CONTROL);
  robot->setPosition(vpRobot::ARTICULAR_FRAME, qdes);
}

#endif // #ifdef VISP_HAVE_BICLOPS


int main( int argc, char** argv )
{
#ifdef VISP_HAVE_BICLOPS
  ros::init(argc,argv, "RosBiclops");
  ros::NodeHandle n(std::string("~"));

  RosBiclopsNode *node = new RosBiclopsNode(n);

  if( node->setup() != 0 )
  {
    printf( "Biclops setup failed... \n" );
    return -1;
  }

  node->spin();

  delete node;

  printf( "\nQuitting... \n" );
#else
  printf("This node is node available since ViSP was \nnot build with Biclops robot support...\n");
#endif
  return 0;
}
