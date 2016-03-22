#include <stdio.h>
#include <math.h>
#include <sstream>

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <visp3/core/vpMath.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/robot/vpRobotViper850.h>

#include <visp_bridge/3dpose.h> // visp_bridge


#ifdef VISP_HAVE_VIPER850

class RosViper850Node
{
  public:
    RosViper850Node(ros::NodeHandle n);
    virtual ~RosViper850Node();
    
  public:
    int setup();
    void setCameraVel(const geometry_msgs::TwistStampedConstPtr &);
    void setJointVel(const sensor_msgs::JointStateConstPtr &);
    void setRefVel(const geometry_msgs::TwistStampedConstPtr &);
    void spin();
    void publish();
 
  protected:
    ros::NodeHandle n;
    ros::Publisher pose_pub;
    ros::Publisher vel_pub;
    ros::Publisher jointState_pub;
    ros::Publisher jacobian_pub;
    ros::Subscriber cmd_camvel_sub;
    ros::Subscriber cmd_jointvel_sub;
    ros::Subscriber cmd_refvel_sub;
    std::string setControlMode; // "joint_space", "camera_frame" (default), "reference_frame"
    std::string getStateSpace;  // "joint_space", "camera_frame" (default), "reference_frame"
    std::string cmdVelTopicName;  // default to /cmd_vel
    std::string endEffectorType;  // "TOOL_PTGREY_FLEA2_CAMERA" (default), "TOOL_GENERIC_CAMERA", "TOOL_CUSTOM"
    std::string customToolTransformationFileName;

    ros::Time veltime;

    std::string serial_port;

    vpRobotViper850 *robot;
    geometry_msgs::PoseStamped position;
    sensor_msgs::JointState jointState;
    std_msgs::Float64MultiArray jacobian;
    		
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


RosViper850Node::RosViper850Node(ros::NodeHandle nh)
{
  // read in config options
  n = nh;

  ROS_INFO("Launch Viper850 robot ros node");

  robot = NULL;
  n.param<std::string>("set_control_mode", setControlMode, "tool_frame");
  n.param<std::string>("get_state_space", getStateSpace, "tool_frame");
  n.param<std::string>("cmd_vel_topic_name", cmdVelTopicName, "cmd_vel");
  n.param<std::string>("end_effector", endEffectorType, "TOOL_PTGREY_FLEA2_CAMERA");
  n.param<std::string>("custom_tool_transformation_filename", customToolTransformationFileName, "");

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
//  pose_pub = n.advertise<geometry_msgs::PoseStamped>("pose", 1000);
//  vel_pub = n.advertise<geometry_msgs::TwistStamped>("velocity", 1000);
//  jointState_pub = n.advertise<sensor_msgs::JointState>("joint_state", 1000);

  
  // subscribe to services
  if (setControlMode == "joint_space") {
    ROS_INFO("Viper850 robot controlled in joint space");
    jointState_pub = n.advertise<sensor_msgs::JointState>("joint_state", 10);
    jacobian_pub = n.advertise<std_msgs::Float64MultiArray>("jacobian", 10);
    cmd_jointvel_sub = n.subscribe( cmdVelTopicName, 1, (boost::function < void(const sensor_msgs::JointStateConstPtr&)>) boost::bind( &RosViper850Node::setJointVel, this, _1 ));
  }
  else if (setControlMode == "camera_frame") {
    ROS_INFO("Viper850 robot controlled in camera frame");
    pose_pub = n.advertise<geometry_msgs::PoseStamped>("pose", 10);
    vel_pub = n.advertise<geometry_msgs::TwistStamped>("velocity", 10);
    cmd_camvel_sub = n.subscribe( cmdVelTopicName, 1, (boost::function < void(const geometry_msgs::TwistStampedConstPtr&)>) boost::bind( &RosViper850Node::setCameraVel, this, _1 ));
  }
  else if (setControlMode == "reference_frame") {
    ROS_INFO("Viper850 robot controlled in reference frame");
    pose_pub = n.advertise<geometry_msgs::PoseStamped>("pose", 10);
    vel_pub = n.advertise<geometry_msgs::TwistStamped>("velocity", 10);
    cmd_refvel_sub = n.subscribe( cmdVelTopicName, 1, (boost::function < void(const geometry_msgs::TwistStampedConstPtr&)>) boost::bind( &RosViper850Node::setRefVel, this, _1 ));
  }
}

RosViper850Node::~RosViper850Node()
{
  if (robot) {
    robot->stopMotion();
    delete robot;
    robot = NULL;
  }
}

int RosViper850Node::setup()
{
  robot = new vpRobotViper850;

  if (endEffectorType == "TOOL_PTGREY_FLEA2_CAMERA") {
    robot->init(vpViper850::TOOL_PTGREY_FLEA2_CAMERA, vpCameraParameters::perspectiveProjWithDistortion);
  }
  else if (endEffectorType == "TOOL_GENERIC_CAMERA") {
    robot->init(vpViper850::TOOL_GENERIC_CAMERA, vpCameraParameters::perspectiveProjWithDistortion);
  }
  else if (endEffectorType == "TOOL_CUSTOM") {
    if (vpIoTools::checkFilename(customToolTransformationFileName) == false) {
      ROS_ERROR("Viper850: Missing or bad filename for eMt custom tool transformation");
      return -1;
    }

    vpHomogeneousMatrix eMt;
    std::ifstream f(customToolTransformationFileName.c_str());
    try {
      eMt.load(f);
      f.close();
    }
    catch(vpException &e) {
      ROS_ERROR_STREAM("Viper850: Cannot load eMt custom tool transformation from \"" << customToolTransformationFileName << "\": " << e.getStringMessage());
      f.close();
      return -1;
    }

    robot->init(vpViper850::TOOL_CUSTOM, eMt);
  }

  robot->setRobotState(vpRobot::STATE_VELOCITY_CONTROL);

  return 0;
}

void RosViper850Node::spin()
{
	ros::Rate loop_rate(100);
	while(ros::ok()){
		this->publish();
		ros::spinOnce();
		loop_rate.sleep();
	}
}

void RosViper850Node::publish()
{
	double timestamp;
	robot->getPosition(vpRobot::ARTICULAR_FRAME, q, timestamp);

  if (getStateSpace == "joint_space") {
    vpColVector qdot;
    robot->getVelocity(vpRobot::ARTICULAR_FRAME, qdot, timestamp);
    jointState.position.clear();
    jointState.velocity.clear();
    jointState.header.stamp = ros::Time(timestamp);
    for (unsigned int i=0; i<qdot.size(); i++) {
      jointState.position.push_back(q[i]);
      jointState.velocity.push_back(qdot[i]);
    }
    jointState_pub.publish(jointState);

    vpMatrix eJe;
    robot->get_eJe(eJe);
    vpHomogeneousMatrix eMc;
    robot->get_eMc(eMc);
    vpMatrix cJc = vpVelocityTwistMatrix(eMc.inverse()) * eJe;

    jacobian.data.clear();
    jacobian.layout.dim.resize(2);

    jacobian.layout.dim[0].label = "height";
    jacobian.layout.dim[0].size = cJc.getRows();
    jacobian.layout.dim[0].stride = cJc.size();
    jacobian.layout.dim[1].label = "width";
    jacobian.layout.dim[1].size = cJc.getCols();
    jacobian.layout.dim[1].stride = cJc.getCols();
    for(unsigned int i =0; i< cJc.size(); i++)
      jacobian.data.push_back( cJc.data[i]);
    jacobian_pub.publish(jacobian);
  }
  else if (getStateSpace == "camera_frame") {
    wMc = robot->get_fMc(q);
    position.pose = visp_bridge::toGeometryMsgsPose(wMc);
    position.header.stamp = ros::Time(timestamp); // to improve: should be the timestamp returned by getPosition()

    //  ROS_INFO( "Viper850 publish pose at %f s: [%0.2f %0.2f %0.2f] - [%0.2f %0.2f %0.2f %0.2f]",
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
  }
  else if (getStateSpace == "reference_frame") {
    wMc = robot->get_fMc(q);
    position.pose = visp_bridge::toGeometryMsgsPose(wMc);
    position.header.stamp = ros::Time(timestamp); // to improve: should be the timestamp returned by getPosition()

    //  ROS_INFO( "Viper850 publish pose at %f s: [%0.2f %0.2f %0.2f] - [%0.2f %0.2f %0.2f %0.2f]",
    //            position.header.stamp.toSec(),
    //            position.pose.position.x, position.pose.position.y, position.pose.position.z,
    //            position.pose.orientation.w, position.pose.orientation.x, position.pose.orientation.y, position.pose.orientation.z);
    pose_pub.publish(position);

    vpColVector vel(6);
    robot->getVelocity(vpRobot::REFERENCE_FRAME, vel, timestamp);
    geometry_msgs::TwistStamped vel_msg;
    vel_msg.header.stamp = ros::Time(timestamp);
    vel_msg.twist.linear.x = vel[0];
    vel_msg.twist.linear.y = vel[1];
    vel_msg.twist.linear.z = vel[2];
    vel_msg.twist.angular.x = vel[3];
    vel_msg.twist.angular.y = vel[4];
    vel_msg.twist.angular.z = vel[5];
    vel_pub.publish(vel_msg);
  }
}

void
RosViper850Node::setCameraVel(const geometry_msgs::TwistStampedConstPtr &msg)
{
  veltime = ros::Time::now();

  vpColVector vc(6); // Vel in m/s and rad/s

  vc[0] = msg->twist.linear.x;
  vc[1] = msg->twist.linear.y;
  vc[2] = msg->twist.linear.z;

  vc[3] = msg->twist.angular.x;
  vc[4] = msg->twist.angular.y;
  vc[5] = msg->twist.angular.z;

  ROS_INFO( "Viper850 new camera vel at %f s: [%0.2f %0.2f %0.2f] m/s [%0.2f %0.2f %0.2f] rad/s",
            veltime.toSec(),
            vc[0], vc[1], vc[2], vc[3], vc[4], vc[5]);
  robot->setVelocity(vpRobot::CAMERA_FRAME, vc);
}

void
RosViper850Node::setJointVel(const sensor_msgs::JointStateConstPtr &msg)
{
  veltime = ros::Time::now();

  if (msg->velocity.size() != 6) {
    ROS_ERROR("Viper850: Cannot apply a joint velocity vector that is not 6 dimensional");
    return;
  }
  vpColVector qdot(6); // Vel in rad/s for each joint

  for(unsigned int i=0; i<msg->velocity.size(); i++)
    qdot[i] = msg->velocity[i];

  ROS_INFO("Viper850 new joint vel at %f s: [%0.2f %0.2f %0.2f %0.2f %0.2f %0.2f] rad/s",
           veltime.toSec(),
           qdot[0], qdot[1], qdot[2], qdot[3], qdot[4], qdot[5]);

  robot->setVelocity(vpRobot::ARTICULAR_FRAME, qdot);
}

void
RosViper850Node::setRefVel(const geometry_msgs::TwistStampedConstPtr &msg)
{
  veltime = ros::Time::now();

  vpColVector vref(6); // Vel in m/s and rad/s

  vref[0] = msg->twist.linear.x;
  vref[1] = msg->twist.linear.y;
  vref[2] = msg->twist.linear.z;

  vref[3] = msg->twist.angular.x;
  vref[4] = msg->twist.angular.y;
  vref[5] = msg->twist.angular.z;

  ROS_INFO( "Viper850 new reference vel at %f s: [%0.2f %0.2f %0.2f] m/s [%0.2f %0.2f %0.2f] rad/s",
            veltime.toSec(),
            vref[0], vref[1], vref[2], vref[3], vref[4], vref[5]);
  robot->setVelocity(vpRobot::REFERENCE_FRAME, vref);
}

#endif // #ifdef VISP_HAVE_Viper850

int main( int argc, char** argv )
{
#ifdef VISP_HAVE_VIPER850
  ros::init(argc,argv, "RosViper850");
  ros::NodeHandle n(std::string("~"));

  RosViper850Node *node = new RosViper850Node(n);

  if( node->setup() != 0 )
  {
    printf( "Viper850 setup failed... \n" );
    return -1;
  }

  node->spin();

  delete node;

  printf( "\nQuitting... \n" );
#else
  printf("The Viper850 robot is not supported by ViSP...\n");
#endif
  return 0;
}

