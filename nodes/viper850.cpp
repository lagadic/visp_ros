/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2022 by Inria. All rights reserved.
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
 * See http://visp.inria.fr for more information.
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

#include <math.h>
#include <sstream>
#include <stdio.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <visp3/core/vpIoTools.h>
#include <visp3/robot/vpRobotViper850.h>

#include <visp_bridge/3dpose.h>

using namespace std::chrono_literals;

#ifdef VISP_HAVE_VIPER850

class RosViper850Node : public rclcpp::Node
{
public:
  RosViper850Node();
  virtual ~RosViper850Node();

public:
  int setup();
  void setCameraVel( const geometry_msgs::msg::TwistStamped::ConstSharedPtr &msg );
  void setJointVel( const sensor_msgs::msg::JointState::ConstSharedPtr &msg );
  void setRefVel( const geometry_msgs::msg::TwistStamped::ConstSharedPtr &msg );
  void publish();

protected:
  rclcpp::Publisher< geometry_msgs::msg::PoseStamped >::SharedPtr m_pose_pub;
  rclcpp::Publisher< geometry_msgs::msg::TwistStamped >::SharedPtr m_vel_pub;
  rclcpp::Publisher< sensor_msgs::msg::JointState >::SharedPtr m_jointState_pub;
  rclcpp::Publisher< std_msgs::msg::Float64MultiArray >::SharedPtr m_jacobian_pub;

  rclcpp::Subscription< geometry_msgs::msg::TwistStamped >::SharedPtr m_cmd_camvel_sub;
  rclcpp::Subscription< geometry_msgs::msg::TwistStamped >::SharedPtr m_cmd_refvel_sub;
  rclcpp::Subscription< sensor_msgs::msg::JointState >::SharedPtr m_cmd_jointvel_sub;

  unsigned int m_queue_size;

  std::string m_control_mode;       // "joint_space", "camera_frame" (default), "reference_frame"
  std::string m_state_mode;         // "joint_space", "camera_frame" (default), "reference_frame"
  std::string m_cmd_vel_topic_name; // default to /cmd_vel
  int m_tool_type; // See https://visp-doc.inria.fr/doxygen/visp-daily/classvpViper850.html structure vpToolType
  std::string m_tool_custom_transformation_file;

  rclcpp::Time m_vel_time;

  vpRobotViper850 *m_robot;
  geometry_msgs::msg::PoseStamped m_position;
  sensor_msgs::msg::JointState m_jointState;
  std_msgs::msg::Float64MultiArray m_jacobian;

  vpHomogeneousMatrix m_wMc; // world to camera transformation
  vpColVector m_q;           // measured joint position
};

RosViper850Node::RosViper850Node()
  : Node( "viper850_node" )
  , m_queue_size( 10 )
{
  RCLCPP_INFO( this->get_logger(), "Using Viper850 robot" );

  m_robot = NULL;

  m_control_mode                    = this->declare_parameter< std::string >( "control_mode", "tool_frame" );
  m_state_mode                      = this->declare_parameter< std::string >( "state_mode", "tool_frame" );
  m_cmd_vel_topic_name              = this->declare_parameter< std::string >( "cmd_vel_topic_name", "cmd_vel" );
  m_tool_type                       = this->declare_parameter< int >( "tool_type", 1 );
  m_tool_custom_transformation_file = this->declare_parameter< std::string >( "tool_custom_transformation_file", "" );

  // subscribe to services
  if ( m_control_mode == "joint_space" )
  {
    RCLCPP_INFO( this->get_logger(), "Viper850 robot controlled in joint space" );
    m_jointState_pub = this->create_publisher< sensor_msgs::msg::JointState >( "joint_state", m_queue_size );
    m_jacobian_pub   = this->create_publisher< std_msgs::msg::Float64MultiArray >( "jacobian", m_queue_size );

    m_cmd_jointvel_sub = this->create_subscription< sensor_msgs::msg::JointState >(
        m_cmd_vel_topic_name, m_queue_size, std::bind( &RosViper850Node::setJointVel, this, std::placeholders::_1 ) );
  }
  else if ( m_control_mode == "camera_frame" )
  {
    RCLCPP_INFO( this->get_logger(), "Viper850 robot controlled in camera frame" );
    // Create publishers
    m_pose_pub = this->create_publisher< geometry_msgs::msg::PoseStamped >( "pose", m_queue_size );
    m_vel_pub  = this->create_publisher< geometry_msgs::msg::TwistStamped >( "velocity", m_queue_size );

    // Create subscribers
    m_cmd_camvel_sub = this->create_subscription< geometry_msgs::msg::TwistStamped >(
        m_cmd_vel_topic_name, m_queue_size, std::bind( &RosViper850Node::setCameraVel, this, std::placeholders::_1 ) );
  }
  else if ( m_control_mode == "reference_frame" )
  {
    RCLCPP_INFO( this->get_logger(), "Viper850 robot controlled in reference frame" );
    // Create publishers
    m_pose_pub = this->create_publisher< geometry_msgs::msg::PoseStamped >( "pose", m_queue_size );
    m_vel_pub  = this->create_publisher< geometry_msgs::msg::TwistStamped >( "velocity", m_queue_size );

    // Create subscribers
    m_cmd_refvel_sub = this->create_subscription< geometry_msgs::msg::TwistStamped >(
        m_cmd_vel_topic_name, m_queue_size, std::bind( &RosViper850Node::setRefVel, this, std::placeholders::_1 ) );
  }
}

RosViper850Node::~RosViper850Node()
{
  if ( m_robot )
  {
    m_robot->stopMotion();
    delete m_robot;
    m_robot = NULL;
  }
}

int
RosViper850Node::setup()
{
  m_robot = new vpRobotViper850;

  if ( m_tool_type == (int)( vpViper850::TOOL_PTGREY_FLEA2_CAMERA ) )
  {
    m_robot->init( vpViper850::TOOL_PTGREY_FLEA2_CAMERA, vpCameraParameters::perspectiveProjWithDistortion );
  }
  else if ( m_tool_type == (int)( vpViper850::TOOL_GENERIC_CAMERA ) )
  {
    m_robot->init( vpViper850::TOOL_GENERIC_CAMERA, vpCameraParameters::perspectiveProjWithDistortion );
  }
  else if ( m_tool_type == (int)( vpViper850::TOOL_CUSTOM ) )
  {
    if ( vpIoTools::checkFilename( m_tool_custom_transformation_file ) == false )
    {
      RCLCPP_ERROR( this->get_logger(), "Viper850: Missing or bad filename for eMt custom tool transformation" );
      return -1;
    }

    vpHomogeneousMatrix eMt;
    std::ifstream f( m_tool_custom_transformation_file.c_str() );
    try
    {
      eMt.load( f );
      f.close();
    }
    catch ( vpException &e )
    {
      RCLCPP_ERROR( this->get_logger(), "Viper850: Cannot load eMt custom tool transformation from \"%s\"",
                    e.getMessage() );
      f.close();
      return -1;
    }

    m_robot->init( vpViper850::TOOL_CUSTOM, eMt );
  }

  m_robot->setRobotState( vpRobot::STATE_VELOCITY_CONTROL );

  return 0;
}

void
RosViper850Node::publish()
{
  double timestamp = 0;
  m_robot->getPosition( vpRobot::ARTICULAR_FRAME, m_q, timestamp );
  int32_t timestamp_second      = (int32_t)( std::floor( timestamp ) );
  uint32_t timestamp_nanosecond = (uint32_t)( ( timestamp - std::floor( timestamp ) ) * 1e9 );

  if ( m_state_mode == "joint_space" )
  {
    vpColVector qdot;
    m_robot->getVelocity( vpRobot::ARTICULAR_FRAME, qdot, timestamp );
    m_jointState.position.clear();
    m_jointState.velocity.clear();
    m_jointState.header.stamp = rclcpp::Time( timestamp_second, timestamp_nanosecond );
    for ( unsigned int i = 0; i < qdot.size(); i++ )
    {
      m_jointState.position.push_back( m_q[i] );
      m_jointState.velocity.push_back( qdot[i] );
    }
    m_jointState_pub->publish( m_jointState );

    vpMatrix eJe;
    m_robot->get_eJe( eJe );
    vpHomogeneousMatrix eMc;
    m_robot->get_eMc( eMc );
    vpMatrix cJc = vpVelocityTwistMatrix( eMc.inverse() ) * eJe;

    m_jacobian.data.clear();
    m_jacobian.layout.dim.resize( 2 );

    m_jacobian.layout.dim[0].label  = "height";
    m_jacobian.layout.dim[0].size   = cJc.getRows();
    m_jacobian.layout.dim[0].stride = cJc.size();
    m_jacobian.layout.dim[1].label  = "width";
    m_jacobian.layout.dim[1].size   = cJc.getCols();
    m_jacobian.layout.dim[1].stride = cJc.getCols();
    for ( unsigned int i = 0; i < cJc.size(); i++ )
    {
      m_jacobian.data.push_back( cJc.data[i] );
    }
    m_jacobian_pub->publish( m_jacobian );
  }
  else if ( m_state_mode == "camera_frame" )
  {
    m_wMc                   = m_robot->get_fMc( m_q );
    m_position.pose         = visp_bridge::toGeometryMsgsPose( m_wMc );
    m_position.header.stamp = rclcpp::Time( timestamp_second, timestamp_nanosecond );
    // RCLCPP_INFO( this->get_logger(), "Viper850 publish pose at %f s: [%0.2f %0.2f %0.2f] - [%0.2f %0.2f %0.2f
    // %0.2f]",
    //              timestamp, m_position.pose.position.x, m_position.pose.position.y, m_position.pose.position.z,
    //              m_position.pose.orientation.w, m_position.pose.orientation.x, m_position.pose.orientation.y,
    //              m_position.pose.orientation.z );
    m_pose_pub->publish( m_position );

    vpColVector vel( 6 );
    m_robot->getVelocity( vpRobot::CAMERA_FRAME, vel, timestamp );
    geometry_msgs::msg::TwistStamped vel_msg;
    vel_msg.header.stamp    = rclcpp::Time( timestamp );
    vel_msg.twist.linear.x  = vel[0];
    vel_msg.twist.linear.y  = vel[1];
    vel_msg.twist.linear.z  = vel[2];
    vel_msg.twist.angular.x = vel[3];
    vel_msg.twist.angular.y = vel[4];
    vel_msg.twist.angular.z = vel[5];
    m_vel_pub->publish( vel_msg );
  }
  else if ( m_state_mode == "reference_frame" )
  {
    m_wMc                   = m_robot->get_fMc( m_q );
    m_position.pose         = visp_bridge::toGeometryMsgsPose( m_wMc );
    m_position.header.stamp = rclcpp::Time( timestamp_second, timestamp_nanosecond );
    // RCLCPP_INFO( this->get_logger(), "Viper850 publish pose at %f s: [%0.2f %0.2f %0.2f] - [%0.2f %0.2f %0.2f
    // %0.2f]",
    //              timestamp, m_position.pose.position.x, m_position.pose.position.y, m_position.pose.position.z,
    //              m_position.pose.orientation.w, m_position.pose.orientation.x, m_position.pose.orientation.y,
    //              m_position.pose.orientation.z );
    m_pose_pub->publish( m_position );

    vpColVector vel( 6 );
    m_robot->getVelocity( vpRobot::REFERENCE_FRAME, vel, timestamp );
    geometry_msgs::msg::TwistStamped vel_msg;
    vel_msg.header.stamp    = rclcpp::Time( timestamp );
    vel_msg.twist.linear.x  = vel[0];
    vel_msg.twist.linear.y  = vel[1];
    vel_msg.twist.linear.z  = vel[2];
    vel_msg.twist.angular.x = vel[3];
    vel_msg.twist.angular.y = vel[4];
    vel_msg.twist.angular.z = vel[5];
    m_vel_pub->publish( vel_msg );
  }
}

void
RosViper850Node::setCameraVel( const geometry_msgs::msg::TwistStamped::ConstSharedPtr &msg )
{
  m_vel_time = rclcpp::Node::now();

  vpColVector vc( 6 ); // Vel in m/s and rad/s

  vc[0] = msg->twist.linear.x;
  vc[1] = msg->twist.linear.y;
  vc[2] = msg->twist.linear.z;

  vc[3] = msg->twist.angular.x;
  vc[4] = msg->twist.angular.y;
  vc[5] = msg->twist.angular.z;

  // RCLCPP_INFO( this->get_logger(), "Viper850 new camera vel: [%0.2f %0.2f %0.2f] m/s [%0.2f %0.2f %0.2f] rad/s",
  // vc[0], vc[1], vc[2], vc[3], vc[4], vc[5] );
  m_robot->setVelocity( vpRobot::CAMERA_FRAME, vc );
}

void
RosViper850Node::setJointVel( const sensor_msgs::msg::JointState::ConstSharedPtr &msg )
{
  m_vel_time = rclcpp::Node::now();

  if ( msg->velocity.size() != 6 )
  {
    RCLCPP_ERROR( this->get_logger(), "Viper850: Cannot apply a joint velocity vector that is not 6 dimensional" );
    return;
  }
  vpColVector qdot( 6 ); // Vel in rad/s for each joint

  for ( unsigned int i = 0; i < msg->velocity.size(); i++ )
    qdot[i] = msg->velocity[i];

  // RCLCPP_INFO( this->get_logger(), "Viper850 new joint vel: [%0.2f %0.2f %0.2f %0.2f %0.2f %0.2f] rad/s", qdot[0],
  //              qdot[1], qdot[2], qdot[3], qdot[4], qdot[5] );
  m_robot->setVelocity( vpRobot::ARTICULAR_FRAME, qdot );
}

void
RosViper850Node::setRefVel( const geometry_msgs::msg::TwistStamped::ConstSharedPtr &msg )
{
  m_vel_time = rclcpp::Node::now();

  vpColVector vref( 6 ); // Vel in m/s and rad/s

  vref[0] = msg->twist.linear.x;
  vref[1] = msg->twist.linear.y;
  vref[2] = msg->twist.linear.z;

  vref[3] = msg->twist.angular.x;
  vref[4] = msg->twist.angular.y;
  vref[5] = msg->twist.angular.z;

  // RCLCPP_INFO( this->get_logger(), "Viper850 new reference vel: [%0.2f %0.2f %0.2f] m/s [%0.2f %0.2f %0.2f] rad/s",
  //              vref[0], vref[1], vref[2], vref[3], vref[4], vref[5] );
  m_robot->setVelocity( vpRobot::REFERENCE_FRAME, vref );
}

#endif // #ifdef VISP_HAVE_VIPER850

int
main( int argc, char **argv )
{
#ifdef VISP_HAVE_VIPER850
  rclcpp::init( argc, argv );
  auto node = std::make_shared< RosViper850Node >();

  try
  {
    if ( node->setup() != 0 )
    {
      RCLCPP_ERROR( node->get_logger(), "Viper850 setup failed... \n" );
      return EXIT_FAILURE;
    }

    rclcpp::WallRate loop_rate( 100ms );

    while ( rclcpp::ok() )
    {
      node->publish();

      rclcpp::spin_some( node );
      loop_rate.sleep();
    }
  }
  catch ( const vpException &e )
  {
    RCLCPP_ERROR( node->get_logger(), "Catch ViSP exception: %s", e.getMessage() );
  }
  catch ( const rclcpp::exceptions::RCLError &e )
  {
    RCLCPP_ERROR( node->get_logger(), "Unexpectedly failed with %s", e.what() );
  }

  RCLCPP_INFO( node->get_logger(), "Quitting... \n" );
  rclcpp::shutdown();
#else
  std::cout << "This node is not available since ViSP was" << std::endl
            << "not build with Viper850 robot support...\n"
            << std::endl;
#endif
  return EXIT_SUCCESS;
}
