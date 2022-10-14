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

#include <chrono>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <visp3/robot/vpRobotAfma6.h>

#include <visp_bridge/3dpose.h>

using namespace std::chrono_literals;

#ifdef VISP_HAVE_AFMA6

class RosAfma6Node : public rclcpp::Node
{
public:
  RosAfma6Node();
  virtual ~RosAfma6Node();

public:
  int setup();
  void setCameraVel( const geometry_msgs::msg::Twist::ConstSharedPtr &msg );
  void spin();
  void publish();

protected:
  rclcpp::Publisher< geometry_msgs::msg::PoseStamped >::SharedPtr m_pose_pub;
  rclcpp::Publisher< geometry_msgs::msg::TwistStamped >::SharedPtr m_vel_pub;
  rclcpp::Subscription< geometry_msgs::msg::Twist >::SharedPtr m_cmd_camvel_sub;

  unsigned int m_queue_size;
  rclcpp::Time m_vel_time;

  std::string m_serial_port;

  vpRobotAfma6 *m_robot;
  geometry_msgs::msg::PoseStamped m_position;

  vpHomogeneousMatrix m_wMc; // world to camera transformation
  vpColVector m_q;           // measured joint position

  int m_tool_type = 0; // See https://visp-doc.inria.fr/doxygen/visp-daily/classvpAfma6.html structure vpAfma6ToolType
};

RosAfma6Node::RosAfma6Node()
  : Node( "afma_node" )
  , m_queue_size( 10 )
{
  RCLCPP_INFO( this->get_logger(), "Using Afma6 robot" );

  m_robot = NULL;

  m_tool_type = this->declare_parameter< int >( "tool_type", 0 );

  // Create publishers
  m_pose_pub = this->create_publisher< geometry_msgs::msg::PoseStamped >( "pose", m_queue_size );
  m_vel_pub  = this->create_publisher< geometry_msgs::msg::TwistStamped >( "velocity", m_queue_size );

  // Create subscribers
  m_cmd_camvel_sub = this->create_subscription< geometry_msgs::msg::Twist >(
      "cmd_camvel", m_queue_size, std::bind( &RosAfma6Node::setCameraVel, this, std::placeholders::_1 ) );
}

RosAfma6Node::~RosAfma6Node()
{
  if ( m_robot )
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

  m_robot->init( static_cast< vpAfma6::vpAfma6ToolType >( m_tool_type ),
                 vpCameraParameters::perspectiveProjWithDistortion );
  vpCameraParameters cam;
  m_robot->getCameraParameters( cam, 640, 480 );
  RCLCPP_INFO_STREAM( this->get_logger(), "Camera parameters (640 x 480):\n" << cam );

  m_robot->setRobotState( vpRobot::STATE_VELOCITY_CONTROL );

  return 0;
}

void
RosAfma6Node::publish()
{
  double timestamp = 0;
  m_robot->getPosition( vpRobot::ARTICULAR_FRAME, m_q, timestamp );
  int32_t timestamp_second      = (int32_t)( std::floor( timestamp ) );
  uint32_t timestamp_nanosecond = (uint32_t)( ( timestamp - std::floor( timestamp ) ) * 1e9 );
  m_wMc                         = m_robot->get_fMc( m_q );
  m_position.pose               = visp_bridge::toGeometryMsgsPose( m_wMc );
  m_position.header.stamp       = rclcpp::Time(
            timestamp_second, timestamp_nanosecond ); // to improve: should be the timestamp returned by getPosition()

  //  RCLCPP_INFO( this->get_logger(), "Afma6 publish pose at %f s: [%0.2f %0.2f %0.2f] - [%0.2f %0.2f %0.2f %0.2f]",
  //            timestamp,
  //            m_position.pose.position.x, m_position.pose.position.y, m_position.pose.position.z,
  //            m_position.pose.orientation.w, m_position.pose.orientation.x, m_position.pose.orientation.y,
  //            m_position.pose.orientation.z);
  m_pose_pub->publish( m_position );

  vpColVector vel( 6 );
  m_robot->getVelocity( vpRobot::CAMERA_FRAME, vel, timestamp );
  geometry_msgs::msg::TwistStamped vel_msg;
  vel_msg.header.stamp    = rclcpp::Time( timestamp_second, timestamp_nanosecond );
  vel_msg.twist.linear.x  = vel[0];
  vel_msg.twist.linear.y  = vel[1];
  vel_msg.twist.linear.z  = vel[2];
  vel_msg.twist.angular.x = vel[3];
  vel_msg.twist.angular.y = vel[4];
  vel_msg.twist.angular.z = vel[5];
  m_vel_pub->publish( vel_msg );
}

void
RosAfma6Node::setCameraVel( const geometry_msgs::msg::Twist::ConstSharedPtr &msg )
{
  m_vel_time = rclcpp::Node::now();

  vpColVector vc( 6 ); // Vel in m/s and rad/s

  vc[0] = msg->linear.x;
  vc[1] = msg->linear.y;
  vc[2] = msg->linear.z;

  vc[3] = msg->angular.x;
  vc[4] = msg->angular.y;
  vc[5] = msg->angular.z;

  // RCLCPP_INFO( this->get_logger(), "Afma6 new camera vel: [%0.2f %0.2f %0.2f] m/s [%0.2f %0.2f %0.2f] rad/s", vc[0],
  //              vc[1], vc[2], vc[3], vc[4], vc[5] );
  m_robot->setVelocity( vpRobot::CAMERA_FRAME, vc );
}

#endif // #ifdef VISP_HAVE_AFMA6

int
main( int argc, char **argv )
{
#ifdef VISP_HAVE_AFMA6
  rclcpp::init( argc, argv );
  auto node = std::make_shared< RosAfma6Node >();

  try
  {
    if ( node->setup() != 0 )
    {
      RCLCPP_ERROR( node->get_logger(), "Afma6 setup failed... \n" );
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
            << "not build with Afma6 robot support...\n"
            << std::endl;
#endif
  return EXIT_SUCCESS;
}
