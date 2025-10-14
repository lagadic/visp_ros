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

//! \example joy2twist_afma6.cpp
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <visp3/core/vpMath.h>

using namespace std::chrono_literals;

class Joy2twist: public rclcpp::Node
{
private:
  rclcpp::Subscription< sensor_msgs::msg::Joy >::SharedPtr m_sub_joy;
  rclcpp::Publisher< geometry_msgs::msg::TwistStamped >::SharedPtr m_pub_cmd;

  bool m_send_cmd{ false };
  unsigned int m_queue_size;

public:
  void callbackJoy( const sensor_msgs::msg::Joy::ConstSharedPtr &msg );
  Joy2twist();
  virtual ~Joy2twist(){};
};

Joy2twist::Joy2twist()
  : Node( "joy2twist_node" )
  , m_queue_size( 10 )
{
  RCLCPP_INFO( this->get_logger(), "Using joy2twist" );

  m_sub_joy = this->create_subscription< sensor_msgs::msg::Joy >(
      "joy", m_queue_size, std::bind( &Joy2twist::callbackJoy, this, std::placeholders::_1 ) );

  m_pub_cmd  = this->create_publisher< geometry_msgs::msg::TwistStamped >( "/cmd_camvel", m_queue_size );
}

void
Joy2twist::callbackJoy( const sensor_msgs::msg::Joy::ConstSharedPtr &msg )
{
  std::ostringstream strs;
  strs << std::endl << "axes: [";
  for ( size_t i = 0; i < msg->axes.size(); i++ )
    strs << msg->axes[i] << ",";
  strs << "]" << std::endl;
  strs << "button: [";
  for ( size_t i = 0; i < msg->buttons.size(); i++ )
    strs << msg->buttons[i] << ",";
  strs << "]" << std::endl;
  std::string str;
  str = strs.str();
  //ROS_DEBUG( "%s", str.c_str() );

  // Use Logitech wireless gamepad F310
  // See http://wiki.ros.org/joy#Logitech_Wireless_Gamepad_F710_.28DirectInput_Mode.29
  // button 4 (LB) or 5 (RB) active : DMS to send commands 
  // axis 0 (left right) = rotation around camera y axis
  // axis 1 (top down) = rotation around camera x axis
  // axis 3 (left right) = linear vel along x camera
  // axis 4 (top down)   = linear vel along z camera
  if ( msg->buttons[4] || msg->buttons[5] )
    m_send_cmd = true;
  else
    m_send_cmd = false;

  geometry_msgs::msg::TwistStamped cmd_vel;
  if ( m_send_cmd )
  {
    cmd_vel.header.stamp    = rclcpp::Clock(RCL_ROS_TIME).now();
    cmd_vel.twist.angular.x = -vpMath::rad( msg->axes[1] * 180 / 20. );
    cmd_vel.twist.angular.y = -vpMath::rad( msg->axes[0] * 180 / 20. );
    cmd_vel.twist.linear.x  = -msg->axes[3] / 10.;
    cmd_vel.twist.linear.z  = msg->axes[4] / 10.;
  }
  m_pub_cmd->publish( cmd_vel );
}

int
main( int argc, char **argv )
{
  rclcpp::init( argc, argv );
  auto node = std::make_shared< Joy2twist >();
  try
  {
    rclcpp::WallRate loop_rate( 100ms );

    while ( rclcpp::ok() )
    {
      rclcpp::spin_some( node );
      loop_rate.sleep();
    }
  }
  catch ( const rclcpp::exceptions::RCLError &e )
  {
    RCLCPP_ERROR( node->get_logger(), "Unexpectedly failed with %s", e.what() );
  }

  RCLCPP_INFO( node->get_logger(), "Quitting joy2twist... \n" );
  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
