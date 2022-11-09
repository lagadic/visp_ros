/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2022 by INRIA. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 *
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * Camera video capture for ROS image.
 *
 *****************************************************************************/

/*!
  \file vpROSGrabber.cpp
  \brief class for cameras video capture using ROS middleware.
*/

#include <visp_ros/vpROSGrabber.h>

#if defined( VISP_HAVE_OPENCV )

#include <visp3/core/vpFrameGrabberException.h>
#include <visp3/core/vpImageConvert.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <iostream>
#include <math.h>

using namespace std::chrono_literals;

/*!
  Basic Constructor that subscribes to the "image" and "camera_info" ROS topics.
*/
vpROSGrabber::vpROSGrabber()
  : /*Node( "visp_ros_grabber" )
  ,*/
  m_isInitialized( false )
  , m_flip( false )
  , m_rectify( false )
  , m_mutex_img( true )
  , m_first_img_received( false )
  , m_first_cam_info_received( false )
  , m_subscribe_camera_info( false )
  , m_sec( 0 )
  , m_nanosec( 0 )
  , m_topic_image( "image" )
{
}

/*!
  Basic destructor that calls the close() method.

  \sa close()
*/
vpROSGrabber::~vpROSGrabber() { close(); }

/*!
  Initialization of the grabber.
  Generic initialization of the grabber using parameter from the main function.
  To be used to create ros node that can be started with `ros2 run`.

  \param argc : number of arguments from the main function

  \param argv : arguments from the main function

*/
void
vpROSGrabber::open( int argc, char **argv )
{
  if ( !m_isInitialized )
  {
    std::string str;
    if ( !rclcpp::ok() )
    {
      rclcpp::init( argc, argv );
    }
    m_nh = rclcpp::Node::make_shared( "ros_grabber" );
    RCLCPP_INFO_STREAM( m_nh->get_logger(), "Subscribe to raw image on " << m_topic_image << " topic" );
    if ( m_subscribe_camera_info )
    {
      RCLCPP_INFO( m_nh->get_logger(), "Subscribe also to camera_info topic" );
      m_img_cam_sub = image_transport::create_camera_subscription(
          m_nh.get(), m_topic_image,
          std::bind( &vpROSGrabber::imageAndCamInfoCb, this, std::placeholders::_1, std::placeholders::_2 ), "raw" );
    }
    else
    {
      m_img_sub = image_transport::create_subscription(
          m_nh.get(), m_topic_image, std::bind( &vpROSGrabber::imageCb, this, std::placeholders::_1 ), "raw" );
    }
    m_width         = 0;
    m_height        = 0;
    m_isInitialized = true;

    rclcpp::spin_some( m_nh );
  }
}

/*!
  Initialization of the grabber.

  Generic initialization of the grabber.

  \exception vpFrameGrabberException::initializationError If ROS has already been initialised with a different
  master_URI.

*/
void
vpROSGrabber::open()
{
  if ( !m_isInitialized )
  {
    int argc = 1;
    char *argv[1];
    argv[0] = new char[255];

    std::string exe = "ros_grabber_exe";
    strcpy( argv[0], exe.c_str() );
    open( argc, argv );

    // Wait for a first image
    rclcpp::WallRate loop_rate( 40ms );
    while ( !m_first_img_received )
    {
      rclcpp::spin_some( m_nh );
      loop_rate.sleep();
    }
    delete[] argv[0];
  }
}

/*!
  Initialization of the grabber.

  Call the generic initialization method.

  \param I : Gray level image. This parameter is not used.

  \sa open()
*/
void
vpROSGrabber::open( vpImage< unsigned char > &I )
{
  open();
  acquire( I );
}

/*!
  Initialization of the grabber.

  Call the generic initialization method.

  \param I : Color image. This parameter is not used.

  \sa open()
*/
void
vpROSGrabber::open( vpImage< vpRGBa > &I )
{
  open();
  acquire( I );
}

/*!
  Grab a gray level image with timestamp

  \param I : Acquired gray level image.

  \param timestamp : timestamp of the acquired image.

  \exception vpFrameGrabberException::initializationError If the

  initialization of the grabber was not done previously.
*/
void
vpROSGrabber::acquire( vpImage< unsigned char > &I, struct timespec &timestamp )
{
  rclcpp::spin_some( m_nh );

  if ( m_isInitialized == false )
  {
    close();
    throw( vpFrameGrabberException( vpFrameGrabberException::initializationError, "Grabber not initialized." ) );
  }
  rclcpp::WallRate loop_rate( 100us );
  while ( !m_mutex_img || !m_first_img_received )
  {
    rclcpp::spin_some( m_nh );
    loop_rate.sleep();
  }
  m_mutex_img       = false;
  timestamp.tv_sec  = m_sec;
  timestamp.tv_nsec = m_nanosec;
  vpImageConvert::convert( m_img, I, m_flip );
  m_first_img_received = false;
  m_mutex_img          = true;
}

/*!
  Grab a gray level image with timestamp

  \param I : Acquired gray level image.

  \param timestamp_second : timestamp expressed in [s] of the acquired image.

  \exception vpFrameGrabberException::initializationError If the

  initialization of the grabber was not done previously.
*/
void
vpROSGrabber::acquire( vpImage< unsigned char > &I, double &timestamp_second )
{
  struct timespec timestamp;
  acquire( I, timestamp );
  timestamp_second = timestamp.tv_sec + static_cast< double >( timestamp.tv_nsec ) * 1e-9;
}

/*!
  Grab a gray level image with timestamp without waiting.

  \param I : Acquired gray level image.

  \param timestamp : timestamp of the acquired image.

  \return true if a new image was acquired or false if the image is the same than the previous one.

  \exception vpFrameGrabberException::initializationError If the initialization of the grabber was not done previously.
*/
bool
vpROSGrabber::acquireNoWait( vpImage< unsigned char > &I, struct timespec &timestamp )
{
  rclcpp::spin_some( m_nh );

  bool new_image = false;
  if ( m_isInitialized == false )
  {
    close();
    throw( vpFrameGrabberException( vpFrameGrabberException::initializationError, "Grabber not initialized." ) );
  }
  rclcpp::WallRate loop_rate( 100us );
  while ( !m_mutex_img )
  {
    rclcpp::spin_some( m_nh );
    loop_rate.sleep();
  }
  m_mutex_img       = false;
  timestamp.tv_sec  = m_sec;
  timestamp.tv_nsec = m_nanosec;
  vpImageConvert::convert( m_img, I, m_flip );
  new_image            = m_first_img_received;
  m_first_img_received = false;
  m_mutex_img          = true;

  return new_image;
}

/*!
  Grab a color image with timestamp

  \param I : Acquired color image.

  \param timestamp : timestamp of the acquired image.

  \exception vpFrameGrabberException::initializationError If the initialization of the grabber was not done previously.
*/
void
vpROSGrabber::acquire( vpImage< vpRGBa > &I, struct timespec &timestamp )
{
  rclcpp::spin_some( m_nh );

  if ( m_isInitialized == false )
  {
    close();
    throw( vpFrameGrabberException( vpFrameGrabberException::initializationError, "Grabber not initialized." ) );
  }
  rclcpp::WallRate loop_rate( 100us );
  while ( !m_mutex_img || !m_first_img_received )
  {
    rclcpp::spin_some( m_nh );
    loop_rate.sleep();
  }
  m_mutex_img       = false;
  timestamp.tv_sec  = m_sec;
  timestamp.tv_nsec = m_nanosec;
  vpImageConvert::convert( m_img, I, m_flip );
  m_first_img_received = false;
  m_mutex_img          = true;
}

/*!
  Grab a color image with timestamp

  \param I : Acquired color image.

  \param timestamp_second : timestamp expressed in [s] of the acquired image.

  \exception vpFrameGrabberException::initializationError If the initialization of the grabber was not done previously.
*/
void
vpROSGrabber::acquire( vpImage< vpRGBa > &I, double &timestamp_second )
{
  struct timespec timestamp;
  acquire( I, timestamp );
  timestamp_second = timestamp.tv_sec + static_cast< double >( timestamp.tv_nsec ) * 1e-9;
}

/*!
  Grab a color image with timestamp without waiting.

  \param I : Acquired color image.

  \param timestamp : timestamp of the acquired image.

  \return true if a new image was acquired or false if the image is the same than the previous one.

  \exception vpFrameGrabberException::initializationError If the initialization of the grabber was not done previously.
*/
bool
vpROSGrabber::acquireNoWait( vpImage< vpRGBa > &I, struct timespec &timestamp )
{
  rclcpp::spin_some( m_nh );

  bool new_image = false;
  if ( m_isInitialized == false )
  {
    close();
    throw( vpFrameGrabberException( vpFrameGrabberException::initializationError, "Grabber not initialized." ) );
  }
  rclcpp::WallRate loop_rate( 100us );
  while ( !m_mutex_img )
  {
    rclcpp::spin_some( m_nh );
    loop_rate.sleep();
  }
  m_mutex_img       = false;
  timestamp.tv_sec  = m_sec;
  timestamp.tv_nsec = m_nanosec;
  vpImageConvert::convert( m_img, I, m_flip );
  new_image            = m_first_img_received;
  m_first_img_received = false;
  m_mutex_img          = true;
  return new_image;
}

/*!
  Grab an image directly in the OpenCV format.

  \param timestamp : timestamp of the acquired image.

  \return Acquired image.

  \exception vpFrameGrabberException::initializationError If the
  initialization of the grabber was not done previously.
*/
cv::Mat
vpROSGrabber::acquire( struct timespec &timestamp )
{
  rclcpp::spin_some( m_nh );

  cv::Mat img;
  if ( m_isInitialized == false )
  {
    close();
    throw( vpFrameGrabberException( vpFrameGrabberException::initializationError, "Grabber not initialized." ) );
  }
  rclcpp::WallRate loop_rate( 100us );
  while ( !m_mutex_img || !m_first_img_received )
  {
    rclcpp::spin_some( m_nh );
    loop_rate.sleep();
  }

  m_mutex_img          = false;
  timestamp.tv_sec     = m_sec;
  timestamp.tv_nsec    = m_nanosec;
  img                  = m_img.clone();
  m_first_img_received = false;
  m_mutex_img          = true;
  return img;
}

/*!
  Grab a gray level image

  \param I : Acquired gray level image.

  \exception vpFrameGrabberException::initializationError If the initialization of the grabber was not done
  previously.
*/
void
vpROSGrabber::acquire( vpImage< unsigned char > &I )
{
  struct timespec timestamp;
  acquire( I, timestamp );
}

/*!
  Grab a gray level image without waiting.

  \param I : Acquired gray level image.

  \return true if a new image was acquired or false if the image is the same than the previous one.

  \exception vpFrameGrabberException::initializationError If the initialization of the grabber was not done
  previously.
*/
bool
vpROSGrabber::acquireNoWait( vpImage< unsigned char > &I )
{
  struct timespec timestamp;
  return acquireNoWait( I, timestamp );
}

/*!
  Grab a color image

  \param I : Acquired color image.

  \exception vpFrameGrabberException::initializationError If the initialization of the grabber was not done
  previously.
*/
void
vpROSGrabber::acquire( vpImage< vpRGBa > &I )
{
  struct timespec timestamp;
  acquire( I, timestamp );
}

/*!
  Grab a color image without waiting.

  \param I : Acquired color image.

  \return true if a new image was acquired or false if the image is the same than the previous one.

  \exception vpFrameGrabberException::initializationError If the initialization of the grabber was not done
  previously.
*/
bool
vpROSGrabber::acquireNoWait( vpImage< vpRGBa > &I )
{
  struct timespec timestamp;
  return acquireNoWait( I, timestamp );
}

/*!
  Grab an image directly in the OpenCV format.

  \return Acquired image.

  \exception vpFrameGrabberException::initializationError If the
  initialization of the grabber was not done previously.
*/
cv::Mat
vpROSGrabber::acquire()
{
  struct timespec timestamp;
  return acquire( timestamp );
}

void
vpROSGrabber::close()
{
  if ( m_isInitialized )
  {
    m_isInitialized = false;
  }
}

/*!
  Set the boolean variable flip to the expected value.

  \param flip : When true the image is flipped during each image acquisition.

*/
void
vpROSGrabber::setFlip( bool flip )
{
  m_flip = flip;
}

/*!
  Set the boolean variable rectify to the expected value.

  \param rectify : When true the image is rectified during each image
  acquisition by using camera parameters.

  \warning Rectification will happen only if the `cam_info` topic is published
  besides the image topic.
*/
void
vpROSGrabber::setRectify( bool rectify )
{
  m_rectify = rectify;
}

/*!
  Get the width of the image.

  \param width : width of the image.

*/
void
vpROSGrabber::getWidth( unsigned int &width ) const
{
  width = getWidth();
}

/*!
  Get the height of the image.

  \param height : height of the image.

*/

void
vpROSGrabber::getHeight( unsigned int &height ) const
{
  height = getHeight();
}

/*!
  Get the width of the image.

  \return width of the image.

*/
unsigned int
vpROSGrabber::getWidth() const
{
  return m_width;
}

/*!
  Get the height of the image.

  \return height of the image.

*/
unsigned int
vpROSGrabber::getHeight() const
{
  return m_height;
}

/*!

  Set the ROS image topic name. Default is `image_raw`.

  \param topic_name : Name of the image topic.

  \sa subscribeCameraInfoTopic()
*/
void
vpROSGrabber::subscribeImageTopic( const std::string &topic_name )
{
  m_topic_image = topic_name;
}

/*!

  Enable/disable camera info subscription. By default, camera info subscription is disabled.

  \param enable : When true, subscribe also to `camera_info` topic.
  When false, this feature is disabled.

  \sa subscribeImageTopic()
*/
void
vpROSGrabber::subscribeCameraInfoTopic( bool enable )
{
  m_subscribe_camera_info = enable;
}

/*!
  Get the vpCameraParameters from the camera.

  \param cam : Parameter of the camera.

  \return true if the parameters are available, false otherwise.
*/
bool
vpROSGrabber::getCameraInfo( vpCameraParameters &cam )
{
  if ( !m_isInitialized )
  {
    close();
    throw( vpFrameGrabberException( vpFrameGrabberException::initializationError, "Grabber not initialized." ) );
  }

  // Wait for an image
  rclcpp::WallRate loop_rate( 100us );
  while ( !m_first_img_received )
  {
    rclcpp::spin_some( m_nh );
    loop_rate.sleep();
  }

  // If we get an image (m_first_img_received=true) we should have the camera parameters
  if ( !m_first_cam_info_received )
  {
    return false;
  }

  cam = m_cam;

  return true;
}

void
vpROSGrabber::imageCb( const sensor_msgs::msg::Image::ConstSharedPtr &msg )
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare( msg, "bgr8" );
  }
  catch ( cv_bridge::Exception &e )
  {
    RCLCPP_ERROR( m_nh->get_logger(), "cv_bridge exception: %s", e.what() );
    return;
  }
  if ( m_rectify && m_p.initialized() )
  {
    m_p.rectifyImage( cv_ptr->image, m_img );
  }
  else
  {
    cv_ptr->image.copyTo( m_img );
  }
  cv::Size data_size   = m_img.size();
  m_width              = static_cast< unsigned int >( data_size.width );
  m_height             = static_cast< unsigned int >( data_size.height );
  m_sec                = msg->header.stamp.sec;
  m_nanosec            = msg->header.stamp.nanosec;
  m_first_img_received = true;
  m_mutex_img          = true;
}

void
vpROSGrabber::imageAndCamInfoCb( const sensor_msgs::msg::Image::ConstSharedPtr image_msg,
                                 const sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg )
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare( image_msg, "bgr8" );
  }
  catch ( cv_bridge::Exception &e )
  {
    RCLCPP_ERROR( m_nh->get_logger(), "cv_bridge exception: %s", e.what() );
    return;
  }

  m_cam = visp_bridge::toVispCameraParameters( info_msg );
  m_p.fromCameraInfo( info_msg );
  if ( m_rectify && m_p.initialized() )
  {
    m_p.rectifyImage( cv_ptr->image, m_img );
  }
  else
  {
    cv_ptr->image.copyTo( m_img );
  }
  cv::Size data_size        = m_img.size();
  m_width                   = static_cast< unsigned int >( data_size.width );
  m_height                  = static_cast< unsigned int >( data_size.height );
  m_sec                     = image_msg->header.stamp.sec;
  m_nanosec                 = image_msg->header.stamp.nanosec;
  m_first_img_received      = true;
  m_first_cam_info_received = true;
  m_mutex_img               = true;
}

#endif
