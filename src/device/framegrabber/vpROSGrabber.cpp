/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2021 by INRIA. All rights reserved.
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
 * Camera video capture for ROS image_transort_compressed.
 *
 * Authors:
 * Francois Pasteau
 * Fabien Spindler
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
#include <sensor_msgs/CompressedImage.h>

#include <iostream>
#include <math.h>

/*!
  Basic Constructor that subscribes to the "image" and "camera_info" ROS topics.
*/
vpROSGrabber::vpROSGrabber()
  : m_isInitialized( false )
  , m_flip( false )
  , m_rectify( true )
  , m_mutex_image( true )
  , m_mutex_param( true )
  , m_first_img_received( false )
  , m_first_param_received( false )
  , m_sec( 0 )
  , m_nsec( 0 )
  , m_master_uri( "http://localhost:11311" )
  , m_topic_image( "image" )
  , m_topic_cam_info( "camera_info" )
  , m_nodespace( "" )
  , m_image_transport( "raw" )
{
}

/*!
  Basic destructor that calls the close() method.

  \sa close()
*/
vpROSGrabber::~vpROSGrabber() { close(); }

/*!
  Initialization of the grabber.
    Generic initialization of the grabber using parameter from the main function
    To be used to create ros node that can be started with rosrun

  \param argc : number of arguments from the main function

  \param argv : arguments from the main function

*/

void
vpROSGrabber::open( int argc, char **argv )
{

  if ( !m_isInitialized )
  {
    std::string str;
    if ( !ros::isInitialized() )
      ros::init( argc, argv, "visp_node", ros::init_options::AnonymousName );
    m_n = new ros::NodeHandle;
    if ( m_image_transport == "raw" )
    {
      if ( ros::param::get( "~image_transport", str ) )
      {
        m_image_transport = str;
      }
      else
      {
        m_image_transport = "raw";
        ros::param::set( "~image_transport", "raw" );
      }
    }
    if ( m_image_transport == "raw" )
    {
      std::cout << "Subscribe to raw image on " << m_nodespace + m_topic_image << " topic" << std::endl;
      m_img_sub = m_n->subscribe( m_nodespace + m_topic_image, 1, &vpROSGrabber::imageCallbackRaw, this,
                                  ros::TransportHints().tcpNoDelay() );
    }
    else
    {
      std::cout << "Subscribe to image on " << m_nodespace + m_topic_image << " topic" << std::endl;
      m_img_sub = m_n->subscribe( m_nodespace + m_topic_image, 1, &vpROSGrabber::imageCallback, this,
                                  ros::TransportHints().tcpNoDelay() );
    }

    std::cout << "Subscribe to camera_info on " << m_nodespace + m_topic_cam_info << " topic" << std::endl;
    m_cam_info_sub = m_n->subscribe( m_nodespace + m_topic_cam_info, 1, &vpROSGrabber::paramCallback, this,
                                     ros::TransportHints().tcpNoDelay() );

    m_spinner = new ros::AsyncSpinner( 1 );
    m_spinner->start();
    m_width         = 0;
    m_height        = 0;
    m_isInitialized = true;
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
  if ( ros::isInitialized() && ros::master::getURI() != m_master_uri )
  {
    close();
    throw( vpFrameGrabberException( vpFrameGrabberException::initializationError,
                                    "ROS grabber already initialised with a different master_URI (" +
                                        ros::master::getURI() + " != " + m_master_uri + ")" ) );
  }
  if ( !m_isInitialized )
  {
    int argc = 2;
    char *argv[2];
    argv[0] = new char[255];
    argv[1] = new char[255];

    std::string exe = "ros.exe", arg1 = "__master:=";
    strcpy( argv[0], exe.c_str() );
    arg1.append( m_master_uri );
    strcpy( argv[1], arg1.c_str() );
    open( argc, argv );

    // Wait for a first image
    while ( !m_first_img_received )
      vpTime::wait( 40 );

    delete[] argv[0];
    delete[] argv[1];
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
  if ( m_isInitialized == false )
  {
    close();
    throw( vpFrameGrabberException( vpFrameGrabberException::initializationError, "Grabber not initialized." ) );
  }
  while ( !m_mutex_image || !m_first_img_received )
    ;
  m_mutex_image     = false;
  timestamp.tv_sec  = m_sec;
  timestamp.tv_nsec = m_nsec;
  vpImageConvert::convert( m_img, I, m_flip );
  m_first_img_received = false;
  m_mutex_image        = true;
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
  bool new_image = false;
  if ( m_isInitialized == false )
  {
    close();
    throw( vpFrameGrabberException( vpFrameGrabberException::initializationError, "Grabber not initialized." ) );
  }
  while ( !m_mutex_image )
    ;
  m_mutex_image     = false;
  timestamp.tv_sec  = m_sec;
  timestamp.tv_nsec = m_nsec;
  vpImageConvert::convert( m_img, I, m_flip );
  new_image            = m_first_img_received;
  m_first_img_received = false;
  m_mutex_image        = true;
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
  if ( m_isInitialized == false )
  {
    close();
    throw( vpFrameGrabberException( vpFrameGrabberException::initializationError, "Grabber not initialized." ) );
  }
  while ( !m_mutex_image || !m_first_img_received )
    ;
  m_mutex_image     = false;
  timestamp.tv_sec  = m_sec;
  timestamp.tv_nsec = m_nsec;
  vpImageConvert::convert( m_img, I, m_flip );
  m_first_img_received = false;
  m_mutex_image        = true;
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
  bool new_image = false;
  if ( m_isInitialized == false )
  {
    close();
    throw( vpFrameGrabberException( vpFrameGrabberException::initializationError, "Grabber not initialized." ) );
  }
  while ( !m_mutex_image )
    ;
  m_mutex_image     = false;
  timestamp.tv_sec  = m_sec;
  timestamp.tv_nsec = m_nsec;
  vpImageConvert::convert( m_img, I, m_flip );
  new_image            = m_first_img_received;
  m_first_img_received = false;
  m_mutex_image        = true;
  return new_image;
}

/*!
  Grab an image direclty in the OpenCV format.

  \param timestamp : timestamp of the acquired image.

  \return Acquired image.

  \exception vpFrameGrabberException::initializationError If the
  initialization of the grabber was not done previously.
*/
cv::Mat
vpROSGrabber::acquire( struct timespec &timestamp )
{
  cv::Mat img;
  if ( m_isInitialized == false )
  {
    close();
    throw( vpFrameGrabberException( vpFrameGrabberException::initializationError, "Grabber not initialized." ) );
  }
  while ( !m_mutex_image || !m_first_img_received )
    ;
  m_mutex_image        = false;
  timestamp.tv_sec     = m_sec;
  timestamp.tv_nsec    = m_nsec;
  img                  = m_img.clone();
  m_first_img_received = false;
  m_mutex_image        = true;
  return img;
}

/*!
  Grab a gray level image

  \param I : Acquired gray level image.

  \exception vpFrameGrabberException::initializationError If the initialization of the grabber was not done previously.
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

  \exception vpFrameGrabberException::initializationError If the initialization of the grabber was not done previously.
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

  \exception vpFrameGrabberException::initializationError If the initialization of the grabber was not done previously.
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

  \exception vpFrameGrabberException::initializationError If the initialization of the grabber was not done previously.
*/
bool
vpROSGrabber::acquireNoWait( vpImage< vpRGBa > &I )
{
  struct timespec timestamp;
  return acquireNoWait( I, timestamp );
}

/*!
  Grab an image direclty in the OpenCV format.

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
    m_spinner->stop();
    delete m_spinner;
    delete m_n;
  }
}

/*!
  Set the boolean variable flip to the expected value.

  \param flipType : Expected value of the variable flip. True means that the
  image is flipped during each image acquisition.

  \warning This function is only useful under Windows.

  \note The aim of this function is to fix a problem which appears under Windows.
  Indeed with several cameras the aquired images are flipped.
*/
void
vpROSGrabber::setFlip( bool flipType )
{
  m_flip = flipType;
}

/*!
  Set the boolean variable rectify to the expected value.

  \param rectify : Expected value of the variable rectify. True means
  that the image is rectified during each image acquisition.

  \warning Rectification will happen only if the CameraInfo are correctly received.
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
vpROSGrabber::getWidth( unsigned int width ) const
{
  width = getWidth();
}

/*!
  Get the height of the image.

  \param height : height of the image.

*/

void
vpROSGrabber::getHeight( unsigned int height ) const
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

  Set the ROS topic name for CameraInfo

  \param topic_name : Name of the topic.

*/
void
vpROSGrabber::setCameraInfoTopic( const std::string &topic_name )
{
  m_topic_cam_info = topic_name;
}

/*!

  Set the ROS topic name for Images

  \param topic_name : Name of the topic.

*/
void
vpROSGrabber::setImageTopic( const std::string &topic_name )
{
  m_topic_image = topic_name;
}

/*!

  Set the URI for ROS Master

  \param master_uri : URI of the master ("http://127.0.0.1:11311")

*/
void
vpROSGrabber::setMasterURI( const std::string &master_uri )
{
  m_master_uri = master_uri;
}

/*!

  Set the nodespace

  \param nodespace Namespace of the connected camera (nodespace is appended to the all topic names)

*/
void
vpROSGrabber::setNodespace( const std::string &nodespace )
{
  m_nodespace = nodespace;
}

void
setImageTransport( const std::string image_transport );

/*!

  Set the image_transport type of the image topic. Values should be :
  - "raw" if images are not compressed
  - any other value if the images are compressed (ie "jpeg",...).

  \param image_transport type of transport of the image topic

*/
void
vpROSGrabber::setImageTransport( const std::string &image_transport )
{
  m_image_transport = image_transport;
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

  // Test: if we get an image (m_first_img_received=true) we should have the camera parameters
  // (m_first_param_received=true) if they are available
  if ( m_first_img_received && !m_first_param_received )
    return false;
  while ( !m_mutex_param || !m_first_img_received )
    ;
  m_mutex_param = false;
  cam           = m_cam;
  m_mutex_param = true;

  return true;
}

void
vpROSGrabber::imageCallback( const sensor_msgs::CompressedImage::ConstPtr &msg )
{
  cv::Mat img        = cv::imdecode( msg->data, 1 );
  cv::Size data_size = img.size();

  while ( !m_mutex_image )
    ;
  m_mutex_image = false;
  if ( m_rectify && m_p.initialized() )
  {
    m_p.rectifyImage( img, m_img );
  }
  else
  {
    img.copyTo( m_img );
  }
  m_width              = static_cast< unsigned int >( data_size.width );
  m_height             = static_cast< unsigned int >( data_size.height );
  m_sec                = msg->header.stamp.sec;
  m_nsec               = msg->header.stamp.nsec;
  m_first_img_received = true;
  m_mutex_image        = true;
}

void
vpROSGrabber::imageCallbackRaw( const sensor_msgs::Image::ConstPtr &msg )
{
  while ( !m_mutex_image )
    ;
  m_mutex_image = false;
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare( msg, "bgr8" );
  }
  catch ( cv_bridge::Exception &e )
  {
    ROS_ERROR( "cv_bridge exception: %s", e.what() );
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
  m_nsec               = msg->header.stamp.nsec;
  m_first_img_received = true;
  m_mutex_image        = true;
}

void
vpROSGrabber::paramCallback( const sensor_msgs::CameraInfo::ConstPtr &msg )
{
  if ( m_rectify )
  {
    while ( !m_mutex_image )
      ;
    m_mutex_image = false;
    m_cam         = visp_bridge::toVispCameraParameters( *msg );
    m_p.fromCameraInfo( msg );
    m_first_param_received = true;
    m_mutex_image          = true;
  }
}

#endif
