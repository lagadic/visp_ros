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
  \file vpROSGrabber.h
  \brief class for Camera video capture for ROS middleware.
*/

#ifndef vpROSGrabber_h
#define vpROSGrabber_h

#include <visp3/core/vpConfig.h>

#if defined( VISP_HAVE_OPENCV )

#include <visp3/core/vpFrameGrabber.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpRGBa.h>

#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visp_bridge/camera.h>

#if VISP_HAVE_OPENCV_VERSION >= 0x020101
#include <opencv2/highgui/highgui.hpp>
#else
#include <highgui.h>
#endif

/*!
  \class vpROSGrabber

  \ingroup Framegrabber CameraDriver
  \brief Class for cameras video capture for ROS cameras.

  The code below shows how to use this class.
  \code
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp_ros/vpROSGrabber.h>

int main()
{
#if defined(VISP_HAVE_OPENCV)
  vpImage<unsigned char> I; // Create a gray level image container
  vpROSGrabber g;        // Create a grabber for ROS
  g.setCameraInfoTopic("/camera/camera_info");
  g.setImageTopic("/camera/image_raw");
  g.setRectify(true);

  g.open(I);                           // Open the framegrabber
  g.acquire(I);                        // Acquire an image
  vpImageIo::writePGM(I, "image.pgm"); // Write image on the disk
#endif
}
  \endcode

 */
class VISP_EXPORT vpROSGrabber : public vpFrameGrabber //, rclcpp::Node
{
protected:
  image_transport::Subscriber m_img_sub;
  image_transport::CameraSubscriber m_img_cam_sub;

  bool m_isInitialized;
  unsigned int m_width;
  unsigned int m_height;
  image_geometry::PinholeCameraModel m_p;
  cv::Mat m_img;
  bool m_flip;
  bool m_rectify;
  bool m_mutex_img;
  void imageCb( const sensor_msgs::msg::Image::ConstSharedPtr &msg );
  void imageAndCamInfoCb( const sensor_msgs::msg::Image::ConstSharedPtr image_msg,
                          const sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg );
  bool m_first_img_received;
  bool m_first_cam_info_received;
  bool m_subscribe_camera_info;
  uint32_t m_sec, m_nanosec;
  std::string m_topic_image;
  vpCameraParameters m_cam;
  std::shared_ptr< rclcpp::Node > m_nh;

public:
  vpROSGrabber();
  virtual ~vpROSGrabber();

  void open();
  void open( int argc, char **argv );
  void open( vpImage< unsigned char > &I );
  void open( vpImage< vpRGBa > &I );

  void acquire( vpImage< unsigned char > &I );
  void acquire( vpImage< vpRGBa > &I );
  cv::Mat acquire();
  bool acquireNoWait( vpImage< unsigned char > &I );
  bool acquireNoWait( vpImage< vpRGBa > &I );

  void acquire( vpImage< unsigned char > &I, struct timespec &timestamp );
  void acquire( vpImage< vpRGBa > &I, struct timespec &timestamp );
  void acquire( vpImage< unsigned char > &I, double &timestamp_second );
  void acquire( vpImage< vpRGBa > &I, double &timestamp_second );
  cv::Mat acquire( struct timespec &timestamp );
  bool acquireNoWait( vpImage< unsigned char > &I, struct timespec &timestamp );
  bool acquireNoWait( vpImage< vpRGBa > &I, struct timespec &timestamp );

  void close();

  void subscribeImageTopic( const std::string &topic_name );
  void subscribeCameraInfoTopic( bool enable );
  void setFlip( bool flip );
  void setRectify( bool rectify );

  bool getCameraInfo( vpCameraParameters &cam );
  void getWidth( unsigned int &width ) const;
  void getHeight( unsigned int &height ) const;
  unsigned int getWidth() const;
  unsigned int getHeight() const;
};

#endif
#endif
