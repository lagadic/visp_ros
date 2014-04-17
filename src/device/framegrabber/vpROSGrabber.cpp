/****************************************************************************
 *
 * $Id: vpROSGrabber.cpp 3530 2012-01-03 10:52:12Z fpasteau $
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2012 by INRIA. All rights reserved.
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
 *
 * Description:
 * Camera video capture for ROS image_transort_compressed.
 *
 * Authors:
 * Francois Pasteau
 *
 *****************************************************************************/

/*!
  \file vpROSGrabber.cpp
  \brief class for cameras video capture using ROS middleware.
*/

#include <visp_ros/vpROSGrabber.h>

#if defined(VISP_HAVE_OPENCV)

#include <visp/vpImageConvert.h>
#include <visp/vpFrameGrabberException.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <math.h>

/*!
  Basic Constructor.
*/
vpROSGrabber::vpROSGrabber() :
  isInitialized(false),
  mutex_image(true),
  mutex_param(true),
  first_img_received(false),
  first_param_received(false),
  _rectify(true),
  flip(false),
  _topic_image("image"),
  _topic_info("camera_info"),
  _master_uri("http://127.0.0.1:11311"),
  _nodespace(""),
  _image_transport("raw"),
  _sec(0),
  _nsec(0)
{

}


/*!
  Basic destructor that calls the close() method.

  \sa close()
*/
vpROSGrabber::~vpROSGrabber( )
{
  close();
}



/*!
  Initialization of the grabber.
    Generic initialization of the grabber using parameter from the main function
    To be used to create ros node that can be started with rosrun

  \param argc : number of arguments from the main function

  \param argv : arguments from the main function

*/

void vpROSGrabber::open(int argc, char **argv){

  if(!isInitialized){
    std::string str;
    if(!ros::isInitialized()) ros::init(argc, argv, "visp_node", ros::init_options::AnonymousName);
    n = new ros::NodeHandle;
    if(_image_transport == "raw"){
      if (ros::param::get("~image_transport",  str)){
        _image_transport = str;
      }else{
        _image_transport = "raw";
        ros::param::set("~image_transport", "raw");
      }
    }
    if(_image_transport == "raw")
      image_data = n->subscribe(_nodespace + _topic_image, 1, &vpROSGrabber::imageCallbackRaw,this,ros::TransportHints().tcpNoDelay());
    else
      image_data = n->subscribe(_nodespace + _topic_image, 1, &vpROSGrabber::imageCallback,this,ros::TransportHints().tcpNoDelay());

    image_info = n->subscribe(_nodespace + _topic_info, 1, &vpROSGrabber::paramCallback,this,ros::TransportHints().tcpNoDelay());

    spinner = new ros::AsyncSpinner(1);
    spinner->start();
    usWidth = 640;
    usHeight = 480;
    isInitialized = true;
  }
}





/*!
  Initialization of the grabber.

    Generic initialization of the grabber.

    \exception vpFrameGrabberException::initializationError If ROS has already been initialised with a different master_URI.

*/

void vpROSGrabber::open(){
  if(ros::isInitialized() && ros::master::getURI() != _master_uri){
    close();
    throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
                                   "ROS already initialised with a different master_URI (" + ros::master::getURI() +" != " + _master_uri + ")") );
  }
  if(!isInitialized){
    int argc = 2;
    char *argv[2];
    std::string exe = "ros.exe", arg1 = "__master:=";
    strcpy(argv[0], exe.c_str());
    arg1.append(_master_uri);
    strcpy(argv[1], arg1.c_str());
    open(argc, argv);
  }
}


/*!
  Initialization of the grabber.

  Call the generic initialization method.

  \param I : Gray level image. This parameter is not used.

  \sa open()
*/
void vpROSGrabber::open(vpImage<unsigned char> &I)
{
  open();
}


/*!
  Initialization of the grabber.

  Call the generic initialization method.

  \param I : Color image. This parameter is not used.

  \sa open()
*/
void vpROSGrabber::open(vpImage<vpRGBa> &I)
{
  open();
}




/*!
    Grab a gray level image with timestamp

  \param I : Acquired gray level image.

    \param timestamp : timestamp of the acquired image.

  \exception vpFrameGrabberException::initializationError If the

  initialization of the grabber was not done previously.
*/
void vpROSGrabber::acquire(vpImage<unsigned char> &I, struct timespec &timestamp)
{
  if (isInitialized==false)
  {
    close();
    throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
                                   "Initialization not done") );
  }
  while(!mutex_image || !first_img_received);
  mutex_image = false;
  timestamp . tv_sec = _sec;
  timestamp . tv_nsec = _nsec;
  vpImageConvert::convert(data, I, flip);
  first_img_received = false;
  mutex_image = true;
}



/*!
    Grab a gray level image with timestamp without waiting.


    \param I : Acquired gray level image.

    \param timestamp : timestamp of the acquired image.

    \return true if a new image was acquired

    \exception vpFrameGrabberException::initializationError If the

    initialization of the grabber was not done previously.
*/
bool vpROSGrabber::acquireNoWait(vpImage<unsigned char> &I, struct timespec &timestamp)
{
  bool new_image = false;
  if (isInitialized==false)
  {
    close();
    throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
                                   "Initialization not done") );
  }
  while(!mutex_image);
  mutex_image = false;
  timestamp . tv_sec = _sec;
  timestamp . tv_nsec = _nsec;
  vpImageConvert::convert(data, I, flip);
  new_image = first_img_received;
  first_img_received = false;
  mutex_image = true;
  return new_image;
}


/*!
    Grab a color image with timestamp

    \param I : Acquired color image.

    \param timestamp : timestamp of the acquired image.

    \exception vpFrameGrabberException::initializationError If the

    initialization of the grabber was not done previously.
*/
void vpROSGrabber::acquire(vpImage<vpRGBa> &I, struct timespec &timestamp)
{
  if (isInitialized==false)
  {
    close();
    throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
                                   "Initialization not done") );
  }
  while(!mutex_image || !first_img_received);
  mutex_image = false;
  timestamp . tv_sec = _sec;
  timestamp . tv_nsec = _nsec;
  vpImageConvert::convert(data, I, flip);
  first_img_received = false;
  mutex_image = true;
}



/*!
    Grab a color image with timestamp without waiting.


    \param I : Acquired color image.

    \param timestamp : timestamp of the acquired image.

    \return true if a new image was acquired

    \exception vpFrameGrabberException::initializationError If the

    initialization of the grabber was not done previously.
*/
bool vpROSGrabber::acquireNoWait(vpImage<vpRGBa> &I, struct timespec &timestamp)
{
  bool new_image = false;
  if (isInitialized==false)
  {
    close();
    throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
                                   "Initialization not done") );
  }
  while(!mutex_image);
  mutex_image = false;
  timestamp . tv_sec = _sec;
  timestamp . tv_nsec = _nsec;
  vpImageConvert::convert(data, I, flip);
  new_image = first_img_received;
  first_img_received = false;
  mutex_image = true;
  return new_image;
}



/*!
  Grab an image direclty in the OpenCV format.

  \param timestamp : timestamp of the acquired image.

  \return Acquired image.

  \exception vpFrameGrabberException::initializationError If the
  initialization of the grabber was not done previously.
*/
cv::Mat vpROSGrabber::acquire(struct timespec &timestamp)
{
  cv::Mat retour;
  if (isInitialized==false)
  {
    close();
    throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
                                   "Initialization not done") );
  }
  while(!mutex_image || !first_img_received);
  mutex_image = false;
  timestamp . tv_sec = _sec;
  timestamp . tv_nsec = _nsec;
  retour = data.clone();
  first_img_received = false;
  mutex_image = true;
  return retour;
}



/*!
    Grab a gray level image

    \param I : Acquired gray level image.

    \exception vpFrameGrabberException::initializationError If the

    initialization of the grabber was not done previously.
*/
void vpROSGrabber::acquire(vpImage<unsigned char> &I)
{
  struct timespec timestamp;
  acquire(I, timestamp);
}



/*!
    Grab a gray level image without waiting.


    \param I : Acquired gray level image.

    \return true if a new image was acquired

    \exception vpFrameGrabberException::initializationError If the

    initialization of the grabber was not done previously.
*/
bool vpROSGrabber::acquireNoWait(vpImage<unsigned char> &I)
{
  struct timespec timestamp;
  return acquireNoWait(I, timestamp);
}


/*!
    Grab a color image

    \param I : Acquired color image.

    \exception vpFrameGrabberException::initializationError If the

    initialization of the grabber was not done previously.
*/
void vpROSGrabber::acquire(vpImage<vpRGBa> &I)
{
  struct timespec timestamp;
  acquire(I, timestamp);
}



/*!
    Grab a color image without waiting.

    \param I : Acquired color image.

    \return true if a new image was acquired

    \exception vpFrameGrabberException::initializationError If the

    initialization of the grabber was not done previously.
*/
bool vpROSGrabber::acquireNoWait(vpImage<vpRGBa> &I)
{
  struct timespec timestamp;
  return acquireNoWait(I, timestamp);
}



/*!
  Grab an image direclty in the OpenCV format.

  \return Acquired image.

  \exception vpFrameGrabberException::initializationError If the
  initialization of the grabber was not done previously.
*/
cv::Mat vpROSGrabber::acquire()
{
  struct timespec timestamp;
  return acquire(timestamp);
}


void vpROSGrabber::close(){
  if(isInitialized){
    isInitialized = false;
    spinner->stop();
    delete spinner;
    delete n;
  }
}


/*!
  Set the boolean variable flip to the expected value.

  \param flipType : Expected value of the variable flip. True means that the image is flipped during each image acquisition.

  \warning This function is only useful under Windows.

  \note The aim of this function is to fix a problem which appears under Windows. Indeed with several cameras the aquired images are flipped.
*/
void vpROSGrabber::setFlip(bool flipType)
{
  flip = flipType;
}


/*!
    Set the boolean variable rectify to the expected value.

    \param rectify : Expected value of the variable rectify. True means that the image is rectified during each image acquisition.

    \warning Rectification will happen only if the CameraInfo are correctly received
*/
void vpROSGrabber::setRectify(bool rectify)
{
  _rectify = rectify;
}


/*!
  Get the width of the image.

  \param width : width of the image.

*/
void vpROSGrabber::getWidth(unsigned short &width) const
{
  width = getWidth();
}


/*!
  Get the height of the image.

  \param height : height of the image.

*/

void vpROSGrabber::getHeight(unsigned short &height)const
{
  height = getHeight();
}


/*!
  Get the width of the image.

  \return width of the image.

*/
unsigned short vpROSGrabber::getWidth()const
{
  return usWidth;
}

/*!
  Get the height of the image.

  \return height of the image.

*/
unsigned short vpROSGrabber::getHeight()const
{
  return usHeight;
}


/*!

  Set the ROS topic name for CameraInfo

  \param topic_name name of the topic.

*/
void vpROSGrabber::setCameraInfoTopic(std::string topic_name)
{
  _topic_info = topic_name;
}


/*!

    Set the ROS topic name for Images

    \param topic_name name of the topic.

*/
void vpROSGrabber::setImageTopic(std::string topic_name)
{
  _topic_image = topic_name;
}


/*!

    Set the URI for ROS Master

    \param master_uri URI of the master ("http://127.0.0.1:11311")

*/
void vpROSGrabber::setMasterURI(std::string master_uri)
{
  _master_uri = master_uri;
}

/*!

    Set the nodespace

    \param nodespace Namespace of the connected camera (nodespace is appended to the all topic names)

*/
void vpROSGrabber::setNodespace(std::string nodespace)
{
  _nodespace = nodespace;
}


void setImageTransport(std::string image_transport);

/*!

    Set the image_transport type of the image topic

    \param image_transport type of transport of the image topic

*/
void vpROSGrabber::setImageTransport(std::string image_transport)
{
  _image_transport = image_transport;
}

/*!
  Get the vpCameraParameters from the camera

  \param cam parameter of the camera

*/

void vpROSGrabber::getCameraInfo(vpCameraParameters &cam){
  while(!mutex_param || !first_param_received);
  mutex_param = false;
  cam = _cam;
  mutex_param = true;
}


void vpROSGrabber::imageCallback(const sensor_msgs::CompressedImage::ConstPtr& msg){

  cv::Mat data_t = cv::imdecode(msg->data,1);
  cv::Size data_size = data_t.size();

  while(!mutex_image);
  mutex_image = false;
  if(_rectify && p.initialized()){
    p.rectifyImage(data_t,data);
  }else{
    data_t.copyTo(data);
  }
  usWidth = data_size.width;
  usHeight = data_size.height;
  _sec = msg->header.stamp.sec;
  _nsec = msg->header.stamp.nsec;
  first_img_received = true;
  mutex_image = true;
}


void vpROSGrabber::imageCallbackRaw(const sensor_msgs::Image::ConstPtr& msg){
  while(!mutex_image);
  mutex_image = false;
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  if(_rectify && p.initialized()){
    p.rectifyImage(cv_ptr->image,data);
  }else{
    cv_ptr->image.copyTo(data);
  }
  cv::Size data_size = data.size();
  usWidth = data_size.width;
  usHeight = data_size.height;
  _sec = msg->header.stamp.sec;
  _nsec = msg->header.stamp.nsec;
  first_img_received = true;
  mutex_image = true;
}

void vpROSGrabber::paramCallback(const sensor_msgs::CameraInfo::ConstPtr& msg){
  while(!mutex_param);
  mutex_param = false;
  _cam = visp_bridge::toVispCameraParameters(*msg);
  p.fromCameraInfo(msg);
  first_param_received = true;
  mutex_param = true;
}

#endif

