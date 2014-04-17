/****************************************************************************
 *
 * $Id: vpROSRobot.h 3778 2012-10-31 14:12:07Z fpasteau $
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
 * vpRobot implementation for ROS middleware.
 *
 * Authors:
 * Francois Pasteau
 *
 *****************************************************************************/


#ifndef vpROSRobot_H
#define vpROSRobot_H

/*!
\file vpROSRobot.h
\brief vpRobot implementation for ROS middleware.
*/

#include <visp/vpRobot.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
/*!
\class vpROSRobot
\brief vpRobot implementation for Quickie Salsa M wheelchair with ROS.
*/

class VISP_EXPORT vpROSRobot : public vpRobot
{

  private:
    ros::NodeHandle *n;
    ros::Publisher cmdvel;
    ros::Subscriber odom;
    ros::AsyncSpinner *spinner;

  protected:
    bool isInitialized;

    vpQuaternionVector q;
    vpTranslationVector p;
    vpColVector pose_prev;
    vpColVector displacement;
    uint32_t _sec, _nsec;
    volatile bool odom_mutex;
    std::string _master_uri;
    std::string _topic_cmd;
    std::string _topic_odom;
    std::string _nodespace;

  private: // Set as private since not implemented
    void get_eJe(vpMatrix & eJe){}

    vpROSRobot(const vpROSRobot &robot);
    /*!
    Get the robot Jacobian expressed in the robot reference (or world) frame.
    \warning Not implemented.
  */
    void get_fJe(vpMatrix & /*fJe*/) {}

    /*!
    Get a displacement expressed in the joint space between two successive position control.
    \warning Not implemented.
  */
    void getArticularDisplacement(vpColVector  & /*qdot*/) {};


    void getVelocity (const vpRobot::vpControlFrameType frame, vpColVector & velocity);
    vpColVector getVelocity (const vpRobot::vpControlFrameType frame);

    /*!
    Set a displacement (frame has to be specified) in position control.
    \warning Not implemented.
  */
    void setPosition(const vpRobot::vpControlFrameType /*frame*/, const vpColVector &/*q*/) {};
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void getCameraDisplacement(vpColVector & /*v*/);

  public:
    //! constructor
    vpROSRobot() ;
    //! destructor
    virtual ~vpROSRobot() ;

    void getDisplacement(const vpRobot::vpControlFrameType /*frame*/, vpColVector &/*q*/);
    void getDisplacement(const vpRobot::vpControlFrameType /*frame*/, vpColVector &/*q*/, struct timespec &timestamp);
    void getPosition(const vpRobot::vpControlFrameType /*frame*/, vpColVector &/*q*/);

    //! basic initialization
    void init() ;
    void init(int argc, char **argv) ;

    void setVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &vel);
    void stopMotion();
} ;

#endif

/*
* Local variables:
* c-basic-offset: 2
* End:
*/


