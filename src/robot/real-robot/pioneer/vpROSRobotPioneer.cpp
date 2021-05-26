/****************************************************************************
 *
 * $Id: vpROSRobotPioneer.cpp 4574 2014-01-09 08:48:51Z fpasteau $
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2014 by INRIA. All rights reserved.
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
 * Interface for Pioneer robots based on ROS middleware.
 *
 * Authors:
 * Francois Pasteau
 *
 *****************************************************************************/

#include <visp/vpConfig.h>
#include <visp/vpMath.h>
#include <visp/vpRobotException.h>
#include <std_srvs/Empty.h>
#include <visp_ros/vpROSRobotPioneer.h>


/*!
  Default constructor that initializes Aria.
  */
vpROSRobotPioneer::vpROSRobotPioneer() : vpPioneer()
{

}

//! destructor
vpROSRobotPioneer::~vpROSRobotPioneer()
{
  stopMotion();
  disableMotors();
}

//! basic initialization
void vpROSRobotPioneer::init()
{
  vpROSRobot::init();
  enableMotors();
}

void vpROSRobotPioneer::init(int argc, char **argv)
{
  vpROSRobot::init(argc, argv);
  enableMotors();
}

/*!
  Set the velocity (frame has to be specified) that will be applied to the robot.

  \param frame : Control frame. For the moment, only vpRobot::REFERENCE_FRAME to control translational and
  rotational velocities is implemented.

  \param vel : A two dimension vector that corresponds to the velocities to apply to the robot.
  - If the frame is vpRobot::REFERENCE_FRAME, first value is the translation velocity in m/s.
    Second value is the rotational velocity in rad/s.

  Note that to secure the usage of the robot, velocities are saturated to the maximum allowed
  which can be obtained by getMaxTranslationVelocity() and getMaxRotationVelocity(). To change
  the default values, use setMaxTranslationVelocity() and setMaxRotationVelocity().

  \exception vpRobotException::dimensionError : Velocity vector is not a 2 dimension vector.
  \exception vpRobotException::wrongStateError : If the specified control frame is not supported.
  */
void vpROSRobotPioneer::setVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &vel)
{
  if (! isInitialized) {
    throw (vpRobotException(vpRobotException::notInitialized,
                            "ROS robot pioneer is not initialized") );
  }

  if (vel.size() != 2)
  {
    throw(vpRobotException(vpRobotException::dimensionError, "Velocity vector is not a 2 dimension vector"));
  }

  vpColVector vel_max(2);
  vpColVector vel_sat;
  vpColVector vel_robot(6);

  if (frame == vpRobot::REFERENCE_FRAME)
  {
    vel_max[0] = getMaxTranslationVelocity();
    vel_max[1] = getMaxRotationVelocity();

    vel_sat = vpRobot::saturateVelocities(vel, vel_max, true);
    vel_robot[0] = vel_sat[0];
    vel_robot[1] = 0;
    vel_robot[2] = 0;
    vel_robot[3] = 0;
    vel_robot[4] = 0;
    vel_robot[5] = vel_sat[1];
    vpROSRobot::setVelocity(frame, vel_robot);
  }
  else
  {
    throw vpRobotException (vpRobotException::wrongStateError,
                            "Cannot send the robot velocity in the specified control frame");
  }
}

void vpROSRobotPioneer::enableMotors()
{
  ros::ServiceClient client = n->serviceClient<std_srvs::Empty>("/RosAria/enable_motors");
  std_srvs::Empty srv;
  if (!client.call(srv))
  {
    ROS_ERROR("Unable to enable motors");
  }
}

void vpROSRobotPioneer::disableMotors()
{
  ros::ServiceClient client = n->serviceClient<std_srvs::Empty>("/RosAria/disable_motors");
  std_srvs::Empty srv;
  if (!client.call(srv))
  {
    ROS_ERROR("Unable to disable motors");
  }
}

