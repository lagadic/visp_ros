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
#include <visp_ros/vpROSRobotPioneer.h>


/*!
  Default constructor that initializes Aria.
  */
vpROSRobotPioneer::vpROSRobotPioneer() : vpPioneer()
{

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
  init();

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
    vel_robot[0] = 0;
    vel_robot[0] = 0;
    vel_robot[0] = 0;
    vel_robot[0] = 0;
    vel_robot[0] = vel_sat[1];
    vpROSRobot::setVelocity(frame, vel_robot);
  }
  else
  {
    throw vpRobotException (vpRobotException::wrongStateError,
                            "Cannot send the robot velocity in the specified control frame");
  }
}

