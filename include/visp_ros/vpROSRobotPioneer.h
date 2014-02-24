/****************************************************************************
 *
 * $Id: vpROSRobotPioneer.h 4574 2014-01-09 08:48:51Z fpasteau $
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
 * Interface for Pioneer mobile robots based on ROS
 *
 * Authors:
 * Francois Pasteau
 *
 *****************************************************************************/
#ifndef VPROSROBOTPIONEER_H
#define VPROSROBOTPIONEER_H

#include <visp/vpConfig.h>
#include <visp/vpRobot.h>
#include <visp_ros/vpROSRobot.h>
#include <visp/vpPioneer.h>

/*!

  \class vpROSRobotPioneer

  \ingroup Pioneer RobotDriver

  \brief Interface for Pioneer mobile robots based on ROS.

  This class provides a position and speed control interface for Pioneer mobile robots.
  It inherits from the vpROSRobot class. For more information about the model of the robot,
  see vpPioneer documentation.

*/
class VISP_EXPORT vpROSRobotPioneer: public vpROSRobot, public vpPioneer
{
private: /* Not allowed functions. */

  /*!
    Copy constructor not allowed.
   */
  vpROSRobotPioneer(const vpROSRobotPioneer &robot);

public:
  vpROSRobotPioneer();

  /*!
    Get the robot Jacobian expressed at point E, the point located at the
    middle between the two wheels.

    \param eJe : Robot jacobian such as \f$(v_x, w_z) = {^e}{\bf J}e \; {\bf v}\f$ with
    \f$(v_x, w_z)\f$ respectively the translational and rotational control velocities
    of the mobile robot, \f$\bf v\f$ the six dimention velocity skew, and where

    \sa get_eJe()

  */
  void get_eJe(vpMatrix & eJe)
  {
    eJe = vpUnicycle::get_eJe();
  }

private: // Set as private since not implemented
  /*!
    Get the robot Jacobian expressed in the robot reference (or world) frame.
    \warning Not implemented.
  */
  void get_fJe(vpMatrix & /*fJe*/) {} ;

public:
  void getVelocity (const vpRobot::vpControlFrameType frame, vpColVector & velocity)  
  {
	throw (vpRobotException(vpRobotException::notImplementedError, "getVelocity not implemented with ROS") );
  }
  vpColVector getVelocity (const vpRobot::vpControlFrameType frame)
  {
	throw (vpRobotException(vpRobotException::notImplementedError, "getVelocity not implemented with ROS") );
  }

private: // Set as private since not implemented
  /*!
    Set a displacement (frame has to be specified) in position control.
    \warning Not implemented.
  */
  void setPosition(const vpRobot::vpControlFrameType /*frame*/, const vpColVector &/*q*/) {};

public:
  void setVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &vel);

  /*!
    Enable or disable sonar device usage.
    */
  void useSonar(bool usage)
  {
	throw (vpRobotException(vpRobotException::notImplementedError, "useSonar not implemented with ROS") );
  }

protected:
  bool isInitialized;
};

#endif // vpROSRobotPioneer_H

