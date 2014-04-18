/****************************************************************************
 *
 * $Id: movePioneer.cpp 4574 2014-01-09 08:48:51Z fspindle $
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
 * Example that shows how to control a Pioneer mobile robot in ViSP.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#include <iostream>

#include <visp/vpTime.h>
#include <visp_ros/vpROSRobotPioneer.h>


/*!
  \example movePioneer.cpp example showing how to connect and send
  direct basic motion commands to a Pioneer mobile robot.

  WARNING: this program does no sensing or avoiding of obstacles, the robot WILL
  collide with any objects in the way!   Make sure the robot has about 2-3
  meters of free space around it before starting the program.

  This program will work either with the MobileSim simulator or on a real
  robot's onboard computer.  (Or use -remoteHost to connect to a wireless
  ethernet-serial bridge.)
*/
int main(int argc, char **argv)
{
  try {
    std::cout << "\nWARNING: this program does no sensing or avoiding of obstacles, \n"
                 "the robot WILL collide with any objects in the way! Make sure the \n"
                 "robot has approximately 3 meters of free space on all sides.\n"
              << std::endl;

    vpROSRobotPioneer robot;

    robot.setCmdVelTopic("/RosAria/cmd_vel");
    robot.init();
    vpColVector v(2), v_mes(2);

    //for (int i=0; i < 100000; i++)
      while(1)
    {
      double t = vpTime::measureTimeMs();

      v = 0;
      v[0] = 0.01;//i/1000.; // Translational velocity in m/s
      //v[1] = vpMath::rad(i/5.); // Rotational velocity in rad/sec
      robot.setVelocity(vpRobot::REFERENCE_FRAME, v);


      vpTime::wait(t, 40);
    }

   return 0;
  }
  catch(vpException e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return 1;
  }
}
