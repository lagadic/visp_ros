/*!
  \example tutorial-ros-pioneer.cpp example showing how to connect and send
  direct basic motion commands to a Pioneer mobile robot.

  WARNING: this program does no sensing or avoiding of obstacles, the robot WILL
  collide with any objects in the way!   Make sure the robot has about 2-3
  meters of free space around it before starting the program.

*/

#include <iostream>

#include <visp/vpTime.h>
#include <visp_ros/vpROSRobotPioneer.h>

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

    for (int i=0; i < 100; i++)
    {
      double t = vpTime::measureTimeMs();

      v = 0;
      v[0] = 0.01; // Translational velocity in m/s
      //v[1] = vpMath::rad(2); // Rotational velocity in rad/sec
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
