/*
 * \example  tutorial-franka-coppeliasim-joint-impedance-control.cpp
 *
 *  Created on: May 1, 2021
 *      Author: oliva
 */

//! \example tutorial-franka-coppeliasim-joint-impedance-control.cpp

#include <iostream>

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/visual_features/vpFeatureThetaU.h>
#include <visp3/visual_features/vpFeatureTranslation.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>
#include <visp3/gui/vpPlot.h>

#include <visp_ros/vpROSGrabber.h>
#include <visp_ros/vpROSRobotFrankaCoppeliasim.h>

int main(int argc, char **argv)
{
  bool opt_verbose = false;
  bool opt_plot = false;
  bool opt_coppeliasim_sync_mode = false;


  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--verbose" || std::string(argv[i]) == "-v") {
      opt_verbose = true;
    }
    else if (std::string(argv[i]) == "--enable-coppeliasim-sync-mode") {
      opt_coppeliasim_sync_mode = true;
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << argv[0]
                << "[--enable-coppeliasim-sync-mode] "
                << "[--verbose] [-v] "
                << "[--help] [-h]" << std::endl;;
      return EXIT_SUCCESS;
    }
  }
  try{
    //    ROS node    //
    ros::init(argc, argv, "visp_ros");
    ros::NodeHandlePtr n = boost::make_shared<ros::NodeHandle>();
    ros::Rate loop_rate(1000);
    ros::spinOnce();

    vpROSRobotFrankaCoppeliasim robot;
    robot.setVerbose(opt_verbose);
    robot.connect();

    std::cout << "Coppeliasim sync mode enabled: " << (opt_coppeliasim_sync_mode ? "yes" : "no") << std::endl;
    robot.coppeliasimStopSimulation(); // Allows to reset simulation, moving the robot to initial position
    robot.setCoppeliasimSyncMode(false);
    robot.coppeliasimStartSimulation();

    vpPlot *plotter = nullptr;

    plotter = new vpPlot(4, 800, 800, 10, 10, "Real time curves plotter");
    plotter->setTitle(0, "Joint Positions [rad]");
    plotter->initGraph(0, 7);
    plotter->setLegend(0, 0, "q1");
    plotter->setLegend(0, 1, "q2");
    plotter->setLegend(0, 2, "q3");
    plotter->setLegend(0, 3, "q4");
    plotter->setLegend(0, 4, "q5");
    plotter->setLegend(0, 5, "q6");
    plotter->setLegend(0, 6, "q7");

    plotter->setTitle(1, "Joint position error [rad]");
    plotter->initGraph(1, 7);
    plotter->setLegend(1, 0, "e_q1");
    plotter->setLegend(1, 1, "e_q2");
    plotter->setLegend(1, 2, "e_q3");
    plotter->setLegend(1, 3, "e_q4");
    plotter->setLegend(1, 4, "e_q5");
    plotter->setLegend(1, 5, "e_q6");
    plotter->setLegend(1, 6, "e_q7");

    plotter->setTitle(2, "Joint torque command [Nm]");
    plotter->initGraph(2, 7);
    plotter->setLegend(2, 0, "Tau1");
    plotter->setLegend(2, 1, "Tau2");
    plotter->setLegend(2, 2, "Tau3");
    plotter->setLegend(2, 3, "Tau4");
    plotter->setLegend(2, 4, "Tau5");
    plotter->setLegend(2, 5, "Tau6");
    plotter->setLegend(2, 6, "Tau7");

    plotter->setTitle(3, "Joint error norm [rad]");
    plotter->initGraph(3, 1);
    plotter->setLegend(3, 0, "||qd - d||");

    // Create joint array
    vpColVector q(7,0), qd(7,0),dq(7,0), dqd(7,0), ddqd(7,0), tau_d(7,0), C(7,0), pos(6,0), q0(7,0);
    vpMatrix J(6,7), B(7,7);

    std::cout << "Reading current joint position" << std::endl;
    robot.getPosition(vpRobot::JOINT_STATE, q0);
    std::cout << "Initial joint position: " << q0.t() << std::endl;

    robot.setRobotState(vpRobot::STATE_FORCE_TORQUE_CONTROL);
    qd = q0;

    bool final_quit = false;
    bool first_time = true;

    double k = 800;
    double d = 2*sqrt(k);

    double sim_time = robot.getCoppeliasimSimulationTime();
    double sim_time_start = sim_time;
    double sim_time_prev = sim_time;

    while (!final_quit) {
      ros::spinOnce();

      sim_time = robot.getCoppeliasimSimulationTime();

      robot.getPosition(vpRobot::JOINT_STATE, q);
      robot.getVelocity(vpRobot::JOINT_STATE, dq);
      robot.getMass(B);
      robot.getCoriolis(C);

      if (first_time){
        sim_time_start = sim_time;
        first_time = false;
      }
      qd[0] = q0[0] + std::sin(2*M_PI*0.25*(sim_time - sim_time_start));
      dqd[0] = 2*M_PI*0.25*std::cos(2*M_PI*0.25*(sim_time - sim_time_start));
      ddqd[0] = -std::pow(2*0.25*M_PI,2)*std::sin(2*M_PI*0.25*(sim_time - sim_time_start));
      qd[3] = q0[3] + (M_PI/16)*std::sin(2*M_PI*(sim_time - sim_time_start));
      dqd[3] = M_PI*(M_PI/8)*std::cos(2*M_PI*(sim_time - sim_time_start));
      ddqd[3] = - M_PI*M_PI*(M_PI/4)*std::sin(2*M_PI*(sim_time - sim_time_start));

      // Compute the control law
      tau_d = B*(k*(qd - q) + d*(dqd - dq) + ddqd) + C;

      // Send command to the torque robot
      robot.setForceTorque(vpRobot::JOINT_STATE, tau_d);

      vpColVector norm(1,std::sqrt((qd-q).sumSquare()));
      plotter->plot(0, sim_time, q);
      plotter->plot(1, sim_time, qd - q);
      plotter->plot(2, sim_time, tau_d);
      plotter->plot(3, sim_time, norm);

      std::stringstream ss;
      ss << "Loop time [s]: " << std::round((sim_time - sim_time_prev)*1000.)/1000.;
      ss << " Simulation time [s]: " << sim_time;
      sim_time_prev = sim_time;
      vpDisplay::displayText(plotter->I, 40, 20, ss.str(), vpColor::red);

      robot.wait(sim_time, 0.010); // Simulate a loop at 100 Hz

      vpMouseButton::vpMouseButtonType button;
      if (vpDisplay::getClick(plotter->I, button, false)) {
        if (button == vpMouseButton::button3) {
          final_quit = true;
        }
      }
    }

    if (plotter != nullptr) {
      delete plotter;
      plotter = nullptr;
    }

    robot.coppeliasimStopSimulation();
  }
  catch(const vpException &e) {
    std::cout << "ViSP exception: " << e.what() << std::endl;
    std::cout << "Stop the robot " << std::endl;
    return EXIT_FAILURE;
  }

  return 0;
}



