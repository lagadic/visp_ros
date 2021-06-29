/*
 * \example tutorial-franka-coppeliasim-ibvs-apriltag.cpp
 */

#include <iostream>
#include <mutex>

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpPlot.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include <visp_ros/vpROSGrabber.h>
#include <visp_ros/vpROSRobotFrankaCoppeliasim.h>

static bool s_simStepDone = true;
static std::mutex s_mutex_ros;
static float s_simTime = 0;

void
simStepDone_callback( const std_msgs::Bool &msg )
{
  std::lock_guard< std::mutex > lock( s_mutex_ros );
  s_simStepDone = msg.data;
}

void
simTime_callback( const std_msgs::Float32 &msg )
{
  std::lock_guard< std::mutex > lock( s_mutex_ros );
  s_simTime = msg.data;
}

int
main( int argc, char **argv )
{

  try
  {
    //------------------------------------------------------------------------//
    //------------------------------------------------------------------------//
    // ROS node
    ros::init( argc, argv, "visp_ros" );
    ros::NodeHandlePtr n = boost::make_shared< ros::NodeHandle >();
    ros::Rate loop_rate( 1000 );
    ros::spinOnce();

    vpROSRobotFrankaCoppeliasim robot;
    robot.setVerbose( true );
    robot.setTopicJointState( "/vrep/franka/joint_state" );
    robot.setTopic_eMc( "/vrep/franka/eMc" );
    robot.connect();

    ros::Publisher enableSyncMode_pub  = n->advertise< std_msgs::Bool >( "/enableSyncMode", 1 );
    ros::Publisher startSimTrigger_pub = n->advertise< std_msgs::Bool >( "/startSimulation", 1 );
    ros::Publisher stopSimTrigger_pub  = n->advertise< std_msgs::Bool >( "/stopSimulation", 1 );
    std_msgs::Bool trigger;
    std_msgs::Bool syncMode;
    std_msgs::Bool startStopSim;

    std::string simulationStepDone_topic_name = "/simulationStepDone";
    std::cout << "Subscribe to " << simulationStepDone_topic_name << std::endl;
    ros::Subscriber sub_simStepDone       = n->subscribe( simulationStepDone_topic_name, 1, simStepDone_callback );
    std::string simulationTime_topic_name = "/simulationTime";
    std::cout << "Subscribe to " << simulationTime_topic_name << std::endl;
    ros::Subscriber sub_simulationTime = n->subscribe( simulationTime_topic_name, 1, simTime_callback );

    startStopSim.data = true;
    startSimTrigger_pub.publish( startStopSim );
    vpTime::wait( 1000 );
    stopSimTrigger_pub.publish( startStopSim );
    vpTime::wait( 1000 );
    syncMode.data = true;
    enableSyncMode_pub.publish( trigger );
    startSimTrigger_pub.publish( startStopSim );
    vpTime::wait( 1000 );

    robot.setRobotState( vpRobot::STATE_VELOCITY_CONTROL );

    double t_start     = vpTime::measureTimeSecond();
    double t_init      = vpTime::measureTimeMs();
    double t_init_prev = t_init;

    float t_simTime_start = 0, t_simTime = 0, t_simTime_prev = 0;
    ;
    s_mutex_ros.lock();
    t_simTime_start = t_simTime_prev = s_simTime;
    s_mutex_ros.unlock();
    t_init_prev = vpTime::measureTimeMs();

    vpColVector q_init, q_final;
    robot.getPosition( vpRobot::JOINT_STATE, q_init );
    std::cout << "q initial: " << 180. / M_PI * q_init.t() << " deg" << std::endl;

    while ( vpTime::measureTimeSecond() - t_start < 10 )
    {
      ros::spinOnce();
      t_init = vpTime::measureTimeMs();
      s_mutex_ros.lock();
      t_simTime = s_simTime;
      s_mutex_ros.unlock();

      vpColVector qdot( 7, 0 );
      qdot[0] = vpMath::rad( 10 );
      robot.setVelocity( vpRobot::JOINT_STATE, qdot );
      vpTime::sleepMs( 50 );

      std::stringstream ss;
      ss << "Loop time: " << t_init - t_init_prev << " - " << t_simTime - t_simTime_prev << " ms";
      //      std::cout << ss.str() << std::endl;
      t_init_prev    = t_init;
      t_simTime_prev = t_simTime;
    }
    std::cout << "Elapsed time: " << vpTime::measureTimeSecond() - t_start << " seconds - "
              << t_simTime - t_simTime_start << std::endl;

    robot.getPosition( vpRobot::JOINT_STATE, q_final );
    std::cout << "q final: " << 180 / M_PI * q_final.t() << " deg" << std::endl;
    std::cout << "Joint displacement: " << 180. / M_PI * ( q_final - q_init ).t() << " deg" << std::endl;

    stopSimTrigger_pub.publish( startStopSim );
  }
  catch ( const vpException &e )
  {
    std::cout << "ViSP exception: " << e.what() << std::endl;
    std::cout << "Stop the robot " << std::endl;
    return EXIT_FAILURE;
  }

  return 0;
}
