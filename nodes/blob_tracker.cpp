#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <boost/thread/mutex.hpp>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>

#include <cv_bridge/cv_bridge.h>

#include <visp_bridge/3dpose.h>

#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/blob/vpDot2.h>
#include <visp3/vision/vpPose.h>


#include <visp_ros/BlobTracker.h>

namespace {
class BlobTracker {
  enum STATE_T {
    START, //!< Start process, waiting a user click before entering in INIT state
    INIT,  //!< Initialize the tracker from user click
    TRACK, //!< Run the tracker
    END,   //!< Stop the tracker
    QUIT   //!< Exit
  };

  ros::NodeHandle m_nh;
  image_transport::ImageTransport m_it;
  image_transport::Subscriber m_image_sub;
  ros::Publisher m_tracker_publisher;
  ros::Publisher m_status_publisher;
  unsigned int m_queue_size;
  STATE_T m_state;
  boost::mutex m_lock;
#ifdef VISP_HAVE_X11
  vpDisplayX *m_display;
#elif defined(VISP_HAVE_GDI)
  vpDisplayGDI *m_display;
#elif defined(VISP_HAVE_OPENCV)
  vpDisplayOpenCV *m_display;
#endif
  vpImage<unsigned char> m_I_grayscale;
  std::vector<vpDot2> m_blob;
  unsigned int m_nblobs;
  double m_square_size;
  vpCameraParameters m_cam;
  std::vector<vpPoint> m_points_3d;
  std::vector<vpImagePoint> m_points_2d;
  vpHomogeneousMatrix m_cMo;
  unsigned int m_thickness;
  double m_cam_px;
  double m_cam_py;
  double m_cam_u0;
  double m_cam_v0;
  double m_cam_kud;
  double m_cam_kdu;

public:
  BlobTracker() : m_it(m_nh), m_image_sub(), m_tracker_publisher(), m_status_publisher(), m_queue_size(1), m_state(START),
    m_lock(), m_display(NULL), m_I_grayscale(), m_blob(), m_nblobs(4), m_square_size(0.12), m_cam(), m_points_3d(), m_points_2d(), m_cMo(), m_thickness(2),
    m_cam_px(-1), m_cam_py(-1), m_cam_u0(-1), m_cam_v0(-1), m_cam_kud(-1), m_cam_kdu(-1)
  {
    m_image_sub = m_it.subscribe("/camera/image_raw", m_queue_size, &BlobTracker::callback, this);
    m_tracker_publisher = m_nh.advertise<visp_ros::BlobTracker>("blob_tracker/data", m_queue_size);
    m_status_publisher = m_nh.advertise<std_msgs::Int8>("blob_tracker/status", m_queue_size);

    m_blob.resize(m_nblobs);
    m_points_3d.resize(m_nblobs);
    m_points_2d.resize(m_nblobs);

    // Load object size
    m_nh.getParam("square_size", m_square_size);

    // Load camera parameters
    m_nh.getParam("cam_px", m_cam_px);
    m_nh.getParam("cam_py", m_cam_py);
    m_nh.getParam("cam_u0", m_cam_u0);
    m_nh.getParam("cam_v0", m_cam_v0);
    m_nh.getParam("cam_kud", m_cam_kud);
    m_nh.getParam("cam_kdu", m_cam_kdu);

    if (m_cam_px < 0 || m_cam_py < 0 || m_cam_u0 < 0 || m_cam_v0 < 0) {
      ROS_ERROR("Camera parameters are not set");
    }

    if (m_cam_kud == -1 || m_cam_kdu == -1) {
      m_cam.initPersProjWithoutDistortion(m_cam_px, m_cam_py, m_cam_u0, m_cam_v0);
    }
    else {
      m_cam.initPersProjWithDistortion(m_cam_px, m_cam_py, m_cam_u0, m_cam_v0, m_cam_kud, m_cam_kdu);
    }

    // Construct 3D model
    //
    //   pt0       pt1
    //  x         x
    //
    //       --------------> x
    //       |
    //   pt3 |      pt2
    //  x    |     x
    //       |
    //       \/ y

    m_points_3d[0] = vpPoint(-m_square_size / 2., -m_square_size / 2., 0); // top/left
    m_points_3d[1] = vpPoint( m_square_size / 2., -m_square_size / 2., 0); // top/right
    m_points_3d[2] = vpPoint( m_square_size / 2.,  m_square_size / 2., 0); // bottom/right
    m_points_3d[3] = vpPoint(-m_square_size / 2.,  m_square_size / 2., 0); // bottom/left
  }

  void spin() {
    ros::Rate loop_rate(60);
    bool quit = false;
    while (m_nh.ok() && ! quit) {
      {
        boost::mutex::scoped_lock(m_lock);
        switch (m_state) {
        case START:
          break;

        case INIT:
          break;

        case TRACK:
          break;

        case END:
          break;

        case QUIT:
          quit = true;
          break;

        default:
          break;
        }
      }

      ros::spinOnce();
      loop_rate.sleep();
    }
    if (m_display) {
      delete m_display;
      m_display = NULL;
    }
    std::cout << "Image processing stopped..." << std::endl;
  }

  void init_display () {
#ifdef VISP_HAVE_X11
    m_display = new vpDisplayX;
#elif VISP_HAVE_GDI
    m_display = new vpDisplayGDI;
#elif VISP_HAVE_OPENCV
    m_display = new vpDisplayOpenCV;
#endif
    if (m_display) {
      std::cout << "Image size: " <<  m_I_grayscale.getWidth() << " x " << m_I_grayscale.getHeight() << std::endl;
      std::cout << "Camera parameters:\n" << m_cam << std::endl;
      m_display->init(m_I_grayscale, 80, m_I_grayscale.getHeight()+20, "Image processing");
    }
  }

  void display(const sensor_msgs::ImageConstPtr& msg) {
    try {
      cv::Mat cv_frame = cv_bridge::toCvShare(msg, "bgr8")->image;
      vpImageConvert::convert(cv_frame, m_I_grayscale);

      if (m_display == NULL) {
        init_display();
      }

      vpDisplay::display(m_I_grayscale);
      vpDisplay::displayText(m_I_grayscale, 20, 20, "Click to start initialisation", vpColor::red);
      vpDisplay::flush(m_I_grayscale);

      if (vpDisplay::getClick(m_I_grayscale, false)) {
        m_state = INIT;
      }
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }

  void callback(const sensor_msgs::ImageConstPtr& msg) {
    boost::mutex::scoped_lock(m_lock);

    switch (m_state) {
    case START:
    {
      std_msgs::Int8 status_msg;
      status_msg.data = 0;
      m_status_publisher.publish(status_msg);

      display(msg);
      break;
    }

    case INIT:
    {
      std_msgs::Int8 status_msg;
      status_msg.data = 0;
      m_status_publisher.publish(status_msg);

      cv::Mat cv_frame = cv_bridge::toCvShare(msg, "bgr8")->image;
      vpImageConvert::convert(cv_frame, m_I_grayscale);

      vpDisplay::display(m_I_grayscale);
      vpDisplay::displayText(m_I_grayscale, 20, 20, "Click in the 4 blobs clockwise to initialise the tracker", vpColor::red);
      vpDisplay::flush(m_I_grayscale);
      try {
        for (unsigned int i = 0; i < m_blob.size(); i++) {
          m_blob[i].setGraphics(true);
          m_blob[i].setGraphicsThickness(m_thickness);
          m_blob[i].initTracking(m_I_grayscale);
          m_blob[i].track(m_I_grayscale);
          m_points_2d[i] = m_blob[i].getCog();

          std::stringstream ss;
          ss << i;
          vpDisplay::displayText(m_I_grayscale, m_blob[i].getCog() + vpImagePoint(-20, 20), ss.str(), vpColor::red);
          vpDisplay::flush(m_I_grayscale);
        }
        compute_pose(m_points_3d, m_points_2d, m_cam, true, m_cMo);

        m_state = TRACK;
        vpDisplay::flush(m_I_grayscale);
      } catch (...) {
        std::cout << "Tracking failed during initialisation" << std::endl;
        m_state = START;
      }

      break;
    }

    case TRACK:
    {
      cv::Mat cv_frame = cv_bridge::toCvShare(msg, "bgr8")->image;
      vpImageConvert::convert(cv_frame, m_I_grayscale);

      vpDisplay::display(m_I_grayscale);
      vpDisplay::displayText(m_I_grayscale, 20, 20, "Tracking in progress", vpColor::red);
      vpDisplay::displayText(m_I_grayscale, 40, 20, "Right click to quit", vpColor::red);

      visp_ros::BlobTracker blob_tracker_msg;
      try {
        for (unsigned int i = 0; i < m_blob.size(); i++) {
          m_blob[i].track(m_I_grayscale);
          m_points_2d[i] = m_blob[i].getCog();
          std::stringstream ss;
          ss << i;
          vpDisplay::displayText(m_I_grayscale, m_blob[i].getCog() + vpImagePoint(-20, 20), ss.str(), vpColor::red);
        }
        compute_pose(m_points_3d, m_points_2d, m_cam, false, m_cMo);
        vpDisplay::displayFrame(m_I_grayscale, m_cMo, m_cam, m_square_size / 2., vpColor::none, m_thickness);

        // Publish tracker results
        ros::Time now = ros::Time::now();
        geometry_msgs::PoseStamped msg_pose;
        msg_pose.header.stamp = now;
        msg_pose.header.frame_id = msg->header.frame_id;
        msg_pose.pose = visp_bridge::toGeometryMsgsPose(m_cMo); //convert
        blob_tracker_msg.pose_stamped = msg_pose;
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg);
        cv_ptr->toImageMsg(blob_tracker_msg.image);

        for (unsigned int i = 0; i < m_blob.size(); i++) {
          visp_ros::ImagePoint ip;
          ip.i = m_blob[i].getCog().get_i();
          ip.j = m_blob[i].getCog().get_j();
          blob_tracker_msg.blob_cogs.push_back(ip);

          visp_ros::ProjectedPoint pp;
          vpPixelMeterConversion::convertPoint(m_cam, m_blob[i].getCog(), pp.x, pp.y);
          blob_tracker_msg.blob_proj.push_back(pp);
        }
        vpDisplay::flush(m_I_grayscale);

        std_msgs::Int8 status_msg;
        status_msg.data = 1;

        vpMouseButton::vpMouseButtonType button;
        if (vpDisplay::getClick(m_I_grayscale, button, false)) {
          if (button == vpMouseButton::button3) {
            m_state = END;
            status_msg.data = 2; // To stop visual servo controller
          }
        }

        m_tracker_publisher.publish(blob_tracker_msg);
        m_status_publisher.publish(status_msg);
      } catch (...) {
        std::cout << "Tracking failed" << std::endl;
        m_state = START;

        std_msgs::Int8 status_msg;
        status_msg.data = 0;
        m_status_publisher.publish(status_msg);
      }

      break;
    }

    case END:
    {
      std_msgs::Int8 status_msg;
      status_msg.data = 0;
      m_status_publisher.publish(status_msg);

      if (m_display) {
        delete m_display;
        m_display = NULL;
      }
      m_state = QUIT;

      break;
    }

    default:
      break;
    }
  }

  double compute_pose(std::vector<vpPoint> &points_3d, const std::vector<vpImagePoint> &points_2d, const vpCameraParameters &cam,
                     bool init, vpHomogeneousMatrix &cMo)
  {
    vpPose pose;
    double x = 0, y = 0;
    for (unsigned int i = 0; i < points_3d.size(); i++) {
      vpPixelMeterConversion::convertPoint(cam, points_2d[i], x, y);
      points_3d[i].set_x(x);
      points_3d[i].set_y(y);
      pose.addPoint(points_3d[i]);
    }

    if (init == true) {
      vpHomogeneousMatrix cMo_dem;
      vpHomogeneousMatrix cMo_lag;
      double residual_dem = std::numeric_limits<double>::max();
      double residual_lag = std::numeric_limits<double>::max();

      try {
        pose.computePose(vpPose::DEMENTHON, cMo_dem);
        residual_dem = pose.computeResidual(cMo_dem);
      } catch (...) {
        residual_dem = std::numeric_limits<double>::max();
      }
      try {
        pose.computePose(vpPose::LAGRANGE, cMo_lag);
        residual_lag = pose.computeResidual(cMo_lag);
      } catch (...) {
        residual_lag = std::numeric_limits<double>::max();
      }
      if (residual_dem < residual_lag)
        cMo = cMo_dem;
      else
        cMo = cMo_lag;
    }
    pose.computePose(vpPose::VIRTUAL_VS, cMo);
    double residual = pose.computeResidual(cMo);

    return residual;
  }
};
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "visp_ros_blob_tracker_node");
  BlobTracker blobTracker;
  blobTracker.spin();
  return 0;
}
