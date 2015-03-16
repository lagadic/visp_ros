//! \example tutorial-ros-grabber.cpp
#include <visp/vpDisplayX.h>
#include <visp/vpImage.h>
#include <visp_ros/vpROSGrabber.h>

int main(int argc, const char** argv)
{
  try {
    bool opt_use_camera_info = false;
    for (int i=0; i<argc; i++) {
      if (std::string(argv[i]) == "--use-camera-info")
        opt_use_camera_info = true;
      else if (std::string(argv[i]) == "--help") {
        std::cout << "Usage: " << argv[0]
                  << " [--use-camera-info] [--help]"
                  << std::endl;
        return 0;
      }
    }
    std::cout << "Use camera info: " << ((opt_use_camera_info == true) ? "yes" : "no") << std::endl;

    //vpImage<unsigned char> I; // Create a gray level image container
    vpImage<vpRGBa> I; // Create a color image container
    //! [Construction]
    vpROSGrabber g; // Create a grabber based on ROS
    //! [Construction]

    //! [Setting camera topic]
    g.setImageTopic("/camera/image_raw");
    //! [Setting camera topic]
    //! [Setting camera info]
    if (opt_use_camera_info) {
      g.setCameraInfoTopic("/camera/camera_info");
      g.setRectify(true);
    }
    //! [Setting camera info]

    //! [Opening]
    g.open(I);
    //! [Opening]
    std::cout << "Image size: " << I.getWidth() << " " << I.getHeight() << std::endl;

#ifdef VISP_HAVE_X11
    vpDisplayX d(I);
#else
    std::cout << "No image viewer is available..." << std::endl;
#endif

    while(1) {
      //! [Acquisition]
      g.acquire(I);
      //! [Acquisition]
      vpDisplay::display(I);
      vpDisplay::displayText(I, 20, 20, "A click to quit...", vpColor::red);
      vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false))
        break;
    }
  }
  catch(vpException e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
}
