/*! \example tutorial-ros-grabber.cpp */
#include <visp/vpDisplayX.h>
#include <visp/vpImage.h>
#include <visp_ros/vpROSGrabber.h>

int main()
{
  try {
    //vpImage<unsigned char> I; // Create a gray level image container
    vpImage<vpRGBa> I; // Create a color image container
    vpROSGrabber g; // Create a grabber based on ROS

    g.setCameraInfoTopic("/camera/camera_info");
    g.setImageTopic("/camera/image_raw");
    g.setRectify(true);
    g.open(I);
    std::cout << "Image size: " << I.getWidth() << " " << I.getHeight() << std::endl;

#ifdef VISP_HAVE_X11
    vpDisplayX d(I);
#else
    std::cout << "No image viewer is available..." << std::endl;
#endif

    while(1) {
      g.acquire(I);
      vpDisplay::display(I);
      vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false))
        break;
    }
  }
  catch(vpException e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
}
