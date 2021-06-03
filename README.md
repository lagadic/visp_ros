visp_ros
========

A basket of generic ros nodes based on ViSP library.

# Installation

## Prerequisities

Install `ros-<version>-visp` package that matches your ros distribution (kinetic, melodic, noetic), as for example:

	$ sudo apt-get install ros-melodic-visp

If you want to use the nodes that allow to control real robots such as Biclops PT head, Viper 650, Viper 850, Afma4 or Afma6 robots, you need to build ViSP from source and install ViSP in '/opt/ros/<ros-version>' in order to overwrite the version that was installed using the previous line. 

	$ cd visp-ws
	$ git clone https://github.com/lagadic/visp.git
	$ mkdir visp-build-ros; cd visp-build-ros
	$ cmake -DCMAKE_INSTALL_PREFIX=/opt/ros/$ROS_DISTRO ../visp
	$ make -j4; sudo make install

## Get the source

Enter in catkin source folder and source ROS setup:

	$ cd ~/catkin_ws/src
	$ source /opt/ros/<ros-version>

Get vision_visp stack that contains visp_bridge package:

	$ git clone https://github.com/lagadic/vision_visp.git -b $ROS_DISTRO

Get visp_ros package:

	$ git clone https://github.com/lagadic/visp_ros.git

## Build visp_ros package

	$ cd ~/catkin_ws
	$ catkin_make -DCMAKE_BUILD_TYPE=Release --pkg visp_ros

## Build documentation and tutorials

	$ cd ~/catkin_ws
	$ rosdoc_lite src/visp_ros

Documentation is available in `~/catkin_ws/doc/html/index.html`


# Usage

	$ source ~/catkin_ws/devel/setup.bash
