visp_ros
========

`visp_ros` is an extension of [ViSP](https://visp.inria.fr/) library developed by Inria [Rainbow](https://team.inria.fr/rainbow/) team. While ViSP is independent to ROS, in `visp_ros` we benefit from ROS features.

`visp_ros` is compatible with ROS kinetic, melodic and noetic.

`visp_ros` contains a library:

- with new C++ classes (vpROSGrabber, vpROSRobot, vpROSRobotPioneer, vpROSRobotFrankaCoppeliasim) that could be used like usual ViSP classes. They are based on ROS, but to use them there is no need to know so much about ROS;
- that makes possible to use ROS in a transparent way, either by building classical binaries without catkin, either by building ROS nodes with catkin but without the need to write ROS specific code;
- where creating a ROS node out of ViSP becomes simple.

`visp_ros` contains also a set of ROS nodes that allow to control specific hardware such as for the moment:

- robots that can be controlled only in our lab due to proprietary drivers: Afma6 gantry robot, Biclops PT head, ADEPT Viper 650 and 850 robots described here;
- other robots that anyone can buy and use with open-source drivers interfaced in ViSP: Pioneer mobile robot, Parrot bebop2 drone.

`visp_ros` contains also a tutorial that shows how to simulate a Franka Panda robot using [CoppeliaSim](https://www.coppeliarobotics.com/). The simulation is a physical simulation with a model that has been accurately identified from a real Franka robot. If you are using this simulator we would appreciate that you cite this [paper](http://rainbow-doc.irisa.fr/publi/publi/Gaz19a-eng.html):

*C. Gaz, M. Cognetti, A. Oliva, P. Robuffo Giordano, A. De Luca, Dynamic Identification of the Franka Emika Panda Robot With Retrieval of Feasible Parameters Using Penalty-Based Optimization. IEEE RA-L, 2019.*

# Installation

## Prerequisities

Install `ros-<version>-visp` package that matches your ros distribution (kinetic, melodic, noetic), as for example:

	$ sudo apt-get install ros-melodic-visp

If you want to use the nodes that allow to control real robots such as Biclops PT head, Viper 650, Viper 850, Afma4 or Afma6 robots, you need to build ViSP from source and install ViSP in `/opt/ros/<version>` in order to overwrite the version that was installed using the previous line.

	$ cd visp-ws
	$ git clone https://github.com/lagadic/visp.git
	$ mkdir visp-build-ros; cd visp-build-ros
	$ cmake -DCMAKE_INSTALL_PREFIX=/opt/ros/$ROS_DISTRO ../visp
	$ make -j4; sudo make install

## Get the source

Enter in catkin source folder and source ROS setup:

	$ cd ~/catkin_ws/src
	$ source /opt/ros/<version>/setup.bash

Get `vision_visp` stack that contains `visp_bridge` package:

	$ git clone https://github.com/lagadic/vision_visp.git -b $ROS_DISTRO

Get `visp_ros` package:

	$ git clone https://github.com/lagadic/visp_ros.git

## Build visp_ros package

If you want to use the Franka simulator install `Orocos-kdl` package

    $ sudo apt-get install liborocos-kdl-dev

Then build `visp_ros` package:

	$ cd ~/catkin_ws
	$ catkin_make -DCMAKE_BUILD_TYPE=Release --pkg visp_ros

## Build documentation and tutorials

There is the documentation available in ros wiki page: http://wiki.ros.org/visp_ros

It is also possible to build the package documentation from your own:

	$ cd ~/catkin_ws
	$ rosdoc_lite src/visp_ros

Documentation is available in `~/catkin_ws/doc/html/index.html`

From this main page you will get access to:
- Tutorial: How to use ROS grabber
- Tutorial: How to simulate Franka robot with Coppeliasim
- Visual servoing with Parrot Bebop 2 drone using visp_ros

# Usage

	$ source ~/catkin_ws/devel/setup.bash
