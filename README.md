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

`visp_ros` contains also FrankaSim a simulator for a Franka Panda robot using [CoppeliaSim](https://www.coppeliarobotics.com/). The simulation is a physical simulation with a model that has been accurately identified from a real Franka robot. If you are using this simulator we would appreciate that you cite this [paper](http://rainbow-doc.irisa.fr/publi/publi/Gaz19a-eng.html):

*C. Gaz, M. Cognetti, A. Oliva, P. Robuffo Giordano, A. De Luca, Dynamic Identification of the Franka Emika Panda Robot With Retrieval of Feasible Parameters Using Penalty-Based Optimization. IEEE RA-L, 2019.*

# 1. Installation

## 1.1. Install dependencies 

Either you can install dependencies from existing packages, either from source.

**Install dependencies from packages**

Install `ros-<version>-visp` and `ros-<version>-vision_visp` packages that match your ros distribution (kinetic, melodic, noetic), as for example:

  ```
  $ sudo apt-get install ros-melodic-visp ros-melodic-vision_visp
  ```

Once done, jump to section 1.2. 

**Install dependencies from source**

If you want to use the nodes that allow to control real robots such as Biclops PT head, Viper 650, Viper 850, Afma4, Afma6 or Franka robots, you need to build ViSP from source and install ViSP in `/opt/ros/<version>` in order to overwrite any version that was already installed from packages. There are a couple of [tutorials](https://visp-doc.inria.fr/doxygen/visp-daily/tutorial-install-ubuntu.html) that may help. Below we recall the main instructions

- Install first ViSP required dependencies: OpenCV, X11... 

  ```
  $ sudo apt-get install libopencv-dev libx11-dev liblapack-dev libeigen3-dev libv4l-dev \
         libzbar-dev libpthread-stubs0-dev libjpeg-dev libpng-dev
  ```

- If you want to simulate Panda robot using FrankaSim as explained in this [tutorial](http://docs.ros.org/en/noetic/api/visp_ros/html/tutorial-franka-coppeliasim.html),
  install also Orocos-kdl and PCL

  ```
  $ sudo apt-get install liborocos-kdl-dev libpcl-dev
  ```

- If you want to control a real Panda robot equipped with a Realsense camera as explained in this [tutorial](http://docs.ros.org/en/noetic/api/visp_ros/html/tutorial-franka-coppeliasim.html),
  you need to follow [this tutorial](https://visp-doc.inria.fr/doxygen/visp-daily/tutorial-franka-pbvs.html) to install libfranka and librealsense

- If you want to control a Parrot bebop2 drone follow [this tutorial](http://wiki.ros.org/visp_ros/Tutorials/How%20to%20do%20visual%20servoing%20with%20Parrot%20Bebop%202%20drone%20and%20visp_ros) that explain which are the required dependencies

- Then get, build and install ViSP

  ```
  $ cd visp-ws
  $ git clone https://github.com/lagadic/visp.git
  $ mkdir visp-build-ros; cd visp-build-ros
  $ cmake -DCMAKE_INSTALL_PREFIX=/opt/ros/$ROS_DISTRO ../visp
  $ make -j4; sudo make install
  ```

- Then get `vision_visp` meta package that contains `visp-bridge` package that is a `visp_ros` package dependency:

  ```
  $ cd ~/catkin_ws/src
  $ source /opt/ros/<version>/setup.bash
  $ git clone https://github.com/lagadic/vision_visp.git -b $ROS_DISTRO
  ```

## 1.2. Get visp_ros source

Get `visp_ros` package:

  ```
  $ git clone https://github.com/lagadic/visp_ros.git
  ```

## 1.3. Build visp_ros package

To build `visp_ros` package run:

  ```
  $ cd ~/catkin_ws
  $ catkin_make -DCMAKE_BUILD_TYPE=Release --pkg visp_ros
  ```

## 1.4. Build documentation and tutorials

There is the documentation available in ROS wiki page: http://wiki.ros.org/visp_ros

It is also possible to build the package documentation from your own:

  ```
  $ cd ~/catkin_ws
  $ rosdoc_lite src/visp_ros
  ```

Documentation is available in `~/catkin_ws/doc/html/index.html`.
This documentation is a local version of the documentation available [here](http://docs.ros.org/en/noetic/api/visp_ros/html/index.html).

# 2. Usage

See https://wiki.ros.org/visp_ros
