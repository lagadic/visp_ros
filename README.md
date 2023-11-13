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

`visp_ros` contains also FrankaSim a simulator for a Franka Panda robot using [CoppeliaSim](https://www.coppeliarobotics.com/). The simulation is a physical simulation with a dynamic model that has been accurately identified from a real Franka robot.  [paper](http://rainbow-doc.irisa.fr/publi/publi/Gaz19a-eng.html).

If you are using this simulator we would appreciate that you cite this  [paper](http://rainbow-doc.irisa.fr/publi/publi/Oliva22c-eng.html):

* <em> A. A. Oliva, F. Spindler, P. Robuffo Giordano, F. Chaumette. FrankaSim: A Dynamic Simulator for the Franka Emika Robot with Visual-Servoing Enabled Capabilities. In Int. Conf. on Control, Automation, Robotics and Vision, ICARCV'22, Singapore, December 2022.</em>*

# 1. Installation

## 1.1. Install dependencies 

Either you can install dependencies from existing packages, either from source.

**Install dependencies from packages**

Install `ros-<distro>-visp` and `ros-<distro>-vision_visp` packages that match your ros2 distribution (humble, rolling), as for example:

  ```
  $ sudo apt-get install ros-humble-visp ros-humble-vision_visp
  ```

Note: It may occur that `vision_visp` package is not available for your ros2 distro. 

Once done, or if the packages are not existing jump to section 1.2. 

**Install dependencies from source**

If you want to use the nodes that allow to control real robots such as Biclops PT head, Viper 650, Viper 850, Afma4, Afma6 or Franka robots, you need to build ViSP from source and install ViSP in `/opt/ros/<distro>` in order to overwrite any version that was already installed from packages. There are a couple of [tutorials](https://visp-doc.inria.fr/doxygen/visp-daily/tutorial-install-ubuntu.html) that may help. Below we recall the main instructions

- Install first ViSP required dependencies: OpenCV, X11... 

  ```
  $ sudo apt-get install libopencv-dev libx11-dev liblapack-dev libeigen3-dev libv4l-dev \
                        libzbar-dev libpthread-stubs0-dev libdc1394-dev nlohmann-json3-dev
  ```

- If you want to simulate Panda robot using FrankaSim as explained in this [tutorial](http://docs.ros.org/en/noetic/api/visp_ros/html/tutorial-franka-coppeliasim.html),
  install also Orocos-kdl and PCL

  ```
  $ sudo apt-get install liborocos-kdl-dev libpcl-dev
  ```

- If you want to control a real Panda robot equipped with a Realsense camera as explained in this [tutorial](http://docs.ros.org/en/noetic/api/visp_ros/html/tutorial-franka-coppeliasim.html),
  you need to follow [this tutorial](https://visp-doc.inria.fr/doxygen/visp-daily/tutorial-franka-pbvs.html) to install libfranka and librealsense

- If you want to control a Parrot bebop2 drone follow [this tutorial](http://wiki.ros.org/visp_ros/Tutorials/How%20to%20do%20visual%20servoing%20with%20Parrot%20Bebop%202%20drone%20and%20visp_ros) that explain which are the required dependencies

- Then clone, build and install ViSP

  ```
  $ echo "export VISP_WS=$HOME/visp-ws" >> ~/.bashrc
  $ source ~/.bashrc
  $ mkdir -p $VISP_WS
  $ git clone https://github.com/lagadic/visp.git
  $ mkdir visp-build; cd visp-build
  $ cmake -DCMAKE_INSTALL_PREFIX=/opt/ros/$ROS_DISTRO ../visp
  $ make -j4; sudo make install
  ```

- Create your ros2 workspace and get `vision_visp` meta package that contains `visp-bridge` package that is a `visp_ros` package dependency:

  ```
  $ mkdir -p $HOME/colcon_ws/src
  $ cd $HOME/colcon_ws/src
  $ source /opt/ros/<version>/setup.bash
  $ git clone https://github.com/lagadic/vision_visp.git -b rolling
  ```

## 1.2. Get visp_ros source

Get `visp_ros` package:

  ```
  $ cd $HOME/colcon_ws/src
  $ git clone https://github.com/lagadic/visp_ros.git
  ```

## 1.3. Build visp_ros package

To build `visp_ros` package run:

  ```
  $ cd $HOME/colcon_ws
  $ colcon build --symlink-install --cmake-args -DVISP_DIR=$VISP_WS/visp-build -DCMAKE_BUILD_TYPE=Release
  ```

# 2. Usage

See https://wiki.ros.org/visp_ros
