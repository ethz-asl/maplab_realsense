# maplab_realsense

This repository contains a ros wrapper for the Intel RealSense ZR300 driver, with a focus on its application for visual-inertial mapping.

## Installation

### Dependencies:

First install realsense ros driver package:
* Ubuntu LTS 16.04
  ```
  sudo apt install ros-kinetic-librealsense
  ```
* Ubuntu LTS 14.04
  ```
  sudo apt-get install ros-indigo-librealsense
  ```
  
### Building `maplab_realsense`

If you have no pre-existing catkin workspace, initialize it as follows:
```bash
export ROS_VERSION=kinetic #(Ubuntu 16.04: kinetic, Ubuntu 14.04: indigo)
export CATKIN_WS=~/maplab_ws
mkdir -p $CATKIN_WS/src
cd $CATKIN_WS
catkin init
catkin config --merge-devel # Necessary for catkin_tools >= 0.4.
catkin config --extend /opt/ros/$ROS_VERSION
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
cd src
```

Use `wstool` to checkout all dependencies that are required for `maplab_realsense`:
```bash
cd ${CATKIN_WS}/src
git clone https://github.com/ethz-asl/maplab_realsense.git
wstool init             # If you are using a pre-existing workspace and
                        # have used wstool before, this won't be necessary
wstool merge maplab_realsense/dependencies.rosinstall
wstool update -j4
```

Build `maplab_realsense`:
```bash
catkin build maplab_realsense
```

## Running `maplab_realsense`

Source the catkin workspace:
```bash
source $CATKIN_WS/devel/setup.bash
```

There is an example launch file with a list of all available ros parameters:
```
 roslaunch maplab_realsense maplab_realsense.launch
```
