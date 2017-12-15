# maplab_realsense

This repository contains a ros wrapper for the Intel RealSense ZR300 driver, with a focus on its application for visual-inertial mapping.

Contents:
 * [Installation for maplab users](#installation-for-maplab-users)
 * [Build maplab_realsense](#build-maplab_realsense)
 * [Run maplab_realsense](#run-maplab_realsense)
 * [Stand-alone Installation](#stand-alone-installation)

## Installation for maplab users

This installation instruction assumes you already have maplab installed based on this [page](https://github.com/ethz-asl/maplab/wiki/Installation-Ubuntu).

### Install Realsense driver

* Ubuntu LTS 16.04
  ```
  sudo apt install ros-kinetic-librealsense
  ```
* Ubuntu LTS 14.04
  ```
  sudo apt-get install ros-indigo-librealsense
  ```
  
### Add maplab_realsense and dependencies to your maplab workspace
```
cd $CATKIN_WS/src
git clone https://github.com/ethz-asl/maplab_realsense.git
git clone  https://github.com/ethz-asl/cuckoo_time_translator.git
```

## Build maplab_realsense
```bash
catkin build maplab_realsense
```

## Run maplab_realsense

Source the catkin workspace:
```bash
source $CATKIN_WS/devel/setup.bash
```

There is an example launch file with a list of all available ros parameters:
```
 roslaunch maplab_realsense maplab_realsense.launch
```

## Stand-alone Installation

**Warning:** These instructions have not been tested!

### Install ROS
Skip this part if you have ROS installed.
```bash 
export UBUNTU_VERSION=xenial #(Ubuntu 16.04: xenial, Ubuntu 14.04: trusty)
export ROS_VERSION=kinetic #(Ubuntu 16.04: kinetic, Ubuntu 14.04: indigo)

sudo apt install software-properties-common
sudo add-apt-repository "deb http://packages.ros.org/ros/ubuntu $UBUNTU_VERSION main"
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
sudo apt update

sudo apt install ros-$ROS_VERSION-desktop-full "ros-$ROS_VERSION-tf2-*" "ros-$ROS_VERSION-camera-info-manager*" --yes

```

### Initialize ROS installation
Skip this part if you have ROS installed.
```bash
sudo rosdep init
rosdep update
echo ". /opt/ros/$ROS_VERSION/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Install librealsense
```bash
sudo apt install ros-${ROS_VERSION}-librealsense
```

### Create catkin workspace
Skip this part if you would like to add `maplab_realsense` to an existing workspace.
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

### Clone `maplab_realsense` and dependencies
Be aware that if you would like to add maplab_realsense to an existing workspace, there might be conflicts with the dependencies and/or wstool.
```bash
cd ${CATKIN_WS}/src
git clone https://github.com/ethz-asl/maplab_realsense.git
wstool init             # If you are using a pre-existing workspace and
                        # have used wstool before, this won't be necessary
wstool merge maplab_realsense/dependencies.rosinstall
wstool update -j4
```
