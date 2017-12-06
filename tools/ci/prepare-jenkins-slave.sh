#!/bin/bash -e
echo "Running the prepare script for maplab_realsense.";

if [[ $(uname) == "Linux" ]]; then
  if lsb_release -c 2> /dev/null | grep trusty > /dev/null ; then
    # Ubuntu 14.04 / ROS Indigo.
    sudo apt-get install -y ros-indigo-librealsense
  elif lsb_release -c 2> /dev/null | grep xenial > /dev/null ; then
    # Ubuntu 16.04 / ROS Kinetic.
    sudo apt-get install -y ros-kinetic-librealsense
  else
    echo "Unknown Ubuntu version. Couldn't install all necessary dependencies."
  fi
else
  echo "Platform not supported by the prepare script."
  echo "Please ensure that all necessary dependencies are installed."
fi
