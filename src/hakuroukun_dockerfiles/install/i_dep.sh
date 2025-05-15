#!/bin/sh
set -e
basic_dep="git \
           curl \
           nano \
           vim \
           python3-catkin-tools \
           python3-pip \
	  "

ros_dep="ros-noetic-gazebo-ros-pkgs \
         ros-noetic-gazebo-ros-control \
         ros-noetic-controller-manager \
         ros-noetic-diff-drive-controller \
         ros-noetic-joint-state-controller \
         ros-noetic-move-base \
         ros-noetic-imu-tools \
         ros-noetic-twist-mux \
         ros-noetic-interactive-marker-twist-server \
         ros-noetic-map-server \
         ros-noetic-navigation \
         ros-noetic-rosbridge-server \
         ros-noetic-robot-localization
          "

apt-get update
apt-get upgrade -y
apt-get install -y $basic_dep
DEBIAN_FRONTEND=noninteractive apt-get install -y $ros_dep
apt-get autoremove -y
apt-get clean -y
update-alternatives --install /usr/bin/python python /usr/bin/python3.8 1
python --version
python -m pip install pyserial