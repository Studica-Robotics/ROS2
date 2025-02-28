#!/bin/bash

set -e

echo "Starting Orbbec camera setup..."

ROS2_DIR=$(pwd)

if [ -d "camera" ]; then
    echo "A directory named 'camera' already exists. Please remove it to continue installation."
    exit 1
fi

mkdir -p camera/src
cd camera/src

git clone https://github.com/orbbec/OrbbecSDK_ROS2.git || { echo "Failed to clone OrbbecSDK_ROS2"; exit 1; }

sudo apt install libgflags-dev nlohmann-json3-dev  \
ros-$ROS_DISTRO-image-transport  ros-${ROS_DISTRO}-image-transport-plugins ros-${ROS_DISTRO}-compressed-image-transport \
ros-$ROS_DISTRO-image-publisher ros-$ROS_DISTRO-camera-info-manager \
ros-$ROS_DISTRO-diagnostic-updater ros-$ROS_DISTRO-diagnostic-msgs ros-$ROS_DISTRO-statistics-msgs \
ros-$ROS_DISTRO-backward-ros libdw-dev

cd $ROS2_DIR/camera/src/OrbbecSDK_ROS2/orbbec_camera/scripts
sudo bash install_udev_rules.sh
sudo udevadm control --reload-rules && sudo udevadm trigger

sudo apt install -y ros-$ROS_DISTRO-compressed-image-transport || { echo "Failed to install compressed image transport"; exit 1; }

cd $ROS2_DIR/ros_dependencies/depth_camera 
sudo dpkg -i ros-$ROS_DISTRO-orbbec-camera-msgs_jammy_arm64.deb || { echo "Failed to install orbbec camera msgs package"; exit 1; }
sudo dpkg -i ros-$ROS_DISTRO-orbbec-camera_jammy_arm64.deb || { echo "Failed to install orbbec camera package"; exit 1; }

echo "Camera setup completed successfully!"
