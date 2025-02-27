#!/bin/bash

set -e

echo "Starting ROS2 robot setup"

if [ -z "$ROS_DISTRO" ]; then
    export ROS_DISTRO=humble
fi

ROS_SETUP_LINE=". /opt/ros/$ROS_DISTRO/setup.bash"
if ! grep -Fxq "$ROS_SETUP_LINE" ~/.bashrc; then
    echo "$ROS_SETUP_LINE" >> ~/.bashrc
    echo "Added ROS setup to ~/.bashrc"
fi
source /opt/ros/$ROS_DISTRO/setup.bash

echo "Setting up drivers..."
cd drivers 
sudo make || { echo "Failed to run make for drivers"; exit 1; }
make install || { echo "Failed to run make install for drivers"; exit 1; }

STUDICA_DRIVERS_LINE="export LD_LIBRARY_PATH=/usr/local/lib/studica_drivers:\$LD_LIBRARY_PATH"
STUDICA_CONTROL_LINE="export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/usr/local/lib/studica_control"

if ! grep -Fq "$STUDICA_DRIVERS_LINE" ~/.bashrc; then
    echo "$STUDICA_DRIVERS_LINE" >> ~/.bashrc
    echo "Added studica_drivers path to LD_LIBRARY_PATH"
fi

if ! grep -Fq "$STUDICA_CONTROL_LINE" ~/.bashrc; then
    echo "$STUDICA_CONTROL_LINE" >> ~/.bashrc
    echo "Added studica_control path to LD_LIBRARY_PATH"
fi

ORIGINAL_DIR=$(pwd)
cd .. || { echo "Failed to return to original directory"; exit 1; }

echo "Setting up lidar..."
if [ -f "./setup_lidar" ]; then
    chmod +x ./setup_lidar
    ./setup_lidar || { echo "Failed to run setup_lidar script"; exit 1; }
fi

echo "Setting up m-explore for ROS2..."
if [ -d "m-explore-ros2" ]; then
    echo "m-explore-ros2 directory exists, removing it..."
    rm -rf m-explore-ros2
fi

git clone https://github.com/robo-friends/m-explore-ros2.git || { echo "Failed to clone m-explore-ros2"; exit 1; }
cd m-explore-ros2 
colcon build || { echo "Failed to build m-explore-ros2"; exit 1; }
cd ..

echo "Installing SLAM toolbox..."
sudo apt update
sudo apt install -y ros-$ROS_DISTRO-slam-toolbox || { echo "Failed to install SLAM toolbox"; exit 1; }

echo "Installing Nav2..."
sudo apt install -y ros-$ROS_DISTRO-navigation2 || { echo "Failed to install navigation2"; exit 1; }
sudo apt install -y ros-$ROS_DISTRO-nav2-bringup || { echo "Failed to install nav2-bringup"; exit 1; }

echo "Installing rosbridge..."
sudo apt-get install -y ros-$ROS_DISTRO-rosbridge-suite || { echo "Failed to install rosbridge suite"; exit 1; }

echo "Installing foxglove bridge..."
sudo apt install -y ros-$ROS_DISTRO-foxglove-bridge || { echo "Failed to install foxglove bridge"; exit 1; }

echo "Installing joy package..."
sudo apt install -y ros-$ROS_DISTRO-joy || { echo "Failed to install joy package"; exit 1; }

echo "Installing ROS2 control packages..."
sudo apt install -y ros-$ROS_DISTRO-ros2-control || { echo "Failed to install ros2-control"; exit 1; }
sudo apt install -y ros-$ROS_DISTRO-ros2-controllers || { echo "Failed to install ros2-controllers"; exit 1; }

echo "Installing camera drivers..."
cd ros_dependencies/depth_camera 
sudo dpkg -i ros-$ROS_DISTRO-orbbec-camera_jammy_arm64.deb || { echo "Failed to install orbbec camera package"; exit 1; }
sudo dpkg -i ros-$ROS_DISTRO-orbbec-camera-msgs_jammy_arm64.deb || { echo "Failed to install orbbec camera msgs package"; exit 1; }
sudo apt install -y ros-$ROS_DISTRO-compressed-image-transport || { echo "Failed to install compressed image transport"; exit 1; }

cd "$ORIGINAL_DIR/.."
source ~/.bashrc

echo "Setup script completed successfully! Run 'source ~/.bashrc' to apply all environment changes."
