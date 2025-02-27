#!/bin/bash

set -e

echo "Starting YDLidar setup..."

echo "Creating lidar directory..."
if [ -d "lidar" ]; then
    echo "Lidar directory exists, removing it..."
    rm -rf lidar
fi
mkdir -p lidar
cd lidar 
LIDAR_DIR=$(pwd)

echo "Installing dependencies..."
sudo apt install -y cmake pkg-config || { echo "Failed to install dependencies"; exit 1; }

echo "Setting up YDLidar-SDK..."
git clone https://github.com/YDLIDAR/YDLidar-SDK.git || { echo "Failed to clone YDLidar-SDK"; exit 1; }
cd YDLidar-SDK

if [ -d "build" ]; then
    echo "Build directory exists, removing it..."
    rm -rf build
fi

mkdir -p build
cd build 
cmake .. || { echo "Failed to run cmake"; exit 1; }
make || { echo "Failed to run make"; exit 1; }
sudo make install || { echo "Failed to install YDLidar-SDK"; exit 1; }

cd "$LIDAR_DIR" || { echo "Failed to return to lidar directory"; exit 1; }

echo "Setting up YDLidar ROS2 driver..."
mkdir -p ydlidar_ros2_ws/src
git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git ydlidar_ros2_ws/src/ydlidar_ros2_driver || { echo "Failed to clone ydlidar_ros2_driver"; exit 1; }
cd ydlidar_ros2_ws/src/ydlidar_ros2_driver || { echo "Failed to change to ydlidar_ros2_driver directory"; exit 1; }
git checkout origin/humble || { echo "Failed to checkout humble branch"; exit 1; }

cd "$LIDAR_DIR/ydlidar_ros2_ws" || { echo "Failed to change to ydlidar_ros2_ws directory"; exit 1; }
colcon build --symlink-install || { echo "Failed to build ROS2 package"; exit 1; }

SETUP_LINE="source $LIDAR_DIR/ydlidar_ros2_ws/install/setup.bash"
if grep -Fxq "$SETUP_LINE" ~/.bashrc; then
    echo "Setup line already exists in ~/.bashrc"
else
    echo "$SETUP_LINE" >> ~/.bashrc
    echo "Added setup line to ~/.bashrc"
fi

. ~/.bashrc

echo "YDLidar setup completed successfully! All components installed in: $LIDAR_DIR. Start lidar with: 'ros2 launch ydlidar_ros2_driver ydlidar_launch.py'"