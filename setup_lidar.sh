#!/bin/bash

set -e

echo "Starting YDLidar setup..."

if [ -d "lidar" ]; then
    echo "A directory named 'lidar' already exists. Please remove it to continue installation."
    exit 1
fi

mkdir -p lidar
cd lidar 
LIDAR_DIR=$(pwd)

ORDERED_LIDAR_KEYS=(1 2 3 4 5 6 7 8 9 10 11 12)
declare -A LIDAR_MODELS=(
    [1]="No LiDAR"
    [2]="G1"
    [3]="G2"
    [4]="G6"
    [5]="GS2"
    [6]="TEA"
    [7]="TG"
    [8]="Tmini"
    [9]="Tmini-Plus-SH"
    [10]="X2"
    [11]="X4"
    [12]="X4-Pro"
)

echo "Select your YDLidar model:"
for key in "${ORDERED_LIDAR_KEYS[@]}"; do
    echo "[$key] ${LIDAR_MODELS[$key]}"
done

read -p "Enter the number corresponding to your LiDAR model: " lidar_choice

if [[ -z "${LIDAR_MODELS[$lidar_choice]}" ]]; then
    echo "Invalid choice. Exiting setup."
    exit 1
fi

SELECTED_LIDAR=${LIDAR_MODELS[$lidar_choice]}
echo "You selected: $SELECTED_LIDAR"

echo "Installing dependencies..."
sudo apt install -y cmake pkg-config || { echo "Failed to install dependencies"; exit 1; }

echo "Setting up YDLidar-SDK..."
git clone https://github.com/YDLIDAR/YDLidar-SDK.git || { echo "Failed to clone YDLidar-SDK"; exit 1; }
cd YDLidar-SDK

mkdir -p build
cd build 
cmake .. || { echo "Failed to run cmake"; exit 1; }
make || { echo "Failed to run make"; exit 1; }
sudo make install || { echo "Failed to install YDLidar-SDK"; exit 1; }

cd "$LIDAR_DIR" 

echo "Setting up YDLidar ROS2 driver..."
mkdir -p ydlidar_ros2_ws/src
git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git ydlidar_ros2_ws/src/ydlidar_ros2_driver || { echo "Failed to clone ydlidar_ros2_driver"; exit 1; }
cd ydlidar_ros2_ws/src/ydlidar_ros2_driver || { echo "Failed to change to ydlidar_ros2_driver directory"; exit 1; }
git checkout origin/humble || { echo "Failed to checkout humble branch"; exit 1; }

cd "$LIDAR_DIR/ydlidar_ros2_ws"
colcon build --symlink-install || { echo "Failed to build ROS2 package"; exit 1; }

SETUP_LINE="source $LIDAR_DIR/ydlidar_ros2_ws/install/setup.bash"
if grep -Fxq "$SETUP_LINE" ~/.bashrc; then
    echo "Setup line already exists in ~/.bashrc"
else
    echo "$SETUP_LINE" >> ~/.bashrc
    echo "Added setup line to ~/.bashrc"
fi

. ~/.bashrc

echo "YDLidar setup completed successfully! You selected LiDAR model: $SELECTED_LIDAR."
echo "All components installed in: $LIDAR_DIR."

if [[ "$SELECTED_LIDAR" != "No LiDAR" ]]; then
    cp $LIDAR_DIR/ydlidar_ros2_ws/src/ydlidar_ros2_driver/params/$SELECTED_LIDAR.yaml $LIDAR_DIR/ydlidar_ros2_ws/src/ydlidar_ros2_driver/params/ydlidar.yaml
    echo "Start LiDAR with: 'ros2 launch ydlidar_ros2_driver ydlidar_launch.py'"
else
    echo "No LiDAR selected, but setup is complete in case you add one later."
fi
