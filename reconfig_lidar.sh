#!/bin/bash

set -e

cd ros_dependencies/lidar || { echo "lidar directory not found, please make sure lidar is set up."; exit 1; }

LIDAR_DIR=$(pwd)

ORDERED_LIDAR_KEYS=(2 3 4 5 6 7 8 9 10 11 12)
declare -A LIDAR_MODELS=(
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

lidar_choice="0"

while [ -z "${LIDAR_MODELS[$lidar_choice]}" ]; do
    read -p "Enter the number corresponding to your LiDAR model: " lidar_choice
    if [[ -z "${LIDAR_MODELS[$lidar_choice]}" ]]; then
        echo "Invalid choice. Please enter a number from ${ORDERED_LIDAR_KEYS[0]} to ${ORDERED_LIDAR_KEYS[-1]}."
    fi
done

SELECTED_LIDAR=${LIDAR_MODELS[$lidar_choice]}
echo "You selected: $SELECTED_LIDAR"
echo "Configuring YDLidar ROS2 driver..."
cp $LIDAR_DIR/ydlidar_ros2_ws/src/ydlidar_ros2_driver/params/$SELECTED_LIDAR.yaml $LIDAR_DIR/ydlidar_ros2_ws/src/ydlidar_ros2_driver/params/ydlidar.yaml
echo "Start LiDAR with: 'ros2 launch ydlidar_ros2_driver ydlidar_launch.py'"
