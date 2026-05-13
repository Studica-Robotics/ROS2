#!/bin/bash

set -e

echo "Starting Orbbec camera setup..."

ROS2_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -z "$ROS_DISTRO" ]; then
    . /etc/os-release
    case "${VERSION_ID:-}" in
        22.04) export ROS_DISTRO=humble ;;
        24.04) export ROS_DISTRO=jazzy ;;
        *)
            echo "Set ROS_DISTRO before running this script (e.g. export ROS_DISTRO=jazzy)."
            exit 1
            ;;
    esac
fi

if [ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    echo "ROS 2 not found at /opt/ros/${ROS_DISTRO}. Install it before camera setup."
    exit 1
fi

if [ -d "$ROS2_DIR/camera" ]; then
    echo "A directory named 'camera' already exists. Please remove it to continue installation."
    exit 1
fi

mkdir -p "$ROS2_DIR/camera/src"
cd "$ROS2_DIR/camera/src"

git clone https://github.com/orbbec/OrbbecSDK_ROS2.git || { echo "Failed to clone OrbbecSDK_ROS2"; exit 1; }

sudo apt install libgflags-dev nlohmann-json3-dev  \
ros-$ROS_DISTRO-image-transport  ros-${ROS_DISTRO}-image-transport-plugins ros-${ROS_DISTRO}-compressed-image-transport \
ros-$ROS_DISTRO-image-publisher ros-$ROS_DISTRO-camera-info-manager \
ros-$ROS_DISTRO-diagnostic-updater ros-$ROS_DISTRO-diagnostic-msgs ros-$ROS_DISTRO-statistics-msgs \
ros-$ROS_DISTRO-backward-ros libdw-dev

cd "$ROS2_DIR/camera/src/OrbbecSDK_ROS2/orbbec_camera/scripts"
sudo bash install_udev_rules.sh
sudo udevadm control --reload-rules && sudo udevadm trigger

sudo apt install -y ros-$ROS_DISTRO-compressed-image-transport || { echo "Failed to install compressed image transport"; exit 1; }

. /etc/os-release
MSGS_DEB="$ROS2_DIR/depth_camera/ros-${ROS_DISTRO}-orbbec-camera-msgs_jammy_arm64.deb"
CAM_DEB="$ROS2_DIR/depth_camera/ros-${ROS_DISTRO}-orbbec-camera_jammy_arm64.deb"

if [ "${VERSION_CODENAME:-}" = "jammy" ] && [ -f "$MSGS_DEB" ] && [ -f "$CAM_DEB" ]; then
    echo "Installing Orbbec packages from local jammy arm64 .deb files..."
    cd "$ROS2_DIR/depth_camera"
    sudo dpkg -i "$MSGS_DEB" || { echo "Failed to install orbbec camera msgs package"; exit 1; }
    sudo dpkg -i "$CAM_DEB" || { echo "Failed to install orbbec camera package"; exit 1; }
else
    echo "Building OrbbecSDK_ROS2 from source for ${ROS_DISTRO} (no usable jammy .deb pair under depth_camera, or not on jammy)."
    # shellcheck source=/dev/null
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
    cd "$ROS2_DIR/camera"
    colcon build --symlink-install || { echo "colcon build failed; install missing build deps and retry."; exit 1; }
    CAMERA_SETUP_LINE="source $ROS2_DIR/camera/install/setup.bash"
    if ! grep -Fq "$CAMERA_SETUP_LINE" ~/.bashrc; then
        echo "$CAMERA_SETUP_LINE" >> ~/.bashrc
        echo "Added camera workspace overlay to ~/.bashrc"
    fi
fi

echo "Camera setup completed successfully!"
