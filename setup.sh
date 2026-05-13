#!/bin/bash

set -e

INSTALL_LIDAR=true
INSTALL_CAMERA=true
INSTALL_EXPLORE=true
ORIGINAL_DIR=$(pwd)

echo "Starting ROS2 robot setup. Takes about 5-7 mins."

while getopts ":hlce" opt; do
    case $opt in
        h)
            echo "Usage: $0 [-l] [-c] [-e]]"
            echo "-l Skip lidar installiation"
            echo "-c Skip camera installation"
            echo "-e Skip explore installation"
            echo "-h Show this help message"
            exit 0;
            ;;
        l)
            INSTALL_LIDAR=false
            ;;
        c)
            INSTALL_CAMERA=false
            ;;
        e)
            INSTALL_EXPLORE=false
            ;;
        \?)
            echo "Invalid option: -$OPTARG" >&2
            exit 1
    esac
done

if [ -z "$ROS_DISTRO" ]; then
    # Tier-1 pairing: 22.04 -> Humble, 24.04 -> Jazzy
    . /etc/os-release
    case "${VERSION_ID:-}" in
        22.04) export ROS_DISTRO=humble ;;
        24.04) export ROS_DISTRO=jazzy ;;
        *)
            echo "Set ROS_DISTRO before running this script (unsupported VERSION_ID=${VERSION_ID:-unknown})."
            echo "Example: export ROS_DISTRO=jazzy"
            exit 1
            ;;
    esac
fi

if [ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    _UBUNTU_CODENAME="$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME}}")"
    echo "ROS 2 not found at /opt/ros/${ROS_DISTRO}."
    echo "Ubuntu mirrors do not include ROS packages until you add the ROS 2 apt repository."
    echo "Doc: https://docs.ros.org/en/${ROS_DISTRO}/Installation/Ubuntu-Install-Debs.html"
    echo ""
    echo "On this machine (Ubuntu codename: ${_UBUNTU_CODENAME}), run:"
    cat <<'INST'
  sudo apt update && sudo apt install -y software-properties-common curl
  sudo add-apt-repository universe
  sudo apt update && sudo apt install curl -y
  export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F tag_name | awk -F'"' '{print $4}')
INST
    echo "  curl -L -o /tmp/ros2-apt-source.deb \"https://github.com/ros-infrastructure/ros-apt-source/releases/download/\${ROS_APT_SOURCE_VERSION}/ros2-apt-source_\${ROS_APT_SOURCE_VERSION}.${_UBUNTU_CODENAME}_all.deb\""
    cat <<'INST'
  sudo dpkg -i /tmp/ros2-apt-source.deb
  sudo apt update
INST
    echo "  sudo apt install -y ros-${ROS_DISTRO}-desktop ros-${ROS_DISTRO}-rmw-cyclonedds-cpp ros-dev-tools"
    exit 1
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

cd $ORIGINAL_DIR || { echo "Failed to return to original directory"; exit 1; }

cd ros_dependencies

if [ "$INSTALL_LIDAR" = true ]; then
    echo "Setting up lidar..."
    if [ -f "../setup_lidar.sh" ]; then
        chmod +x ../setup_lidar.sh
        ../setup_lidar.sh || { echo "Failed to run setup_lidar script"; exit 1; }
    else
        echo "setup_lidar.sh not found"
        exit 1
    fi
else
    echo "Skipping lidar installation..."
fi

if [ "$INSTALL_EXPLORE" = true ]; then
    echo "Setting up m-explore for ROS2..."
    if [ -d "m-explore-ros2" ]; then
        echo "A directory named 'm-explore-ros2' already exists. Please remove it to continue installation."
        exit 1
    fi

    git clone https://github.com/robo-friends/m-explore-ros2.git || { echo "Failed to clone m-explore-ros2"; exit 1; }
    cd m-explore-ros2 
    colcon build || { echo "Failed to build m-explore-ros2"; exit 1; }
    cd ..
else
    echo "Skipping explore installation..."
fi

if [ "$INSTALL_CAMERA" = true ]; then
    echo "Installing camera drivers..."
    if [ -f "../setup_camera.sh" ]; then
        chmod +x ../setup_camera.sh
        ../setup_camera.sh || { echo "Failed to run setup_camera script"; exit 1; }
    else
        echo "error: setup_camera.sh not found"
        exit 1
    fi
else
    echo "Skipping camera installation..."
fi

sudo apt update

echo "Configuring Cyclone DDS..."
DDS_SETUP_LINE="export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
sudo apt install -y "ros-${ROS_DISTRO}-rmw-cyclonedds-cpp"
if ! grep -Fxq "$DDS_SETUP_LINE" ~/.bashrc; then
    echo "$DDS_SETUP_LINE" >> ~/.bashrc
    echo "DDS Configuration done."
fi

echo "Installing SLAM toolbox..."
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

cd "$ORIGINAL_DIR/.."
source ~/.bashrc

echo "Build the studica_control workspace (run from your ROS2 repo root, typically where this script lives):"
echo "  cd \"$ORIGINAL_DIR\" && source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --symlink-install"

echo "Setup script completed successfully! Run 'source ~/.bashrc' to apply all environment changes."
