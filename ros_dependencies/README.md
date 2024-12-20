# Install and Build ROS Components

This guide provides an overview of the main components and libraries used in a ROS2 setup, including installation and usage instructions.

## Overview
This document provides instructions for setting up key ROS2 dependencies

**ROS2 Packages**
* **ROS2 control**
* **SLAM**
* **NAV2**
* **rosbridge**
* **Joy**

**Drivers**
* **YDLidar** 
* **Orbbec Depth Camera**

---

## ROS2 Packages
### SLAM
SLAM (Simultaneous Localization and Mapping) is used for mapping the environment using lidar sensors.

#### Installation

```bash
sudo apt install ros-humble-slam-toolbox
```
#### Running
```
ros2 launch slam_toolbox online_sync_launch.py
```

### NAV2
NAV2 is used for robot navigation, allowing path generation and following.

#### Installation

```bash
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
```
#### Running
```
ros2 launch nav2_bringup navigation_launch.py
```


### ROS2 Control (Diff Drive)
ROS2 Control is used to control robot hardware, such as differential-drive robots.

#### Installation

```bash
sudo apt install ros-humble-ros2-control 
sudo apt install ros-humble-ros2-controllers
```
#### Running
```
```
### rosbridge
`rosbridge` is used to relay ROS2 messages to a web-based client (e.g., React).

#### Installation
```bash
sudo apt-get install ros-humble-rosbridge-suite
```
#### Running
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml 
```

### Other Packages
```bash
sudo apt install ros-humble-compressed-image-transport
```

## Camera Drivers

### [Orbbec Depth Camera SDK](https://github.com/orbbec/OrbbecSDK_ROS2)

#### Installation
Expect long installation time.

```bash
# from ROS folder
sudo apt install libgflags-dev nlohmann-json3-dev  \
ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-image-publisher ros-$ROS_DISTRO-camera-info-manager \
ros-$ROS_DISTRO-diagnostic-updater ros-$ROS_DISTRO-diagnostic-msgs ros-$ROS_DISTRO-statistics-msgs \
ros-$ROS_DISTRO-backward-ros libdw-dev

cd  camera_driver/src/OrbbecSDK_ROS2/orbbec_camera/scripts
sudo bash install_udev_rules.sh
sudo udevadm control --reload-rules && sudo udevadm trigger

cd ../../../.. # cd back to camera_driver
source /opt/ros/humble.setup.bash
colcon build --event-handlers  console_direct+  --cmake-args  -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```
#### Running
```bash
ros2 launch orbbec_camera gemini_e.launch.py
```
## Lidar Drivers
### [ydlidar SDK](https://github.com/YDLIDAR/YDLidar-SDK/blob/master/doc/howto/how_to_build_and_install.md)

**Installation**
```bash
sudo apt install cmake pkg-config
sudo apt-get install python swig
sudo apt-get install python-pip
# cd to ROS folder
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
cd YDLidar-SDK
mkdir build && cd build
cmake ..
make
sudo make install
# python API
cd YDLidar-SDK
pip3 install .
```
### [ydlidar ROS2 driver](https://github.com/YDLIDAR/YDLidar-SDK/blob/master/doc/howto/how_to_build_and_install.md)

**Installation**
```bash
cd ydlidar_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```
#### Running
```bash
ros2 launch ydlidar_ros2_driver ydlidar_launch_view.py
```
If using yellow LiDAR, change ydliar.yaml to X2.yaml in launch files.


### Running manual_composition
```
ros2 service call /create_component studica_control/srv/SetData "{component: 'titan', initparams: {can_id: 42, motor_freq: 15600, dist_per_tick: 0.0006830601, speed: 0.8}}" && ros2 service call /create_component studica_control/srv/SetData "{name: 'servo1', component: 'servo', initparams: {pin: 13, servo_type: 'standard'}}" && ros2 service call /create_component studica_control/srv/SetData "{name: 'servo2', component: 'servo', initparams: {pin: 12, servo_type: 'standard'}}" && ros2 service call /create_component studica_control/srv/SetData "{name: 'servo3', component: 'servo', initparams: {pin: 14, servo_type: 'standard'}}" && ros2 service call /create_component studica_control/srv/SetData "{name: 'servo4', component: 'servo', initparams: {pin: 15, servo_type: 'standard'}}"

```


## Controller
* Studica controllers are v1, so several functions will not work such as LED change colour, speakers, audio jack.
* Install ros2 joy package and run joy_node to publish gamepad commands to /joy topic.
* Install ds4drv for more functionality, `ds4drv --hidraw --dump-reports` for hid raw
    * gyroscope control implemented using this. run `ds4drv --hidraw` and `gamepad_gyro_publisher.py` file. Example listener `gyro_drive.py`
    * note change `device_path='/dev/input/event16'` to whatever event number your controller is. `evtest` is a useful tool.  