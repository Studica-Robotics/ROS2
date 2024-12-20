# Install and Build Dependencies

This guide provides an overview of the components and libraries used in Apollo Cloud and drive controls. 

## Overview

### ROS2 Packages
* **ROS2 Control**
    * Drive control package. Provides drive controller for differential drive and mecanum drive (coming soon to humble). Used in  [Studica ROS2 Examples](https://github.com/Studica-Robotics/ROS2-Examples). The ROS2 controller listens to `/cmd_vel` topic to move the robot, enabling communication between navigation nodes like NAV2 and m-explore. 
* **SLAM**
    * 
* **NAV2**
    * 
* **rosbridge**
    * 
* **Joy**
    * 

### External Drivers**
* YDLidar: Driver code for lidar. Init
* Orbbec Depth Camera

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


### ROS2 Control (Differential Drive)
ROS2 Control is used to control robot hardware, such as differential-drive robots.

#### Installation

```bash
sudo apt install ros-humble-ros2-control 
sudo apt install ros-humble-ros2-controllers
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
#### Installation
Installation is so rough with this one [Orbbec Depth Camera SDK](https://github.com/orbbec/OrbbecSDK_ROS2). It never builds and crashes every time. Lucky for you, I made packages for the VMXPi so you can avoid building from source.
#### Build
Follow instructions [Orbbec Depth Camera SDK](https://github.com/orbbec/OrbbecSDK_ROS2).
#### PKG install
For PI only. Build from source if you want drivers on your machine.
```
cd ros_dependencies/depth_camera
sudo dpkg -i ros-humble-orbbec-camera_jammy_arm64.deb
sudo dpkg -i ros-humble-orbbec-camera-msgs_jammy_arm64.deb
```

#### Running
```bash
ros2 launch orbbec_camera gemini_e.launch.py
```


## Lidar Drivers
### [ydlidar SDK](https://github.com/YDLIDAR/YDLidar-SDK/blob/master/doc/howto/how_to_build_and_install.md)


### [ydlidar ROS2 driver](https://github.com/YDLIDAR/YDLidar-SDK/blob/master/doc/howto/how_to_build_and_install.md)

Depending on the Lidar you use, replace the ydlidar.yaml file at
/ydlidar_ros2_driver/params/ydlidar.yaml before building.

Yellow LiDAR: change ydliar.yaml to X2.yaml.
Black LiDAR: ask james

#### Running
```bash
ros2 launch ydlidar_ros2_driver ydlidar_launch_view.py
```


### Running Manual Composition Service:
```bash
ros2 service call /create_component studica_control/srv/SetData "{component: 'titan', initparams: {can_id: 42, motor_freq: 15600, dist_per_tick: 0.0006830601, speed: 0.8}}" && \
ros2 service call /create_component studica_control/srv/SetData "{name: 'servo1', component: 'servo', initparams: {pin: 13, servo_type: 'standard'}}" && \
ros2 service call /create_component studica_control/srv/SetData "{name: 'servo2', component: 'servo', initparams: {pin: 12, servo_type: 'standard'}}" && \
ros2 service call /create_component studica_control/srv/SetData "{name: 'servo3', component: 'servo', initparams: {pin: 14, servo_type: 'standard'}}" && \
ros2 service call /create_component studica_control/srv/SetData "{name: 'servo4', component: 'servo', initparams: {pin: 15, servo_type: 'standard'}}"
```
``


## Controller
* Studica controllers are v1, so several functions will not work such as LED change colour, speakers, audio jack.
* Install ros2 joy package and run joy_node to publish gamepad commands to /joy topic.
* Install ds4drv for more functionality, `ds4drv --hidraw --dump-reports` for hid raw
    * gyroscope control implemented using this. run `ds4drv --hidraw` and `gamepad_gyro_publisher.py` file. Example listener `gyro_drive.py`
    * note change `device_path='/dev/input/event16'` to whatever event number your controller is. `evtest` is a useful tool.  


## Example workflow
Launch the camera node, LiDAR node, and the titan node using manual_composition.

/--

```
run manual composition
srun lidar driver
run camera driver
run slam
run nav2

send service to create components 
open rviz
run exploration
```

/--

```bash
# Terminal 1:
ros2 launch ydlidar_ros2_driver ydlidar_launch_view.py

# Terminal 2:
ros2 launch orbbec_camera gemini_e.launch.py

# Terminal 3:
ros2 run studica_control manual_composition

# Terminal 4:
ros2 service call /
```