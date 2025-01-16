# Install and Build Dependencies

This guide provides an overview of the components and libraries used in Apollo Cloud and drive controls. Run components as needed.

## Overview

### ROS2 Packages
* **ROS2 Control**
    * Drive control package. Provides drive controller for differential drive and mecanum drive (coming soon to humble). Used in  [Studica ROS2 Examples](https://github.com/Studica-Robotics/ROS2-Examples). The ROS2 controller listens to `/cmd_vel` topic to move the robot, enabling communication between navigation nodes like NAV2 and m-explore. 
* **SLAM**
    * Mapping node. Used alongside lidar node, listens to `/scan` data topic. Creates a 2D map, updates the `/map` topic. Provides capa to load and save maps.
* **NAV2**
    * Navigation node. Sends `/cmd_vel` topic and listens to odometry data to move your robot and correct paths. Calculates paths to a target position listening to `/map` (either loaded or live-mapped by SLAM). Has localization features. 
* **rosbridge**
    * Relays ROS2 network through a port, allowing a web-based front-end to recieve and send data. Used in Apollo-Cloud alongside `roslib.js`. I also recommend https://app.foxglove.dev/ + rosbridge as an alternative to RVIZ.
    * Must run if using frontend.
* **RVIZ**
    * Should be included with ROS2 install. Used to view map, odometry, camera feed etc. Very laggy sometimes, NEVER run on raspberry pi.
    * `ros2 run rviz2 rviz2`
* **Joy**
    * Gamepad Node. Listens to gamepad events and publishes them to `/joy` topic. Run this node when using a controller. (This node is a block in Apollo Cloud).

### External Drivers
* **YDLidar**
    * Driver code for lidar. Initializes 2D scanning of the environment, and publishes the data to the `/scan` topic, which are used for SLAM.
* **Orbbec Depth Camera**
    * ROS2 wrapper for Orbbec depth camera. Allows access to camera data, such as Publishes 3 camera feeds: Color, depth, and IR, 3D Point Cloud generation, etc. The camera stream is used in `Camera` tab of Apollo Cloud. 

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
ROS2 Control is used to control robot hardware, such as differential-drive robots. Note mecanum drive source code has been released but the humble package ain't. Try compiling from source https://github.com/ros-controls/ros2_control.

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
Compressed camera feed. A compressed feed is used in Apollo
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
```bash
cd ros_dependencies/depth_camera
sudo dpkg -i ros-humble-orbbec-camera_jammy_arm64.deb
sudo dpkg -i ros-humble-orbbec-camera-msgs_jammy_arm64.deb
```

#### Running
```bash
ros2 launch orbbec_camera gemini_e.launch.py
```


## Lidar Drivers
### YDLiDAR SDK
Compile and install the [YDLiDAR SDK](https://github.com/YDLIDAR/YDLidar-SDK/blob/master/doc/howto/how_to_build_and_install.md) to your environment first in order to use the ROS2 driver for the lidar.

### YDLiDAR ROS2 Driver
Clone and build the [YDLiDAR ROS2 driver](https://github.com/YDLIDAR/ydlidar_ros2_driver) on the `humble` branch to use the LiDAR driver as a ROS2 node.
```bash
# important - checkout the humble branch after cloning
git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git ydlidar_ros2_ws/src/ydlidar_ros2_driver
cd ydlidar_ros2_ws/src/ydlidar_ros2_driver
git checkout origin/humble
cd ../../../
# continue from step 2 in the README
```

#### Running
    ```bash
        ros2 launch ydlidar_ros2_driver ydlidar_launch_view.py # launches RVIZ as well
    ```
**Note:**
* Depending on the type of lidar you use, replace the ydlidar.yaml file at `ydlidar_ros2_driver/params/ydlidar.yaml` before building.
* Yellow LiDAR: Change ydliar.yaml to X2.yaml in `ydlidar_launch_view.py`.
* Black LiDAR: Change ydliar.yaml to TminiPro.yaml in `ydlidar_launch_view.py`. (Ask James if it doesn't work)


## Controller
* Studica controllers are v1, so several functions will not work such as LED change colour, speakers, audio jack.
* Install the joy package and run `joy_node` to publish gamepad commands to `/joy` topic.
    ```bash
    # Install
    sudo apt install ros-humble-joy

    # Run
    ros2 run joy joy_node

    # Note: there is no output until a controller is connected.
    ```
* Install ds4drv for more functionality, `ds4drv --hidraw --dump-reports` for hid raw
    * gyroscope control implemented using this. run `ds4drv --hidraw` and `gamepad_gyro_publisher.py` file. Example listener `gyro_drive.py`
    * note change `device_path='/dev/input/event16'` to whatever event number your controller is. `evtest` is a useful tool.  



### Running Manual Composition Service:
Run this to initialize usage of titan and servos.
```bash
ros2 service call /create_component studica_control/srv/SetData "{component: 'titan', initparams: {can_id: 42, motor_freq: 15600, dist_per_tick: 0.0006830601, speed: 0.8}}" && \
ros2 service call /create_component studica_control/srv/SetData "{name: 'servo1', component: 'servo', initparams: {pin: 13, servo_type: 'standard'}}" && \
ros2 service call /create_component studica_control/srv/SetData "{name: 'servo2', component: 'servo', initparams: {pin: 12, servo_type: 'standard'}}" && \
ros2 service call /create_component studica_control/srv/SetData "{name: 'servo3', component: 'servo', initparams: {pin: 14, servo_type: 'standard'}}" && \
ros2 service call /create_component studica_control/srv/SetData "{name: 'servo4', component: 'servo', initparams: {pin: 15, servo_type: 'standard'}}"
```
``


## Example workflow
Launch a robot that you can drive around with the controller and watch the video feed.

/--

```
run manual composition
run camera driver

send service to create components 
run joy_node
run your apollo program

drive robot with controller
```

/--

```bash
# Terminal 1:
sudo su
source ROS2/src/studica_control/install/setup.bash
ros2 run studica_control manual_composition

# Terminal 2:
ros2 launch orbbec_camera gemini_e.launch.py

# Terminal 3:
sudo su
source ROS2/src/studica_control/install/setup.bash
ros2 service call /create_component studica_control/srv/SetData "{component: 'titan', initparams: {can_id: 42, motor_freq: 15600, dist_per_tick: 0.0006830601, speed: 0.8}}" && \
ros2 service call /create_component studica_control/srv/SetData "{name: 'servo1', component: 'servo', initparams: {pin: 13, servo_type: 'standard'}}" && \
ros2 service call /create_component studica_control/srv/SetData "{name: 'servo2', component: 'servo', initparams: {pin: 12, servo_type: 'standard'}}" && \
ros2 service call /create_component studica_control/srv/SetData "{name: 'servo3', component: 'servo', initparams: {pin: 14, servo_type: 'standard'}}" && \
ros2 service call /create_component studica_control/srv/SetData "{name: 'servo4', component: 'servo', initparams: {pin: 15, servo_type: 'standard'}}"

# Terminal 4:
sudo su
source ROS2/src/studica_control/install/setup.bash
ros2 run joy joy_node

# Terminal 5:
sudo su
source ROS2/src/studica_control/install/setup.bash
cd Apollo-Cloud/Teddy/server/projects/pr/pr
python3 p.py
```

## Example workflow 2
Launch an autonomous mapping robot using discovery.

/--

```
run drive controller
run lidar driver
run SLAM
run nav2

run exploration
```

/--

```bash
# Terminal 1:
ros2 launch studica_drive_control diffdrive.launch.py
( this launch file launches lidar and slam)

# Terminal 2
ros2 launch nav2_bringup navigation_launch.py

# Terminal 3:
ros2 launch explore_lite explore.launch.py

# Terminal 4:
ros2 run rviz2 rviz2
```
