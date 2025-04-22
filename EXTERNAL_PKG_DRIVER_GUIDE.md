# ROS2 Packages and External Drivers

This guide provides an overview of the ROS2 packages and external drivers used in Studica's ROS2 repository. All packages and drivers listed here are automatically installed using the setup script in the compile and install instructions.

## ROS2 Packages

### SLAM
SLAM (Simultaneous Localization and Mapping) is used for mapping the environment using lidar sensors.
Used alongside lidar node, listens to `/scan` and `/odom` topics. Creates a 2D map, updates the `/map` topic. Provides capability to load and save maps.
#### Running
```
ros2 launch slam_toolbox online_sync_launch.py
```
### NAV2
NAV2 is used for robot navigation, allowing path generation and following. Sends `/cmd_vel` topic and listens to odometry data to move your robot and correct paths. Calculates paths to a target position listening to `/map` (either loaded or live-mapped by SLAM). Has localization features. 
#### Running
```
ros2 launch nav2_bringup navigation_launch.py
```
### rosbridge
Relays ROS2 network through a port, allowing a web-based front-end to recieve and send data. Used in Apollo-Cloud alongside `roslib.js`.
Must run rosbridge to make frontend fully functional.
#### Running
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml 
```
### RVIZ
Should be included with ROS2 install. Used to view map, odometry, camera feed etc. 
Use https://app.foxglove.dev/ as an alternative to RVIZ.
#### Running
```
ros2 run rviz2 rviz2
```
### Joy
Gamepad Node. Listens to gamepad events and publishes them to `/joy` topic. Run this node when using a controller. (This node is a block in Apollo Cloud). Studica controllers are v1, so several functions will not work such as LED change colour, speakers, audio jack. Note: there is no output until a controller is connected.
#### Running
```
ros2 run joy joy_node
```
* Install ds4drv for more functionality, `ds4drv --hidraw --dump-reports` for hid raw
    * gyroscope control implemented using this. run `ds4drv --hidraw` and `gamepad_gyro_publisher.py` file. Example listener `gyro_drive.py`
    * note change `device_path='/dev/input/event16'` to whatever event number your controller is. `evtest` is a useful tool. 

## External Drivers
* **YDLidar**
    * Driver code for lidar. Initializes 2D scanning of the environment, and publishes the data to the `/scan` topic, which are used for SLAM.
* **Orbbec Depth Camera**
    * ROS2 wrapper for Orbbec depth camera. Allows access to camera data, such as Publishes 3 camera feeds: Color, depth, and IR, 3D Point Cloud generation, etc. The camera stream is used in `Camera` tab of Apollo Cloud. 


## Camera Drivers

#### Installation
The setup script will install the Camera driver. For manual installation, see below:

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
Ensure rosbridge is running to see camera feed in the camera tab.
```bash
ros2 launch orbbec_camera gemini_e.launch.py
```


## Lidar Drivers
The setup script will install the Lidar driver. For manual installation, see below:

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
        ros2 launch ydlidar_ros2_driver ydlidar_launch.py
        ros2 launch ydlidar_ros2_driver ydlidar_launch_view.py # if you want to launch RVIZ too
    ```
**Note:**
* Depending on the type of lidar you use, replace the ydlidar.yaml file at `ydlidar_ros2_driver/params/ydlidar.yaml` before building.
* Yellow LiDAR: Change ydliar.yaml to X2.yaml in `ydlidar_launch_view.py`.
* Black LiDAR: Change ydliar.yaml to TminiPro.yaml in `ydlidar_launch_view.py`. 
