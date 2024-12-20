# RTAB 3D scan and localization
Create a 3D map of your environment using the orbbec depth camera.
Many other rtabmap examples can be found [here](https://github.com/introlab/rtabmap_ros/tree/ros2/rtabmap_examples/launch), including ones that use LIDAR. You can run the camera on your robot, then see the image and depth topics being published if your local machine is on the same network as the raspberry pi.
It is recommended to run RTAB on your local machine for performance and visualizations.
#
![RTAB 3D scan and localization](https://imgur.com/TKmtMh2.png)
RVIZ running on laptop.
## Install
```
sudo apt install ros-humble-rtabmap-ros
```
## Running
```
python3 rtab.launcher.py
```

Depth camera must be running (either on your local machine or rpi)
```
ros2 launch orbbec_camera gemini_e.launch.py
```

## Notes
This launch file creates it's own odometry data using a rtab package that calculates odometry visually. 