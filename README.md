# ROS2
This repository provides the essential driver library and dependency sources required for running your Studica robot. (IMPORTANT: Requires the [Ubuntu VMXPI Library](https://docs.dev.studica.com/en/latest/docs/VMX/os-images.html) installed)

### How to SSH into VMXPi
1. Setup robot network and get IP address of VMXPi (easiest method is to hook it up to a monitor, mouse, and keyboard). Get the IP address with the following command:
``` bash
sudo apt install net-tools
ifconfig
```
2. Enable SSH (Secure Shell) via the following commands:
``` bash
sudo apt update
sudo apt update sudo apt install openssh-server -ysudo systemctl enable ssh sudo systemctl start ssh
```
3. SSH into the VMXPi from a remote machine. Run this command on your remote machine:
``` bash
ssh vmx@<IP-Address>
```

### Compile & Install
The setup script will install the necessary ROS2 drivers and dependencies. The following commands will execute the setup script:
``` bash
sudo su
cd ROS2
chmod +x setup.sh
./setup.sh
```

You are now ready to go! See Getting Started for configuring your robot and launching ROS2.

## 🚀 Getting Started

### 1. Configure your Robot
All the robot's hardware components (e.g., ultrasonic sensors, drive controllers, servos) are defined in a YAML configuration file at `config/params.yaml`.

- For each component, set `enabled: true` to activate it. And set `enabled: false` if you don't have that component or you don't want to use it.
- You can initialize multiple of the same sensor too. List the sensors or devices under the sensors field and define their individual parameters below it. See an example at the top of `params.yaml`.

### 2. Launch the System
Once everything is configured, launch the robot:

First, build the ROS2 package with the following command:
``` bash
colcon build --executor sequential --packages-select studica_control
```
Everytime the package is built, make sure to source the install folder:
``` bash
source install/setup.bash
```
Now the robot is ready to be launched:
``` bash
ros2 launch studica_control <LAUNCH_FILE.py>
```

### 3. View data
Each of the components publish data to their own ROS2 topic. To view the data from a specific topic, follow these steps:

First, list all of the available topics:
``` bash
ros2 topic list
```
Then, choose one to view (ex. imu):
``` bash
ros2 topic echo <topic-name>
```
You should now be able to view the published data for that component! There is oftentimes a lot of data streaming at a very high rate, so use a ROS2 visualizer such as RViz or Foxglove. This will make it much easier to view your data more intuitively.

### 4. Foxglove
To visualize data with foxglove, we can:

``` bash
source install/setup.bash 
```
Then:
``` bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```
Now, go to foxglove and select localhost:8765
## 🤖 Example – How to get robot moving 

### 1. Enable a drive controller component

Depending on your robot setup, initialize a differential drive component (2 wheels) or a mecanum drive component (4 wheels). To do this, enable a drive controller, set the appropriate parameters for your robot, and optionally, enable the imu. It's as simple as that. If you have any other robot components, you can also enable them now and set their correct parameters. Note: make sure both drive controllers aren't enabled at the same time.

For example:
``` bash
diff_drive_component:
      enabled: true # change to true
      name: "diff_drive_controller"
      can_id: 42
      motor_freq: 15600
      ticks_per_rotation: 1470
      left_port: 2
      right_port: 3
      invert_left: false
      invert_right: false
      wheel_radius: 0.05 # in metres
      wheel_separation: 0.28 # in metres
```
Under the corresponding odometry, you can set what topic the odometry is published to and whether to use the imu data for odometry calculations.
``` bash
diff_drive_odometry:
      name: "differential_odom"
      use_imu: false
      imu_topic: "imu"
      topic: "odom"
```

### 2. Launch the robot
``` bash
colcon build
. install/setup.bash
ros2 launch studica_control studica_launch.py
```

### 3. Start keyboard controller

In another terminal:
``` bash
sudo su
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Then, following the prompted instructions for how to drive your robot around!

### 4. Display odometry data

In another terminal:
``` bash
sudo su
ros2 topic echo /odom
```
You should now see the published odometry data! 

## Service Calls

Services use parameters defined in the `.srv` file, while nodes use those defined in the `.msg` file.

**Service Parameters** (`srv/SetData.srv`)
``` bash
string command
string name
string component
string params
InitializeParams initparams # see Component Parameters below
---
bool success
string message
```

Example for servo component:

To change the angle of your servo, use the following command:
``` bash  
ros2 service call /<servo_name>/set_servo_angle studica_control/srv/SetData "{params: '45'}"
```

Example for DIO component:

If the DIO is on output mode, you can change the DIO state using the following command:
``` bash  
ros2 service call /dio_cmd studica_control/srv/SetData "{params: 'toggle'}"
```

### Notes:
#### Drivers
If there are any changes to the drivers, you must install them again before launching.
```bash
cd drivers
make
make install
```

#### Odometry
To use differential or mecanum drive odometry independent of their drive controllers, 
1. Create a shared pointer to the odometry
2. Pass shared pointer to an executor
3. Call its init function
4. In a timer, call the update function with updated distance values

#### Known Issues

When running lidar and explore scripts, sometimes their respective install scripts aren't sourced properly. Manually source these if necessary:

``` bash
. /home/vmx/ROS2/ros_dependencies/lidar/ydlidar_ros2_ws/install/setup.bash
. /home/vmx/ROS2/ros_dependencies/m-explore-ros2/install/setup.bash
```

Nav2 Issue - https://robotics.stackexchange.com/questions/114875/ros2-running-nav2-and-odom-frame-is-moving-excessively-causing-navigation-to-fa

## Pick your launch file
before running any launch file, make sure to build the ROS2 package and source the install folder.

### Studica_launch.py
- default launch
- basic function in manual_composition.cpp (lidar, gamepad & keyboard control, blocky control)

### mapping2_launch.py
- launch file for mapping
- dual lidar
- 4 Transform Publishers
- Laser Scan Merger
- Foxglove visualization

drive the robot around to map the surronding and use the following command to save the map:

``` bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/home/vmx/ROS2/maps/YOUR_MAP'}}"
```

alternate methods:

``` bash
ros2 run nav2_map_server map_saver_cli -f /home/vmx/ROS2/maps/YOUR_MAP
```

``` bash
rviz2 -d `ros2 pkg prefix slam_toolbox`/share/slam_toolbox/rviz/YOUR_MAP.rviz
```

### office_mapping_launch.py
- Specialized launch file for office environment mapping
- SLAM toolbox integration with custom parameters (office_mapper_params.yaml)
- includes every feature of mapping2_launch.py(except gamepad control)

### nav2_launch.py
- Comprehensive navigation setup with dual LiDAR support
- Nav2 localization and navigation
- 2 Transform Publishers
- ROS bridge for web interfaces
- includes every feature of mapping2_launch.py(except gamepad control)
- Map Loading

### tf_launch.py
- Minimal launch file for transforms and manual composition
- Base transform publishers
- Modular design for use with external LiDAR launches
- 2 Transform Publishers

