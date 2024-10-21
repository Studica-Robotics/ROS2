# ROS2
ROS2 for the VMX

## Compile & Install Drivers
``` bash
cd drivers
make
make install
```

## Build Nodes
``` bash
sudo su
. install/setup.bash
colcon build
```

## Run
``` bash
# if you haven't already, run as super user 
sudo su
. install/setup.bash
ros2 run studica_control studica_control_node
```