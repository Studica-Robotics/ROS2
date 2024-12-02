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
cd src/studica_control
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


## Notes
* Usually run everything in `sudo su` mode or else HAL wont work
* The src/CMakeLists.txt file is very important for everything to compile. Edit this file to set up include files, libraries, deps, components, executables, install, linking etc..
* error while loading shared libraries: libstudica_drivers.so: cannot open shared object file: No such file or directory

fix: `export LD_LIBRARY_PATH=/usr/local/lib/studica_drivers:$LD_LIBRARY_PATH`
* old implementation: studica_control_node


### How to run nodes
building
```
# source your ROS e.g. 
# . /opt/ros/humble/setup.bash
cd src/studica_control
colcon build
```
(requires drivers to be installed first, run make and make install in ../drivers folder)

Running nodes
```
. instal/setup.bash
ros2 run studica_control manual_composition
ros2 run studica_control studica_control_node
```