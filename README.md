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
. /opt/ros/humble/setup.bash
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
**Building**
```bash
# source your ROS e.g. 
. /opt/ros/humble/setup.bash
cd src/studica_control
colcon build
```
(requires drivers to be installed first, run make and make install in ../drivers folder)

**Running nodes**

Terminal 1:
```bash
. /opt/ros/humble/setup.bash
. install/setup.bash
ros2 run studica_control manual_composition
#ros2 run studica_control studica_control_node
```
Terminal 2:
```bash
. /opt/ros/humble/setup.bash
. install/setup.bash
ros2 service call /create_component studica_control/srv/SetData '{<...SetData service request params...>}' # init component
ros2 service call /<service name> studica_control/srv/SetData '{<...SetData service request params...>}' # make subsequent service calls to component
```

---
### Parameters

**Service Parameters** (`srv/SetData.srv`)
```
string command
string name
string component
string params
InitializeParams initparams
---
bool success
string message
```

**Component Parameters** (`msg/InitializeParams.msg`)
```
int32 ping
int32 echo
float32 vref
int32 mux_ch
int32 pin
int32 n_encoder
float32 dist_per_tick
float32 speed
int32 can_id
int32 motor_freq
string servo_type
int32 port_a
int32 port_b
```