# ROS2
This repository provides the essential driver library and dependency sources required. (Requires [VMXPI Library](https://example.com) installed)


## Driver library
Code that directly controls the robot's components, such as servo motors, ultrasonic sensors, and other hardware. This driver library can be used in a ROS2 node, or in plain C++ code.

To do so, make sure to link the driver library.

### Compile & Install
``` bash
cd drivers
make
make install
```
### Example Usage
C++ Examples of using the drivers can be found and built in the examples folder. See 

### Linking Driver Library
To use the driver code in a different project you must link the library .so file.

#### Install Location:
```bash
INCLUDE_INSTALL_PATH = /usr/local/include/studica_drivers # header files
LIB_INSTALL_PATH = /usr/local/lib/studica_drivers/ # .so file
```
#### Linking (CMakeLists.txt):
```bash
# CMakeLists.txt
include_directories( 
    /usr/local/include/vmxpi 
    /usr/local/include/studica_drivers
)
...
target_link_libraries(example_program
    /usr/local/lib/vmxpi/libvmxpi_hal_cpp.so
    /usr/local/lib/studica_drivers/libstudica_drivers.so
)
```
#### Add library path for the linker to search:
You should also add these lines to your .bashrc file to avoid running it every time
```bash
export LD_LIBRARY_PATH=/usr/local/lib/studica_drivers:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/studica_control
```

## ROS2 Components
Components can be loaded in a ROS2 node. This project contains a collection of components use the C++ drivers. To launch them, either use manual_composition or manually launch them individually via terminal. 

**Building**
```bash
# use sudo su mode since HAL requires root permissions
sudo su
# source your ROS e.g. 
. /opt/ros/humble/setup.bash
cd src/studica_control
colcon build
```
(requires drivers to be installed first, run make and make install in ../drivers folder)

**Running nodes**

Terminal 1:
```bash
sudo su
. /opt/ros/humble/setup.bash
cd src/studica_control
. install/setup.bash
ros2 run studica_control manual_composition
```
Terminal 2:
```bash
. /opt/ros/humble/setup.bash
. install/setup.bash
ros2 service call /create_component studica_control/srv/SetData '{<...SetData service request params...>}'
. /opt/ros/humble/setup.bash
. install/setup.bash
ros2 service call /create_component studica_control/srv/SetData '{<...SetData service request params...>}' # init component
ros2 service call /<service name> studica_control/srv/SetData '{<...SetData service request params...>}' # make subsequent service calls to component
```

---
### Parameters  
Datatypes used for communication in nodes. Services use parameters defined in the `.srv` file, while nodes use those defined in the `.msg` file.

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


Values for each parameters can be declared at service requests and responses.
```bash
    ros2 service call /titan_cmd studica_control/srv/SetData "{name: 'tan', component: 'titan', params: 'reset', initparams: {n_encoder: 1, can_id: 45, motor_freq: 15600, dist_per_tick: 0.0006830601, speed: 0.2}}"
```

## Notes
* Usually run everything in `sudo su` mode or else HAL won't work
* The src/CMakeLists.txt file is very important for everything to compile. Edit this file to set up include files, libraries, deps, components, executables, install, linking etc..
* error while loading shared libraries: libstudica_drivers.so: cannot open shared object file: No such file or directory

fix: 
```bash
export LD_LIBRARY_PATH=/usr/local/lib/studica_drivers:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/studica_control
```
* old implementation: studica_control_node


## Development Status
### Drivers
#### Steady
* Analog Input
* Cobra
* DIO
* Titan
* Servo
* Sharp
* Ultrasonic
* IMU
* I2C

#### Revise
* WatchDog (can't trigger hardware watchdog)
* PWM (DNE)
* Encoder (inconsistent, buggy)

### ROS2 Components
#### Revise
> * Porting components from studica_control_node to manual_composition
> * Titan (should be tied to the watchdog, ie. check if the watchdog is fed)
> * Add to manual_composition node, refactor components to not use VMXManager. 
> * Additions: add publisher to components to replace the service response where it makes sense. (e.g. sharp getDistance() -> startPublishingDistance())
    
    * ultrasonic_component
    * sharp_component
    * cobra_component
    * imu_component
