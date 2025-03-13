# ROS2
This repository provides the essential driver library and dependency sources required. (IMPORTANT: Requires the [Ubuntu VMXPI Library](https://docs.dev.studica.com/en/latest/docs/VMX/os-images.html) installed)

### How to SSH into VMXPi
1. Setup robot network and get IP address of VMXPi
``` bash
sudo apt install net-tools
ifconfig
```
2. Enable SSH via the following commands:
``` bash
sudo apt update
sudo apt update sudo apt install openssh-server -ysudo systemctl enable ssh sudo systemctl start ssh
```
3. SSH from a remote machine
``` bash
ssh vmx@<IP-Address>
```

### Compile & Install
``` bash
sudo su
cd ROS2
chmod +x setup.sh
./setup.sh
colcon build --packages-select studica_control
```

### Quick start – How to get robot moving

1. Run manual composition:
``` bash
sudo su
source install/setup.bash
ros2 run studica_control manual_composition
```
2. Initialize robot drive system

In a separate terminal, run one of the following series of commands.

For differential drive robots:
``` bash
sudo su
source install/setup.bash
ros2 service call /create_component studica_control/srv/SetData "{component: 'diffdrive', initparams: {can_id: 42, motor_freq: 15600, ticks_per_rotation: 1470, wheel_radius: 0.05, wheel_separation: 0.29, left: 2, right: 3, invert_left: False, invert_right: False}}" 
```
where `wheel_seperation`, `wheel_radius` are in meters, and `left`, `right` correspond to titan ports. Set `invert_left` and/or `invert_right` to `True` to reverse motor direction.

For mecanum drive robots:
``` bash
sudo su
source install/setup.bash
ros2 service call /create_component studica_control/srv/SetData "{component: 'mecanum', initparams: {can_id: 42, motor_freq: 15600, ticks_per_rotation: 1120, wheel_radius: 0.05, wheelbase: 0.3, width: 0.28, front_left: 2, front_right: 0, rear_left: 3, rear_right: 1, invert_front_left: False, invert_front_right: False, invert_rear_left: False, invert_rear_right: False}}" 
```
where `wheel_radius`, `wheelbase`, `width` are in meters, and `front_left`, `front_right`, `rear_left`, `rear_right` correspond to titan ports. Set `invert_front_left`, `invert_front_right`, `invert_rear_left`, and/or `invert_rear_right` to `True` to reverse motor direction.

See Service Calls for more info. 

3. Start keyboard controller
``` bash
sudo su
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Service Calls

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

**Component Parameters** (`msg/InitializeParams.msg`)
``` bash
int32 ping
int32 echo
float32 vref
int32 pin
int32 n_encoder
float32 dist_per_tick
float32 speed
float32 ticks_per_rotation 
float32 wheel_radius
float32 wheel_separation
float32 wheelbase
float32 width
int32 can_id
int32 motor_freq
string servo_type
int32 port_a
int32 port_b
int32 front_left # titan motor port id (mecanum)
int32 front_right
int32 rear_left
int32 rear_right
bool invert_front_left # reverses direction of motor (mecanum)
bool invert_front_right
bool invert_rear_left
bool invert_rear_right
int32 left # titan motor port id (diffdrive)
int32 right
bool invert_left # reverses direction of motor (diffdrive)
bool invert_right
```

Values for each parameters can be declared at service requests and responses.

Example for ultrasonic:
``` bash
# initialize component
ros2 service call /create_component studica_control/srv/SetData "{component: 'ultrasonic', initparams: {ping: 10, echo: 11}}"   

# start publishing data to /range topic
ros2 service call /ultrasonic_cmd studica_control/srv/SetData "{params: 'start_publishing'}" 
```
