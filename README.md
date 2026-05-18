# Studica Robotics ROS2 Driver

A ROS2 hardware abstraction layer for the **Studica Robotics VMX** platform. Each hardware component — motor controllers, sensors, servos, encoders, and gamepad input — is an independently configurable ROS2 node. Enable only what your robot uses; everything else stays off.

**Repository:** https://github.com/Studica-Robotics/ROS2

---

## Table of Contents

- [Hardware Support](#hardware-support)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Configuration](#configuration)
- [Building](#building)
- [Running](#running)
- [Component Reference](#component-reference)
- [Python Examples](#python-examples)
- [C++ Examples](#c-examples)
- [Standalone Driver Examples](#standalone-driver-examples)
- [Integrating into Your Project](#integrating-into-your-project)

---

## Hardware Support

| Component | Type | Description |
|---|---|---|
| **Titan** | Motor Controller | CAN bus DC motor controller, up to 4 motors per unit |
| **IMU** | 9-axis Sensor | Onboard NavX orientation, velocity, and acceleration |
| **Encoder** | Position Sensor | Quadrature encoder via two digital input channels |
| **DutyCycleEncoder** | Absolute Encoder | Absolute position from PWM duty cycle |
| **Servo** | Actuator | Standard (position), continuous (velocity), or linear |
| **Ultrasonic** | Range Sensor | HC-SR04-style sonar, ~2 cm to 4 m |
| **Sharp** | Range Sensor | GP2Y infrared rangefinder, ~10 cm to 80 cm |
| **DIO** | Digital I/O | General-purpose digital input or output pin |
| **Cobra** | Reflectance Array | 4-channel analog line/surface sensor over I2C |
| **Gamepad** | Input | Joystick/gamepad to `cmd_vel` via `joy` node |

---

## Prerequisites

### System Requirements

- **Hardware:** Studica Robotics VMX (Raspberry Pi-based robotics controller)
- **OS:** Ubuntu 22.04 (Jammy) — pre-installed on the VMX OS image
- **ROS2:** [Humble Hawksbill](https://docs.ros.org/en/humble/Installation.html) (LTS)

### Required SDKs

**VMX HAL** — low-level hardware access library, pre-installed on the official VMX OS image:

> https://learn.studica.com/docs/ws/vmx/os-images

**Studica Drivers** — bundled in this repository's `drivers/` folder. Install steps are in [Installation](#installation) below.

### ROS2 Dependencies

```bash
sudo apt update
sudo apt install -y \
  ros-humble-rclcpp \
  ros-humble-rclcpp-components \
  ros-humble-sensor-msgs \
  ros-humble-geometry-msgs \
  ros-humble-std-msgs \
  ros-humble-rmw-cyclonedds-cpp \
  ros-humble-joy          # only needed for gamepad support
```

CycloneDDS is required because the node runs as root (for pigpio hardware access) and Fast-DDS shared memory segments created by root are not accessible to non-root ROS2 tools (`ros2 topic echo`, etc.). CycloneDDS uses UDP sockets which work correctly across privilege boundaries.

---

## Installation

### 1. Clone the repository

```bash
git clone https://github.com/Studica-Robotics/ROS2.git ~/studica_ws
cd ~/studica_ws
```

Or add the package to an existing workspace:

```bash
cd ~/your_ws/src
git clone https://github.com/Studica-Robotics/ROS2.git studica_ros2
ln -s studica_ros2/src/studica_control studica_control
```

### 2. Build and install the driver library

```bash
cd ~/studica_ws/drivers
make
sudo make install
# Installs headers to /usr/local/include/studica_drivers/
# Installs library to /usr/local/lib/studica_drivers/ and runs ldconfig
cd ~/studica_ws
```

---

## Configuration

All configuration lives in one file:

```
src/studica_control/config/params.yaml
```

**The driver only starts components you explicitly enable.** Everything defaults to `enabled: false`.

### Quick Start

Copy the full reference template to `params.yaml` and enable what you need:

```bash
cp src/studica_control/config/params_template.yaml \
   src/studica_control/config/params.yaml
```

Then edit `params.yaml`. Here is a typical robot with two Titan motor controllers, an IMU, and an ultrasonic sensor:

```yaml
control_server:
  ros__parameters:

    titan:
      enabled: true
      sensors: ["drive", "arm"]
      drive:
        can_id: 42
        motor_freq: 15600
        m_0:
          encoder_mode: "quadrature"
          dist_per_tick: 0.0006830601
        m_1:
          encoder_mode: "quadrature"
          dist_per_tick: 0.0006830601
        m_2:
          encoder_mode: "quadrature"
          dist_per_tick: 0.0006830601
        m_3:
          encoder_mode: "quadrature"
          dist_per_tick: 0.0006830601
      arm:
        can_id: 43
        motor_freq: 15600
        m_0:
          encoder_mode: "quadrature"
          dist_per_tick: 0.0006830601
        m_1:
          encoder_mode: "absolute"   # Cypher encoder -> publishes /arm/m_1/angle
        m_2:
          encoder_mode: "quadrature"
          dist_per_tick: 0.0006830601
        m_3:
          encoder_mode: "quadrature"
          dist_per_tick: 0.0006830601

    imu:
      enabled: true
      name: "imu"
      topic: "imu"
      frame_id: "imu_link"

    ultrasonic:
      enabled: true
      sensors: ["front_sonar"]
      front_sonar:
        ping: 8
        echo: 9
        frame_id: "front_sonar_link"
```

### Multi-Instance Components

The following components support multiple instances by adding names to the `sensors` list and providing a config block for each name:

`cobra`, `duty_cycle`, `dio`, `encoder`, `servo`, `sharp`, `titan`, `ultrasonic`

```yaml
servo:
  enabled: true
  sensors: ["gripper", "wrist", "elbow"]
  gripper:
    port: 14
    type: "standard"
  wrist:
    port: 15
    type: "continuous"
  elbow:
    port: 16
    type: "linear"
```

Topics are auto-generated: `/gripper/cmd`, `/gripper/state`, `/wrist/cmd`, `/wrist/state`, etc.

### Single-Instance Components

`gamepad` and `imu` have a flat configuration (no `sensors` list).

---

## Building

### 1. Set up your shell environment (once)

Add ROS2, the workspace, and CycloneDDS to `~/.bashrc` so every terminal is ready automatically:

```bash
echo -e '\nsource /opt/ros/humble/setup.bash\n[ -f ~/studica_ws/install/setup.bash ] && source ~/studica_ws/install/setup.bash\nexport RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
source ~/.bashrc
```

> The workspace source is guarded — it is silently skipped on a fresh image before the first build, then activates automatically once `colcon build` has run.

### 2. Build

```bash
cd ~/studica_ws
colcon build --packages-select studica_control
```

### 3. Grant hardware permissions (once)

The VMX HAL uses `pigpio` for direct hardware access, which requires root. Run the setup script once after your first build:

```bash
./scripts/setup_permissions.sh
```

> Pass your workspace path if it differs from `~/studica_ws`:
> ```bash
> ./scripts/setup_permissions.sh /path/to/your_ws
> ```

| Step | What it does | Persists? |
|---|---|---|
| sudoers rule | Allows `ros2 launch` to run the node as root without a password prompt | Yes — run once |
| ldconfig | Registers ROS2 and workspace library paths so `sudo` can find them (sudo resets `LD_LIBRARY_PATH`) | Yes — run once |
| udev rule | Makes `/dev/i2c-*` accessible | Yes — run once |

All steps are persistent across reboots and rebuilds. You do not need to re-run this script after `colcon build`.

> **Note:** After editing `params.yaml` only, you do not need to rebuild — just relaunch.

---

## Running

### Launch the Driver

```bash
ros2 launch studica_control studica_launch.py
```

This reads `params.yaml`, initializes every enabled component, and starts publishing sensor data.

To use a config file from your own project instead of the driver's default:

```bash
ros2 launch studica_control studica_launch.py params_file:=/path/to/your/params.yaml
```

### With Gamepad Support

The gamepad component requires a separate `joy` node. Run in separate terminals:

```bash
# Terminal 1 — main HAL
ros2 launch studica_control studica_launch.py

# Terminal 2 — joystick input
ros2 run joy joy_node
```

To find your controller's axis indices before configuring `params.yaml`:

```bash
ros2 topic echo /joy
```

Move each stick and note which `axes[]` index changes.

### Verify Sensor Output

```bash
# List all active topics
ros2 topic list

# Watch IMU data
ros2 topic echo /imu

# Watch motor 0 encoder distance on a Titan named "drive"
ros2 topic echo /drive/m_0/encoder

# Watch motor 0 RPM on the same Titan
ros2 topic echo /drive/m_0/rpm

# Watch ultrasonic range (topic is /<name>/range)
ros2 topic echo /front_sonar/range
```

---

## Component Reference

### Titan — CAN Bus Motor Controller

Topics are auto-generated from the sensor name. Using `"titan0"` as an example:

**Topics (subscribe) — command inputs, always present:**

| Topic | Type | Description |
|---|---|---|
| `/titan0/m_0/cmd` | `std_msgs/Float64` | Duty cycle motor 0, −1.0 to 1.0 |
| `/titan0/m_1/cmd` | `std_msgs/Float64` | Duty cycle motor 1 |
| `/titan0/m_2/cmd` | `std_msgs/Float64` | Duty cycle motor 2 |
| `/titan0/m_3/cmd` | `std_msgs/Float64` | Duty cycle motor 3 |

**Topics (publish) — feedback at 20 Hz, topics depend on `encoder_mode`:**

*Quadrature mode (default):*

| Topic | Type | Description |
|---|---|---|
| `/titan0/m_N/encoder` | `std_msgs/Float64` | Encoder distance (units set by `dist_per_tick`) |
| `/titan0/m_N/rpm` | `std_msgs/Float64` | Motor RPM |

*Absolute mode (Cypher encoder):*

| Topic | Type | Description |
|---|---|---|
| `/titan0/m_N/angle` | `std_msgs/Float64` | Absolute angle in degrees |

Encoder mode is set **per motor** in `params.yaml`. Only the topics for the configured mode are created — the other topics do not exist on the bus at all.

**Service:** `/titan0/titan_cmd` → `studica_control/SetData`

Use the service for configuration and closed-loop control. Direct speed commands should use the `/cmd` topics above.

| Command | Required Fields | Description |
|---|---|---|
| `enable` | — | Power on the controller |
| `disable` | — | Power off; all motors stop immediately |
| `set_speed` | `n_encoder`, `speed` | Set motor duty cycle (−1.0 to 1.0) |
| `set_speed_all` | `speed` | Set all 4 motors to the same duty cycle |
| `stop` | `n_encoder` | Set one motor speed to 0 |
| `disable_motor` | `n_encoder` | Cut power to one motor only |
| `set_target_velocity` | `n_encoder`, `speed` (RPM) | Closed-loop velocity (Titan2 firmware) |
| `set_target_distance` | `n_encoder`, `int_value` | Closed-loop position in encoder counts |
| `set_target_angle` | `n_encoder`, `speed` | Drive to target angle in degrees |
| `set_position_hold` | `n_encoder`, `hold` | Lock motor at current position |
| `configure_encoder` | `n_encoder`, `dist_per_tick` | Override distance per tick at runtime (normally set in params.yaml) |
| `reset_encoder` | `n_encoder` | Zero one encoder |
| `invert_motor` | `n_encoder` | Flip positive direction at runtime (normally set in params.yaml) |
| `get_rpm` | `n_encoder` | Read current RPM |
| `get_encoder_count` | `n_encoder` | Read raw tick count |
| `get_encoder_distance` | `n_encoder` | Read odometry distance |
| `get_firmware_version` | — | Firmware version string |

> **CAN Watchdog:** The Titan enters a safe (stopped) state if no speed command is received within ~150 ms. The driver automatically resends the last commanded speeds at 100 Hz to keep the controller alive.

**params.yaml:**
```yaml
titan:
  enabled: true
  sensors: ["titan0"]
  titan0:
    can_id: 42        # CAN bus ID of the Titan controller
    motor_freq: 15600 # PWM frequency in Hz
    m_0:
      encoder_mode: "quadrature"   # "quadrature" or "absolute"
      dist_per_tick: 0.0006830601  # metres per encoder tick (1.0 = raw counts)
      invert_motor: false          # flip positive motor direction
      invert_encoder: false        # flip encoder count sign
      invert_rpm: false            # flip RPM sign
    m_1:
      encoder_mode: "quadrature"
      dist_per_tick: 0.0006830601
    m_2:
      encoder_mode: "absolute"     # Cypher encoder -> publishes /titan0/m_2/angle
    m_3:
      encoder_mode: "quadrature"
      dist_per_tick: 0.0006830601
```

All per-motor fields default to `encoder_mode: "quadrature"`, `dist_per_tick: 1.0`, and all invert flags `false` if omitted.

---

### IMU — 9-Axis Inertial Measurement Unit

**Topic (publishes):** `<topic>` → `sensor_msgs/Imu`
- Orientation quaternion, angular velocity, linear acceleration
- Rate: 20 Hz

**Service:** `/imu/get_imu_data` → `studica_control/SetData`

| Command | Description |
|---|---|
| `zero_yaw` | Reset the yaw reference to zero at the current heading |
| *(any other string)* | Returns current pitch, yaw, roll as a comma-separated string in `response.message` |

**params.yaml:**
```yaml
imu:
  enabled: true
  name: "imu"
  topic: "imu"
  frame_id: "imu_link"
```

---

### Encoder — Quadrature Encoder

**Topic (publishes):** `<topic>` → `studica_control/EncoderMsg`
- `encoder_count` (int32), `encoder_direction` (string)
- Rate: 20 Hz

**Service:** `/<name>/encoder_cmd` → `studica_control/SetData`

| Command | Description |
|---|---|
| `get_count` | Read current tick count |
| `get_direction` | Read current direction string |

**params.yaml:**
```yaml
encoder:
  enabled: true
  sensors: ["left_wheel"]
  left_wheel:
    port_a: 0   # VMX channel index for phase A
    port_b: 1   # VMX channel index for phase B
    topic: "left_encoder"
```

---

### DutyCycleEncoder — Absolute Encoder

**Topic (publishes):** `<topic>` → `studica_control/DutyCycleEncoderMsg`
- `absolute_angle` (float64), `rollover_count` (int32), `total_rotation` (float64)
- Rate: 20 Hz

**Service:** `/<name>/duty_cycle_encoder_cmd` → `studica_control/SetData`

| Command | Description |
|---|---|
| `get_absolute_position` | Absolute angle within one revolution |
| `get_rollover_count` | Number of complete revolutions |
| `get_total_rotation` | Total rotation across all revolutions |

**params.yaml:**
```yaml
duty_cycle:
  enabled: true
  sensors: ["arm_encoder"]
  arm_encoder:
    port: 8
    topic: "arm_angle"
```

---

### Servo

Topics are auto-generated from the sensor name. Using `"servo"` as an example:

**Topic (subscribes):** `/servo/cmd` → `std_msgs/Float64`
- Directly set the servo target value. Standard: degrees (−150 to 150), Continuous: speed (−100 to 100), Linear: percent (0 to 100).

**Topic (publishes):** `/servo/state` → `std_msgs/Float64`
- Last commanded value, published at 20 Hz.

**Service:** `/servo/set_servo` → `studica_control/SetData`
- Set value via service. Pass the target in `request.initparams.speed`.

**params.yaml:**
```yaml
servo:
  enabled: true
  sensors: ["servo"]
  servo:
    port: 14             # VMX PWM channel index
    type: "standard"     # "standard", "continuous", or "linear"
```

---

### Ultrasonic — HC-SR04 Range Sensor

Topic is auto-generated: `/<name>/range`. Using `"front"` as an example:

**Topic (publishes):** `/front/range` → `sensor_msgs/Range`
- Distance in metres, min 0.02 m, max 4.0 m. Out-of-range readings are published as `inf`.
- Rate: 20 Hz

**Service:** `/front/ultrasonic_cmd` → `studica_control/SetData`

| Command | Description |
|---|---|
| `get_distance` | Distance in millimetres |
| `get_distance_inches` | Distance in inches |
| `get_distance_millimeters` | Distance in millimetres |

**params.yaml:**
```yaml
ultrasonic:
  enabled: true
  sensors: ["front"]
  front:
    ping: 8                   # VMX DIO channel for trigger
    echo: 9                   # VMX DIO channel for echo
    frame_id: "front_link"    # TF frame for the Range message header
```

---

### Sharp — GP2Y Infrared Range Sensor

Topic is auto-generated: `/<name>/range`. Using `"side_ir"` as an example:

**Topic (publishes):** `/side_ir/range` → `sensor_msgs/Range`
- Distance in metres, min 0.1 m, max 0.8 m. Out-of-range readings are published as `inf`.
- Rate: 20 Hz

**Service:** `/side_ir/sharp_cmd` → `studica_control/SetData`

| Command | Description |
|---|---|
| `get_distance` | Distance in centimetres |

**params.yaml:**
```yaml
sharp:
  enabled: true
  sensors: ["side_ir"]
  side_ir:
    port: 22                  # VMX analog input channel
    frame_id: "side_ir_link"  # TF frame for the Range message header
```

---

### DIO — Digital Input/Output

Topics are auto-generated from the sensor name. Using `"limit_switch"` as an example:

**Topic (publishes):** `/limit_switch/state` → `std_msgs/Bool`
- Current pin state, published at 10 Hz. Both input and output pins publish this.

**Topic (subscribes):** `/limit_switch/cmd` → `std_msgs/Bool`
- Set pin high (`true`) or low (`false`). **Output mode only.**

**Service:** `/limit_switch/dio_cmd` → `studica_control/SetData`

| Command | Description |
|---|---|
| `toggle` | Flip the output state (output mode only) |

**params.yaml:**
```yaml
dio:
  enabled: true
  sensors: ["limit_switch"]
  limit_switch:
    pin: 15
    type: "input"    # "input" or "output"
```

---

### Cobra — Reflectance Sensor Array

Topics are auto-generated from the sensor name. Using `"line_sensor"` as an example:

**Topics (publish):** `/line_sensor/ch_0` … `/line_sensor/ch_3` → `std_msgs/Float32`
- One topic per channel. Voltage reading for that channel, published at 20 Hz.

**Service:** `/line_sensor/cobra_cmd` → `studica_control/SetData`

Set `request.initparams.n_encoder` to the channel number (0–3).

| Command | Description |
|---|---|
| `get_raw` | Raw ADC value for the requested channel |
| `get_voltage` | Voltage reading for the requested channel |

**params.yaml:**
```yaml
cobra:
  enabled: true
  sensors: ["line_sensor"]
  line_sensor:
    vref: 5.0   # reference voltage in volts
```

---

### Gamepad — Joystick to cmd_vel

Converts joystick input from a `joy` node into `geometry_msgs/Twist` velocity commands.

**Requires:** `ros2 run joy joy_node` running in a separate terminal.

**Topic (publishes):** `<cmd_vel_topic>` → `geometry_msgs/Twist`
- Published at 10 Hz regardless of joystick input rate.

**Topic (subscribes):** `/gamepad_axis_remap` → `std_msgs/Int32MultiArray`
- Publish `[x_axis, y_axis, z_axis]` indices to remap axes at runtime without restarting. Use `-1` for any axis to leave it unmapped (outputs 0). Uses transient-local QoS so late-joining nodes receive the last mapping.

Set any axis index to `-1` in `params.yaml` to leave it unmapped (output is always 0.0 for that axis).

**params.yaml:**
```yaml
gamepad:
  enabled: true
  axis_linear_x: 1      # joystick axis for forward/back  (-1 = unmapped)
  axis_linear_y: -1     # joystick axis for strafe        (-1 = unmapped)
  axis_angular_z: 0     # joystick axis for rotation      (-1 = unmapped)
  button_turbo: 5       # button index to activate turbo mode
  linear_scale: 1.0
  angular_scale: 1.0
  deadzone: 0.05        # axis values below this are treated as zero
  turbo_multiplier: 2.0
  cmd_vel_topic: "cmd_vel"
```

Run `ros2 topic echo /joy` and move each stick to identify which `axes[]` index to use.

---

## Examples

Both Python and C++ examples are provided for every component. Both are installed as `ros2 run` executables when you build the package:

```
src/studica_control/src/components/examples/python/   ← Python scripts (.py)
src/studica_control/src/components/examples/cpp/      ← C++ sources
```

All examples require the driver to be running first:

```bash
ros2 launch studica_control studica_launch.py
```

---

## Python Examples

Python examples are installed with the package and run via `ros2 run`, the same as C++ examples.

### Titan

Publishes duty cycle to `/titan0/m_0/cmd`, subscribes to `/titan0/m_0/encoder` and `/titan0/m_0/rpm`. Spins motor 0 at 80% for 3 seconds, stops for 2 seconds, repeats three times, then resets the encoder via the service.

```bash
ros2 run studica_control titan_example.py
```

### IMU

Prints orientation quaternion, angular velocity, and linear acceleration at 20 Hz.

```bash
ros2 run studica_control imu_example.py
```

### Ultrasonic

Prints range readings in metres as they arrive.

```bash
ros2 run studica_control ultrasonic_example.py
```

### Servo

Subscribes to `/servo/state` and moves the servo through 90°, −90°, 0°.

```bash
ros2 run studica_control servo_example.py
```

### Gamepad

Shows `cmd_vel` output. Requires the joy node running alongside the launch file.

```bash
# Terminal 1
ros2 launch studica_control studica_launch.py
# Terminal 2
ros2 run joy joy_node
# Terminal 3
ros2 run studica_control gamepad_example.py
```

Additional Python examples for Encoder, DutyCycleEncoder, DIO, Sharp, and Cobra follow the same pattern.

---

## C++ Examples

C++ examples are compiled as part of the package and run via `ros2 run`.

**Build:**

```bash
colcon build --packages-select studica_control
source install/setup.bash
```

### Titan

Timer-driven state machine: motor 0 at 80% for 3 s, stop for 2 s, repeat 3 times,
then reset the encoder via service. Subscribes to `/titan0/m_0/encoder` and `/titan0/m_0/rpm`.

```bash
ros2 run studica_control titan_example
```

Enable in `params.yaml`:
```yaml
titan:
  enabled: true
  sensors: ["titan0"]
  titan0:
    can_id: 42
    motor_freq: 15600
    m_0:
      encoder_mode: "quadrature"
```

### IMU

Prints orientation, angular velocity, and linear acceleration. Calls `zero_yaw` via
service 5 seconds after startup.

```bash
ros2 run studica_control imu_example
```

### Servo

Moves through 90°, −90°, 0° with 3-second intervals. Alternates between publishing
to `/servo/cmd` and calling the `set_servo` service to demonstrate both interfaces.

```bash
ros2 run studica_control servo_example
```

### Ultrasonic

Subscribes to `/ultrasonic/range` and prints distance. Logs a warning when the
reading is `inf` (out of sensor range).

```bash
ros2 run studica_control ultrasonic_example
```

### Sharp

Subscribes to `/sharp/range` and prints IR distance. Logs a warning when the
reading is `inf`.

```bash
ros2 run studica_control sharp_example
```

### Encoder

Subscribes to the encoder topic and prints count and direction. Also polls the
`get_count` service every 5 seconds as a cross-check.

```bash
ros2 run studica_control encoder_example
```

### DutyCycleEncoder

Subscribes to the duty cycle encoder topic and prints absolute angle, rollover
count, and total rotation. Polls `get_absolute_position` via service every 5 seconds.

```bash
ros2 run studica_control dc_encoder_example
```

### DIO

Drives an output pin high/low via `/dio/cmd` at 0.5 Hz and prints `/dio/state`
feedback. Change `type` to `"input"` in `params.yaml` to observe an input pin instead.

```bash
ros2 run studica_control dio_example
```

### Cobra

Subscribes to all four `/cobra/ch_N` topics and prints voltages side by side at
10 Hz. Calls `get_voltage` on channel 0 via service every 5 seconds.

```bash
ros2 run studica_control cobra_example
```

### Gamepad

Subscribes to `/cmd_vel` and prints linear and angular velocity. Also shows how to
publish to `/gamepad_axis_remap` to remap controller axes at runtime without restarting.

```bash
# Terminal 1
ros2 launch studica_control studica_launch.py
# Terminal 2
ros2 run joy joy_node
# Terminal 3
ros2 run studica_control gamepad_example
```

---

## Standalone Driver Examples

The `drivers/examples/` folder contains standalone C++ programs that exercise the hardware drivers directly, with no ROS2 required. These are useful for testing hardware in isolation.

**Build all examples:**

```bash
cd drivers/examples
make
```

**Build and run a single example:**

```bash
cd drivers/examples/titan_example
make
sudo ./titan_example
```

> `sudo` is required because the VMX HAL uses `pigpio` for GPIO access.

**Available examples:**

| Directory | Description |
|---|---|
| `titan_example` | Configure encoders, enable controller, spin 4 motors, print encoder data |
| `imu_example` | Print pitch, yaw, roll and quaternion at startup |
| `servo_example` | Three sub-examples: `standard`, `continuous`, `linear` |
| `ultrasonic_example` | Print range readings in a loop |
| `sharp_example` | Print IR distance readings in a loop |
| `cobra_example` | Print reflectance voltage for all 4 channels |

**Titan example walkthrough:**

```cpp
// Create controller on CAN ID 45, 15600 Hz PWM, dist_per_tick = 0.000683
studica_driver::Titan titan(45, 15600, 0.0006830601);

// Wait 1s after initialization (required for Titan startup)
std::this_thread::sleep_for(std::chrono::milliseconds(1000));

// Configure encoder distance-per-tick for all 4 motors
titan.ConfigureEncoder(0, 0.0006830601);  // 1464 ticks = 1 rotation

// Enable and run
titan.Enable(true);
titan.SetSpeed(0, -0.2);  // motor 0 at 20% reverse
titan.SetSpeed(1,  0.2);  // motor 1 at 20% forward
printf("RPM: %d  Distance: %f\n", titan.GetRPM(0), titan.GetEncoderDistance(0));

titan.Enable(false);
```

---

## Integrating into Your Project

### Adding studica_control as a Dependency

In your package's `package.xml`:

```xml
<depend>studica_control</depend>
```

In your `CMakeLists.txt`:

```cmake
find_package(studica_control REQUIRED)

# For custom messages and services:
rosidl_get_typesupport_target(cpp_typesupport_target
  "studica_control" "rosidl_typesupport_cpp")

target_link_libraries(your_node
  "${cpp_typesupport_target}"
)
ament_target_dependencies(your_node
  studica_control
  rclcpp
  # ... other deps
)
```

### Calling the Titan Service from C++

```cpp
#include "studica_control/srv/set_data.hpp"

auto client = node->create_client<studica_control::srv::SetData>("titan/titan_cmd");
client->wait_for_service();

auto request = std::make_shared<studica_control::srv::SetData::Request>();
request->params = "set_speed";
request->initparams.n_encoder = 0;
request->initparams.speed = 0.5f;  // 50% forward

auto future = client->async_send_request(request);
```

### Calling the Titan Service from Python

```python
from studica_control.srv import SetData

client = node.create_client(SetData, 'titan/titan_cmd')
client.wait_for_service()

req = SetData.Request()
req.params = 'set_speed'
req.initparams.n_encoder = 0
req.initparams.speed = 0.5

future = client.call_async(req)
rclpy.spin_until_future_complete(node, future)
print(future.result().message)
```

### Subscribing to Sensor Data from Python

```python
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Float64, Float32

# IMU
node.create_subscription(Imu, 'imu', lambda msg: ..., 10)

# Titan — per-motor topics (encoder distance and RPM for quadrature mode)
node.create_subscription(Float64, '/titan0/m_0/encoder', lambda msg: ..., 10)
node.create_subscription(Float64, '/titan0/m_0/rpm', lambda msg: ..., 10)

# Titan — absolute angle (absolute mode)
node.create_subscription(Float64, '/titan0/m_2/angle', lambda msg: ..., 10)

# Ultrasonic range (/<name>/range)
node.create_subscription(Range, '/front/range', lambda msg: ..., 10)

# Cobra per-channel voltage
node.create_subscription(Float32, '/line_sensor/ch_0', lambda msg: ..., 10)
```

### Custom params.yaml in Your Package

If you bundle a `params.yaml` in your own package, point the launch file at it:

```python
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

params_file = os.path.join(
    get_package_share_directory('your_package'), 'config', 'params.yaml')

Node(
    package='studica_control',
    executable='manual_composition',
    name='control_server',
    output='screen',
    parameters=[params_file],
)
```

Install the config in your `CMakeLists.txt`:

```cmake
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
```

---

## VMX Channel Reference

Use these tables when assigning `port`, `pin`, `port_a`, `port_b`, `ping`, and `echo` values in `params.yaml`.

| Channel Type | Index Range | Used By |
|---|---|---|
| DIO | 0–21 | Encoder (A/B), DIO, Ultrasonic (ping/echo) |
| PWM | 0–21 | Servo |
| Analog Input | 22–25 | Sharp |
| Duty Cycle | 0, 2, 4, 6, 8, 10 | DutyCycleEncoder |

Consult the VMX hardware documentation for the physical pin mapping on your board.

---

## License

See [LICENSE](LICENSE).
