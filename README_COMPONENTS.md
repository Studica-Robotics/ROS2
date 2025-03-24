
This guide provides an overview of each component in this ROS2 repo.

## Cobra Component
Line follower. Ranges from 0V-5V. The higher the voltage, the more brightness is detected. Publishes to /cobra.

``` bash
ros2 service call /create_component studica_control/srv/SetData "{component: 'cobra', initparams: {vref: 5.0}}"
```

## Differential Drive Component
This component controls differential drive robots and publishes odometry to /odom. Run imu component simultaneously to integrate imu data into odometry.

Initialize
``` bash
ros2 service call /create_component studica_control/srv/SetData "{component: 'diffdrive', initparams: {can_id: 42, motor_freq: 15600, ticks_per_rotation: 1470, wheel_radius: 0.05, wheel_separation: 0.29, left: 2, right: 3, invert_left: False, invert_right: False}}" 
```

Set the wheel_radius (metres), wheel_separation (metres), titan ports (left and right), and inversions according to your robot setup.

## Differential Drive Odometry
Calculates and publishes odometry for differential drive robots. 

## Dio Component
Publishes to /dio_state.
``` bash
ros2 service call /create_component studica_control/srv/SetData "{component: 'dio', initparams: {pin: 10}}"
```

## Encoder Component
Publishes to /encoder_count and /encoder_direction.
``` bash
ros2 service call /create_component studica_control/srv/SetData "{component: 'encoder', initparams: {port_a: 10, port_b: 11}}"
```

## IMU Component
Publishes to /imu topic.
``` bash
ros2 service call /create_component studica_control/srv/SetData "{component: 'imu'}"
```

## Mecanum Drive Component
This component controls mecanum drive robots and publishes odometry to /odom. Run imu component simultaneously to integrate imu data into odometry.

Initialize
``` bash
ros2 service call /create_component studica_control/srv/SetData "{component: 'mecanum', initparams: {can_id: 42, motor_freq: 15600, ticks_per_rotation: 1470, wheel_radius: 0.05, wheelbase: 0.3, width: 0.29, front_left: 2, front_right: 3, rear_left: 0, rear_right: 0, invert_front_left: False, invert_front_right: True, invert_rear_left: True, invert_rear_right: False}}" 
```

Set the wheel_radius (metres), wheelbase (metres), width (metres), titan ports, inversions according to your robot setup.

## Mecanum Drive Odometry
Calculates and publishes odometry for mecanum drive robots.


## Servo Component
Controls the servo component. Types include standard, continuous, and linear.
``` bash
ros2 service call /create_component studica_control/srv/SetData "{component: 'servo', initparams: {pin: 10, servo_type: linear}}"
```

## Sharp Component
Controls the Sharp IR range sensor. Publishes to /ir_range.
``` bash
ros2 service call /create_component studica_control/srv/SetData "{component: 'sharp', initparams: {port: 10}}"
```

## Titan Component
Publishes titan distance values to "titan". Index corresponds to titan port. Must initialize titan ports to receive values. Set dist_per_tick to 1 for pure encoder values. dist_per_tick in meters.
``` bash
ros2 service call /create_component studica_control/srv/SetData "{component: 'titan', initparams: {can_id: 42, motor_freq: 15600, ticks_per_rotation: 1470, wheel_radius: 0.05}}"
ros2 service call /titan_cmd studica_control/srv/SetData "{params: 'configure_encoder', initparams: {n_encoder: 2, dist_per_tick: 0.0002}}"
ros2 service call /titan_cmd studica_control/srv/SetData "{params: 'enable'}"
```

## Ultrasonic Component
Gets range from ultrasonic sensor. Publishes to "ultrasonic_range". Output in meters.
``` bash
ros2 service call /create_component studica_control/srv/SetData "{component: 'ultrasonic', initparams: {ping: 10, echo: 11}}"
```
