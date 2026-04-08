#!/usr/bin/env python3
"""
motor_demo.py  —  Studica Robotics ROS2 Motor Demo
====================================================

PURPOSE
-------
This script gets the robot's motors moving and shows you how the ROS2
architecture works. It is meant as a first step for people who are new
to this repo.

HOW TO RUN
----------
You need TWO terminals.

Terminal 1  (launches the robot hardware stack, needs root for hardware access):
    sudo su
    cd /home/vmx/ROS2
    . /opt/ros/humble/setup.bash
    . install/setup.bash
    ros2 launch studica_control studica_launch.py

Wait until you see "Managed nodes are active" in Terminal 1, then:

Terminal 2  (runs this demo script):
    cd /home/vmx/ROS2
    . /opt/ros/humble/setup.bash
    . install/setup.bash
    python3 examples/motor_demo.py

WHAT IT DOES
------------
Runs a short movement sequence: forward → turn left → backward → turn right → stop

VISUALISATION (optional, from a laptop on the same network)
-------------------------------------------------------------
After launching the robot stack, run these on your laptop:

    # See the full node/topic graph visually:
    rqt_graph

    # Watch live IMU data:
    ros2 topic echo /imu

    # See the transform tree (coordinate frames):
    ros2 run tf2_tools view_frames

ARCHITECTURE OVERVIEW
---------------------
                 ┌─────────────────────────────────────────────────────┐
                 │               control_server node                   │
                 │  (runs: diff_drive_controller, imu, ultrasonic, …)  │
                 └────────────────┬──────────────────────────────────┬─┘
                                  │ subscribes                       │ publishes
                                  │                                  │
           ┌──────────────────────▼──────┐            ┌─────────────-▼─────────────┐
           │     /cmd_vel_smoothed       │            │  /odom  /imu  /tf  /sharp  │
           │  geometry_msgs/Twist        │            │  (sensor & odometry data)  │
           └──────────────────────▲──────┘            └────────────────────────────┘
                                  │ smoothed output
                 ┌────────────────┴───────────────────┐
                 │          velocity_smoother         │
                 │  (nav2_velocity_smoother)          │
                 │  softens sudden start/stop jerks   │
                 └────────────────▲───────────────────┘
                                  │ raw velocity command
                          ┌───────┴──────────┐
                          │    /cmd_vel      │  ← YOU PUBLISH HERE
                          │ geometry_msgs/   │
                          │ Twist            │
                          └───────▲──────────┘
                                  │
                          ┌───────┴──────────┐
                          │   THIS SCRIPT    │
                          │  (or nav2, or    │
                          │   a gamepad)     │
                          └──────────────────┘

  Twist message fields used for diff-drive:
      linear.x   → forward/backward speed  (metres/second)
      angular.z  → turning rate            (radians/second, positive = left)

CONFIGURATION
-------------
Which drive controller is active is set in the params file:
    /home/vmx/ROS2/src/studica_control/config/params.yaml

To use diff drive (default), make sure this section has "enabled: true":

    diff_drive_component:
      enabled: true
      can_id: 42          # CAN ID of your Titan motor controller
      front_left: 2       # Titan port numbers for each wheel
      front_right: 0
      rear_left: 3
      rear_right: 1
      invert_front_right: true   # flip a motor if it spins the wrong way
      invert_rear_right: true

To use mecanum drive instead, set diff_drive_component.enabled to false
and mecanum_drive_component.enabled to true in the same file. The script
does not need any changes — it still just publishes to /cmd_vel.
"""

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


# ─────────────────────────────────────────────────────────────────────────────
#  TUNE THESE if your robot moves differently than expected
# ─────────────────────────────────────────────────────────────────────────────
FORWARD_SPEED  =  0.3   # m/s   – positive = forward
TURN_SPEED     =  0.8   # rad/s – positive = counter-clockwise (left)
MOVE_DURATION  =  2.0   # seconds per movement step
STOP_DURATION  =  0.5   # short pause between steps
CMD_VEL_TOPIC  = "/cmd_vel"
# ─────────────────────────────────────────────────────────────────────────────


class MotorDemo(Node):
    """
    Publishes geometry_msgs/Twist messages to /cmd_vel.

    Data flow:
      motor_demo  →  /cmd_vel  →  velocity_smoother  →  /cmd_vel_smoothed
                                                      →  diff_drive_controller
                                                      →  Titan motor controller
                                                      →  wheels
    """

    def __init__(self):
        super().__init__("motor_demo")
        self.cmd_vel_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.get_logger().info("MotorDemo node started.")

    def _make_twist(self, linear_x=0.0, angular_z=0.0) -> Twist:
        twist = Twist()
        twist.linear.x  = float(linear_x)
        twist.angular.z = float(angular_z)
        return twist

    def send_velocity(self, linear_x=0.0, angular_z=0.0):
        """Publish one velocity command."""
        self.cmd_vel_pub.publish(self._make_twist(linear_x, angular_z))

    def stop(self):
        """Publish a zero-velocity command to halt the robot."""
        self.cmd_vel_pub.publish(self._make_twist())

    def drive_for(self, label: str, linear_x=0.0, angular_z=0.0, duration=MOVE_DURATION):
        """
        Drive at a constant velocity for `duration` seconds.
        Publishes at 20 Hz so the velocity smoother gets a continuous stream.
        """
        print(f"\n>>> {label}")
        print(f"    linear.x={linear_x:+.2f} m/s   angular.z={angular_z:+.2f} rad/s")

        end_time = time.time() + duration
        while time.time() < end_time:
            self.send_velocity(linear_x, angular_z)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.05)  # 20 Hz

    def pause(self, duration=STOP_DURATION):
        """Stop and wait briefly before the next move."""
        self.stop()
        end_time = time.time() + duration
        while time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)


# ─────────────────────────────────────────────────────────────────────────────
#  DEMO SEQUENCE  –  feel free to edit this to try different moves
# ─────────────────────────────────────────────────────────────────────────────

def run_demo(node: MotorDemo):
    print("\n" + "="*60)
    print(" MOTOR DEMO  –  starting movement sequence")
    print("="*60)
    print(" Make sure the robot is on the floor with space to move!")
    input(" Press ENTER when ready…")

    node.drive_for("FORWARD",           linear_x= FORWARD_SPEED, angular_z=0.0)
    node.pause()
    node.drive_for("TURN LEFT",         linear_x=0.0, angular_z= TURN_SPEED)
    node.pause()
    node.drive_for("BACKWARD",          linear_x=-FORWARD_SPEED, angular_z=0.0)
    node.pause()
    node.drive_for("TURN RIGHT",        linear_x=0.0, angular_z=-TURN_SPEED)
    node.pause()

    node.stop()
    print("\n>>> STOP — sequence complete")

    print("\n" + "="*60)
    print(" DONE!  The motors responded to /cmd_vel Twist messages.")
    print()
    print(" Next steps:")
    print("   • Run 'rqt_graph' on your laptop to see the full node graph")
    print("   • Run 'ros2 topic echo /odom' to watch live odometry")
    print("   • Run 'ros2 topic echo /imu'  to watch live IMU data")
    print("   • Connect a gamepad: ros2 launch studica_control gamepad_launch.py")
    print("   • Open RViz2 to visualize the robot in 3D")
    print("="*60 + "\n")


# ─────────────────────────────────────────────────────────────────────────────

def main():
    print(__doc__)

    print("\nInitialising ROS2…")
    rclpy.init()

    node = MotorDemo()

    try:
        run_demo(node)
    except KeyboardInterrupt:
        print("\nInterrupted – stopping motors.")
        node.stop()
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
