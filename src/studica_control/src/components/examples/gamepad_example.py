#!/usr/bin/env python3
"""Gamepad — subscribe to cmd_vel output from the gamepad controller.

Setup (3 terminals):
  1. ros2 launch studica_control studica_launch.py   # main HAL — gamepad.enabled must be true in params.yaml
  2. ros2 run joy joy_node                            # publishes raw joystick input to /joy
  3. python3 gamepad_example.py                       # this script — shows cmd_vel output

To find your controller's axis indices:
  ros2 topic echo /joy   — move each stick and note which axes[] index changes

Configure axes in params.yaml under control_server.gamepad:
  axis_linear_x, axis_linear_y, axis_angular_z, button_turbo

Topic: subscribes to '/cmd_vel' (geometry_msgs/Twist) — no topic to configure, cmd_vel is fixed
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class GamepadExample(Node):
    def __init__(self):
        super().__init__('gamepad_example')
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.on_twist, 10)
        self.get_logger().info('Listening on /cmd_vel...')

    def on_twist(self, msg):
        self.get_logger().info(
            f'Linear: ({msg.linear.x:.2f}, {msg.linear.y:.2f})  '
            f'Angular: {msg.angular.z:.2f}')


def main():
    rclpy.init()
    rclpy.spin(GamepadExample())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
