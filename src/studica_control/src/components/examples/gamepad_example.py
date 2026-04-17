#!/usr/bin/env python3
"""Gamepad — subscribe to cmd_vel output from the gamepad controller.

Run:  python3 gamepad_example.py
Requires: studica_launch.py running, gamepad enabled in params.yaml,
          and a joystick node publishing to /joy (e.g. ros2 run joy joy_node)

Topic: subscribes to '/cmd_vel' (geometry_msgs/Twist)
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
