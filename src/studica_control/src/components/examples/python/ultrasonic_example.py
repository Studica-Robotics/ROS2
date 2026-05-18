#!/usr/bin/env python3
"""Ultrasonic — subscribe to range readings.

Run:  python3 ultrasonic_example.py
Requires: studica_launch.py running, ultrasonic enabled in params.yaml

Topic:   subscribes to 'ultrasonic_range' (sensor_msgs/Range) — set ultrasonic.<sensor>.topic: ultrasonic_range in params.yaml
Service: '/<sensor_name>/ultrasonic_cmd' (SetData)
  Commands: get_distance, get_distance_inches, get_distance_millimeters
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range


class UltrasonicExample(Node):
    def __init__(self):
        super().__init__('ultrasonic_example')
        self.sub = self.create_subscription(Range, 'ultrasonic_range', self.on_range, 10)
        self.get_logger().info('Listening on ultrasonic_range topic...')

    def on_range(self, msg):
        self.get_logger().info(f'Range: {msg.range:.3f} m  (min: {msg.min_range}, max: {msg.max_range})')


def main():
    rclpy.init()
    rclpy.spin(UltrasonicExample())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
