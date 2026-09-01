#!/usr/bin/env python3
"""Sharp IR — subscribe to infrared range readings.

Run:  python3 sharp_example.py
Requires: studica_launch.py running, sharp enabled in params.yaml

Topic:   subscribes to 'ir_range' (sensor_msgs/Range) — set sharp.<sensor>.topic: ir_range in params.yaml
Service: '/<sensor_name>/sharp_cmd' (SetData)
  Commands: get_distance
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range


class SharpExample(Node):
    def __init__(self):
        super().__init__('sharp_example')
        self.sub = self.create_subscription(Range, 'ir_range', self.on_range, 10)
        self.get_logger().info('Listening on ir_range topic...')

    def on_range(self, msg):
        self.get_logger().info(f'IR range: {msg.range:.3f} m')


def main():
    rclpy.init()
    rclpy.spin(SharpExample())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
