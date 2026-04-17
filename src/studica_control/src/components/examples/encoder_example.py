#!/usr/bin/env python3
"""Encoder — subscribe to quadrature encoder count and direction.

Run:  python3 encoder_example.py
Requires: studica_launch.py running, encoder enabled in params.yaml

Topic:   subscribes to encoder topic (studica_control/EncoderMsg)
Service: '/<encoder_name>/encoder_cmd' (SetData)
  Commands: get_count, get_direction
"""
import rclpy
from rclpy.node import Node
from studica_control.msg import EncoderMsg


class EncoderExample(Node):
    def __init__(self):
        super().__init__('encoder_example')
        # Subscribe to the first encoder's topic (default: rr_enc)
        self.sub = self.create_subscription(EncoderMsg, 'rr_enc', self.on_encoder, 10)
        self.get_logger().info('Listening on rr_enc topic...')

    def on_encoder(self, msg):
        self.get_logger().info(f'Count: {msg.encoder_count}  Direction: {msg.encoder_direction}')


def main():
    rclpy.init()
    rclpy.spin(EncoderExample())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
