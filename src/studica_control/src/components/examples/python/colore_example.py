#!/usr/bin/env python3
"""Colore — subscribe to the color the sensor sees, and the matched color.

the colore is a color sensor (CAN or USB). /color is the color as RGB (each
channel 0.0-1.0). if color matching is enabled it also reports which taught
color it sees on /color_match.

Run:  python3 colore_example.py
Requires: studica_launch.py running, colore enabled in params.yaml
          for the match output, add "match" to publish_outputs (see README)

Topics:
  '/color0/color'        (std_msgs/ColorRGBA)          — RGB, each channel 0.0-1.0
  '/color0/color_match'  (studica_control/ColoreMatch) — nearest taught color + confidence
Service: '/color0/colore_cmd' (studica_control/SetData)
  set_brightness        — white LED 0-100, value in initparams.n_encoder
  learn_color,<name>    — teach the current color under <name>
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from studica_control.msg import ColoreMatch
from studica_control.srv import SetData

SENSOR = 'color0'


class ColoreExample(Node):
    def __init__(self):
        super().__init__('colore_example')
        self.create_subscription(ColorRGBA, f'/{SENSOR}/color', self.on_color, 10)
        self.create_subscription(ColoreMatch, f'/{SENSOR}/color_match', self.on_match, 10)
        self.client = self.create_client(SetData, f'/{SENSOR}/colore_cmd')
        self.get_logger().info(f'Listening on /{SENSOR}/color ...')

    def on_color(self, msg):
        self.get_logger().info(f'color  r={msg.r:.2f} g={msg.g:.2f} b={msg.b:.2f}')

    def on_match(self, msg):
        self.get_logger().info(f'match  {msg.label} (confidence {msg.confidence:.2f})')

    def set_brightness(self, percent: int):
        """Set the white LED brightness (0-100) via service."""
        if not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('colore_cmd service not available')
            return
        req = SetData.Request()
        req.params = 'set_brightness'
        req.initparams.n_encoder = percent  # 0-100
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            self.get_logger().info(f'set_brightness: {future.result().message}')


def main():
    rclpy.init()
    node = ColoreExample()
    node.set_brightness(50)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
