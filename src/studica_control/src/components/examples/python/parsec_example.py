#!/usr/bin/env python3
"""Parsec — subscribe to multi-zone ToF readings (100 frames, then exit).

Run:  ros2 run studica_control parsec_example.py
Requires: studica_launch.py running, parsec enabled in params.yaml
          sensors: ["parsec"]  (name must match SENSOR below)

Topics:
  /parsec/zones     (studica_control/ParsecZoneMsg)
  /parsec/min_range (sensor_msgs/Range)
Service: /parsec/parsec_cmd (SetData)
  Commands: get_config, get_zone_distance, get_min_distance
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from studica_control.msg import ParsecZoneMsg
from studica_control.srv import SetData

SENSOR = 'parsec'
MAX_PRINTS = 100


def _zone_cell(d: int) -> str:
    if d == -1:
        return '    off'
    if d == -2 or d < 1 or d > 4000:
        return '    ---'
    return f'{d:4d} mm'


class ParsecExample(Node):
    def __init__(self):
        super().__init__('parsec_example')
        self.create_subscription(ParsecZoneMsg, f'/{SENSOR}/zones', self.on_zones, 10)
        self.create_subscription(Range, f'/{SENSOR}/min_range', self.on_min_range, 10)
        self.client = self.create_client(SetData, f'/{SENSOR}/parsec_cmd')
        self.create_timer(5.0, self.poll_service)
        self.print_count = 0
        self.get_logger().info(f'Listening on /{SENSOR}/zones and /{SENSOR}/min_range')

    def on_zones(self, msg: ParsecZoneMsg):
        side = 8 if msg.zones == 64 else 4
        center = msg.zones // 2
        center_mm = msg.fdist[center] if center < len(msg.fdist) else -2
        self.get_logger().info(
            f'{self.print_count:3d} | zones seq={msg.seq} count={msg.zones} center={center_mm} mm'
        )
        for row in range(side):
            row_vals = msg.fdist[row * side:(row + 1) * side]
            if not row_vals:
                break
            line = '  ' + ' '.join(_zone_cell(d) for d in row_vals)
            self.get_logger().info(line)

        self.print_count += 1
        if self.print_count >= MAX_PRINTS:
            self.get_logger().info(f'printed {MAX_PRINTS} frames, exiting')
            rclpy.shutdown()

    def on_min_range(self, msg: Range):
        if math.isinf(msg.range):
            self.get_logger().warn('no valid zones in frame')
        else:
            self.get_logger().info(f'nearest valid zone: {msg.range:.3f} m')

    def poll_service(self):
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('parsec_cmd service not available')
            return
        req = SetData.Request()
        req.params = 'get_min_distance'
        future = self.client.call_async(req)
        future.add_done_callback(self._on_service_response)

    def _on_service_response(self, future):
        resp = future.result()
        if resp.success:
            self.get_logger().info(f'service get_min_distance: {resp.message} mm')
        else:
            self.get_logger().warn(f'service get_min_distance failed: {resp.message}')


def main():
    rclpy.init()
    rclpy.spin(ParsecExample())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
