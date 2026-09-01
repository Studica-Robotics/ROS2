#!/usr/bin/env python3
"""Duty Cycle Encoder — subscribe to absolute position and rotation data.

Run:  python3 dc_encoder_example.py
Requires: studica_launch.py running, duty_cycle enabled in params.yaml

Topic:   subscribes to 'duty_cycle_encoder' (studica_control/DutyCycleEncoderMsg) — set duty_cycle.<sensor>.topic: duty_cycle_encoder in params.yaml
Service: '/<sensor_name>/duty_cycle_encoder_cmd' (SetData)
  Commands: get_absolute_position, get_rollover_count, get_total_rotation
"""
import rclpy
from rclpy.node import Node
from studica_control.msg import DutyCycleEncoderMsg


class DcEncoderExample(Node):
    def __init__(self):
        super().__init__('dc_encoder_example')
        self.sub = self.create_subscription(
            DutyCycleEncoderMsg, 'duty_cycle_encoder', self.on_data, 10)
        self.get_logger().info('Listening on duty_cycle_encoder topic...')

    def on_data(self, msg):
        self.get_logger().info(
            f'Angle: {msg.absolute_angle:.2f}  '
            f'Rollovers: {msg.rollover_count}  '
            f'Total rotation: {msg.total_rotation:.2f}')


def main():
    rclpy.init()
    rclpy.spin(DcEncoderExample())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
