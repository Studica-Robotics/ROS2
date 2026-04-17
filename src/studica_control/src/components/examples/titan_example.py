#!/usr/bin/env python3
"""Titan motor controller — subscribe to encoder data and command motors.

Run:  python3 titan_example.py
Requires: studica_launch.py running, titan enabled in params.yaml

Topics:  subscribes to '<sensor_name>/titan_encoders' (Float32MultiArray, 4 encoder counts)
           set titan.<sensor>.topic: titan_encoders in params.yaml — sensor name sets the prefix
Service: '<sensor_name>/titan_cmd' (SetData)
  Commands: set_speed, stop, enable, disable, reset, invert_motor,
            get_rpm, get_encoder_count, get_encoder_distance,
            setup_encoder, configure_encoder
"""
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from studica_control.srv import SetData


class TitanExample(Node):
    def __init__(self):
        super().__init__('titan_example')
        self.sub = self.create_subscription(
            Float32MultiArray, 'titan_encoders', self.on_encoder, 10)
        self.client = self.create_client(SetData, 'titan_cmd')
        self.get_logger().info('Listening on titan_encoders...')

    def on_encoder(self, msg):
        self.get_logger().info(f'Encoder counts: {list(msg.data)}')

    def call_titan(self, command, motor=0, speed=0.0):
        req = SetData.Request()
        req.params = command
        req.initparams.n_encoder = motor
        req.initparams.speed = speed
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if future.result():
            self.get_logger().info(f'{command}: {future.result().message}')
        else:
            self.get_logger().error(f'{command}: no response')


def main():
    rclpy.init()
    node = TitanExample()

    if not node.client.wait_for_service(timeout_sec=3.0):
        node.get_logger().error('titan_cmd service not available')
        rclpy.shutdown()
        return

    # enable motor 0 first, then spin at 30% for 3 seconds, stop for 2 — repeat 3 times
    node.call_titan('enable', motor=0)
    for _ in range(3):
        node.call_titan('set_speed', motor=0, speed=0.3)
        end = time.time() + 3.0
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        node.call_titan('stop', motor=0)
        end = time.time() + 2.0
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
