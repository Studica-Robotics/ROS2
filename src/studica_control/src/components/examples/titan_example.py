#!/usr/bin/env python3
"""Titan motor controller — subscribe to encoder data and command motors.

Run:  python3 titan_example.py
Requires: studica_launch.py running, titan enabled in params.yaml

Topics:  subscribes to 'titan_encoders' (Float32MultiArray, 4 encoder counts)
Service: '/titan/titan_cmd' (SetData)
  Commands: set_speed, stop, enable, disable, reset, invert_motor,
            get_rpm, get_encoder_count, get_encoder_distance,
            setup_encoder, configure_encoder
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from studica_control.srv import SetData


class TitanExample(Node):
    def __init__(self):
        super().__init__('titan_example')
        self.sub = self.create_subscription(
            Float32MultiArray, 'titan_encoders', self.on_encoder, 10)
        self.client = self.create_client(SetData, '/titan/titan_cmd')
        self.get_logger().info('Listening on titan_encoders...')

    def on_encoder(self, msg):
        self.get_logger().info(f'Encoder counts: {list(msg.data)}')

    def call_titan(self, command, motor=0, speed=0.0):
        req = SetData.Request()
        req.params = command
        req.initparams.n_encoder = motor
        req.initparams.speed = speed
        future = self.client.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(f'{command}: {f.result().message}'))


def main():
    rclpy.init()
    node = TitanExample()

    # Example: spin motor 0 at 30% for 3 seconds, then stop
    def run_motor():
        node.call_titan('set_speed', motor=0, speed=0.3)
        node.create_timer(3.0, lambda: node.call_titan('stop', motor=0))

    node.create_timer(2.0, run_motor)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
