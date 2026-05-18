#!/usr/bin/env python3
"""Cobra — subscribe to reflectance readings from all 4 channels.

the cobra is a 4-channel infrared reflectance sensor. each channel outputs
a voltage between 0 and vref — low over dark surfaces, high over light ones.
use the 4 channel readings together to follow a line or detect surface changes.

Run:  python3 cobra_example.py
Requires: studica_launch.py running, cobra enabled in params.yaml

Topic:   subscribes to 'cobra' (std_msgs/Float32MultiArray) — set cobra.<sensor>.topic: cobra in params.yaml
           data[0..3] = voltage in volts for channels 0-3
Service: 'cobra_cmd' (studica_control/SetData)
  Commands:
    get_raw <channel>     — raw adc value for one channel (0-3)
    get_voltage <channel> — voltage in volts for one channel (0-3)
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from studica_control.srv import SetData


class CobraExample(Node):
    def __init__(self):
        super().__init__('cobra_example')
        self.sub = self.create_subscription(
            Float32MultiArray, 'cobra', self.on_reading, 10)
        self.client = self.create_client(SetData, 'cobra_cmd')
        self.get_logger().info('Listening on cobra topic (4 channels)...')

    def on_reading(self, msg):
        channels = msg.data
        if len(channels) < 4:
            return
        self.get_logger().info(
            f'ch0: {channels[0]:.3f}V  ch1: {channels[1]:.3f}V  '
            f'ch2: {channels[2]:.3f}V  ch3: {channels[3]:.3f}V')

    def get_voltage(self, channel: int):
        """Request the voltage for a single channel via service."""
        if not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('cobra_cmd service not available')
            return
        req = SetData.Request()
        req.params = f'get_voltage {channel}'
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            self.get_logger().info(
                f'channel {channel} voltage: {future.result().message} V')


def main():
    rclpy.init()
    node = CobraExample()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
