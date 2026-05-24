#!/usr/bin/env python3
"""Light Tower — cycle through every state via the /light_tower/set service.

Run:      ros2 run studica_control light_tower_example.py
Requires: studica_launch.py running, light_tower enabled in params.yaml

Service:  /light_tower/set  (studica_control/SetData)
  Commands:
    "off"              — everything off
    "<color>"          — solid on  (red, green, yellow, buzzer)
    "<color>:blink"    — software blink at default_blink_hz
    "<color>:blink_hw" — hardware blink (continuous pin LOW)
    "<color>:<hz>"     — software blink at specific Hz

Topic:    /light_tower/state  (std_msgs/String) — current state at 1 Hz
"""
import itertools

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from studica_control.srv import SetData

STATES = [
    'red',
    'green',
    'yellow',
    'buzzer',
    'red:blink',
    'green:blink_hw',
    'yellow:2.5',
    'off',
]


class LightTowerExample(Node):
    def __init__(self):
        super().__init__('light_tower_example')
        self._cycle = itertools.cycle(STATES)

        self.sub = self.create_subscription(String, '/light_tower/state', self.on_state, 10)
        self.client = self.create_client(SetData, '/light_tower/set')
        self.create_timer(3.0, self.next_state)
        self.get_logger().info('Light tower example ready — cycling every 3 s')

    def on_state(self, msg):
        self.get_logger().info(f'state: {msg.data}')

    def next_state(self):
        cmd = next(self._cycle)
        req = SetData.Request()
        req.params = cmd
        future = self.client.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(f'set "{cmd}" → {f.result().message}'))


def main():
    rclpy.init()
    rclpy.spin(LightTowerExample())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
