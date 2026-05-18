#!/usr/bin/env python3
"""Titan motor controller — command motors via topics and read encoder/RPM or angle feedback.

Run:  python3 titan_example.py
Requires: studica_launch.py running, titan enabled in params.yaml

Topic tree (sensor name "titan0"):

  Commands (publish to):
    /titan0/m_0/cmd  (std_msgs/Float64) — duty cycle -1.0 to 1.0
    /titan0/m_1/cmd  ...
    /titan0/m_2/cmd  ...
    /titan0/m_3/cmd  ...

  Feedback (subscribe to) — topics present depend on encoder_mode in params.yaml:
    quadrature mode:
      /titan0/m_N/encoder  (std_msgs/Float64) — encoder distance
      /titan0/m_N/rpm      (std_msgs/Float64) — RPM
    absolute mode:
      /titan0/m_N/angle    (std_msgs/Float64) — absolute angle in degrees

Service: /titan0/titan_cmd (SetData)
  Use for configuration: configure_encoder, reset_encoder, set_current_limit,
  set_target_velocity, set_target_angle, invert_motor, get_cypher_angle, etc.
  See titan_component.h for the full command reference.
"""
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from studica_control.srv import SetData

SENSOR = 'titan0'  # must match the sensor name in params.yaml


class TitanExample(Node):
    def __init__(self):
        super().__init__('titan_example')

        # command publishers — one per motor, always present
        self.cmd_pubs = [
            self.create_publisher(Float64, f'/{SENSOR}/m_{i}/cmd', 10)
            for i in range(4)
        ]

        # quadrature feedback — subscribe only if encoder_mode is "quadrature"
        self.create_subscription(Float64, f'/{SENSOR}/m_0/encoder', self.on_encoder, 10)
        self.create_subscription(Float64, f'/{SENSOR}/m_0/rpm',     self.on_rpm,     10)

        # absolute feedback — subscribe only if encoder_mode is "absolute"
        # self.create_subscription(Float64, f'/{SENSOR}/m_0/angle', self.on_angle, 10)

        self.client = self.create_client(SetData, f'/{SENSOR}/titan_cmd')
        self.get_logger().info(f'Titan example ready. Sensor: {SENSOR}')

    def on_encoder(self, msg):
        self.get_logger().info(f'm_0 encoder distance: {msg.data:.4f}')

    def on_rpm(self, msg):
        self.get_logger().info(f'm_0 rpm: {msg.data:.1f}')

    def on_angle(self, msg):
        self.get_logger().info(f'm_0 absolute angle: {msg.data:.2f} deg')

    def set_speed(self, motor: int, duty: float):
        """Publish a duty cycle command directly to the motor topic."""
        msg = Float64()
        msg.data = float(duty)
        self.cmd_pubs[motor].publish(msg)

    def call_service(self, command: str, motor: int = 0, speed: float = 0.0,
                     int_value: int = 0, dist_per_tick: float = 0.0):
        """Send a configuration or closed-loop command via the service."""
        if not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('titan_cmd service not available')
            return None
        req = SetData.Request()
        req.params = command
        req.initparams.n_encoder = motor
        req.initparams.speed = speed
        req.initparams.int_value = int_value
        req.initparams.dist_per_tick = dist_per_tick
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        result = future.result()
        if result:
            self.get_logger().info(f'{command}: {result.message}')
        else:
            self.get_logger().error(f'{command}: no response')
        return result


def main():
    rclpy.init()
    node = TitanExample()

    # spin motor 0 at 80% for 3 seconds, stop for 2 — repeat 3 times
    for _ in range(3):
        node.set_speed(0, 0.8)
        end = time.time() + 3.0
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

        node.set_speed(0, 0.0)
        end = time.time() + 2.0
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

    # reset the encoder via service after the run
    node.call_service('reset_encoder', motor=0)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
