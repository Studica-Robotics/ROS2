#!/usr/bin/env python3
"""Titan motor controller — command motors via topics and read encoder/RPM feedback.

Before the run loop: Titan2 setup via titan_cmd (PID type, autotune, target velocity).
Then runs motor 0 at target RPM for 3 seconds, stops for 2 seconds,
then resets the encoder via the service.

Run:  ros2 run studica_control titan_example.py
Requires: studica_launch.py running, titan enabled in params.yaml
          sensor name "titan0", m_0 encoder_mode: quadrature

Topics published:   /titan0/m_N/cmd      (Float64)  duty cycle -1.0 to 1.0
Topics subscribed:  /titan0/m_0/encoder, /titan0/m_0/rpm
Service:            /titan0/titan_cmd    (SetData)
"""
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from studica_control.srv import SetData

SENSOR = 'titan0'

PID_MCV2 = 2
MOTOR = 0
TARGET_RPM = 10.0
AUTOTUNE_WAIT_S = 15


class TitanExample(Node):
    def __init__(self):
        super().__init__('titan_example')

        self.cmd_pubs = [
            self.create_publisher(Float64, f'/{SENSOR}/m_{i}/cmd', 10)
            for i in range(4)
        ]

        self.create_subscription(Float64, f'/{SENSOR}/m_0/encoder', self.on_encoder, 10)
        self.create_subscription(Float64, f'/{SENSOR}/m_0/rpm', self.on_rpm, 10)

        self.client = self.create_client(SetData, f'/{SENSOR}/titan_cmd')
        self.get_logger().info(f'Titan example ready. Sensor: {SENSOR}')

    def on_encoder(self, msg):
        self.get_logger().info(f'm_0 encoder distance: {msg.data:.4f}')

    def on_rpm(self, msg):
        self.get_logger().info(f'm_0 rpm: {msg.data:.2f}')

    def set_speed(self, motor: int, duty: float):
        msg = Float64()
        msg.data = float(duty)
        self.cmd_pubs[motor].publish(msg)

    def call_service(self, command: str, motor: int = 0, speed: float = 0.0,
                     int_value: int = 0, dist_per_tick: float = 0.0):
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('titan_cmd service not available')
            return None
        req = SetData.Request()
        req.params = command
        req.initparams.n_encoder = motor
        req.initparams.speed = float(speed)
        req.initparams.int_value = int_value
        req.initparams.dist_per_tick = dist_per_tick
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        result = future.result()
        if result and result.success:
            self.get_logger().info(f'{command}: {result.message}')
        elif result:
            self.get_logger().error(f'{command}: {result.message}')
        else:
            self.get_logger().error(f'{command}: no response')
        return result


def main():
    rclpy.init()
    node = TitanExample()

    if not node.client.wait_for_service(timeout_sec=10.0):
        node.get_logger().error('titan_cmd not available — is studica_launch running?')
        rclpy.shutdown()
        return

    node.get_logger().info('setup: set_pid_type MCV2 on all motors')
    if not node.call_service('set_pid_type', int_value=PID_MCV2):
        rclpy.shutdown()
        return

    node.get_logger().info(f'setup: autotune all ({AUTOTUNE_WAIT_S} s)')
    if not node.call_service('autotune'):
        rclpy.shutdown()
        return

    end = time.time() + AUTOTUNE_WAIT_S
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)

    if not node.call_service('set_target_velocity', motor=MOTOR, speed=TARGET_RPM):
        rclpy.shutdown()
        return

    node.get_logger().info(f'running at {TARGET_RPM:.1f} rpm')
    end = time.time() + 3.0
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)

    node.call_service('set_target_velocity', motor=MOTOR, speed=0.0)
    node.set_speed(MOTOR, 0.0)
    node.get_logger().info('stopped')

    end = time.time() + 2.0
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)

    node.call_service('reset_encoder', motor=MOTOR)
    node.get_logger().info('done — encoder reset')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
