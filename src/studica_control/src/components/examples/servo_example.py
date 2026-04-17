#!/usr/bin/env python3
"""Servo — read current angle and command a new angle.

Run:  python3 servo_example.py
Requires: studica_launch.py running, servo enabled in params.yaml

Topic:   subscribes to servo topic (std_msgs/Float32, last commanded angle)
Service: '/<servo_name>/set_servo_angle' (SetData)
  Command: pass the target angle as the 'params' string (e.g. "90")
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from studica_control.srv import SetData


class ServoExample(Node):
    def __init__(self):
        super().__init__('servo_example')
        self.sub = self.create_subscription(Float32, 'servo_angle', self.on_angle, 10)
        self.client = self.create_client(SetData, '/servo/set_servo_angle')
        self.get_logger().info('Listening on servo_angle topic...')

    def on_angle(self, msg):
        self.get_logger().info(f'Current angle: {msg.data}')

    def set_angle(self, degrees):
        req = SetData.Request()
        req.params = str(degrees)
        future = self.client.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(f'set_angle: {f.result().message}'))


def main():
    rclpy.init()
    node = ServoExample()
    # Example: move to 90 degrees after 2 seconds
    node.create_timer(2.0, lambda: node.set_angle(90))
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
