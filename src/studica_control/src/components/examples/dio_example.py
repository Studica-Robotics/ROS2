#!/usr/bin/env python3
"""DIO — subscribe to digital pin state and toggle it.

Run:  python3 dio_example.py
Requires: studica_launch.py running, dio enabled in params.yaml

Topic:   subscribes to dio topic (std_msgs/Bool)
Service: '/<dio_name>/dio_cmd' (SetData)
  Commands: toggle
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from studica_control.srv import SetData


class DioExample(Node):
    def __init__(self):
        super().__init__('dio_example')
        self.sub = self.create_subscription(Bool, 'dio_state', self.on_state, 10)
        self.client = self.create_client(SetData, '/dio/dio_cmd')
        self.get_logger().info('Listening on dio_state topic...')

    def on_state(self, msg):
        self.get_logger().info(f'DIO state: {msg.data}')

    def toggle(self):
        req = SetData.Request()
        req.params = 'toggle'
        future = self.client.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(f'toggle: {f.result().message}'))


def main():
    rclpy.init()
    node = DioExample()
    # Example: toggle pin after 2 seconds
    node.create_timer(2.0, node.toggle)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
