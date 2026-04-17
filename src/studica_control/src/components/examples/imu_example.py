#!/usr/bin/env python3
"""IMU — subscribe to orientation, angular velocity, and linear acceleration.

Run:  python3 imu_example.py
Requires: studica_launch.py running, imu enabled in params.yaml

Topic:   subscribes to 'imu' (sensor_msgs/Imu) — set imu.<sensor>.topic: imu in params.yaml
Service: '/imu/get_imu_data' (SetData) — returns pitch, yaw, roll
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuExample(Node):
    def __init__(self):
        super().__init__('imu_example')
        self.sub = self.create_subscription(Imu, 'imu', self.on_imu, 10)
        self.get_logger().info('Listening on imu topic...')

    def on_imu(self, msg):
        q = msg.orientation
        av = msg.angular_velocity
        la = msg.linear_acceleration
        self.get_logger().info(
            f'Orientation: ({q.x:.3f}, {q.y:.3f}, {q.z:.3f}, {q.w:.3f})  '
            f'Gyro: ({av.x:.3f}, {av.y:.3f}, {av.z:.3f})  '
            f'Accel: ({la.x:.3f}, {la.y:.3f}, {la.z:.3f})')


def main():
    rclpy.init()
    rclpy.spin(ImuExample())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
