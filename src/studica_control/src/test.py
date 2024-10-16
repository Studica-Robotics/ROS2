import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32  # Assuming the encoder publishes values as Int32

class EncoderSubscriber(Node):

    def __init__(self):
        super().__init__('encoder_subscriber')
        self.subscription = self.create_subscription(
            Int32,
            'encoder_value',  # Topic name
            self.listener_callback,
            10)  # QoS history depth
        self.subscription  # prevent unused variable warning
        self.last_value = None

    def listener_callback(self, msg):
        current_value = msg.data
        if self.last_value is None or self.last_value != current_value:
            self.get_logger().info(f'Encoder value changed: {current_value}')
            self.last_value = current_value


def main(args=None):
    rclpy.init(args=args)
    encoder_subscriber = EncoderSubscriber()
    rclpy.spin(encoder_subscriber)
    encoder_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
