import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, LivelinessPolicy
from std_msgs.msg import String

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        qos_profile = QoSProfile(depth=10, liveliness=LivelinessPolicy.BEST_AVAILABLE)
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            qos_profile)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()