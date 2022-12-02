import rclpy

from rclpy.node import Node
from std_msgs.msg import String


class TestNode(Node):

    def __init__(self):
        super().__init__('testnode')
        self.tmr = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        pub_cnt = self.count_publishers('/chatter')
        self.get_logger().info('Publisher Count: "{0}"'.format(pub_cnt))
        sub_cnt = self.count_subscribers('/chatter')
        self.get_logger().info('Subscriber Count: "{0}"'.format(sub_cnt))


def main(args=None):
    rclpy.init(args=args)

    node = TestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
