import rclpy
from rclpy.node import Node
from rclpy.timer import TimerInfo

from std_msgs.msg import String


class Talker(Node):

    def __init__(self):
        super().__init__('talker')
        self.count = 0
        hz_param = self.declare_parameter('hz', 1.0)
        self.publisher = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0 / hz_param.value, self.callback)

    def callback(self, info: TimerInfo):
        data = 'Hello World: {0}'.format(self.count)
        self.get_logger().info('Publishing: "{0}"'.format(data))
        self.publisher.publish(String(data=data))
        self.count += 1
        print(info.expected_call_time)
        print(info.actual_call_time)


def main(args=None):
    rclpy.init(args=args)

    node = Talker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
