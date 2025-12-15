import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

class ClockPublisher(Node):
    def __init__(self):
        super().__init__('clock_publisher')
        self.publisher_ = self.create_publisher(Clock, '/clock', 10)
        publish_hz = 10
        self.timer = self.create_timer(1/publish_hz, self.timer_callback)
        self.get_logger().info('Publishing rosgraph_msgs/Clock to /clock')

    def timer_callback(self):
        now = self.get_clock().now().to_msg()
        clock_msg = Clock(clock=now)
        self.publisher_.publish(clock_msg)
        self.get_logger().debug(f'Published Clock: {clock_msg.clock.sec}.{clock_msg.clock.nanosec:09d}')

def main(args=None):
    rclpy.init(args=args)
    node = ClockPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
