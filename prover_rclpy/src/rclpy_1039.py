import time
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import Empty


class TestNode(Node):

    def __init__(self) -> None:
        super().__init__('test_node')

        my_callback_group = ReentrantCallbackGroup()

        self.create_subscription(Empty, 'trigger1', self.callback1, 1, callback_group=my_callback_group)
        self.create_subscription(Empty, 'trigger2', self.callback2, 1, callback_group=my_callback_group)
        self.create_timer(1, self.callback3, callback_group=my_callback_group)

    def callback1(self, data) -> None:
        self.get_logger().warning("Start Trigger 1")
        time.sleep(10)
        self.get_logger().warning("End of Trigger 1")

    def callback2(self, data) -> None:
        self.get_logger().warning("Trigger 2")

    def callback3(self) -> None:
        self.get_logger().warning("Trigger Timer")


def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    ex.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()