import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class TestNode(Node):
    def __init__(self):
        super().__init__('zf9p')
        self.tmr = self.create_timer(1, self.tmr_callback)

    def tmr_callback(self):
        1/0
        print('Timer callback fired')


def single_executor():
    test_node = TestNode()

    while rclpy.ok() is True:
        rclpy.spin(test_node)

    test_node.destroy_timer(test_node.tmr)


def multi_executor():
    test_node = TestNode()

    # Moved to multi threaded executor so none of Actions block publishing of msgs
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(test_node)

    while rclpy.ok() is True:
        executor.spin()

    test_node.destroy_timer(test_node.tmr)


def main(args=None):
    rclpy.init()

    #single_executor()  # un comment to verify proper operation 
    multi_executor()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
