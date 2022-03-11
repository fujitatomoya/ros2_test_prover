import rclpy
from rclpy.node import Node

import threading


class Child(Node):

    def __init__(self):
        super().__init__("segfault_node")

        self._thread = threading.Thread(
            target=self._run,
            name='segfault_thread',
        )
        self._thread.start()
        self.get_logger().info('segfault thread started')

    def _run(self):
        from rclpy.executors import SingleThreadedExecutor
        executor = SingleThreadedExecutor(context=None)
        executor.add_node(self)
        i = 0
        while i < 5:
            i = i + 1
            executor.spin_once(timeout_sec=1.0)

def main(args=None):
    rclpy.init(args=args)
    child = Child()
    dummy_node = rclpy.create_node("dummy")
    dummy_node.destroy_node()
    dummy_node.get_logger().info('dummy node destroyed')
    child.destroy_node() # segfault here
    child.get_logger().info('child node destroyed')

if __name__ == '__main__':
    main()

