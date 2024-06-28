import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

class TestNode(Node):
    def __init__(self, n=10):
        super().__init__('test_node')
        # Create a timer to give the executor something to do
        self.create_timer(0., callback=lambda: None)


def main(args=None):
    rclpy.init()
    executor = MultiThreadedExecutor()
    node = TestNode()


    # Spin once to give the executor some work to do (creating threads)
    rclpy.spin_once(node, executor=executor)

    print("Shutting down")
    rclpy.shutdown()
    node.destroy_node()
    # This will shutdown the ThreadpoolExecutor
    #executor._executor.shutdown()
    executor.shutdown()

    print(f"Leftover Threads: {threading.enumerate()}")


if __name__ == '__main__':
    main()