import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
import time

class DummyNode(Node):
    def __init__(self):
        super().__init__('dummy_node')
        self.timer_called = False
        self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("Timer callback called.")
        self.timer_called = True

def main():
    rclpy.init()

    node = DummyNode()
    executor = SingleThreadedExecutor()
    #executor = MultiThreadedExecutor()
    executor.add_node(node)

    # Spin once manually
    rclpy.spin_once(node, executor=executor, timeout_sec=0.05)  # This succeeded
    #rclpy.spin_once(node, timeout_sec=0.05)  # This failed

    # Now try to spin the executor
    print("Spinning executor after spin_once...")
    executor.spin_once(timeout_sec=1.0)

    time.sleep(1)  # give some time to consolidate the thread pool

    if not node.timer_called:
        print("Timer callback was NOT called after spin_once (node likely removed!)")
    else:
        print("Timer callback called.")

    rclpy.shutdown()

if __name__ == "__main__":
    main()
