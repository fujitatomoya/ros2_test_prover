import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.executors import SingleThreadedExecutor

class SimpleNode(Node):
    def __init__(self, n=10):
        super().__init__('simple_node')
        self.n = n
        # Create a one-shot timer with a zero duration to start tasks after initialization
        self.timer = self.create_timer(0, self.start_tasks)

    def start_tasks(self):
        self.timer.cancel()
        # Scheduling tasks to print their execution order
        tasks = [self.executor.create_task(self.print_value(i)) for i in range(self.n)]

    async def print_value(self, value):
        print(f"Task executed with value: {value}")

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()  # Spin the node to process callbacks
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as e:
        node.get_logger().error(f'Unhandled exception in executor: {e}')
    finally:
        executor.shutdown()
        node.destroy_node()

if __name__ == '__main__':
    main()