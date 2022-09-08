import time
import rclpy

from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from action_tutorials_interfaces.action import Fibonacci
from example_interfaces.srv import AddTwoInts


class SubNode(Node):
    def __init__(self):
        super().__init__("sub_node")
        self.execute_callback_group = MutuallyExclusiveCallbackGroup()
        self.action_server = ActionServer(
            self,
            Fibonacci,
            "fibonacci",
            self.action_callback,
            callback_group=self.execute_callback_group,
        )
        self.service_server = self.create_service(
            AddTwoInts,
            "add_two_ints",
            self.service_callback,
            callback_group=self.execute_callback_group,
        )

    def action_callback(self, target_handle):
        self.get_logger().info("Received action request")
        time.sleep(2)
        result = Fibonacci.Result()
        result.sequence = [0, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55]
        target_handle.succeed()
        self.get_logger().info("Action request processed")
        return result

    def service_callback(self, request, response):
        self.get_logger().info("Received service request")
        time.sleep(1)
        response.sum = request.a + request.b
        self.get_logger().info("Service request processed")
        return response


def main(args=None):
    rclpy.init(args=args)

    sub_node = SubNode()

    executor = MultiThreadedExecutor()
    rclpy.spin(sub_node, executor)


if __name__ == "__main__":
    main()