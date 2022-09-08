import time
import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient

from action_tutorials_interfaces.action import Fibonacci
from example_interfaces.srv import AddTwoInts


class PubNode(Node):
    def __init__(self):
        super().__init__("pub_node")
        self.action_client = ActionClient(self, Fibonacci, "fibonacci")
        self.action_client.wait_for_server()
        self.service_client = self.create_client(AddTwoInts, "add_two_ints")
        self.service_client.wait_for_service()

    def send_action(self):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = 10
        print("Sending action")
        self._send_goal_future = self.action_client.send_goal_async(goal_msg)

    def send_service(self):
        request = AddTwoInts.Request()
        request.a = 1
        request.b = 2
        print("Sending service")
        self.future = self.service_client.call_async(request)


def main(args=None):
    rclpy.init(args=args)

    pub_node = PubNode()

    pub_node.send_service()
    pub_node.send_service()

    time.sleep(3)

    pub_node.send_action()
    pub_node.send_action()

    time.sleep(3)


if __name__ == "__main__":
    main()