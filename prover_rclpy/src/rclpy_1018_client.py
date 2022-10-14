from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


class MinimalClient(Node):
    def __init__(self):
        super().__init__("minimal_client_async")
        self.cb = None
        self.timer_cb = MutuallyExclusiveCallbackGroup()
        self.cli = self.create_client(
            AddTwoInts, "add_two_ints", callback_group=self.cb
        )
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.req = AddTwoInts.Request()

        # Create a timer for the main loop execution
        self.timer = self.create_timer(
            1.0, self.main_loop, callback_group=self.timer_cb
        )

    async def send_request(self, a, b):
        self.get_logger().info("send_request: enter")
        self.req.a = a
        self.req.b = b

        # Only works once, then never executes the timer again
        #self.future = self.cli.call_async(self.req)
        #rclpy.spin_until_future_complete(self, self.future)
        #self.get_logger().info("send_request: exit")
        #return self.future.result()

        # Suggested example by @llapx
        #self.future = self.cli.call_async(self.req)
        #while rclpy.ok():
        #    if self.future.done() and self.future.result():
        #        self.get_logger().info("send_request: exit")
        #        return self.future.result()

        return await self.cli.call_async(self.req)

        # Used to work, but blocks now
        # return self.cli.call(self.req)

    async def main_loop(self) -> None:
        response = await self.send_request(4, 2)
        self.get_logger().info(
            "Result of add_two_ints: for %d + %d = %d" % (4, 2, response.sum)
        )


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClient()

    executor = MultiThreadedExecutor()
    executor.add_node(minimal_client)
    executor.spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
