import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future

# Random service example.
from rcl_interfaces.srv import GetParameters


class AsyncClient(Node):
    def __init__(self):
        super().__init__("async_client_node")
        callback_group = ReentrantCallbackGroup()

        self.cli = self.create_client(
            GetParameters, "test_service", callback_group=callback_group
        )
        self.service_call_timer = self.create_timer(
            1.0, self.call_service_cb, callback_group=callback_group
        )
        self.log_timer = self.create_timer(
            2.0,
            self.log_cb,
            callback_group=callback_group,
        )

    def log_cb(self):
        self.get_logger().info("Logging timer fired.")

    async def call_service_cb(self):
        self.service_call_timer.cancel()

        self.get_logger().info("Calling service")
        # I know there's no service, this is intentional and I expect this task to not complete,
        # but other tasks should still be able to complete!
        response = await self.cli.call_async(GetParameters.Request())
        self.get_logger().info(f"Service call response: {response.message}")


def main(args=None):
    rclpy.init()
    node = AsyncClient()

    # If I run this, my log_cb is called.
    #rclpy.spin_until_future_complete(node, Future())
    # If I run this, my log_cb isn't called!
    rclpy.spin_until_future_complete(node, Future(), timeout_sec=5.0)


if __name__ == '__main__':
    main()