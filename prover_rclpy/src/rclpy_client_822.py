import sys

from prover_interfaces.srv import Huge
import rclpy
from rclpy.node import Node

import time


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(Huge, 'huge')
        while not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Huge.Request()

    def send_request(self):
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    for _ in range(1000):
        minimal_client.get_logger().info('Sending async request...')
        minimal_client.send_request()

        while rclpy.ok():
            rclpy.spin_once(minimal_client)
            time.sleep(0.1)
            if minimal_client.future.done():
                try:
                    response = minimal_client.future.result()
                except Exception as e:
                    minimal_client.get_logger().info(
                        'Service call failed %r' % (e,))
                else:
                    minimal_client.get_logger().info('Result received!!!')
                break


    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()