import sys

from prover_interfaces.srv import Huge
import rclpy
from rclpy.node import Node

import time


class MinimalServerAsync(Node):

    def __init__(self):
        super().__init__('minimal_server')
        self.cli = self.create_service(Huge, 'huge', self.callback)

    def callback(self, request, response):
        self.get_logger().info('Incoming request...')
        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_server = MinimalServerAsync()

    try:
        rclpy.spin(minimal_server)
    except KeyboardInterrupt:
        pass

    minimal_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()