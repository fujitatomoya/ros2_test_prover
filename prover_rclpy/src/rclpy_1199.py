#!/usr/bin/env python3

import rclpy
import rclpy.node

from rclpy.node import Node

class TestNode(Node):

    def __init__(self):
        super().__init__('test_node')
        self.get_logger().info("Info log")
        self.get_logger().warn("Warn log")


def main(args=None):
    rclpy.init(args=args)

    node = TestNode()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
