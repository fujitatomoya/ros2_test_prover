#!/usr/bin/env python3

import threading
import time

import rclpy
import rclpy.node
import std_msgs.msg


def thread2(node):
    while True:
        sub = node.create_subscription(
            std_msgs.msg.Bool,
            "rclpy_760_mre",
            lambda msg: None,
            10)
        time.sleep(0.01)
        node.destroy_subscription(sub)


def main():
    rclpy.init()
    node = rclpy.node.Node("rclpy_760_mre")

    thread = threading.Thread(target=thread2, args=(node,), daemon=True)
    thread.start()

    rclpy.spin(node)
    rclpy.shutdown()

    thread.join()


if __name__ == '__main__':
    main()