from functools import partial
from rclpy.timer import TimerInfo

import rclpy
import rclpy.node
import rclpy.executors
import rclpy.timer


def main(args=None):
    rclpy.init()
    node = rclpy.node.Node("test_node")
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    executor.spin_once(timeout_sec=0)

    try:
        print("----- test 1")
        # pass, i think correct behavior, info is appended as extra argument.
        def timer_callback(info: TimerInfo):
            print(info)
            print(info.expected_call_time)
            print(info.actual_call_time)

        timer = node.create_timer(1, timer_callback)
        executor.spin_once(2)
        timer.cancel()
        node.destroy_timer(timer)
    finally:
        pass

    try:
        print("----- test 2")
        # pass, i think correct behavior
        def timer_callback(node):
            print(node)

        timer = node.create_timer(1, partial(timer_callback, node))
        executor.spin_once(2)
        timer.cancel()
        node.destroy_timer(timer)
    finally:
        pass

    try:
        print("----- test 3")
        # pass, correct behavior
        def timer_callback(node, executor, info: TimerInfo):
            print(node)
            print(executor)
            print(info)
            print(info.expected_call_time)
            print(info.actual_call_time)

        timer = node.create_timer(1, partial(timer_callback, node, executor))
        executor.spin_once(2)
        node.destroy_timer(timer)
    finally:
        pass

    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
