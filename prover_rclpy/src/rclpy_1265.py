from functools import partial
import rclpy
import rclpy.node
import rclpy.executors

def main(args=None):
    rclpy.init()
    node = rclpy.node.Node("test_node")
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    executor.spin_once(timeout_sec=0)

    try:
        print("----- test 1")
        # pass
        def timer_callback(info):
            print(info)

        timer = node.create_timer(1, timer_callback)
        executor.spin_once(2)
        timer.cancel()
        node.destroy_timer(timer)
    finally:
        pass

    try:
        print("----- test 2")
        # pass, but self is going to be TimerInfo
        def timer_callback(self):
            print(self)

        timer = node.create_timer(1, timer_callback)
        executor.spin_once(2)
        timer.cancel()
        node.destroy_timer(timer)
    finally:
        pass

    try:
        print("----- test 3")
        # pass
        def timer_callback(info):
            print(info)

        timer = node.create_timer(1, partial(timer_callback))
        executor.spin_once(2)
        timer.cancel()
        node.destroy_timer(timer)
    finally:
        pass

    try:
        print("----- test 4")
        # pass
        def timer_callback(node):
            print(node)

        timer = node.create_timer(1, partial(timer_callback, node))
        executor.spin_once(2)
        timer.cancel()
        node.destroy_timer(timer)
    finally:
        pass

    try:
        print("----- test 5")
        # pass
        def timer_callback(node, executor):
            print(node)
            print(executor)

        timer = node.create_timer(1, partial(timer_callback, node, executor))
        executor.spin_once(2)
        node.destroy_timer(timer)
    finally:
        pass

    try:
        print("----- test 6")
        # pass
        def timer_callback(node, info):
            print(info)
            print(node)

        timer = node.create_timer(1, partial(timer_callback, node))
        executor.spin_once(2)
        timer.cancel()
        node.destroy_timer(timer)
    finally:
        pass

    try:
        print("----- test 7")
        # pass but node is gonna be TimerInfo
        def timer_callback(node):
            print(node)

        timer = node.create_timer(1, partial(timer_callback))
        executor.spin_once(2)
        timer.cancel()
        node.destroy_timer(timer)
    finally:
        pass

    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
