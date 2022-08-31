import rclpy
import rclpy.node
import rclpy.executors

def main(args=None):
    rclpy.init()
    node = rclpy.node.Node("test_node")
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    future = rclpy.Future()

    async def work():
        node.create_timer(0.1, lambda: future.set_result(True))
        await future

    task = executor.create_task(work())
    #executor.spin_until_future_complete(task, 1)  # NG
    executor.spin_until_future_complete(task, None)  # OK
    print(task.done())

    rclpy.shutdown()


if __name__ == '__main__':
    main()
