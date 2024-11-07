import rclpy


async def test_cancel():
    future = rclpy.task.Future()
    future.cancel()

    print(future, future.cancelled(), future.done())

    return await future


def main(args=None):
    rclpy.init()

    node = rclpy.create_node("test")

    def timer_callback():
        node.get_logger().info("Logging timer fired.")

    node.create_timer(1.0, timer_callback)

    executor = rclpy.get_global_executor()
    executor.add_node(node)
    task = executor.create_task(test_cancel)

    print(f"{task.cancelled()=}")
    #task.cancel()
    executor.spin_until_future_complete(task)
    print(f"{task.done()=}")
    print(f"{task.cancelled()=}")

if __name__ == '__main__':
    main()
