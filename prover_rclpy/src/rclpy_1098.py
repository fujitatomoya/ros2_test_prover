import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.executors import MultiThreadedExecutor


async def throwing_task():
    raise FileNotFoundError()


async def test_1():
    try:
        await throwing_task()
    except FileNotFoundError:
        print("Exception caught")
        pass
    return True


async def test_2(executor):
    task = executor.create_task(throwing_task)
    try:
        print("Awaiting task (before)")
        await task
        print("Awaiting task (after)")
    except FileNotFoundError:
        print("Error but should keep going")
    return True


def main(args=None):
    rclpy.init()

    node = rclpy.create_node("test_wait")
    executor = SingleThreadedExecutor()
    #executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    # task1 (this is no problem, exception is caught in user created task)
    task = executor.create_task(test_1)
    executor.spin_until_future_complete(task, timeout_sec=3.0)
    assert task.result()

    # task2
    task = executor.create_task(test_2, executor)
    executor.spin_until_future_complete(task, timeout_sec=3.0)
    assert not task.result()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
