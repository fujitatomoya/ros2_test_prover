import asyncio
import time

import rclpy


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
        await task
        print("Awaiting task")
    except FileNotFoundError:
        print("Error but should keep going")
    return True


def main(args=None):
    rclpy.init()

    node = rclpy.create_node("test_wait")
    executor = rclpy.get_global_executor()
    executor.add_node(node)
    
    # task1
    task = executor.create_task(test_1)
    executor.spin_until_future_complete(task)
    assert task.result()

    #task2
    task = executor.create_task(test_2, executor)
    executor.spin_until_future_complete(task)
    assert task.result()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
