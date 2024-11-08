
import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


def sleep_async(node: Node, time: float) -> Future:
    """
    Sleeps for a given amount of time.

    Creates a timer with the sleep time as frequency and destroys it on the first callback.

    args:
        node: The node to sleep on. Future will be created with the nodes executor.
        time: The time to sleep in seconds
    
    returns:
        A future that will be done after the given amount of time.
    """
    future = Future(executor=node.executor)
    timer = node.create_timer(timer_period_sec=time, callback=lambda: _timer_callback(future), callback_group=MutuallyExclusiveCallbackGroup())

    def _timer_callback(future: Future):
        future.set_result(None)
        node.destroy_timer(timer)

    return future

async def sleep(self, seconds):
    for i in range(1, seconds+1):
        await sleep_async(self, 1)
        print(f"Slept for {i} seconds")


def main(args=None):
    rclpy.init()

    node = rclpy.create_node("test")
    executor = rclpy.get_global_executor()
    executor.add_node(node)
    task = executor.create_task(sleep, node, 5)

    task.cancel()
    print(f"{task.cancelled()=}")
    executor.spin_until_future_complete(task)

    print(f"{task.done()=}")
    print(f"{task.cancelled()=}")

if __name__ == '__main__':
    main()
