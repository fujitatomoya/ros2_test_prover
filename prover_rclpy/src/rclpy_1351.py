import threading
import time

from concurrent.futures import ThreadPoolExecutor

import rclpy
from rclpy.executors import Executor
from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import SingleThreadedExecutor
from rclpy.task import Future


def main(args=None):
    rclpy.init(args=args)
    executor = SingleThreadedExecutor()

    future1 = Future(executor=executor)
    future2 = Future(executor=executor)

    # I believe that we need to use ThreadPoolExecutor here...
    #thread1 = threading.Thread(target=executor.spin_until_future_complete, args=(future1, 10))
    #thread2 = threading.Thread(target=executor.spin_until_future_complete, args=(future2, 10))

    # rolling does not fail with this...
    with ThreadPoolExecutor(max_workers=5) as exe:
        exe.submit(executor.spin_until_future_complete, future1, 10)
        exe.submit(executor.spin_until_future_complete, future2, 10)

        time.sleep(1.0)
        future1.set_result(True)
        future2.set_result(True)

    #thread1.start()
    #time.sleep(0.5)
    #thread2.start()

    #future1.set_result(True)
    #future2.set_result(True)

    #thread1.join()
    #thread2.join()
    executor.shutdown()


if __name__ == '__main__':
    main()
