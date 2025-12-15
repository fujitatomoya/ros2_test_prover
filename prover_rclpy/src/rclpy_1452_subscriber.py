import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rosgraph_msgs.msg import Clock
import time
from collections import deque
import numpy as np

class ClockSubscriberWithTimer(Node):
    def __init__(self):
        super().__init__('clock_sub_with_timer')

        # Subscribe to /clock
        if True:
            self.subscription = self.create_subscription(
                Clock,
                '/clock',
                self.clock_callback,
                10
            )

        # Timer
        timer_hz = 10
        self.timer_period = 1/timer_hz  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Period tracking
        self.timer_periods = deque(maxlen=10)
        self.clock_periods = deque(maxlen=10)

        # Last timestamps
        self.last_timer_time = time.time()
        self.last_clock_time = time.time()

        # Throttle
        self.log_throttle_sec = 5.0

        self.get_logger().info("Node started with multithreaded executor")

    def timer_callback(self):

        now = time.time()
        period = now - self.last_timer_time
        self.last_timer_time = now
        self.timer_periods.append(period)

        if len(self.timer_periods) < 5:
            return

        avg = np.average(self.timer_periods)
        std =  np.std(self.timer_periods)
        freq = 1.0 / avg if avg > 0 else float('inf')

        self.get_logger().info(
            f"[timer] freq avg: {freq:.2f} Hz | period avg: {avg * 1000:.1f} ms ± {std * 1000:.1f} ms",
            throttle_duration_sec=self.log_throttle_sec
        )

    def clock_callback(self, msg: Clock):

        now = time.time()
        period = now - self.last_clock_time
        self.last_clock_time = now
        self.clock_periods.append(period)

        if len(self.clock_periods) < 5:
            return

        avg = np.average(self.clock_periods)
        std =  np.std(self.clock_periods)
        freq = 1.0 / avg if avg > 0 else float('inf')

        self.get_logger().info(
            f"[clock] freq avg: {freq:.2f} Hz | period avg: {avg * 1000:.1f} ms ± {std * 1000:.1f} ms",
            throttle_duration_sec=self.log_throttle_sec
        )


def main(args=None):
    rclpy.init(args=args)

    node = ClockSubscriberWithTimer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        exit(0)

if __name__ == '__main__':
    main()

