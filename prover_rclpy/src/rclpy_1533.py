import rclpy
from rclpy.node import Node

#from sensor_msgs.msg import Imu # comment out this line to fix the issue?
import time

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.get_logger().info('Initialised')
        
        self.timer = self.create_timer(1, self.timer_callback)

    # Can be unreliable and unpredictable.
    # Python's garbage collector determines when to call it, which might not happen immediately when you expect.
    def __del__(self):
        print("Deleting node")

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.shutdown()

    def shutdown(self):
        print("Shutting down node")

    def timer_callback(self):
        self.get_logger().info('Called!')
        time.sleep(0.9)

def main(args=None):
    rclpy.init(args=args)
    with MinimalPublisher() as node:
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info('Keyboard Interrupt (SIGINT)')
            pass


if __name__ == '__main__':
    main()
