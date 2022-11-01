import gc
import psutil
import time

import rclpy

def main(args=None):
    rclpy.init(args=args)
    while True:
        node = rclpy.create_node('minimal_client')
        node.destroy_node()
        rss = psutil.Process().memory_info().rss
        gc.collect()
        print(rss)
        time.sleep(1.0)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
