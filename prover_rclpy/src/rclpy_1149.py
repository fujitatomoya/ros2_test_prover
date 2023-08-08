import gi
import rclpy
import sys


def main():
    rclpy.init(args=sys.argv)
    print("init")

    gi.require_version("Gst", "1.0")
    from gi.repository import Gst

    print("before node")
    node = rclpy.create_node("gstreamer_crash")
    print("after node")
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
