import rclpy
import sys

def main():
    rclpy.init(args=sys.argv)
    node = rclpy.create_node("test")
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
