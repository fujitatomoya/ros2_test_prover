import rclpy
import sys

def main():
    rclpy.init(args=sys.argv)
    node = rclpy.create_node("test")
    node.declare_parameter('param', rclpy.Parameter.Type.INTEGER)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
