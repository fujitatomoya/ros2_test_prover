import signal
import rclpy

from rclpy.node import Node
from rcl_interfaces.msg import ParameterType


class ParamTest(Node):
    def __init__(self):
        Node.__init__(self, 'param_test')

        # parameter declaration and info
        self._param = self.declare_parameter('param', 'abc').get_parameter_value().string_value
        self.get_logger().info(f'String Param: {self._param}')


def main(args=None):
    def raise_keyboard_int(_, __):
        rclpy.logging.get_logger('param_test_node').info('stopping')
        raise KeyboardInterrupt()

    signal.signal(signal.SIGINT, raise_keyboard_int)

    rclpy.init(args=args)
    node = ParamTest()

    try:
        rclpy.spin_once(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()



if __name__ == '__main__':
    main()