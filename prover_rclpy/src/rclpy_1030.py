import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult


class MinimalParam(Node):
    def __init__(self):
        super().__init__('minimal_param_node')
        self.log = self.get_logger()
        self.declare_parameter('my_parameter', Parameter.Type.STRING)

        self.add_on_set_parameters_callback(self.callback)

    def callback(self, parameters):
        for p in parameters:
            if p.name == 'my_parameter':
                self.log.info(f"Got my_parameter: {p.value}")
        return SetParametersResult(successful=True)


def main():
    rclpy.init()
    node = MinimalParam()
    rclpy.spin(node)

if __name__ == '__main__':
    main()