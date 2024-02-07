import rclpy
import rclpy.node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, ParameterValue


class ParamDemo(rclpy.node.Node):
    def __init__(self):
        super().__init__("param_demo")

        self.declare_parameter(
            "param_1",
            "param_1_default_value",
            descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING),
        )

        #self.declare_parameter( # this should generate ValueError
        #    "param_2",
        #    descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING),
        #)

        self.declare_parameter(
            "param_2",
            #value=ParameterValue(),
            descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_NOT_SET, dynamic_typing=True),
        )

def main():
    rclpy.init()
    node = ParamDemo()
    rclpy.spin(node)


if __name__ == "__main__":
    main()