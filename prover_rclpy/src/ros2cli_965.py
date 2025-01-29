import rclpy
import rclpy.node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType


class ParamDemo(rclpy.node.Node):
    def __init__(self):
        super().__init__("param_demo_py")

        #self.declare_parameter( # this should generate ValueError but humble
        #    "param_2_py",
        #    descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING),
        #)

        self.declare_parameter(
            "param_2_py",
            #value=ParameterValue(),
            descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_NOT_SET, dynamic_typing=True),
        )

def main():
    rclpy.init()
    node = ParamDemo()
    rclpy.spin(node)


if __name__ == "__main__":
    main()