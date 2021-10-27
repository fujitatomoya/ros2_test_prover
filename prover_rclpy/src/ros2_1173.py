# Refer to the following API:
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

class ParametersNode(Node):
    def __init__(self):
        super().__init__('ParametersNode', automatically_declare_parameters_from_overrides=True)
        self.get_logger().info("Initialized Parameters Node.")
        #self._timer = self.create_timer(1, self.printParam)
        self.declare_parameter("self_declared_param", 45)
        # self.set_parameters_callback(self.onParameterChange)
        # self.add_on_set_parameters_callback(self.onParameterChange)

    def printParam(self):
        params = ["test", "group.list_param", "group.string_param", "group2.complex_param",
                  "group.group2.float_param", "group.group2.bool_param"]
        for p in params:
            try:
                obj = self.get_parameter(p)
                value = obj._value
                self.get_logger().info(f"Parameter {p}: {value} ({type(value)}), ({obj.type_})")
            except rclpy.exceptions.ParameterNotDeclaredException:
                self.get_logger().info(f"Parameter {p} not defined!")

    def onParameterChange(self, params):
        txt = "\n".join(f"   {p.name}={p.value}" for p in params)
        self.get_logger().info("Parameter changed callback called! Changed parameters:\n" + txt)
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)

    node = ParametersNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
