import sys
import rclpy

from rclpy.node import Node
from rclpy.parameter import Parameter

# Setup
# Both ns1.bar and ns2.bar parameters are declared.
#   * ns1.bar is declared using the namespace argument of declare_parameters()
#   * ns2.bar is declared with the namespace argument blank, but the parameter name prefixed with "ns1."

# Expected Result:
# Both declaration modes should have equivalent behavior. Both modes do in fact result in the nodes _parameters
# collection having the expected namespace prefix.
# This example program should set params {foo, ns1.bar, ns2.bar} to {p1, p2, p3} respectively.

# Actual Result:
# Parameters using the namespace argument of declare_parameters() cannot be loaded from sysargs (using -p) or
# from yaml config files.
# This example program outputs params {foo, ns1.bar, ns2.bar} as {p1, default_bar1, p3} respectively.

# Observations:
#   * ns1.bar fails (unset, shows default)
#   * ns2.bar works (shows value from parameter file)


# Run with a parameter file and see:
# python3 nsparamtest.py --ros-args --params-file nsparamtest.yaml

# Run with parameters on the command line:
# python3 nsparamtest.py --ros-args -p foo:=p1 -p ns1.bar:=p2 -p ns2.bar:=p3

class NamespaceParamTest(Node):

    def __init__(self):
        super().__init__('nstest')

        # prm "foo" without namespace
        self.declare_parameters(
            namespace="",
            parameters=[("foo", "default_foo")])

        # FAILS - always shows default value
        # using the namespace argument declares the parameter with the
        # correct name but the parameter will not load from sysargs or
        # yaml parameter file
        self.declare_parameters(
            namespace="ns1",
            parameters=[("bar", "default_bar1")],
        )

        # WORKS - can load from yaml or command line
        # this parameter works because instead of using the namespace
        # argument we apply the namespace directly to each parameter
        self.declare_parameters(
            namespace="",
            parameters=[("ns2.bar", "default_bar2")],
        )

        print(f'foo = {self.get_parameter("foo").value}')
        print(f'ns1.bar = {self.get_parameter("ns1.bar").value}')
        print(f'ns2.bar = {self.get_parameter("ns2.bar").value}')

        # print our node parameters and the current value
        print('node parameters => ', [(k, p.value) for k, p in self._parameters.items()])


def main():
    rclpy.init(args=sys.argv)
    node = NamespaceParamTest()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

### nsparamtest.yaml
# nstest:
#   ros__parameters:
#     foo: p1
#     ns1:
#       bar: p2
#     ns2:
#       bar: p3

### CLI to test
# ros2 run prover_rclpy rclpy_1132 --ros-args --params-file nsparamtest.yaml
# ros2 run prover_rclpy rclpy_1132 --ros-args -p foo:=p1 -p ns1.bar:=p2 -p ns2.bar:=p3