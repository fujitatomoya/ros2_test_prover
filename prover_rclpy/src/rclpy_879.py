#!/usr/bin/env python3

import threading
import time

import rclpy
import rclpy.node
import std_msgs.msg

from rcl_interfaces.msg import FloatingPointRange
from rcl_interfaces.msg import IntegerRange
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import GetParameters
from rclpy.parameter import Parameter

def main():
    rclpy.init()
    node = rclpy.node.Node("rclpy_879")
    node.declare_parameter(
        'foo',
        'hello',
        ParameterDescriptor(
            name='foo',
            type=ParameterType.PARAMETER_STRING,
            additional_constraints='some constraints',
            read_only=True,
            floating_point_range=[FloatingPointRange(from_value=-2.0, to_value=2.0, step=0.1)],
            integer_range=[IntegerRange(from_value=-10, to_value=10, step=2)]
        )
    )
    node.declare_parameter(
        'bar',
        10,
        ParameterDescriptor(
            name='bar',
            type=ParameterType.PARAMETER_DOUBLE,
            additional_constraints='some more constraints',
            read_only=True,
            floating_point_range=[FloatingPointRange(from_value=-20.0, to_value=20.0, step=0.5)],
            integer_range=[IntegerRange(from_value=-20, to_value=20, step=3)]
        )
    )
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()