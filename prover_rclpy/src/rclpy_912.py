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
    node = rclpy.node.Node("rclpy_912")

    node.declare_parameter(
        #"str_list", [""], # this works as string array
        "str_list", [], # this deduced as byte array always
        #name = "str_list", # if default is not provided, rclpy applies the descriptor info
        ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING_ARRAY,
            description="optionally some strings",
        )
    )
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()