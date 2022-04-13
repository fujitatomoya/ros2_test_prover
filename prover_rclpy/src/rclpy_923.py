#!/usr/bin/env python3

import threading
import time
import traceback
import logging

import rclpy
import rclpy.node
import rclpy.serialization
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
    msg = std_msgs.msg.String()
    msg.data = 'foobar'
    msg_serialized = rclpy.serialization.serialize_message(msg)
    msg_deserialized = rclpy.serialization.deserialize_message(msg_serialized, std_msgs.msg.String)

    try:
        msg_deserialized = rclpy.serialization.deserialize_message(b"", std_msgs.msg.String)
    except Exception as e:
        print(e)
        #logging.error(traceback.format_exc())

    rclpy.shutdown()


if __name__ == '__main__':
    main()