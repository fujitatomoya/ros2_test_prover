#!/usr/bin/env python3

import threading
import time

import rclpy
import rclpy.node
import std_msgs.msg

from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time_source import USE_SIM_TIME_NAME

class TestNode(Node):

    def __init__(self):
        super().__init__('test_node')
        #self.assertTrue(self.has_parameter(USE_SIM_TIME_NAME))
        #self.assertFalse(self.get_parameter(USE_SIM_TIME_NAME).value)
        # Set `use_sim_time` parameter enabled
        results = self.set_parameters([
            #Parameter(USE_SIM_TIME_NAME, Parameter.Type.BOOL, False)
            Parameter(USE_SIM_TIME_NAME, Parameter.Type.BOOL, True)
        ])
        #self.assertTrue(all(isinstance(result, SetParametersResult) for result in results))
        #self.assertTrue(all(result.successful for result in results))
        #self.assertEqual(self.get_parameter(USE_SIM_TIME_NAME).value, True)

        def timer_callback():
            time_now = self.get_clock().now()
            self.get_logger().info('Current time: {}'.format(time_now.to_msg()))

        # Set timer to expire in 2 seconds
        timer = self.create_timer(2.0, timer_callback)


def main(args=None):
    rclpy.init(args=args)

    node = TestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
