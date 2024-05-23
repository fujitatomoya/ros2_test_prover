#!/usr/bin/env python3

import signal
import sys

import rclpy
import rclpy.node

from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions

def shutdown_handler():
    print("exit")

def main(args=None):
    def signal_handler(sig, frame):
        # this handler should be called before deferred signal handler
        print("signal receive\n")
        rclpy.shutdown()
        #rclpy.try_shutdown()
    signal.signal(signal.SIGINT, signal_handler)

    rclpy.init(args=args)
    #rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    try:
        node = rclpy.create_node("test_node")

        node.context.on_shutdown(shutdown_handler)
        rclpy.get_default_context().on_shutdown(shutdown_handler)

        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)

    #rclpy.shutdown()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
