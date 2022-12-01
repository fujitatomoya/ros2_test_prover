import gc
import psutil
import time

import rclpy
from rclpy.qos import QoSPolicyKind
from rclpy.qos_overriding_options import QoSOverridingOptions

from std_msgs.msg import String

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('qos_override_node')
    qos_override_opts = QoSOverridingOptions(
        policy_kinds = (
            QoSPolicyKind.HISTORY,
            QoSPolicyKind.DEPTH,
            QoSPolicyKind.RELIABILITY,
            QoSPolicyKind.DEADLINE, # failure with TypeError: 'int' object is not callable
            QoSPolicyKind.LIFESPAN, # failure with TypeError: 'int' object is not callable
            QoSPolicyKind.LIVELINESS_LEASE_DURATION,  # failure with TypeError: 'int' object is not callable
            )
        )
    pub = node.create_publisher(
        String, 'qos_overrides_chatter', 10, qos_overriding_options = qos_override_opts
        )
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
