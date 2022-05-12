import traceback, time, sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Int32, Bool
from rclpy.qos import ReliabilityPolicy, QoSProfile

from rclpy.qos import LivelinessPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.duration import Duration

class FooNode(Node):
    def __init__(self):
        super().__init__('foo_node')
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            lifespan=Duration(seconds=300),
            liveliness=LivelinessPolicy.AUTOMATIC,
            liveliness_lease_duration=Duration(seconds=2))
        self.publish_foo_health = self.create_publisher(
            Bool, 'foo_health', qos_profile=qos_profile)   

        self.get_logger().info(f'Foo node started!')
    
def main(args=None):
    rclpy.init(args=args)

    foo_node = FooNode()
    rclpy.spin(foo_node)

    foo_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except:
        traceback.print_exc()
