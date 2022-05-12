import traceback, time
import rclpy

from rclpy.node import Node
from std_msgs.msg import String, Bool

from rclpy.qos import ReliabilityPolicy, QoSProfile, LivelinessPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.duration import Duration

class BarNode(Node):
    def __init__ (self):
        super().__init__('bar_node')
        QoS_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            lifespan=Duration(seconds=300),
            liveliness=LivelinessPolicy.AUTOMATIC,
            liveliness_lease_duration=Duration(seconds=2))
        self.publish_bar_health = self.create_publisher(
            Bool, 'bar_health', qos_profile=QoS_profile)
        
        self.get_logger().info(f'Bar node started!')

def main(args=None):

    rclpy.init(args=args)
    bar_node = BarNode()
    rclpy.spin(bar_node)

    bar_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except:
        traceback.print_exc()
