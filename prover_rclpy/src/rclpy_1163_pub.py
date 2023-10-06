import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from std_msgs.msg import Bool


class PubNode(Node):
    def __init__(self):
        super().__init__("pub_node")

        qos = QoSProfile(
            depth=5,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        self.pub = self.create_publisher(Bool, "/some_topic", qos)
        self.pub.publish(Bool(data=False))


def main(args=None):
    rclpy.init(args=args)
    node = PubNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()