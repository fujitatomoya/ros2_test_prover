from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import Bool
from rclpy.duration import Duration
import rclpy


class SubNode(Node):
    def __init__(self):
        super().__init__("sub_node")

        qos = QoSProfile(
            depth=5,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            lifespan=Duration(seconds=2),
        )
        print(qos)
        self.sub = self.create_subscription(
            Bool,
            "/some_topic",
            self.my_callback,
            qos,
        )

    def my_callback(self, msg: Bool):
        self.get_logger().info("Received: {}".format(msg.data))


def main(args=None):
    rclpy.init(args=args)
    node = SubNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()