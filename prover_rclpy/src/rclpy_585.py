import rclpy
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.task import Future
import os

latching_qos = QoSProfile(depth=1,
    durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)

def main():

    rclpy.init()
    
    # Set up publisher
    pubnode = Node('pubnode_' + str(os.getpid()))
    pub1 = pubnode.create_publisher(String, 'topic1', latching_qos)
    msg1 = String()
    msg1.data = "hello1"
    pubnode.get_logger().info("Publishing hello1")
    pub1.publish(msg1)

    # Set up listener
    future_msgs = Future()
    subnode = Node('subnode_' + str(os.getpid()))
    subnode.create_subscription(String, 'topic1', lambda msg : ([
            subnode.get_logger().info("Received message on topic1"),
            future_msgs.set_result(msg)
    ]), latching_qos)

    # Start nodes
    exe = MultiThreadedExecutor()
    exe.add_node(pubnode)
    exe.add_node(subnode)

    future_msgs.add_done_callback(lambda fut : print("Future is done"))
    exe.spin_until_future_complete(future_msgs)

if __name__ == '__main__':
    main()