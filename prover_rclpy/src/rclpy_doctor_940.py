import traceback
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from rclpy.qos import ReliabilityPolicy, QoSProfile, LivelinessPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.qos_event import SubscriptionEventCallbacks
from rclpy.duration import Duration

class DoctorNodeHelp(Node):
    def __init__(self):
        super().__init__('doctor_node_help')
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            lifespan=Duration(seconds=300),
            liveliness=LivelinessPolicy.AUTOMATIC,
            liveliness_lease_duration=Duration(seconds=2.2))
        self.subscribe_bar_node = self.create_subscription(
            Bool, 'bar_health', callback=self.bar_callback, qos_profile=qos_profile,
            event_callbacks=SubscriptionEventCallbacks(liveliness=self.bar_live))
        self.subscribe_foo_node = self.create_subscription(
            Bool, 'foo_health', self.foo_callback, qos_profile=qos_profile,
            event_callbacks=SubscriptionEventCallbacks(liveliness=self.foo_live))
        self.get_logger().info(f'Doctor node started!')
            
    def bar_live(self, events):
        print(f'Bar: {events}')

    def foo_live(self, events):
        print(f'Foo: {events}')

    def bar_callback(self, msg):
        pass
    def foo_callback(self, msg):
        pass

def main(args=None):
    rclpy.init(args=args)
    doctor_node = DoctorNodeHelp()
    rclpy.spin(doctor_node)

    doctor_node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    try:
        main()
    except:
        traceback.print_exc()