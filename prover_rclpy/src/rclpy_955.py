import rclpy
from rclpy.node import Node

class MonitoTopics(Node):

    def __init__(self):
        super().__init__('monitor_topics')
        self._timer = self.create_timer(
            timer_period_sec=0.1,
            callback=self._monitor_topics)

    def _monitor_topics(self):
        for topic_name, topic_types in self.get_topic_names_and_types():
            del topic_types
            for topic_info in self.get_publishers_info_by_topic(topic_name):
                del topic_info
            for topic_info in self.get_subscriptions_info_by_topic(topic_name):
                del topic_info


def main(args=None):
    rclpy.init(args=args)

    node = MonitoTopics()
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()