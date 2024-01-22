import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from std_msgs.msg import String
import datetime
import rosbag2_py

topics = [
    ['/chatter1', 'std_msgs/msg/String', String],
    ['/chatter2', 'std_msgs/msg/String', String],
    #['/chatter3', 'std_msgs/msg/String', String],
    #['/chatter4', 'std_msgs/msg/String', String],
]


class SimpleBagRecorder(Node):
    def __init__(self,topics):
        super().__init__('simple_bag_recorder')
        self.callbacks = []
        self.subscriptions_list = []
        self.topic_infos = []
        self.writer = rosbag2_py.SequentialWriter()
        self.topics = topics
        storage_options = rosbag2_py._storage.StorageOptions(
            uri=datetime.datetime.now().strftime('rosbag2_%Y_%m_%d-%H_%M_%S'),
            storage_id='sqlite3')
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)

        i = 0
        for topic in self.topics:
            self.topic_infos.append(rosbag2_py._storage.TopicMetadata(
                name=topic[0],
                type=topic[1],
                serialization_format='cdr'))
            self.writer.create_topic(self.topic_infos[i])
            self.subscriptions_list.append( self.create_subscription(
                topic[2],
                topic[0],
                lambda msg, topic=topic[0]:  self.topic_callback(msg, topic),
                10))
            i+=1
    def topic_callback(self, msg,  topic):
        print('Called : ',topic)
        self.writer.write(
            topic,
            serialize_message(msg),
            self.get_clock().now().nanoseconds)


def main(args=None):
    rclpy.init(args=args)
    sbr = SimpleBagRecorder(topics)
    rclpy.spin(sbr)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
