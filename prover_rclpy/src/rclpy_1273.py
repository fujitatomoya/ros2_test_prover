from std_msgs.msg import String
from rclpy.serialization import deserialize_message, serialize_message

# Serialized message data
#serialized_data = b'\x01\x03\x00\x00\x00\xb3\xa8\xbfFb\x89\xc8\x17\x00\x01\x00\x00I\x00\x00\x00Hello, world! Time(nanoseconds=1713770713401001821, clock_type=ROS_TIME)\x00\x00\x00\x00'

def main():
    msg = String()
    msg.data = 'Hello, world!'
    msg_serialized = serialize_message(msg)
    print(msg_serialized)
    msg_deserialized = deserialize_message(msg_serialized, String)
    print(msg_deserialized)

    #message_type = String()
    #decoded_data = deserialize_message(serialized_data, String)
    #print(decoded_data)

if __name__ == "__main__":
    main()
