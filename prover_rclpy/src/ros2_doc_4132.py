import rclpy

from std_msgs.msg import String

def main(args=None):
    counter = 0
    rclpy.init(args=args)
    node = rclpy.create_node('test')
    pub = node.create_publisher(String, 'chatter', 10)

    rate1 = node.create_rate(2) # 2Hz
    while rclpy.ok() and counter < 10:
      msg = String()
      msg.data = 'Hello World: {0}'.format(counter)
      counter += 1
      node.get_logger().info('Publishing: "{0}"'.format(msg.data))
      pub.publish(msg)
      rclpy.spin_once(node)

    rate1.sleep() # this can be waken, since last spin will set the event for the timer object.
    rate1.destroy()
    node.get_logger().info('rate1 destroy')

    rate2 = node.create_rate(1) # 1Hz, so that expectation is publish the data with 1Hz, but rate1 timer is still alive
    node.get_logger().info('rate2 created')
    while rclpy.ok() and counter < 20:
      msg = String()
      msg.data = 'Hello World: {0}'.format(counter)
      counter += 1
      node.get_logger().info('Publishing: "{0}"'.format(msg.data))
      pub.publish(msg)
      rclpy.spin_once(node)

    rate2.sleep() # this can be woken, since last spin will set the event for the timer object.

    rate2.destroy()
    node.get_logger().info('rate2 destroy')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
