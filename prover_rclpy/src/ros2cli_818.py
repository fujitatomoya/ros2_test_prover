from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import qos_profile_services_default

class ServiceProvider(Node):

    def __init__(self):
        super().__init__('service_provider')
        # qos_profile_sensor_data (keep_last, depth is 5 and reliability is best_effort)
        # qos_profile_services_default (keep_last, depth is 10 and reliability is reliable)
        self.srv = self.create_service(
            #AddTwoInts, 'my_add_two_ints', self.add_two_ints_callback, qos_profile=qos_profile_services_default)
            AddTwoInts, 'my_add_two_ints', self.add_two_ints_callback, qos_profile=qos_profile_sensor_data)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request a: %d b: %d' % (request.a, request.b))
        self.get_logger().info('Outbound result : %d' % (response.sum))

        return response


def main(args=None):
    rclpy.init(args=args)

    service = ServiceProvider()

    rclpy.spin(service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()