#!/usr/bin/env python3

"""
Simple service client example that demonstrates ROS 2 service client functionality.
"""

import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class SimpleServiceClient(Node):
    def __init__(self):
        super().__init__('simple_service_client')

        # Create client
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for service
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.request = AddTwoInts.Request()

    def send_request(self, a, b):
        self.request.a = a
        self.request.b = b
        self.future = self.client.call_async(self.request)
        return self.future


def main(args=None):
    rclpy.init(args=args)
    client = SimpleServiceClient()

    # Get values from command line arguments
    a = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    b = int(sys.argv[2]) if len(sys.argv) > 2 else 2

    future = client.send_request(a, b)

    try:
        while rclpy.ok():
            rclpy.spin_once(client)
            if future.done():
                response = future.result()
                client.get_logger().info(f'Result: {response.sum}')
                break
    except KeyboardInterrupt:
        print('Shutting down service client...')
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()