#!/usr/bin/env python3

"""
Simple service server example that demonstrates ROS 2 service functionality.
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class SimpleServiceServer(Node):
    def __init__(self):
        super().__init__('simple_service_server')

        # Create service
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_callback)

    def add_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {request.a} + {request.b} = {response.sum}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SimpleServiceServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Shutting down service server...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()