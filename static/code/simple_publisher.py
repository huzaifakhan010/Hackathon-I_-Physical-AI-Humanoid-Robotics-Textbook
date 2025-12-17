#!/usr/bin/env python3

"""
Simple publisher example that demonstrates basic ROS 2 publisher functionality.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')

        # Create publisher
        self.publisher = self.create_publisher(String, 'chatter', 10)

        # Create timer to publish every 0.5 seconds
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    node = SimplePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Shutting down publisher node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()