#!/usr/bin/env python3

"""
Example demonstrating ROS 2 parameters usage.
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String


class ParameterExample(Node):
    def __init__(self):
        super().__init__('parameter_example')

        # Declare parameters
        self.declare_parameter('robot_name', 'default_robot')
        self.declare_parameter('max_speed', 1.0)

        # Get parameter values
        self.robot_name = self.get_parameter('robot_name').value
        self.max_speed = self.get_parameter('max_speed').value

        # Create publisher
        self.publisher = self.create_publisher(String, 'robot_status', 10)

        # Create timer
        self.timer = self.create_timer(1.0, self.status_callback)

        # Set parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        """Callback for parameter changes"""
        for param in params:
            if param.name == 'robot_name' and param.type_ == Parameter.Type.STRING:
                self.robot_name = param.value
                self.get_logger().info(f'Robot name changed to: {param.value}')
            elif param.name == 'max_speed' and param.type_ == Parameter.Type.DOUBLE:
                self.max_speed = param.value
                self.get_logger().info(f'Max speed changed to: {param.value}')

        return SetParametersResult(successful=True)

    def status_callback(self):
        msg = String()
        msg.data = f'{self.robot_name} operating at {self.max_speed} m/s'
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ParameterExample()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Shutting down parameter example...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()