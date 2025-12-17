---
sidebar_position: 3
---

# rclpy Usage Guide

This guide provides practical examples and best practices for using rclpy, the Python client library for ROS 2. It covers common patterns and techniques for building ROS 2 nodes in Python.

## Getting Started with rclpy

### Basic Node Structure

Every rclpy node follows this basic pattern:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        # Initialize the node with a name
        super().__init__('node_name')

        # Node initialization code goes here
        # (create publishers, subscribers, services, etc.)

def main(args=None):
    # Initialize the ROS 2 client library
    rclpy.init(args=args)

    # Create the node
    node = MyNode()

    try:
        # Keep the node running and processing callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        pass
    finally:
        # Clean up resources
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Creating Publishers

Publishers send messages to topics:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')

        # Create a publisher
        # Parameters: message_type, topic_name, queue_size
        self.publisher = self.create_publisher(String, 'topic_name', 10)

        # Optional: create a timer to publish periodically
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        # Create and populate message
        msg = String()
        msg.data = f'Hello World: {self.i}'

        # Publish the message
        self.publisher.publish(msg)

        # Log the action
        self.get_logger().info(f'Publishing: "{msg.data}"')

        # Increment counter
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Creating Subscribers

Subscribers receive messages from topics:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')

        # Create a subscription
        # Parameters: message_type, topic_name, callback, queue_size
        self.subscription = self.create_subscription(
            String,           # Message type
            'topic_name',     # Topic name
            self.listener_callback,  # Callback function
            10                # Queue size
        )

        # Prevent unused variable warning
        self.subscription

    def listener_callback(self, msg):
        # This function is called when a message arrives
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Creating Services

Services provide request-response communication:

### Service Server
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class ServiceServerNode(Node):
    def __init__(self):
        super().__init__('service_server')

        # Create a service
        # Parameters: service_type, service_name, callback
        self.srv = self.create_service(
            AddTwoInts,           # Service type
            'add_two_ints',       # Service name
            self.add_two_ints_callback  # Callback function
        )

    def add_two_ints_callback(self, request, response):
        # Process the request
        response.sum = request.a + request.b

        # Log the action
        self.get_logger().info(f'Returning {request.a} + {request.b} = {response.sum}')

        # Return the response
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ServiceServerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Client
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class ServiceClientNode(Node):
    def __init__(self):
        super().__init__('service_client')

        # Create a client
        # Parameters: service_type, service_name
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for the service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        # Create a request object
        self.request = AddTwoInts.Request()

    def send_request(self, a, b):
        # Set the request parameters
        self.request.a = a
        self.request.b = b

        # Call the service asynchronously
        self.future = self.cli.call_async(self.request)

        # Add callback for when response arrives
        self.future.add_done_callback(self.response_callback)

        return self.future

    def response_callback(self, future):
        # Process the response when it arrives
        try:
            response = future.result()
            self.get_logger().info(f'Result of add_two_ints: {response.sum}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    client = ServiceClientNode()

    # Send a request
    future = client.send_request(1, 2)

    try:
        # Keep spinning until the response comes back
        while rclpy.ok():
            rclpy.spin_once(client)
            if future.done():
                try:
                    response = future.result()
                    client.get_logger().info(f'Result: {response.sum}')
                except Exception as e:
                    client.get_logger().error(f'Service call failed: {e}')
                break
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Working with Parameters

Nodes can declare and use parameters:

```python
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values
        self.declare_parameter('param_name', 'default_value')
        self.declare_parameter('int_param', 42)
        self.declare_parameter('double_param', 3.14)
        self.declare_parameter('bool_param', True)

        # Get parameter values
        self.param_value = self.get_parameter('param_name').value
        self.int_value = self.get_parameter('int_param').value
        self.double_value = self.get_parameter('double_param').value
        self.bool_value = self.get_parameter('bool_param').value

        # Set callback for parameter changes
        self.add_on_set_parameters_callback(self.parameters_callback)

    def parameters_callback(self, params):
        """Callback for parameter changes"""
        for param in params:
            if param.name == 'param_name' and param.type_ == Parameter.Type.STRING:
                self.get_logger().info(f'param_name changed to: {param.value}')
                self.param_value = param.value
            elif param.name == 'int_param' and param.type_ == Parameter.Type.INTEGER:
                self.get_logger().info(f'int_param changed to: {param.value}')
                self.int_value = param.value

        # Return successful result
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = ParameterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Using Timers

Timers allow periodic execution:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TimerNode(Node):
    def __init__(self):
        super().__init__('timer_node')

        # Create publisher for timer output
        self.publisher = self.create_publisher(String, 'timer_topic', 10)

        # Create timer that calls callback every 0.5 seconds
        self.timer = self.create_timer(0.5, self.timer_callback)

        # Counter to track calls
        self.i = 0

    def timer_callback(self):
        # This method is called every 0.5 seconds
        msg = String()
        msg.data = f'Timer has called {self.i} times'

        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = TimerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Quality of Service (QoS) Settings

QoS settings control message delivery behavior:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class QoSNode(Node):
    def __init__(self):
        super().__init__('qos_node')

        # Different QoS profiles for different needs

        # Reliable delivery (guaranteed delivery, may be slower)
        reliable_qos = QoSProfile(
            depth=10,  # Queue size
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        # Best effort (faster but may lose messages)
        best_effort_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        # Create publishers with different QoS
        self.reliable_publisher = self.create_publisher(String, 'reliable_topic', reliable_qos)
        self.best_effort_publisher = self.create_publisher(String, 'best_effort_topic', best_effort_qos)

def main(args=None):
    rclpy.init(args=args)
    node = QoSNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced Patterns

### Multiple Publishers and Subscribers
```python
class MultiTopicNode(Node):
    def __init__(self):
        super().__init__('multi_topic_node')

        # Multiple publishers
        self.pub1 = self.create_publisher(String, 'topic1', 10)
        self.pub2 = self.create_publisher(Int64, 'topic2', 10)

        # Multiple subscribers
        self.sub1 = self.create_subscription(String, 'input1', self.callback1, 10)
        self.sub2 = self.create_subscription(Float64, 'input2', self.callback2, 10)
        self.sub3 = self.create_subscription(Bool, 'input3', self.callback3, 10)

    def callback1(self, msg):
        self.get_logger().info(f'Received from input1: {msg.data}')

    def callback2(self, msg):
        self.get_logger().info(f'Received from input2: {msg.data}')

    def callback3(self, msg):
        self.get_logger().info(f'Received from input3: {msg.data}')
```

### Node with Services and Actions
```python
from rclpy.action import ActionServer
from example_interfaces.action import Fibonacci

class ComplexNode(Node):
    def __init__(self):
        super().__init__('complex_node')

        # Publishers and subscribers
        self.pub = self.create_publisher(String, 'output', 10)
        self.sub = self.create_subscription(String, 'input', self.sub_callback, 10)

        # Services
        self.srv = self.create_service(AddTwoInts, 'add_service', self.service_callback)

        # Actions
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_fibonacci
        )

    def sub_callback(self, msg):
        # Process subscription
        pass

    def service_callback(self, request, response):
        # Process service request
        response.sum = request.a + request.b
        return response

    def execute_fibonacci(self, goal_handle):
        # Process action goal
        feedback_msg = Fibonacci.Feedback()
        result = Fibonacci.Result()

        # Execute action with feedback
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.sequence = feedback_msg.sequence
                return result

            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])

            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result.sequence = feedback_msg.sequence
        return result
```

## Error Handling and Best Practices

### Proper Error Handling
```python
class SafeNode(Node):
    def __init__(self):
        super().__init__('safe_node')
        self.subscription = self.create_subscription(String, 'input', self.safe_callback, 10)

    def safe_callback(self, msg):
        try:
            # Process message with error handling
            result = self.process_message(msg)
            # Publish result
            self.publish_result(result)
        except ValueError as e:
            self.get_logger().error(f'Invalid message format: {e}')
        except Exception as e:
            self.get_logger().error(f'Unexpected error: {e}')
```

### Resource Management
```python
def main(args=None):
    rclpy.init(args=args)
    node = MyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {e}')
    finally:
        # Always clean up resources
        node.destroy_node()
        rclpy.shutdown()
```

## Common Message Types and Usage

### Basic Types
```python
from std_msgs.msg import String, Int32, Float64, Bool, ColorRGBA

# String message
msg = String()
msg.data = "Hello, ROS 2!"

# Integer message
msg = Int32()
msg.data = 42

# Float message
msg = Float64()
msg.data = 3.14159

# Boolean message
msg = Bool()
msg.data = True

# Color message
msg = ColorRGBA()
msg.r = 1.0  # Red
msg.g = 0.0  # Green
msg.b = 0.0  # Blue
msg.a = 1.0  # Alpha (opacity)
```

### Geometry Types
```python
from geometry_msgs.msg import Twist, Pose, Point

# Velocity command (linear and angular)
twist = Twist()
twist.linear.x = 1.0    # Forward velocity
twist.angular.z = 0.5   # Angular velocity

# 3D point
point = Point()
point.x = 1.0
point.y = 2.0
point.z = 3.0

# Pose (position and orientation)
pose = Pose()
pose.position.x = 1.0
pose.position.y = 2.0
pose.position.z = 0.0
pose.orientation.w = 1.0  # No rotation (quaternion)
```

### Sensor Types
```python
from sensor_msgs.msg import LaserScan, Image, JointState

# Laser scan (LIDAR data)
scan = LaserScan()
scan.ranges = [1.0, 1.1, 1.2, 1.3]  # Distance measurements

# Joint states
joint_state = JointState()
joint_state.name = ['joint1', 'joint2']
joint_state.position = [0.1, 0.2]
joint_state.velocity = [0.0, 0.0]
joint_state.effort = [0.0, 0.0]
```

## Testing and Debugging

### Logging
```python
class LoggingNode(Node):
    def __init__(self):
        super().__init__('logging_node')

        # Different log levels
        self.get_logger().debug('Debug message')
        self.get_logger().info('Info message')
        self.get_logger().warn('Warning message')
        self.get_logger().error('Error message')
        self.get_logger().fatal('Fatal message')
```

### Debugging Tips
1. Use `rclpy.logging` for consistent logging
2. Check topics with `ros2 topic list` and `ros2 topic echo`
3. Use `rclpy.qos` profiles appropriately
4. Always handle exceptions in callbacks
5. Use meaningful node and topic names

## Command Line Tools for rclpy

### Useful Commands
```bash
# List all nodes
ros2 node list

# List topics
ros2 topic list

# Echo messages from a topic
ros2 topic echo /topic_name std_msgs/msg/String

# List services
ros2 service list

# Call a service
ros2 service call /service_name example_interfaces/srv/AddTwoInts "{a: 1, b: 2}"

# List parameters
ros2 param list node_name

# Get parameter value
ros2 param get node_name param_name

# Run a Python node
ros2 run package_name script_name.py
```

## Best Practices Summary

1. **Always use try-finally** for proper cleanup
2. **Handle exceptions** in all callbacks
3. **Use appropriate QoS** for your use case
4. **Log appropriately** for debugging
5. **Validate inputs** before processing
6. **Use standard message types** when possible
7. **Follow naming conventions** (lowercase with underscores)
8. **Comment your code** explaining complex logic
9. **Test thoroughly** before deployment
10. **Use meaningful names** for nodes, topics, and parameters

## Common Pitfalls to Avoid

1. **Forgetting to initialize rclpy** before creating nodes
2. **Not handling KeyboardInterrupt** properly
3. **Creating nodes before initializing** the library
4. **Not cleaning up resources** in finally blocks
5. **Using blocking calls** in callbacks
6. **Mismatched message types** between publishers and subscribers
7. **Incorrect QoS profiles** causing connection issues
8. **Not waiting for services** before calling them
9. **Using magic numbers** instead of named constants
10. **Not validating** parameter values

This guide provides the essential knowledge needed to work effectively with rclpy. Use these patterns as a foundation for building robust ROS 2 nodes in Python.