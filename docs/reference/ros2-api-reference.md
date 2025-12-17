---
sidebar_position: 1
---

# ROS 2 API Reference Guide

This reference guide provides an overview of key ROS 2 APIs and concepts for beginners. It serves as a quick reference for common ROS 2 programming patterns and functionality.

## Core Concepts

### Nodes
Nodes are the fundamental building blocks of a ROS 2 system.

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        # Initialize the node with a name
        super().__init__('node_name')

# Main execution pattern
def main(args=None):
    rclpy.init(args=args)  # Initialize ROS 2
    node = MyNode()        # Create node instance
    try:
        rclpy.spin(node)   # Keep node running
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()  # Clean up
        rclpy.shutdown()     # Shutdown ROS 2
```

### Publishers
Publishers send messages to topics.

```python
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        # Create publisher: (message_type, topic_name, queue_size)
        self.publisher = self.create_publisher(String, 'topic_name', 10)

    def publish_message(self, data):
        msg = String()
        msg.data = data
        self.publisher.publish(msg)
```

### Subscribers
Subscribers receive messages from topics.

```python
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        # Create subscription: (message_type, topic_name, callback, queue_size)
        self.subscription = self.create_subscription(
            String,           # Message type
            'topic_name',     # Topic name
            self.callback,    # Callback function
            10                # Queue size
        )

    def callback(self, msg):
        # Process received message
        self.get_logger().info(f'Received: {msg.data}')
```

### Services
Services provide request-response communication.

```python
from example_interfaces.srv import AddTwoInts

class ServiceServerNode(Node):
    def __init__(self):
        super().__init__('service_server')
        # Create service: (service_type, service_name, callback)
        self.srv = self.create_service(
            AddTwoInts,           # Service type
            'add_two_ints',       # Service name
            self.add_callback     # Callback function
        )

    def add_callback(self, request, response):
        # Process request and fill response
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {response.sum}')
        return response

class ServiceClientNode(Node):
    def __init__(self):
        super().__init__('service_client')
        # Create client: (service_type, service_name)
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        # Call service asynchronously
        future = self.cli.call_async(request)
        return future
```

### Actions
Actions handle long-running tasks with feedback.

```python
from example_interfaces.action import Fibonacci
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        # Create action server
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

    def goal_callback(self, goal_request):
        # Accept or reject goal
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        # Accept or reject cancel request
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        # Execute the action with feedback
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Fibonacci.Result()

            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])

            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        return result
```

## Common Message Types

### Standard Messages
- `std_msgs/Bool` - Boolean values
- `std_msgs/Int32`, `std_msgs/Float64` - Numeric values
- `std_msgs/String` - Text strings
- `std_msgs/ColorRGBA` - Color values

### Geometry Messages
- `geometry_msgs/Twist` - Velocity commands (linear and angular)
- `geometry_msgs/Pose` - Position and orientation
- `geometry_msgs/Point` - 3D point coordinates

### Sensor Messages
- `sensor_msgs/LaserScan` - LIDAR data
- `sensor_msgs/Image` - Camera images
- `sensor_msgs/Imu` - Inertial measurement unit data
- `sensor_msgs/JointState` - Robot joint positions

## Quality of Service (QoS) Settings

QoS settings control how messages are handled:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Reliable delivery
reliable_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE
)

# Best effort (good for sensor data)
best_effort_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE
)

# Create publisher with custom QoS
publisher = self.create_publisher(String, 'topic', reliable_qos)
```

## Parameters

Nodes can declare and use parameters:

```python
class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameter with default value
        self.declare_parameter('my_param', 'default_value')

        # Get parameter value
        self.my_param = self.get_parameter('my_param').value

        # Set parameter callback for dynamic changes
        self.add_on_set_parameters_callback(self.parameters_callback)

    def parameters_callback(self, params):
        for param in params:
            if param.name == 'my_param' and param.type_ == Parameter.Type.STRING:
                self.my_param = param.value
        return SetParametersResult(successful=True)
```

## Timers

Timers allow periodic execution:

```python
class TimerNode(Node):
    def __init__(self):
        super().__init__('timer_node')

        # Create timer that calls callback every 0.5 seconds
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        self.get_logger().info(f'Timer callback {self.i}')
        self.i += 1
```

## TF (Transforms)

For coordinate frame transformations:

```python
from tf2_ros import TransformBroadcaster
import tf2_ros
from geometry_msgs.msg import TransformStamped

class TransformNode(Node):
    def __init__(self):
        super().__init__('transform_node')
        # Create transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

    def broadcast_transform(self):
        t = TransformStamped()

        # Set transform header
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'parent_frame'
        t.child_frame_id = 'child_frame'

        # Set transform translation and rotation
        t.transform.translation.x = 1.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        # Broadcast transform
        self.tf_broadcaster.sendTransform(t)
```

## Common Patterns

### Publisher with Timer
```python
class PublisherWithTimer(Node):
    def __init__(self):
        super().__init__('publisher_timer')
        self.publisher = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello {self.i}'
        self.publisher.publish(msg)
        self.i += 1
```

### Client with Future Handling
```python
def call_service_with_future(self):
    future = self.cli.call_async(self.request)
    # Add callback for when response arrives
    future.add_done_callback(self.service_response_callback)

def service_response_callback(self, future):
    try:
        response = future.result()
        self.get_logger().info(f'Response: {response.sum}')
    except Exception as e:
        self.get_logger().error(f'Service call failed: {e}')
```

## Error Handling

Always include proper error handling:

```python
def safe_subscription_callback(self, msg):
    try:
        # Process message
        result = self.process_message(msg)
        # Publish result
        self.result_publisher.publish(result)
    except ValueError as e:
        self.get_logger().error(f'Invalid message format: {e}')
    except Exception as e:
        self.get_logger().error(f'Unexpected error: {e}')
```

## Best Practices

1. **Always clean up resources** in a finally block
2. **Use appropriate QoS settings** for your application
3. **Handle exceptions** in callbacks
4. **Use meaningful names** for topics, services, and parameters
5. **Follow naming conventions** (lowercase with underscores)
6. **Include logging** for debugging
7. **Validate inputs** before processing
8. **Use standard message types** when possible

## Common Command Line Tools

- `ros2 topic list` - Show all topics
- `ros2 topic echo <topic_name>` - Print messages from a topic
- `ros2 service list` - Show all services
- `ros2 node list` - Show all nodes
- `ros2 param list <node_name>` - Show node parameters
- `ros2 run <package> <executable>` - Run a node
- `ros2 launch <package> <launch_file>` - Launch a system