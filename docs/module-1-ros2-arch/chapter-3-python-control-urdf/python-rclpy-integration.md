---
sidebar_position: 1
---

# Python-ROS 2 Integration with rclpy

In this section, we'll explore how to connect Python applications to ROS 2 using the rclpy library. rclpy is the Python client library for ROS 2 that allows you to create nodes, publish and subscribe to topics, and use services and actions - all from Python code.

## What is rclpy?

rclpy (ROS Client Library for Python) is the official Python interface to ROS 2. Think of it as the bridge that connects your Python code to the ROS 2 ecosystem. It provides all the functionality you need to create ROS 2 nodes, handle messages, and communicate with other nodes in the system.

### Key Components of rclpy

#### Node
- The basic building block of a ROS 2 system
- Created using `rclpy.node.Node`
- Contains publishers, subscribers, services, and other ROS 2 elements
- Manages the lifecycle of your Python ROS 2 application

#### Publishers and Subscribers
- For topic-based communication
- Publishers send messages to topics
- Subscribers receive messages from topics
- Uses the publish/subscribe pattern we learned about

#### Services and Clients
- For request/response communication
- Service servers respond to requests
- Service clients make requests
- Uses the request/response pattern we learned about

## Setting Up rclpy

Before you can use rclpy, you need to initialize the ROS 2 Python client library:

```python
import rclpy
from rclpy.node import Node

# Initialize the ROS 2 client library
rclpy.init()

# Create your node
my_node = MyNodeClass()

# Spin the node (keep it running)
rclpy.spin(my_node)

# Clean up when done
rclpy.shutdown()
```

## Creating Your First rclpy Node

Let's create a simple node that publishes a "Hello World" message:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HelloWorldPublisher(Node):
    def __init__(self):
        # Initialize the node with the name 'hello_world_publisher'
        super().__init__('hello_world_publisher')

        # Create a publisher that sends String messages to the 'hello_topic' topic
        self.publisher = self.create_publisher(String, 'hello_topic', 10)

        # Create a timer that calls the timer_callback method every 0.5 seconds
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Counter to keep track of messages sent
        self.i = 0

    def timer_callback(self):
        # Create a message
        msg = String()
        msg.data = f'Hello World: {self.i}'

        # Publish the message
        self.publisher.publish(msg)

        # Log the message to the console
        self.get_logger().info(f'Publishing: "{msg.data}"')

        # Increment the counter
        self.i += 1

def main(args=None):
    # Initialize the ROS 2 client library
    rclpy.init(args=args)

    # Create the node
    hello_world_publisher = HelloWorldPublisher()

    try:
        # Keep the node running
        rclpy.spin(hello_world_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        hello_world_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Creating a Subscriber Node

Now let's create a node that subscribes to the messages published by our first node:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HelloWorldSubscriber(Node):
    def __init__(self):
        # Initialize the node with the name 'hello_world_subscriber'
        super().__init__('hello_world_subscriber')

        # Create a subscription to the 'hello_topic' topic
        self.subscription = self.create_subscription(
            String,           # Message type
            'hello_topic',    # Topic name
            self.listener_callback,  # Callback function
            10                # Queue size
        )

        # Make sure the subscription is properly created
        self.subscription  # Prevent unused variable warning

    def listener_callback(self, msg):
        # This function is called when a message arrives
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    # Initialize the ROS 2 client library
    rclpy.init(args=args)

    # Create the subscriber node
    hello_world_subscriber = HelloWorldSubscriber()

    try:
        # Keep the node running
        rclpy.spin(hello_world_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        hello_world_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Working with Services

Let's create a simple service server and client using rclpy:

### Service Server
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        # Create a service that responds to AddTwoInts requests
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        # Process the request
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {request.a} + {request.b} = {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()

    try:
        rclpy.spin(minimal_service)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Client
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')
        # Create a client for the AddTwoInts service
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
        return self.future

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClient()

    # Send a request
    future = minimal_client.send_request(1, 2)

    try:
        # Keep spinning until the response comes back
        while rclpy.ok():
            rclpy.spin_once(minimal_client)
            if future.done():
                try:
                    response = future.result()
                    minimal_client.get_logger().info(
                        f'Result of add_two_ints: {response.sum}')
                except Exception as e:
                    minimal_client.get_logger().error(f'Service call failed: {e}')
                break
    except KeyboardInterrupt:
        pass
    finally:
        minimal_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## rclpy Best Practices

### 1. Proper Resource Management
Always clean up resources properly:

```python
def main(args=None):
    rclpy.init(args=args)
    node = MyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up resources
        node.destroy_node()
        rclpy.shutdown()
```

### 2. Use Appropriate QoS Settings
Consider your application's requirements:

```python
# For real-time control
qos_profile = rclpy.qos.QoSProfile(
    depth=1,
    reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
    durability=rclpy.qos.DurabilityPolicy.VOLATILE
)

publisher = self.create_publisher(String, 'topic', qos_profile)
```

### 3. Error Handling
Handle potential errors gracefully:

```python
def listener_callback(self, msg):
    try:
        # Process the message
        processed_data = self.process_message(msg)
        # Publish results
        self.result_publisher.publish(processed_data)
    except Exception as e:
        self.get_logger().error(f'Error processing message: {e}')
```

## Common rclpy Patterns

### Parameter Handling
```python
class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values
        self.declare_parameter('my_param', 'default_value')

        # Get parameter value
        self.my_param = self.get_parameter('my_param').value
```

### Timer-Based Operations
```python
class TimerNode(Node):
    def __init__(self):
        super().__init__('timer_node')

        # Create a timer that calls a method at regular intervals
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        # This runs every 0.1 seconds
        pass
```

### Multiple Publishers/Subscribers
```python
class MultiTopicNode(Node):
    def __init__(self):
        super().__init__('multi_topic_node')

        # Multiple publishers
        self.pub1 = self.create_publisher(String, 'topic1', 10)
        self.pub2 = self.create_publisher(Int64, 'topic2', 10)

        # Multiple subscribers
        self.sub1 = self.create_subscription(String, 'topic3', self.callback1, 10)
        self.sub2 = self.create_subscription(Float64, 'topic4', self.callback2, 10)
```

## Connecting AI Agents to ROS 2

The power of rclpy becomes clear when connecting AI agents to robot systems:

```python
import rclpy
from rclpy.node import Node
import numpy as np  # Example AI library

class AIAgentNode(Node):
    def __init__(self):
        super().__init__('ai_agent_node')

        # Subscribe to sensor data
        self.sensor_sub = self.create_subscription(
            LaserScan, '/laser_scan', self.scan_callback, 10)

        # Publish control commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # AI model (simplified example)
        self.ai_model = self.initialize_ai_model()

    def scan_callback(self, msg):
        # Process sensor data
        sensor_data = self.process_scan(msg)

        # Use AI to make decisions
        action = self.ai_model.decide_action(sensor_data)

        # Convert AI decision to robot command
        cmd_msg = self.create_command_message(action)

        # Send command to robot
        self.cmd_pub.publish(cmd_msg)

    def initialize_ai_model(self):
        # Initialize your AI model here
        return SimpleNavigationModel()

    def process_scan(self, scan_msg):
        # Convert ROS message to format suitable for AI
        return np.array(scan_msg.ranges)

    def create_command_message(self, action):
        # Convert AI action to ROS message
        cmd = Twist()
        cmd.linear.x = action.linear_speed
        cmd.angular.z = action.angular_speed
        return cmd
```

## Key Takeaways

- rclpy is the Python client library for ROS 2
- It allows you to create nodes, publishers, subscribers, services, and clients
- Proper initialization and cleanup are important for resource management
- rclpy enables the connection between Python AI agents and ROS 2 robot systems
- Common patterns include timer-based operations and proper error handling
- AI agents can use rclpy to receive sensor data and send control commands

## Exercise

Create a simple rclpy node that:
1. Subscribes to a topic called `/sensor_data` that publishes Float64 messages
2. Processes the sensor data by multiplying it by 2
3. Publishes the result to a topic called `/processed_data`
4. Logs the original and processed values

Test your node by creating a simple publisher that sends test data to `/sensor_data`.