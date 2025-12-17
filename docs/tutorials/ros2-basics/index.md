---
sidebar_position: 1
---

# Basic ROS 2 Tutorial

This tutorial will guide you through the fundamentals of ROS 2 by creating simple publisher and subscriber nodes. By the end of this tutorial, you'll understand the basic concepts of ROS 2 and be able to create your own simple ROS 2 nodes.

## Prerequisites

Before starting this tutorial, you should have:
- ROS 2 installed on your system
- Basic Python programming knowledge
- Familiarity with command-line tools

## What You'll Learn

In this tutorial, you'll learn how to:
1. Create a basic ROS 2 package
2. Write a publisher node that sends messages
3. Write a subscriber node that receives messages
4. Run and test your nodes
5. Understand the basic ROS 2 concepts

## Creating Your First ROS 2 Package

First, let's create a workspace and package for our tutorial:

```bash
# Create a workspace directory
mkdir -p ~/ros2_workspace/src
cd ~/ros2_workspace

# Create a package for our tutorial
colcon build
source install/setup.bash

# Now create the package (replace with your ROS 2 distro, e.g., humble, iron, jazzy)
ros2 pkg create --build-type ament_python tutorial_pkg
```

## Creating a Publisher Node

Let's create a simple publisher that sends "Hello World" messages:

**File: `tutorial_pkg/tutorial_pkg/publisher_member_function.py`**

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Creating a Subscriber Node

Now let's create a subscriber that receives and logs the messages:

**File: `tutorial_pkg/tutorial_pkg/subscriber_member_function.py`**

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    try:
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Making Files Executable and Updating setup.py

Make the Python files executable:

```bash
chmod +x tutorial_pkg/tutorial_pkg/publisher_member_function.py
chmod +x tutorial_pkg/tutorial_pkg/subscriber_member_function.py
```

Update the `setup.py` file in your package to include the entry points:

**File: `tutorial_pkg/setup.py`**

```python
from setuptools import setup
import os
from glob import glob

package_name = 'tutorial_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Basic ROS 2 tutorial package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = tutorial_pkg.publisher_member_function:main',
            'listener = tutorial_pkg.subscriber_member_function:main',
        ],
    },
)
```

## Building and Running the Nodes

Now let's build and run our nodes:

```bash
# From your workspace directory
cd ~/ros2_workspace
colcon build --packages-select tutorial_pkg
source install/setup.bash

# Run the publisher node in one terminal
ros2 run tutorial_pkg talker

# In another terminal, run the subscriber node
ros2 run tutorial_pkg listener
```

You should see the publisher sending messages and the subscriber receiving them.

## Understanding the Code

Let's break down the key components of our publisher:

### Node Initialization
```python
class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')  # Initialize the node with a name
```

### Creating a Publisher
```python
self.publisher_ = self.create_publisher(String, 'topic', 10)
```
- `String`: The message type
- `'topic'`: The topic name
- `10`: The queue size for outgoing messages

### Creating a Timer
```python
timer_period = 0.5  # seconds
self.timer = self.create_timer(timer_period, self.timer_callback)
```
This creates a timer that calls `timer_callback` every 0.5 seconds.

### Publishing Messages
```python
def timer_callback(self):
    msg = String()
    msg.data = f'Hello World: {self.i}'
    self.publisher_.publish(msg)
```
This creates a message and publishes it to the topic.

For the subscriber:
### Creating a Subscription
```python
self.subscription = self.create_subscription(
    String,           # Message type
    'topic',          # Topic name
    self.listener_callback,  # Callback function
    10)               # Queue size
```

### Processing Messages
```python
def listener_callback(self, msg):
    self.get_logger().info(f'I heard: "{msg.data}"')
```
This function is called whenever a message arrives on the topic.

## Exploring ROS 2 Tools

ROS 2 provides many command-line tools to help you understand and debug your system:

```bash
# List all active topics
ros2 topic list

# Echo messages from a specific topic
ros2 topic echo /topic

# Get information about a topic
ros2 topic info /topic

# List all active nodes
ros2 node list

# Get information about a node
ros2 node info /minimal_publisher

# List all services
ros2 service list
```

## Creating a Service Server and Client

Let's create a simple service that adds two numbers:

**Service Server (`tutorial_pkg/tutorial_pkg/service_server.py`):**
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
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

**Service Client (`tutorial_pkg/tutorial_pkg/service_client.py`):**
```python
import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class MinimalClient(Node):

    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClient()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    minimal_client.get_logger().info(f'Result of add_two_ints: {response.sum}')

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Update your `setup.py` to add these new executables:

```python
entry_points={
    'console_scripts': [
        'talker = tutorial_pkg.publisher_member_function:main',
        'listener = tutorial_pkg.subscriber_member_function:main',
        'add_server = tutorial_pkg.service_server:main',
        'add_client = tutorial_pkg.service_client:main',
    ],
},
```

Run the service:
```bash
# Terminal 1: Start the server
ros2 run tutorial_pkg add_server

# Terminal 2: Call the service
ros2 run tutorial_pkg add_client 2 3
```

## Understanding the Architecture

ROS 2 uses a distributed architecture where:

1. **Nodes** are the basic computational elements
2. **Topics** enable publish/subscribe communication
3. **Services** enable request/response communication
4. **Actions** handle long-running tasks with feedback

The communication is peer-to-peer, meaning nodes can communicate directly without a central server (though there is a ROS 2 daemon that helps with discovery).

## Best Practices

1. **Always clean up resources** in a finally block
2. **Use meaningful node and topic names**
3. **Handle exceptions** in your callbacks
4. **Use appropriate queue sizes** for publishers and subscribers
5. **Log important information** for debugging
6. **Use standard message types** when possible

## Troubleshooting Common Issues

### Nodes can't communicate
- Check that the message types match between publisher and subscriber
- Verify that topic names are identical
- Make sure both nodes are on the same ROS domain

### Nodes not appearing
- Ensure `rclpy.init()` is called before creating nodes
- Check that nodes are properly initialized with `super().__init__()`

### Messages not appearing
- Verify that the publisher is actually sending messages
- Check that the subscriber is subscribed to the correct topic
- Ensure QoS profiles are compatible

## Next Steps

Now that you understand the basics of ROS 2, you can:
1. Explore more complex message types
2. Learn about parameters and configuration
3. Understand Quality of Service (QoS) settings
4. Create more sophisticated robot applications
5. Integrate with simulation environments like Gazebo

This tutorial provided a foundation for understanding ROS 2 concepts. The publish/subscribe and request/response patterns you've learned form the basis of most ROS 2 applications.