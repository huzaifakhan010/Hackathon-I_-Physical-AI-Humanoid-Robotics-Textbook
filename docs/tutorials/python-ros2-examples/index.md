---
sidebar_position: 2
---

# Python-ROS 2 Examples

This tutorial provides practical examples of connecting Python applications to ROS 2 systems. These examples demonstrate common patterns and best practices for Python-ROS 2 integration.

## Prerequisites

Before working through these examples, you should:
- Understand basic ROS 2 concepts (nodes, topics, services)
- Have Python knowledge at an intermediate level
- Have ROS 2 installed with Python support

## Example 1: Simple Publisher-Subscriber Pair

Let's start with a simple example that demonstrates the basic publisher-subscriber pattern.

### Publisher Node

**File: `python_examples/simple_publisher.py`**

```python
#!/usr/bin/env python3

"""
Simple publisher example that sends random temperature values.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import random


class TemperaturePublisher(Node):
    def __init__(self):
        super().__init__('temperature_publisher')

        # Create publisher for temperature data
        self.publisher = self.create_publisher(Float64, 'temperature', 10)

        # Create timer to publish data every 2 seconds
        self.timer = self.create_timer(2.0, self.publish_temperature)

        self.get_logger().info('Temperature publisher started')

    def publish_temperature(self):
        # Generate random temperature value (simulating sensor data)
        temp_msg = Float64()
        temp_msg.data = round(random.uniform(18.0, 30.0), 2)  # Room temperature range

        self.publisher.publish(temp_msg)
        self.get_logger().info(f'Published temperature: {temp_msg.data}°C')


def main(args=None):
    rclpy.init(args=args)

    temp_publisher = TemperaturePublisher()

    try:
        rclpy.spin(temp_publisher)
    except KeyboardInterrupt:
        temp_publisher.get_logger().info('Shutting down temperature publisher...')
    finally:
        temp_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Subscriber Node

**File: `python_examples/simple_subscriber.py`**

```python
#!/usr/bin/env python3

"""
Simple subscriber example that receives and processes temperature data.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class TemperatureSubscriber(Node):
    def __init__(self):
        super().__init__('temperature_subscriber')

        # Create subscription to temperature topic
        self.subscription = self.create_subscription(
            Float64,
            'temperature',
            self.temperature_callback,
            10
        )

        self.get_logger().info('Temperature subscriber started')

    def temperature_callback(self, msg):
        temperature = msg.data

        # Process the temperature data
        if temperature < 20.0:
            status = "Cold"
        elif temperature > 25.0:
            status = "Warm"
        else:
            status = "Comfortable"

        self.get_logger().info(f'Received temperature: {temperature}°C ({status})')

        # You could add more processing here
        # For example, triggering actions based on temperature


def main(args=None):
    rclpy.init(args=args)

    temp_subscriber = TemperatureSubscriber()

    try:
        rclpy.spin(temp_subscriber)
    except KeyboardInterrupt:
        temp_subscriber.get_logger().info('Shutting down temperature subscriber...')
    finally:
        temp_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Example 2: Parameter-Based Node

This example shows how to use parameters to configure node behavior.

**File: `python_examples/parameter_example.py`**

```python
#!/usr/bin/env python3

"""
Example demonstrating ROS 2 parameters for node configuration.
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String


class ParameterExampleNode(Node):
    def __init__(self):
        super().__init__('parameter_example_node')

        # Declare parameters with default values
        self.declare_parameter('robot_name', 'my_robot')
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('sensor_range', 5.0)
        self.declare_parameter('debug_mode', False)

        # Get parameter values
        self.robot_name = self.get_parameter('robot_name').value
        self.max_speed = self.get_parameter('max_speed').value
        self.sensor_range = self.get_parameter('sensor_range').value
        self.debug_mode = self.get_parameter('debug_mode').value

        # Create publisher
        self.publisher = self.create_publisher(String, 'robot_status', 10)

        # Create timer
        self.timer = self.create_timer(1.0, self.status_callback)

        # Set parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.get_logger().info(f'Node initialized with robot: {self.robot_name}, '
                              f'max_speed: {self.max_speed}, debug: {self.debug_mode}')

    def parameter_callback(self, params):
        """Callback for parameter changes"""
        for param in params:
            if param.name == 'robot_name' and param.type_ == Parameter.Type.STRING:
                self.robot_name = param.value
                self.get_logger().info(f'Robot name changed to: {param.value}')
            elif param.name == 'max_speed' and param.type_ == Parameter.Type.DOUBLE:
                self.max_speed = param.value
                self.get_logger().info(f'Max speed changed to: {param.value}')
            elif param.name == 'debug_mode' and param.type_ == Parameter.Type.BOOL:
                self.debug_mode = param.value
                status = 'enabled' if param.value else 'disabled'
                self.get_logger().info(f'Debug mode {status}')

        return SetParametersResult(successful=True)

    def status_callback(self):
        msg = String()
        msg.data = f'{self.robot_name} is operating at {self.max_speed} m/s'
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    param_node = ParameterExampleNode()

    try:
        rclpy.spin(param_node)
    except KeyboardInterrupt:
        param_node.get_logger().info('Shutting down parameter example node...')
    finally:
        param_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Example 3: Service Client and Server

This example demonstrates request-response communication.

### Service Server

**File: `python_examples/calculator_server.py`**

```python
#!/usr/bin/env python3

"""
Calculator service server example.
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class CalculatorServer(Node):
    def __init__(self):
        super().__init__('calculator_server')

        # Create service
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_callback
        )

        self.get_logger().info('Calculator service server started')

    def add_callback(self, request, response):
        """Callback to handle addition requests"""
        result = request.a + request.b
        response.sum = result

        self.get_logger().info(f'{request.a} + {request.b} = {result}')

        return response


def main(args=None):
    rclpy.init(args=args)

    calc_server = CalculatorServer()

    try:
        rclpy.spin(calc_server)
    except KeyboardInterrupt:
        calc_server.get_logger().info('Shutting down calculator server...')
    finally:
        calc_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Service Client

**File: `python_examples/calculator_client.py`**

```python
#!/usr/bin/env python3

"""
Calculator service client example.
"""

import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class CalculatorClient(Node):
    def __init__(self):
        super().__init__('calculator_client')

        # Create client
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.request = AddTwoInts.Request()

    def send_request(self, a, b):
        """Send request to the service"""
        self.request.a = a
        self.request.b = b

        # Call service asynchronously
        self.future = self.client.call_async(self.request)
        self.future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        """Handle service response"""
        try:
            response = future.result()
            self.get_logger().info(f'Result: {response.sum}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


def main(args=None):
    rclpy.init(args=args)

    calc_client = CalculatorClient()

    # Get numbers from command line arguments or use defaults
    a = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    b = int(sys.argv[2]) if len(sys.argv) > 2 else 2

    calc_client.send_request(a, b)

    try:
        # Keep spinning to handle the async response
        while rclpy.ok():
            rclpy.spin_once(calc_client)
            if calc_client.future.done():
                break
    except KeyboardInterrupt:
        calc_client.get_logger().info('Shutting down calculator client...')
    finally:
        calc_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Example 4: Multiple Sensor Fusion

This example demonstrates how to work with multiple sensors and combine their data.

**File: `python_examples/sensor_fusion.py`**

```python
#!/usr/bin/env python3

"""
Example of sensor fusion: combining data from multiple sensors.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import numpy as np


class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')

        # Storage for sensor data
        self.laser_data = None
        self.imu_data = None

        # Create subscribers for different sensors
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Create publisher for fused data
        self.fused_pub = self.create_publisher(Float64, 'safety_score', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer for fusion processing
        self.timer = self.create_timer(0.1, self.fusion_callback)  # 10 Hz

        self.get_logger().info('Sensor fusion node started')

    def laser_callback(self, msg):
        """Handle laser scan data"""
        self.laser_data = np.array(msg.ranges)
        # Replace invalid ranges (inf, nan) with max range
        self.laser_data = np.nan_to_num(self.laser_data, nan=msg.range_max, posinf=msg.range_max, neginf=0.0)

    def imu_callback(self, msg):
        """Handle IMU data"""
        self.imu_data = {
            'orientation': [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w],
            'angular_velocity': [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
            'linear_acceleration': [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        }

    def fusion_callback(self):
        """Main fusion logic"""
        if self.laser_data is None or self.imu_data is None:
            return  # Wait for both sensors to provide data

        # Calculate safety score based on laser data
        # (closer obstacles = lower safety score)
        min_distance = np.min(self.laser_data)
        safety_score = Float64()

        if min_distance < 0.5:  # Less than 50cm to obstacle
            safety_score.data = 0.1  # Very unsafe
        elif min_distance < 1.0:  # Between 50cm and 1m
            safety_score.data = 0.5  # Moderately unsafe
        elif min_distance < 2.0:  # Between 1m and 2m
            safety_score.data = 0.8  # Moderately safe
        else:  # More than 2m from obstacles
            safety_score.data = 1.0  # Very safe

        # Publish safety score
        self.fused_pub.publish(safety_score)

        # Simple navigation based on safety
        cmd = Twist()
        if safety_score.data > 0.7:  # Safe to move forward
            cmd.linear.x = 0.5  # Move forward at 0.5 m/s
        elif safety_score.data > 0.3:  # Some risk
            cmd.linear.x = 0.1  # Move forward slowly
            cmd.angular.z = 0.2  # Gentle turn
        else:  # Unsafe
            cmd.linear.x = 0.0  # Stop
            cmd.angular.z = 0.5  # Turn away from obstacle

        # Publish velocity command
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)

    fusion_node = SensorFusionNode()

    try:
        rclpy.spin(fusion_node)
    except KeyboardInterrupt:
        fusion_node.get_logger().info('Shutting down sensor fusion node...')
    finally:
        fusion_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Example 5: Quality of Service (QoS) Settings

This example demonstrates different QoS profiles for various use cases.

**File: `python_examples/qos_examples.py`**

```python
#!/usr/bin/env python3

"""
Example demonstrating Quality of Service (QoS) settings.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class QoSExampleNode(Node):
    def __init__(self):
        super().__init__('qos_example_node')

        # Different QoS profiles for different needs:

        # Reliable communication (guaranteed delivery, higher latency)
        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        # Best effort communication (faster, may lose messages)
        best_effort_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        # Keep last value (for configuration parameters)
        transient_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # Create publishers with different QoS
        self.reliable_pub = self.create_publisher(String, 'reliable_topic', reliable_qos)
        self.best_effort_pub = self.create_publisher(String, 'best_effort_topic', best_effort_qos)
        self.transient_pub = self.create_publisher(String, 'config_topic', transient_qos)

        # Create subscriptions with matching QoS
        self.reliable_sub = self.create_subscription(
            String, 'reliable_topic', self.reliable_callback, reliable_qos)
        self.best_effort_sub = self.create_subscription(
            String, 'best_effort_topic', self.best_effort_callback, best_effort_qos)

        # Timer for publishing
        self.timer = self.create_timer(1.0, self.publish_messages)
        self.counter = 0

        self.get_logger().info('QoS example node started')

    def publish_messages(self):
        """Publish messages with different QoS profiles"""
        msg = String()
        msg.data = f'Message {self.counter}'

        # Publish with different QoS
        self.reliable_pub.publish(msg)
        self.best_effort_pub.publish(msg)
        self.transient_pub.publish(msg)

        self.counter += 1

    def reliable_callback(self, msg):
        self.get_logger().info(f'Reliable: {msg.data}')

    def best_effort_callback(self, msg):
        self.get_logger().info(f'Best effort: {msg.data}')


def main(args=None):
    rclpy.init(args=args)

    qos_node = QoSExampleNode()

    try:
        rclpy.spin(qos_node)
    except KeyboardInterrupt:
        qos_node.get_logger().info('Shutting down QoS example node...')
    finally:
        qos_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Running the Examples

### 1. Make the scripts executable:
```bash
chmod +x python_examples/*.py
```

### 2. Source your ROS 2 environment:
```bash
source /opt/ros/<your_ros2_distro>/setup.bash
# or if you built from source:
source ~/ros2_ws/install/setup.bash
```

### 3. Run the examples:

**For publisher-subscriber:**
```bash
# Terminal 1
python3 python_examples/simple_publisher.py

# Terminal 2
python3 python_examples/simple_subscriber.py
```

**For services:**
```bash
# Terminal 1
python3 python_examples/calculator_server.py

# Terminal 2
python3 python_examples/calculator_client.py 5 3
```

**For parameters:**
```bash
# Terminal 1
python3 python_examples/parameter_example.py

# Terminal 2 - change parameters at runtime
ros2 param set /parameter_example_node robot_name "new_robot_name"
ros2 param set /parameter_example_node max_speed 2.0
```

## Best Practices from Examples

### 1. Error Handling
Always handle potential exceptions:
```python
def safe_callback(self, msg):
    try:
        # Process message
        result = self.process_data(msg)
        self.publish_result(result)
    except ValueError as e:
        self.get_logger().error(f'Invalid data format: {e}')
    except Exception as e:
        self.get_logger().error(f'Unexpected error: {e}')
```

### 2. Resource Management
Properly clean up resources:
```python
def main(args=None):
    rclpy.init(args=args)
    node = MyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Received interrupt signal')
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

### 3. Logging
Use appropriate log levels:
```python
self.get_logger().debug('Detailed debug information')
self.get_logger().info('General information')
self.get_logger().warn('Warning message')
self.get_logger().error('Error message')
```

### 4. Parameter Validation
Validate parameters before use:
```python
def validate_parameters(self):
    max_speed = self.get_parameter('max_speed').value
    if max_speed <= 0 or max_speed > 10.0:  # reasonable limits
        self.get_logger().warn(f'Invalid max_speed: {max_speed}, using default')
        max_speed = 1.0
        self.set_parameters([Parameter('max_speed', Parameter.Type.DOUBLE, max_speed)])
```

## Common Patterns

### Pattern 1: Node with Multiple Publishers/Subscribers
```python
class MultiTopicNode(Node):
    def __init__(self):
        super().__init__('multi_topic_node')

        # Multiple publishers
        self.pub1 = self.create_publisher(Type1, 'topic1', 10)
        self.pub2 = self.create_publisher(Type2, 'topic2', 10)

        # Multiple subscribers
        self.sub1 = self.create_subscription(Type1, 'input1', self.cb1, 10)
        self.sub2 = self.create_subscription(Type2, 'input2', self.cb2, 10)
```

### Pattern 2: Node with Timer and State
```python
class StatefulNode(Node):
    def __init__(self):
        super().__init__('stateful_node')

        self.state = 'IDLE'  # Current state
        self.timer = self.create_timer(0.1, self.state_machine)

    def state_machine(self):
        if self.state == 'IDLE':
            self.idle_behavior()
        elif self.state == 'ACTIVE':
            self.active_behavior()
```

## Summary

These examples demonstrate essential Python-ROS 2 integration patterns:

1. **Basic communication**: Publisher/subscriber and service patterns
2. **Configuration**: Using parameters for runtime configuration
3. **Data fusion**: Combining multiple sensor inputs
4. **Quality of service**: Different communication guarantees
5. **Best practices**: Error handling, resource management, and logging

Use these examples as templates for your own ROS 2 Python applications. Each pattern can be adapted and extended for specific use cases in robotics development.