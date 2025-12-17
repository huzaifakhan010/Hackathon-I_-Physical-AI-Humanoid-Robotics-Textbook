---
sidebar_position: 5
---

# Common Beginner Mistakes and Mental Models

Learning ROS 2 communication patterns can be challenging, and beginners often make certain mistakes. This section identifies the most common errors and provides mental models to help you avoid them.

## Common Topic-Related Mistakes

### 1. Topic Name Mismatches
**The Mistake**: Publisher and subscriber use slightly different topic names.

**Example of the Mistake**:
```python
# Publisher node
self.publisher = self.create_publisher(String, 'camera_image', 10)  # Wrong: no slash

# Subscriber node
self.subscription = self.create_subscription(String, '/camera_image', callback, 10)  # Wrong: has slash
```

**Correct Approach**:
```python
# Both publisher and subscriber must use identical topic names
# Publisher
self.publisher = self.create_publisher(String, '/sensor/camera/image_raw', 10)

# Subscriber
self.subscription = self.create_subscription(String, '/sensor/camera/image_raw', callback, 10)
```

**Mental Model**: Think of topic names like email addresses - they must match exactly for communication to work.

### 2. Message Type Mismatches
**The Mistake**: Publisher and subscriber use different message types.

**Example of the Mistake**:
```python
# Publisher sends image data
from sensor_msgs.msg import Image
# But subscriber expects string data
from std_msgs.msg import String  # Wrong type!
```

**Correct Approach**: Always use matching message types:
```python
# Both use the same message type
from sensor_msgs.msg import Image  # Both publisher and subscriber
```

### 3. Publishing Too Fast or Too Slow
**The Mistake**: Publishing at a rate that overwhelms subscribers or is too slow for real-time applications.

**Mental Model**: Think of publishing rate like a conversation pace - too fast and listeners can't follow, too slow and the conversation becomes awkward.

### 4. Ignoring QoS Settings
**The Mistake**: Using default QoS settings without considering application requirements.

**Common Issues**:
- Using reliable delivery for sensor data that can tolerate some loss
- Using best-effort for critical control commands
- Not setting appropriate history depth for message queues

**Correct Approach**: Choose QoS based on your needs:
- For sensor data: Best effort, small history
- For control commands: Reliable, appropriate history
- For configuration: Transient local for late joiners

## Common Service-Related Mistakes

### 1. Not Checking Service Availability
**The Mistake**: Trying to call a service before it's available.

**Example of the Mistake**:
```python
# This might fail if service isn't ready yet
future = self.client.call_async(request)  # Could fail immediately
```

**Correct Approach**:
```python
# Always check if service is available first
while not self.client.wait_for_service(timeout_sec=1.0):
    self.get_logger().info('Service not available, waiting again...')

future = self.client.call_async(request)  # Safe to call now
```

### 2. Blocking the Main Thread with Synchronous Calls
**The Mistake**: Using synchronous service calls that block other operations.

**Mental Model**: Think of service calls like phone calls - if you make a call and wait for the person to answer, you can't do other things during that time.

**Better Approach**: Use asynchronous calls:
```python
def call_service_async(self):
    future = self.client.call_async(request)
    future.add_done_callback(self.response_callback)  # Handle response when it arrives
    # Main thread continues with other work
```

### 3. Not Handling Service Timeouts
**The Mistake**: Not setting or handling timeouts for service calls.

**Correct Approach**:
```python
# Set timeouts and handle them appropriately
try:
    response = self.client.call(request, timeout_sec=5.0)
except Exception as e:
    self.get_logger().error(f'Service call failed: {e}')
```

## Common Mental Model Mistakes

### 1. Thinking Topics Are Like Variables
**Wrong Mental Model**: "I set the topic value and other nodes can read it like a shared variable."

**Correct Mental Model**: Topics are like radio broadcasts - you send data out and any number of nodes can receive it, but there's no shared memory or direct connection.

### 2. Confusing Synchronous and Asynchronous Communication
**Wrong Mental Model**: "When I publish a message, the subscriber processes it immediately."

**Correct Mental Model**: Publishing is asynchronous - the message goes into a queue and subscribers process it when they can. There's no guarantee of immediate processing.

### 3. Assuming All Messages Are Guaranteed to Arrive
**Wrong Mental Model**: "Every message I publish will definitely reach every subscriber."

**Correct Mental Model**: With default settings, messages are best-effort. For guaranteed delivery, you need to configure QoS settings appropriately.

### 4. Believing Nodes Must Run on the Same Computer
**Wrong Mental Model**: "All nodes must be on the same computer because they need to communicate."

**Correct Mental Model**: ROS 2 is designed for distributed systems - nodes can run on different computers, robots, or cloud services and still communicate seamlessly.

## Architecture Design Mistakes

### 1. Creating Too Many Direct Connections
**The Mistake**: Every node communicates directly with every other node it needs data from.

**Better Approach**: Use topics for data sharing - one publisher, many subscribers.

### 2. Making Nodes Too Complex
**The Mistake**: Creating nodes that try to do too many things.

**Better Approach**: Follow single responsibility principle - each node should do one thing well.

### 3. Not Planning for Failure
**The Mistake**: Not considering what happens when a node fails.

**Better Approach**: Design your system to handle node failures gracefully - other nodes should continue operating when possible.

## Debugging Tips for Communication Issues

### 1. Use ROS 2 Command Line Tools
```bash
# Check what topics exist
ros2 topic list

# Check what services exist
ros2 service list

# Echo messages on a topic to see if data is flowing
ros2 topic echo /topic_name

# Check topic type
ros2 topic type /topic_name
```

### 2. Add Logging to Your Nodes
```python
def callback(self, msg):
    self.get_logger().info(f'Received message: {msg}')
    # Process message
```

### 3. Check Node Connections
```python
# In your node, check connection status
def on_connect(self, subscription):
    self.get_logger().info('New subscriber connected')

def on_disconnect(self, subscription):
    self.get_logger().warn('Subscriber disconnected')
```

## Best Practices to Avoid Mistakes

### 1. Use Consistent Naming Conventions
- Use forward slashes to create hierarchy: `/sensor/camera/image_raw`
- Use descriptive names that indicate content and purpose
- Follow community conventions when possible

### 2. Plan Your Message Types Carefully
- Use standard message types when possible
- Create custom message types only when necessary
- Document your custom message types well

### 3. Test Communication Patterns Separately
- Test publishers before adding subscribers
- Test individual nodes before connecting them
- Use simple test nodes to verify communication works

### 4. Consider Timing Requirements
- Match publishing rates to processing capabilities
- Use appropriate QoS settings for your application
- Plan for real-time vs. best-effort requirements

## Common Analogies That Help Understanding

### Topics are Like:
- **Radio stations**: One broadcaster, many listeners
- **Newspapers**: One publisher, many readers
- **Social media feeds**: One poster, many followers

### Services are Like:
- **Phone calls**: Direct, two-party communication
- **Customer service**: Request specific help, get specific response
- **ATM machines**: Request specific information, get specific response

### Nodes are Like:
- **Specialized workers**: Each does one job well
- **Organ departments**: Each handles specific functions
- **Body organs**: Each performs specific vital functions

## Exercise: Identify the Mistakes

Look at these code snippets and identify the communication mistakes:

### Code Snippet 1:
```python
# Publisher node
self.pub = self.create_publisher(String, 'temperature', 10)
msg = String()
msg.data = str(25.0)
self.pub.publish(msg)
```

```python
# Subscriber node
self.sub = self.create_subscription(Float32, 'temperature', callback, 10)  # Mistake?
```

### Code Snippet 2:
```python
# Service client
future = self.client.call_async(request)
response = future.result()  # Mistake?
```

### Code Snippet 3:
```python
# Publisher sending 1000 messages per second
timer = self.create_timer(0.001, publish_data)  # Mistake?
```

For each snippet, identify the mistake and explain how to fix it.

## Key Takeaways

- Topic names must match exactly between publishers and subscribers
- Message types must be consistent across publishers and subscribers
- Always check service availability before calling
- Use appropriate QoS settings for your application
- Design nodes with single responsibilities
- Test communication patterns separately
- Use ROS 2 tools to debug communication issues
- Think of topics as broadcasts, not shared variables
- Plan for failure and graceful degradation

Understanding these common mistakes and mental models will help you avoid the most frequent pitfalls when working with ROS 2 communication patterns.