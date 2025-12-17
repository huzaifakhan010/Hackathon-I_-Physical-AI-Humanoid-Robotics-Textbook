---
sidebar_position: 2
---

# Topics and Data Flow

In this section, we'll explore how topics work in ROS 2, which enable continuous data communication between nodes. Topics work like radio broadcasts - one node sends information out, and multiple nodes can receive it simultaneously.

## What are Topics?

Topics are communication channels in ROS 2 that enable a **publish/subscribe** pattern. Think of them like radio stations - a publisher node broadcasts information on a topic, and any number of subscriber nodes can tune in to receive that information.

### Key Characteristics of Topics

#### One-to-Many Communication
- One publisher can send data to multiple subscribers
- Multiple subscribers can receive the same data stream
- No direct connection between publisher and subscribers
- Like a radio station broadcasting to many listeners

#### Continuous Data Flow
- Publishers send data continuously at regular intervals
- Subscribers receive data as long as they're "tuned in"
- Perfect for sensor data, robot status, or any continuous information
- Unlike services, which are request-response based

#### Typed Messages
- All data on a topic follows the same message format
- Publishers and subscribers must agree on message type
- This ensures data consistency across the system
- Like how radio stations broadcast in specific formats (AM/FM)

## Radio Station Analogy

Think of ROS 2 topics like a radio broadcasting system:

| Radio System | ROS 2 Topic System | Function |
|--------------|-------------------|----------|
| Radio Station | Publisher Node | Sends out information |
| Radio Frequency | Topic Name | Channel of communication |
| Radio Listeners | Subscriber Nodes | Receive the broadcast |
| Audio Signal | Message Data | The actual information |

Just as multiple people can listen to the same radio station simultaneously, multiple nodes can subscribe to the same topic and receive the same data stream.

## Common Use Cases for Topics

### Sensor Data Broadcasting
- Camera nodes publishing image streams
- LIDAR nodes publishing distance measurements
- IMU nodes publishing orientation data
- GPS nodes publishing location information

### Robot Status Updates
- Joint position feedback from motor controllers
- Battery level monitoring
- Robot pose estimation
- System health status

### Control Commands
- Velocity commands to robot base
- Joint angle commands to arms
- LED status updates

## Topic Communication Flow

Here's how topic-based communication works:

### 1. Publisher Setup
```python
# This is a conceptual example, not executable code
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Message type

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        # Create a publisher on the 'topic_name' topic
        self.publisher = self.create_publisher(String, 'topic_name', 10)
        # 10 is the queue size for messages

    def publish_data(self, message_data):
        msg = String()
        msg.data = message_data
        self.publisher.publish(msg)  # Send the message
```

### 2. Subscriber Setup
```python
# This is a conceptual example, not executable code
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Must match publisher

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        # Create a subscription to 'topic_name'
        self.subscription = self.create_subscription(
            String,           # Message type
            'topic_name',     # Topic name
            self.listener_callback,  # Function to call when message received
            10               # Queue size
        )

    def listener_callback(self, msg):
        # This function is called whenever a message arrives
        self.get_logger().info(f'Received: {msg.data}')
```

### 3. The Communication Process
1. **Publisher registers** with ROS 2 system to publish on a specific topic
2. **Subscriber registers** to listen to that same topic
3. **ROS 2 discovers** the connection and enables communication
4. **Publisher sends** data continuously (or when available)
5. **Subscriber receives** data automatically when published
6. **Multiple subscribers** can receive the same data stream

## Topic Names and Organization

### Naming Convention
- Use forward slashes to create hierarchy: `/sensor/camera/image_raw`
- Start with meaningful names: `/robot/status`, `/navigation/goal`
- Be descriptive but concise
- Follow ROS 2 naming standards

### Common Topic Examples
- `/camera/image_raw` - Raw camera image data
- `/scan` - LIDAR scan data
- `/cmd_vel` - Velocity commands for robot base
- `/joint_states` - Current positions of all robot joints
- `/tf` - Transform data between coordinate frames

## Quality of Service (QoS) Settings

Topics have QoS settings that control how messages are handled:

### Reliability
- **Reliable**: Every message is guaranteed to arrive (like TCP)
- **Best effort**: Messages might be lost (like UDP, good for sensor data)

### Durability
- **Transient**: Late-joining subscribers get the last message
- **Volatile**: Only new messages are sent to subscribers

### History
- **Keep last N**: Store only the most recent N messages
- **Keep all**: Store all messages (use carefully!)

## Advantages of Topic-Based Communication

### Decoupling
- Publishers don't need to know about subscribers
- Subscribers don't need to know about publishers
- Nodes can be added or removed without affecting others

### Scalability
- Multiple subscribers can receive the same data
- No performance degradation with more subscribers
- Easy to add new consumers of data

### Robustness
- Failure of one subscriber doesn't affect others
- Network interruptions are handled gracefully
- Built-in message queuing and buffering

## Common Beginner Mistakes

### Topic Name Mismatches
- **Problem**: Publisher and subscriber use different topic names
- **Solution**: Verify topic names match exactly, including case and slashes

### Message Type Mismatches
- **Problem**: Publisher and subscriber use different message types
- **Solution**: Ensure both use the same message type definition

### QoS Profile Mismatches
- **Problem**: Publisher and subscriber have incompatible QoS settings
- **Solution**: Use compatible QoS profiles for the same topic

### Data Rate Issues
- **Problem**: Publishing too fast for subscribers to handle
- **Solution**: Match publishing rate to subscriber processing capability

## Key Takeaways

- Topics enable one-to-many communication in ROS 2
- Use the publish/subscribe pattern for continuous data streams
- Topics work like radio broadcasts - one sender, many receivers
- Always match message types and topic names between publishers and subscribers
- Choose appropriate QoS settings based on your application needs
- Topics are perfect for sensor data, status updates, and control commands

## Exercise

Think of a robot vacuum cleaner that needs to share sensor data with multiple processing nodes. Design a topic-based communication system that includes:

1. What topics would the robot need for its different sensors?
2. What nodes would subscribe to each topic?
3. What message types would be appropriate for each topic?

Sketch out the publisher-subscriber relationships and explain how the data would flow through the system.