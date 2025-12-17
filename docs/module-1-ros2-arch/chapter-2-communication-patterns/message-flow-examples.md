---
sidebar_position: 4
---

# Message Flow Examples

In this section, we'll explore real-world examples of how messages flow between nodes in ROS 2 systems. These examples will help you visualize how topics, services, and other communication patterns work together in practical robot applications.

## Example 1: Robot Navigation System

Let's examine how a simple robot navigation system works, showing the flow of messages between different nodes.

### Scenario: Robot Moving to a Goal Location

**Nodes Involved:**
- **Navigation Node**: Plans the path and sends movement commands
- **LIDAR Node**: Publishes obstacle detection data
- **Motor Controller Node**: Receives velocity commands and controls wheels
- **Map Server Node**: Provides static map data via service
- **Transform Node**: Shares coordinate frame relationships

### Message Flow Diagram (Text Description)

```
User Request → Service Call → Navigation Node
     ↓
Navigation Node → Topic (/scan) → LIDAR Node
     ↓
LIDAR Node → Topic (/scan) → Navigation Node (obstacle data)
     ↓
Navigation Node → Topic (/cmd_vel) → Motor Controller Node
     ↓
Motor Controller → Feedback → Navigation Node
```

### Step-by-Step Flow

1. **User sends navigation goal** via service request to Navigation Node
2. **Navigation Node requests map** from Map Server via service call
3. **LIDAR Node continuously publishes** sensor data to `/scan` topic
4. **Navigation Node subscribes** to `/scan` topic to detect obstacles
5. **Navigation Node calculates path** based on map and sensor data
6. **Navigation Node publishes velocity commands** to `/cmd_vel` topic
7. **Motor Controller subscribes** to `/cmd_vel` topic and moves robot
8. **Motor Controller provides feedback** through transforms and odometry

### Code Example: Navigation Node
```python
# This is a conceptual example, not executable code
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.srv import GetMap

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')

        # Publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber for LIDAR data
        self.scan_subscriber = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # Service client for map requests
        self.map_client = self.create_client(GetMap, '/static_map')

        # Timer for navigation loop
        self.timer = self.create_timer(0.1, self.navigation_loop)

    def scan_callback(self, msg):
        # Process LIDAR data to detect obstacles
        self.lidar_data = msg

    def navigation_loop(self):
        # This runs continuously to navigate the robot
        if self.has_goal() and self.lidar_data:
            # Check for obstacles and calculate safe velocity
            safe_velocity = self.calculate_safe_velocity()

            # Publish velocity command
            cmd_msg = Twist()
            cmd_msg.linear.x = safe_velocity.linear.x
            cmd_msg.angular.z = safe_velocity.angular.z
            self.cmd_vel_publisher.publish(cmd_msg)
```

## Example 2: Robot Arm Control System

Let's look at how a robot arm system manages communication between perception, planning, and control nodes.

### Scenario: Robot Arm Picking Up an Object

**Nodes Involved:**
- **Vision Node**: Detects objects using camera
- **Motion Planning Node**: Plans arm movements
- **Arm Controller Node**: Controls joint positions
- **Gripper Node**: Controls gripper state
- **TF Node**: Manages coordinate transformations

### Message Flow

```
Camera Feed → Topic (/camera/image_raw) → Vision Node
     ↓
Vision Node → Topic (/detected_object) → Motion Planning Node
     ↓
Motion Planning → Topic (/arm_joint_trajectory) → Arm Controller Node
     ↓
Motion Planning → Service (/control_gripper) → Gripper Node
     ↓
Arm Controller → Feedback → Motion Planning Node
```

### Key Communication Patterns

- **Topics**: For continuous sensor data and state updates
- **Services**: For discrete actions like gripper control
- **Actions**: For long-running tasks like trajectory execution

## Example 3: Humanoid Robot Walking System

Let's examine a more complex system for humanoid robot locomotion.

### Scenario: Humanoid Robot Walking Forward

**Nodes Involved:**
- **Walking Controller Node**: Coordinates walking pattern
- **IMU Node**: Provides balance and orientation data
- **Joint State Publisher**: Publishes current joint positions
- **Foot Pressure Sensors**: Detect ground contact
- **Pattern Generator**: Creates walking gait patterns

### Message Flow

```
IMU Data → Topic (/imu/data) → Walking Controller
     ↓
Foot Sensors → Topic (/foot_pressure) → Walking Controller
     ↓
Walking Controller → Topic (/joint_commands) → Joint Controllers
     ↓
Joint Controllers → Topic (/joint_states) → Walking Controller
     ↓
Walking Controller → Topic (/robot_pose) → Other Nodes
```

### Synchronization Requirements

- **Real-time constraints**: Joint commands must be sent at precise intervals
- **Feedback integration**: Balance adjustments based on sensor data
- **Safety monitoring**: Emergency stops based on sensor readings

## Example 4: Multi-Robot Coordination System

For advanced applications, multiple robots may need to coordinate.

### Scenario: Two Robots Collaborating

**Communication Elements:**
- **Robot-to-Robot Topics**: Share position and status
- **Coordinator Node**: Manages team behavior
- **Shared Services**: Access common resources

### Message Flow

```
Robot 1 → Topic (/robot1/status) → Coordinator
     ↓
Robot 2 → Topic (/robot2/status) → Coordinator
     ↓
Coordinator → Topic (/robot1/command) → Robot 1
     ↓
Coordinator → Topic (/robot2/command) → Robot 2
```

## Common Message Flow Patterns

### 1. Sensor-Processing-Actuation Loop
```
Sensor Node → Topic → Processing Node → Topic → Actuator Node
```
- Most common pattern in robotics
- Enables modular design and testing
- Allows multiple processing nodes to use same sensor data

### 2. Hierarchical Control
```
High-level → Service → Mid-level → Topic → Low-level
```
- High-level makes decisions
- Mid-level translates to specific commands
- Low-level handles direct hardware control

### 3. Feedback Integration
```
Process → Topic → Monitor → Topic → Controller → Topic → Process
```
- Closed-loop control for stability
- Essential for precise robot control
- Enables adaptive behavior

## Best Practices for Message Flow Design

### 1. Minimize Direct Dependencies
- Use topics instead of direct connections when possible
- Allow for multiple subscribers to the same data
- Design for system expansion and modification

### 2. Appropriate Communication Pattern Selection
- Use topics for continuous data (sensors, status)
- Use services for discrete requests (configuration, actions)
- Use actions for long-running tasks with feedback

### 3. Message Rate Considerations
- Match publishing rate to subscriber processing capability
- Consider network bandwidth for distributed systems
- Use message filters to reduce unnecessary processing

### 4. Error Handling
- Design fallback behaviors when nodes fail
- Implement timeouts for service calls
- Monitor connection status between nodes

## Visualization Tips

When designing your own message flows:

1. **Start with the use case**: What does the robot need to accomplish?
2. **Identify required data**: What information does each component need?
3. **Choose appropriate patterns**: Topics for continuous data, services for requests
4. **Consider timing requirements**: Real-time vs. best-effort communication
5. **Plan for failure**: How will the system behave if nodes fail?

## Exercise

Design a message flow for a security robot that patrols a building. Your system should include:

1. A camera system for detecting obstacles
2. A navigation system for path planning
3. A motor control system for movement
4. A communication system to report status to a base station

Draw the message flow using text descriptions showing:
- Which nodes exist in your system
- What topics and services they use
- The direction of information flow
- How the robot would handle an obstacle detection

Consider both normal operation and error scenarios in your design.