---
sidebar_position: 2
---

# Bridging AI Agents to Robot Controllers

In this section, we'll explore how to connect AI agents to robot controllers using ROS 2 and Python. This is where the intelligence of your AI system meets the physical capabilities of your robot, enabling autonomous behavior and decision-making.

## The AI-Robot Bridge Concept

Think of the AI-robot bridge as the connection between your AI agent's "mind" and the robot's "body." The AI agent processes information and makes decisions, while the robot executes those decisions in the physical world.

### The Bridge Components

#### Perception Pipeline
- **Sensors** → **ROS 2 Topics** → **AI Agent** → **Processing**
- Raw sensor data (cameras, LIDAR, IMU) flows to the AI agent
- AI agent processes this data to understand the environment

#### Decision Pipeline
- **AI Agent** → **Control Commands** → **ROS 2 Topics/Services** → **Robot Actuators**
- AI agent decides what actions to take based on perception
- Commands are sent to robot actuators (motors, grippers, etc.)

## Connecting AI Agents to ROS 2

### Basic AI Agent Structure

```python
import rclpy
from rclpy.node import Node
import numpy as np  # Common AI library

class AIAgentNode(Node):
    def __init__(self):
        super().__init__('ai_agent_node')

        # Subscribe to sensor data from the robot
        self.sensor_subscription = self.create_subscription(
            LaserScan,      # Laser scanner data
            '/scan',        # Topic name
            self.scan_callback,
            10
        )

        # Subscribe to camera data
        self.camera_subscription = self.create_subscription(
            Image,          # Camera image data
            '/camera/image_raw',
            self.camera_callback,
            10
        )

        # Publish control commands to the robot
        self.cmd_publisher = self.create_publisher(
            Twist,          # Velocity commands
            '/cmd_vel',
            10
        )

        # Initialize your AI model
        self.ai_model = self.initialize_model()

        # Timer for decision-making loop
        self.timer = self.create_timer(0.1, self.decision_loop)

    def initialize_model(self):
        # Initialize your AI model here
        # This could be a neural network, rule-based system, etc.
        return SimpleNavigationModel()

    def scan_callback(self, msg):
        # Process laser scan data
        self.laser_data = msg.ranges

    def camera_callback(self, msg):
        # Process camera data
        self.camera_data = msg.data

    def decision_loop(self):
        # This runs continuously to make decisions
        if hasattr(self, 'laser_data') and hasattr(self, 'camera_data'):
            # Process sensor data with AI
            action = self.ai_model.decide_action(self.laser_data, self.camera_data)

            # Convert AI decision to ROS message
            cmd_msg = self.create_command_from_action(action)

            # Send command to robot
            self.cmd_publisher.publish(cmd_msg)

    def create_command_from_action(self, action):
        # Convert AI action to ROS message
        cmd = Twist()
        cmd.linear.x = action.linear_velocity
        cmd.angular.z = action.angular_velocity
        return cmd

def main(args=None):
    rclpy.init(args=args)
    ai_agent = AIAgentNode()

    try:
        rclpy.spin(ai_agent)
    except KeyboardInterrupt:
        pass
    finally:
        ai_agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Common AI-Robot Integration Patterns

### 1. Sensor Fusion Pattern

Combine data from multiple sensors for better decision-making:

```python
class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')

        # Multiple sensor subscriptions
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Store sensor data
        self.sensor_data = {
            'imu': None,
            'odom': None,
            'scan': None
        }

        # Publish fused data
        self.fused_pub = self.create_publisher(SensorData, '/fused_sensors', 10)

    def imu_callback(self, msg):
        self.sensor_data['imu'] = msg
        self.process_fusion()

    def odom_callback(self, msg):
        self.sensor_data['odom'] = msg
        self.process_fusion()

    def scan_callback(self, msg):
        self.sensor_data['scan'] = msg
        self.process_fusion()

    def process_fusion(self):
        # Combine all sensor data into a unified representation
        if all(data is not None for data in self.sensor_data.values()):
            fused_data = self.fuse_sensors(self.sensor_data)
            self.fused_pub.publish(fused_data)
```

### 2. Behavior Tree Pattern

Use behavior trees to structure complex AI behaviors:

```python
class BehaviorTreeNode(Node):
    def __init__(self):
        super().__init__('behavior_tree_node')

        # Subscriptions for sensor data
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer for behavior tree execution
        self.timer = self.create_timer(0.05, self.behavior_tree)

        # Behavior state
        self.state = 'SEARCHING'  # SEARCHING, APPROACHING, AVOIDING, etc.

    def behavior_tree(self):
        # Behavior tree logic
        if self.state == 'SEARCHING':
            if self.detect_object():
                self.state = 'APPROACHING'
            else:
                self.search_behavior()
        elif self.state == 'APPROACHING':
            if self.is_safe_to_approach():
                self.approach_behavior()
            else:
                self.state = 'AVOIDING'
        elif self.state == 'AVOIDING':
            self.avoid_behavior()
            if self.is_clear():
                self.state = 'SEARCHING'

    def detect_object(self):
        # Process sensor data to detect objects
        return True  # Simplified

    def search_behavior(self):
        # Send commands for searching behavior
        cmd = Twist()
        cmd.linear.x = 0.2
        cmd.angular.z = 0.5  # Turn slowly
        self.cmd_pub.publish(cmd)
```

### 3. State Machine Pattern

Use finite state machines for clear behavior transitions:

```python
class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_node')

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.state_machine)

        # Define states
        self.STATES = {
            'IDLE': self.idle_state,
            'MOVING': self.moving_state,
            'OBSTACLE_AVOIDANCE': self.obstacle_avoidance_state,
            'GOAL_APPROACH': self.goal_approach_state
        }

        self.current_state = 'IDLE'
        self.obstacle_detected = False

    def state_machine(self):
        # Execute current state's behavior
        self.STATES[self.current_state]()

    def idle_state(self):
        # Robot is idle, waiting for command
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

        # Transition condition
        if self.should_start_moving():
            self.current_state = 'MOVING'

    def moving_state(self):
        # Robot is moving toward goal
        cmd = Twist()
        cmd.linear.x = 0.5  # Move forward
        self.cmd_pub.publish(cmd)

        # Check for obstacles
        if self.obstacle_detected:
            self.current_state = 'OBSTACLE_AVOIDANCE'

    def obstacle_avoidance_state(self):
        # Avoid obstacles
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.5  # Turn to avoid
        self.cmd_pub.publish(cmd)

        if not self.obstacle_detected:
            self.current_state = 'MOVING'
```

## Handling Sensor Data for AI

### Converting ROS Messages to AI-Ready Format

```python
class AIPreprocessingNode(Node):
    def __init__(self):
        super().__init__('ai_preprocessing_node')

        # Subscribe to raw sensor data
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.camera_sub = self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)

        # Publish preprocessed data for AI
        self.ai_pub = self.create_publisher(AIPreprocessedData, '/ai_input', 10)

    def scan_callback(self, msg):
        # Convert laser scan to AI-friendly format
        processed_scan = self.process_laser_scan(msg)

        # Create AI-ready message
        ai_msg = AIPreprocessedData()
        ai_msg.scan_data = processed_scan.flatten().tolist()  # Convert to list
        ai_msg.header.stamp = self.get_clock().now().to_msg()

        self.ai_pub.publish(ai_msg)

    def process_laser_scan(self, scan_msg):
        # Process raw scan data for AI consumption
        ranges = np.array(scan_msg.ranges)

        # Handle invalid ranges (NaN, infinity)
        ranges = np.nan_to_num(ranges, nan=0.0, posinf=10.0, neginf=0.0)

        # Normalize if needed
        ranges = np.clip(ranges, 0.0, 10.0)  # Clip to 0-10m range

        return ranges
```

## Sending Control Commands from AI

### Converting AI Decisions to Robot Commands

```python
class AIDecisionNode(Node):
    def __init__(self):
        super().__init__('ai_decision_node')

        # Subscribe to AI decisions
        self.decision_sub = self.create_subscription(AIDecision, '/ai_decision', self.decision_callback, 10)

        # Publish to various robot controllers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_pub = self.create_publisher(JointCommand, '/arm_controller/command', 10)
        self.gripper_pub = self.create_publisher(GripperCommand, '/gripper_controller/command', 10)

    def decision_callback(self, msg):
        # Convert high-level AI decision to specific robot commands
        if msg.action_type == 'NAVIGATE':
            self.send_navigation_command(msg)
        elif msg.action_type == 'GRASP':
            self.send_grasp_command(msg)
        elif msg.action_type == 'SPEAK':
            self.send_speak_command(msg)

    def send_navigation_command(self, decision):
        # Convert navigation decision to velocity command
        cmd = Twist()
        cmd.linear.x = decision.linear_velocity
        cmd.angular.z = decision.angular_velocity
        self.cmd_vel_pub.publish(cmd)

    def send_grasp_command(self, decision):
        # Convert grasp decision to arm and gripper commands
        arm_cmd = JointCommand()
        arm_cmd.positions = decision.arm_positions
        self.arm_pub.publish(arm_cmd)

        gripper_cmd = GripperCommand()
        gripper_cmd.position = decision.gripper_position
        self.gripper_pub.publish(gripper_cmd)
```

## Error Handling and Safety

### Safe Command Validation

```python
class SafeAIAgentNode(Node):
    def __init__(self):
        super().__init__('safe_ai_agent_node')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Safety parameters
        self.max_linear_vel = 1.0  # m/s
        self.max_angular_vel = 1.5  # rad/s

    def send_command_safely(self, ai_command):
        # Validate and limit command values
        cmd = Twist()

        # Limit linear velocity
        cmd.linear.x = max(-self.max_linear_vel,
                          min(self.max_linear_vel, ai_command.linear.x))

        # Limit angular velocity
        cmd.angular.z = max(-self.max_angular_vel,
                           min(self.max_angular_vel, ai_command.angular.z))

        # Additional safety checks
        if self.is_safe_to_move():
            self.cmd_pub.publish(cmd)
        else:
            # Send stop command if unsafe
            stop_cmd = Twist()
            self.cmd_pub.publish(stop_cmd)

    def is_safe_to_move(self):
        # Check various safety conditions
        # This could check for emergency stops, hardware errors, etc.
        return True  # Simplified
```

## Real-World Example: AI-Controlled Mobile Robot

Here's a complete example of an AI agent controlling a mobile robot:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np

class AINavigationAgent(Node):
    def __init__(self):
        super().__init__('ai_navigation_agent')

        # Subscriptions
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # AI state
        self.laser_data = None
        self.safety_threshold = 0.5  # meters

        # Timer for AI decision making
        self.timer = self.create_timer(0.1, self.ai_decision_loop)

    def scan_callback(self, msg):
        self.laser_data = np.array(msg.ranges)
        # Handle invalid ranges
        self.laser_data = np.nan_to_num(self.laser_data, nan=np.inf)

    def ai_decision_loop(self):
        if self.laser_data is None:
            return

        # Simple AI decision: avoid obstacles and move forward
        if self.is_path_clear():
            cmd = self.move_forward()
        else:
            cmd = self.avoid_obstacle()

        self.cmd_pub.publish(cmd)

    def is_path_clear(self):
        # Check if path ahead is clear of obstacles
        front_scan = self.laser_data[len(self.laser_data)//2-30:len(self.laser_data)//2+30]
        min_distance = np.min(front_scan)
        return min_distance > self.safety_threshold

    def move_forward(self):
        cmd = Twist()
        cmd.linear.x = 0.3
        cmd.angular.z = 0.0
        return cmd

    def avoid_obstacle(self):
        cmd = Twist()
        # Turn away from closest obstacle
        min_idx = np.argmin(self.laser_data)
        if min_idx < len(self.laser_data) / 2:
            # Obstacle on the right, turn left
            cmd.angular.z = 0.5
        else:
            # Obstacle on the left, turn right
            cmd.angular.z = -0.5
        return cmd

def main(args=None):
    rclpy.init(args=args)
    ai_agent = AINavigationAgent()

    try:
        rclpy.spin(ai_agent)
    except KeyboardInterrupt:
        pass
    finally:
        ai_agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Key Considerations for AI-Robot Integration

### 1. Latency
- Minimize communication delays between AI and robot
- Consider real-time requirements for safety-critical applications
- Use appropriate QoS settings for time-sensitive data

### 2. Safety
- Implement safety checks before executing AI commands
- Use emergency stop mechanisms
- Validate command ranges and feasibility

### 3. Robustness
- Handle sensor failures gracefully
- Implement fallback behaviors
- Monitor system health continuously

### 4. Scalability
- Design for multiple AI agents if needed
- Consider distributed computing for complex AI models
- Plan for future expansion of capabilities

## Best Practices

1. **Separate Concerns**: Keep perception, decision-making, and actuation as separate components
2. **Use Standard Interfaces**: Stick to common ROS message types when possible
3. **Implement Safety First**: Always prioritize safety over performance
4. **Log Everything**: Keep detailed logs for debugging AI behavior
5. **Test Thoroughly**: Test both in simulation and on real robots

## Key Takeaways

- The AI-robot bridge connects AI decision-making with physical robot control
- Common patterns include sensor fusion, behavior trees, and state machines
- Proper preprocessing of sensor data is crucial for AI performance
- Safety and validation are critical when connecting AI to physical systems
- Standard ROS interfaces enable interoperability between AI and robot systems
- Error handling and fallback behaviors ensure robust operation

## Exercise

Design an AI agent that can navigate to a goal position while avoiding obstacles. Your agent should:

1. Subscribe to laser scan data for obstacle detection
2. Subscribe to odometry data for robot position
3. Use a simple AI algorithm to plan a path to a goal
4. Publish velocity commands to control the robot
5. Include safety checks to prevent collisions

Implement the basic structure and explain how each component would work together.