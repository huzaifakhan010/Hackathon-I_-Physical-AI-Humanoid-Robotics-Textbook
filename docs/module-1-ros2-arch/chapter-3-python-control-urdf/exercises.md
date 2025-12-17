---
sidebar_position: 6
---

# Chapter 3 Exercises

Complete these exercises to reinforce your understanding of Python control and robot description with URDF.

## Conceptual Exercises

### Exercise 1: rclpy Basics
1. Explain the purpose of `rclpy.init()` and `rclpy.shutdown()` in a ROS 2 Python node.
2. What is the difference between a publisher and a subscriber in rclpy?
3. How do you create a timer in an rclpy node and what is it typically used for?
4. What's the purpose of the `declare_parameter()` method?

### Exercise 2: URDF Structure Understanding
1. What are the three main property sets for each URDF link?
2. Explain the difference between `<visual>` and `<collision>` elements.
3. Name the four main joint types in URDF and describe when you would use each.
4. What is the purpose of the `origin` element in joints, and what do `xyz` and `rpy` represent?

### Exercise 3: AI-Robot Integration Concepts
1. Describe the three main pipelines in AI-robot integration (perception, decision, action).
2. What are the main differences between state machines and behavior trees for AI behavior?
3. Why is sensor fusion important in AI-robot systems?
4. How do you ensure safety when connecting AI agents to physical robots?

## Practical Exercises

### Exercise 4: Simple Publisher-Subscriber Pair
Create a publisher node that publishes random temperature values (Float64) to a topic called `/temperature`, and a subscriber node that receives these values and logs them to the console with a timestamp. Include error handling for invalid temperature values (below absolute zero or above reasonable limits).

### Exercise 5: URDF Robot Design
Design a simple wheeled robot in URDF with:
- A rectangular base body
- Two cylindrical wheels
- One caster wheel for balance
- A camera sensor on top
Include proper visual, collision, and inertial properties for each link, and define the joints connecting them.

### Exercise 6: Sensor Integration Node
Create an rclpy node that subscribes to multiple sensor topics (`/scan` for LIDAR, `/imu/data` for IMU, `/camera/image_raw` for camera) and publishes a combined sensor message that includes the most recent values from each sensor. Add appropriate QoS settings for real-time performance.

### Exercise 7: Simple AI Controller
Implement a simple AI controller that:
- Subscribes to laser scan data
- Implements a basic wall-following algorithm
- Publishes velocity commands to control a simulated robot
- Includes safety checks to prevent collisions

## Integration Exercises

### Exercise 8: Complete Robot System
Design and implement a complete system with:
1. A URDF model of a simple differential drive robot with camera and LIDAR
2. Gazebo plugins for the robot
3. An AI agent node that:
   - Subscribes to camera and LIDAR data
   - Processes sensor data to detect obstacles and features
   - Publishes velocity commands for navigation
4. A launch file to start the complete system

### Exercise 9: Humanoid Robot Arm Control
Create a URDF for a simple humanoid arm with shoulder, elbow, and wrist joints. Then implement:
1. A Python node that publishes joint commands to move the arm
2. A simple inverse kinematics solver to calculate joint angles for end-effector positions
3. A basic trajectory controller that moves the arm smoothly between positions

### Exercise 10: Simulation Environment Setup
Set up a complete simulation environment with:
1. A robot model with proper URDF and Gazebo plugins
2. Multiple sensors (camera, LIDAR, IMU)
3. A realistic environment/world in Gazebo
4. An AI agent that performs a meaningful task (navigation, manipulation, etc.)
5. Performance monitoring and metrics collection

## Advanced Challenges

### Exercise 11: Multi-Robot Coordination
Design a system where two robots coordinate to achieve a common goal:
1. Create URDF models for both robots
2. Implement communication between robots using topics or services
3. Design AI agents that share information and coordinate their actions
4. Test in simulation and document the coordination strategy

### Exercise 12: Real Robot Integration (Conceptual)
Plan how you would transition your simulation-based system to work with a real robot:
1. Identify differences between simulation and reality
2. Design sensor calibration procedures
3. Implement safety measures for real robot operation
4. Plan the testing methodology for safe deployment

## Code Review Exercises

### Exercise 13: Find the Issues
Analyze the following code and identify potential problems:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class BadRobotNode(Node):
    def __init__(self):
        super().__init__('bad_robot')
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 1)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)

    def scan_callback(self, msg):
        cmd = Twist()
        cmd.linear.x = 1.0
        self.publisher.publish(cmd)  # Always moves forward regardless of obstacles!

def main():
    rclpy.init()
    node = BadRobotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()  # Not in a finally block!
```

Identify at least 5 issues with this code and explain how to fix them.

### Exercise 14: URDF Review
Review the following URDF snippet and identify potential problems:

```xml
<robot name="bad_robot">
  <link name="base_link">
    <inertial>
      <mass value="0"/>  <!-- Is this okay? -->
      <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>  <!-- All zeros? -->
    </inertial>
    <visual>
      <geometry>
        <mesh filename="meshes/bad_model.stl"/>  <!-- What if this file doesn't exist? -->
      </geometry>
    </visual>
  </link>

  <joint name="fixed_joint" type="continuous">  <!-- Fixed but continuous? -->
    <parent link="base_link"/>
    <child link="sensor_link"/>
    <limit lower="0" upper="0" effort="1" velocity="1"/>  <!-- Limits for fixed joint? -->
  </joint>
</robot>
```

What problems do you see and how would you fix them?

## Self-Assessment Questions

### Question 1
Which of these is the correct way to properly initialize and clean up an rclpy node?
A) `rclpy.init()` → node creation → `rclpy.spin()` → `rclpy.shutdown()`
B) `rclpy.init()` → node creation → `rclpy.spin()` → node destruction → `rclpy.shutdown()` in a finally block
C) `rclpy.init()` → node creation → `rclpy.spin()` → `rclpy.shutdown()` without node destruction
D) No cleanup is needed in rclpy

### Question 2
In URDF, what's the difference between `<visual>` and `<collision>` elements?
A) There is no difference, they are interchangeable
B) Visual is for rendering, collision is for physics simulation
C) Visual is for simulation, collision is for real robots
D) Visual uses meters, collision uses centimeters

### Question 3
Which joint type would you use for a continuous rotating wheel?
A) Fixed
B) Revolute
C) Continuous
D) Prismatic

### Question 4
What is the purpose of the `use_sim_time` parameter in ROS 2?
A) To speed up simulation
B) To allow nodes to use simulation time instead of system time
C) To enable physics simulation
D) To synchronize all nodes

### Question 5
Which of these is a key benefit of using simulation in robotics development?
A) It's always identical to real-world behavior
B) It provides a safe environment to test without risk to hardware
C) It's faster than developing on real hardware
D) It eliminates the need for real robot testing

### Question 6
In a typical AI-robot integration pattern, what is the correct flow?
A) Robot → AI Agent → Robot
B) AI Agent → Robot → AI Agent
C) Sensors → AI Agent → Actions → Robot
D) Robot → Actions → AI Agent → Sensors

## Answers to Self-Assessment Questions

### Question 1: B) `rclpy.init()` → node creation → `rclpy.spin()` → node destruction → `rclpy.shutdown()` in a finally block
Proper cleanup should happen in a finally block to ensure resources are released even if exceptions occur.

### Question 2: B) Visual is for rendering, collision is for physics simulation
Visual elements define appearance for visualization tools, while collision elements define shapes for physics simulation.

### Question 3: C) Continuous
Continuous joints allow unlimited rotation around their axis, perfect for wheels.

### Question 4: B) To allow nodes to use simulation time instead of system time
This parameter enables nodes to use Gazebo's simulation time rather than system time.

### Question 5: B) It provides a safe environment to test without risk to hardware
Safety is a primary benefit of simulation, allowing testing without physical risks.

### Question 6: C) Sensors → AI Agent → Actions → Robot
The typical flow is sensor data to AI agent, which processes the information and generates actions for the robot.

## Hands-On Projects

### Project 1: Simple Navigation Robot
Create a complete navigation robot that:
1. Uses LIDAR to detect obstacles
2. Implements basic navigation algorithms (wall following, goal seeking)
3. Publishes velocity commands to move safely in an environment
4. Works both in simulation and could be adapted for a real robot

### Project 2: Object Recognition and Grasping
For a robot with a camera and arm:
1. Use camera data to detect objects
2. Calculate grasp positions
3. Control the arm to approach and grasp objects
4. Include proper error handling and safety checks

### Project 3: Multi-Sensor Fusion
Create a node that:
1. Subscribes to multiple sensors (IMU, LIDAR, camera, odometry)
2. Combines sensor data for better environmental understanding
3. Publishes fused information for use by other nodes
4. Implements appropriate filtering and processing techniques

## Key Takeaways Checklist

After completing these exercises, you should be able to:
- [ ] Create ROS 2 nodes using rclpy
- [ ] Design basic URDF models for robots
- [ ] Connect AI agents to robot sensors and actuators
- [ ] Work with simulation environments
- [ ] Implement basic robot control algorithms
- [ ] Use proper error handling and safety measures
- [ ] Integrate multiple sensors for robot perception
- [ ] Design and test robot behaviors in simulation
- [ ] Understand the simulation-to-reality transition

## Extension Activities

1. **Research**: Investigate advanced simulation environments like NVIDIA Isaac Sim
2. **Implementation**: Try implementing your own simple physics simulation
3. **Analysis**: Compare different robot description formats (URDF, SDF, MJCF)
4. **Study**: Explore advanced control algorithms (MPC, RL) for robotics

## Next Steps

Return to the [Module Overview](../../module-1-ros2-arch/index) to see the complete module, or explore the [Reference Materials](../../../reference/ros2-api-reference) and [Tutorials](../../../tutorials/ros2-basics/) for additional learning resources.