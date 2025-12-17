---
sidebar_position: 3
---

# Introduction to URDF for Humanoid Robots

In this section, we'll explore URDF (Unified Robot Description Format), the standard XML-based format for describing robot structure and properties in ROS. URDF is like a blueprint or architectural plan for robots, defining how different parts connect and move relative to each other.

## What is URDF?

URDF (Unified Robot Description Format) is XML-based language that describes robot properties including:
- Physical structure (links and joints)
- Visual appearance (meshes, colors)
- Collision properties
- Inertial properties
- Sensor locations

### The Blueprint Analogy

Think of URDF like a detailed architectural blueprint for a building:
- **Links** are like rooms or structural components
- **Joints** are like doors or hinges connecting the components
- **Visual elements** are like the appearance of walls and fixtures
- **Collision properties** are like the physical boundaries of the structure

## Basic URDF Structure

Here's the basic structure of a URDF file:

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Links define the physical parts -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Joints connect links together -->
  <joint name="base_to_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <origin xyz="0 0.2 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="wheel_link">
    <!-- Wheel link definition -->
  </link>
</robot>
```

## Links: The Building Blocks

Links represent rigid bodies in the robot structure. Each link has:

### Visual Properties
- How the link appears in visualization tools
- Shape (box, cylinder, sphere, mesh)
- Color and material properties

```xml
<link name="link_name">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.1 0.1 0.1"/>  <!-- 10cm cube -->
    </geometry>
    <material name="red">
      <color rgba="1 0 0 1"/>  <!-- Red color -->
    </material>
  </visual>
</link>
```

### Collision Properties
- How the link interacts with physics simulations
- Shape used for collision detection

```xml
<link name="link_name">
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.1 0.1 0.1"/>
    </geometry>
  </collision>
</link>
```

### Inertial Properties
- Physical properties for simulation
- Mass, center of mass, and inertia tensor

```xml
<link name="link_name">
  <inertial>
    <mass value="1.0"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
  </inertial>
</link>
```

## Joints: The Connections

Joints define how links move relative to each other. Types of joints include:

### Fixed Joint
- No movement between links
- Used for rigid connections

```xml
<joint name="fixed_joint" type="fixed">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0.1 0 0" rpy="0 0 0"/>
</joint>
```

### Revolute Joint
- Rotates around a single axis
- Limited by angle range (like an elbow)

```xml
<joint name="revolute_joint" type="revolute">
  <parent link="upper_arm"/>
  <child link="forearm"/>
  <origin xyz="0 0 -0.3" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
</joint>
```

### Continuous Joint
- Rotates continuously (like a wheel)

```xml
<joint name="continuous_joint" type="continuous">
  <parent link="torso"/>
  <child link="head"/>
  <origin xyz="0 0 0.5" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit effort="100" velocity="1"/>
</joint>
```

### Prismatic Joint
- Linear sliding motion

```xml
<joint name="prismatic_joint" type="prismatic">
  <parent link="base"/>
  <child link="slider"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="0" upper="0.5" effort="100" velocity="1"/>
</joint>
```

## A Simple Humanoid Robot Example

Here's a simplified URDF for a basic humanoid torso:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base body/torso -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.2 1.0"/>
      </geometry>
      <material name="light_gray">
        <color rgba="0.7 0.7 0.7 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.2 1.0"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.05"/>
    </inertial>
  </link>

  <!-- Head -->
  <joint name="neck_joint" type="continuous">
    <parent link="base_link"/>
    <child link="head_link"/>
    <origin xyz="0 0 1.0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <link name="head_link">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="skin_color">
        <color rgba="0.8 0.6 0.4 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.004" ixy="0.0" ixz="0.0" iyy="0.004" iyz="0.0" izz="0.004"/>
    </inertial>
  </link>

  <!-- Left Arm -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.2 0 0.6" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1"/>
  </joint>

  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="light_gray"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.002" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>
</robot>
```

## URDF for Humanoid Robots

Humanoid robots have specific structural requirements:

### Key Components
1. **Torso/Body**: Central link connecting all parts
2. **Head**: With sensors (cameras, microphones)
3. **Arms**: With joints for manipulation
4. **Legs**: With joints for locomotion
5. **Hands/Feet**: End effectors

### Common Joint Arrangement
- **6DOF** (degrees of freedom) legs for walking
- **7DOF** arms for dexterity
- **3DOF** head for looking around
- **1-2DOF** spine for flexibility

### Example: Head Structure
```xml
<!-- Head with neck joints -->
<joint name="neck_yaw" type="revolute">
  <parent link="torso"/>
  <child link="neck"/>
  <origin xyz="0 0 0.9" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.0" upper="1.0" effort="10" velocity="2"/>
</joint>

<joint name="neck_pitch" type="revolute">
  <parent link="neck"/>
  <child link="head"/>
  <origin xyz="0 0 0.05" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-0.5" upper="0.5" effort="10" velocity="2"/>
</joint>
```

## Working with URDF in ROS 2

### Loading URDF Files
```python
import rclpy
from rclpy.node import Node
from urdf_parser_py.urdf import URDF

class URDFLoaderNode(Node):
    def __init__(self):
        super().__init__('urdf_loader')

        # Load URDF from file
        self.robot = URDF.from_xml_file('path/to/robot.urdf')

        # Access robot properties
        self.get_logger().info(f'Robot name: {self.robot.name}')
        self.get_logger().info(f'Links: {[link.name for link in self.robot.links]}')
        self.get_logger().info(f'Joints: {[joint.name for joint in self.robot.joints]}')
```

### Publishing Robot Description
To use URDF in ROS 2, you typically publish it to the `/robot_description` topic:

```xml
<!-- In your launch file -->
<node pkg="robot_state_publisher" exec="robot_state_publisher" name="robot_state_publisher">
  <param name="robot_description" value="$(find-pkg-share my_robot_description)/urdf/my_robot.urdf"/>
</node>
```

## Best Practices for URDF

### 1. Use Meaningful Names
- Use descriptive names for links and joints
- Follow consistent naming conventions
- Include functional information in names

### 2. Proper Scaling
- Use meters for all length measurements
- Ensure consistent scale throughout the model
- Verify dimensions match real robot

### 3. Logical Structure
- Create a tree structure (no loops)
- Start with a base link
- Connect all parts in a logical hierarchy

### 4. Physics Accuracy
- Use realistic mass and inertia values
- Proper collision geometry
- Appropriate joint limits

## Common URDF Tools

### Robot State Publisher
- Publishes joint positions to TF (Transform) tree
- Visualizes robot in RViz

### Joint State Publisher
- Publishes joint states for visualization
- Can provide GUI for joint control

### URDF Tutorials
- ROS 2 URDF tutorials for learning
- Example robots to study

## Validation and Debugging

### Check URDF Validity
```bash
# Check if URDF is syntactically correct
check_urdf /path/to/robot.urdf

# View URDF in a graphical format
urdf_to_graphiz /path/to/robot.urdf
```

### Common Issues
- Invalid XML syntax
- Missing parent links
- Joint limits that don't make sense
- Inconsistent units

## Key Takeaways

- URDF describes robot structure using XML
- Links are rigid bodies, joints define connections
- URDF is essential for simulation and visualization
- Humanoid robots have specific structural patterns
- Proper URDF enables robot simulation and control
- Follow best practices for maintainable robot descriptions

## Exercise

Create a simple URDF for a humanoid robot with:
1. A torso link
2. A head with 2 DOF (yaw and pitch)
3. Two arms with shoulder and elbow joints
4. Use proper visual and collision properties
5. Include realistic joint limits

Sketch out the structure and write the basic XML for this robot, focusing on the connection hierarchy and joint types.