---
sidebar_position: 4
---

# Joints, Links, and Sensors in Robot Description

In this section, we'll explore the fundamental components of robot descriptions: joints, links, and sensors. Understanding how these elements work together is crucial for describing humanoid robots in URDF and connecting them to ROS 2 systems.

## Links: The Rigid Bodies

Links represent the rigid, non-moving parts of a robot. Think of them as the "bones" of the robot structure.

### Link Properties

Every link in URDF has three main property sets:

#### 1. Visual Properties
Describes how the link looks in visualization tools:

```xml
<link name="upper_arm">
  <visual>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <geometry>
      <cylinder length="0.3" radius="0.05"/>
    </geometry>
    <material name="arm_material">
      <color rgba="0.8 0.8 0.8 1"/>
    </material>
  </visual>
</link>
```

#### 2. Collision Properties
Defines how the link interacts with physics simulations:

```xml
<link name="upper_arm">
  <collision>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <geometry>
      <cylinder length="0.3" radius="0.05"/>
    </geometry>
  </collision>
</link>
```

#### 3. Inertial Properties
Specifies the physical properties for simulation:

```xml
<link name="upper_arm">
  <inertial>
    <mass value="0.8"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <inertia ixx="0.001" ixy="0.0" ixz="0.0"
             iyy="0.001" iyz="0.0" izz="0.0001"/>
  </inertial>
</link>
```

### Link Hierarchy in Humanoid Robots

Humanoid robots typically follow this link hierarchy:

```
base_link (torso)
├── head_link
├── left_upper_arm
│   └── left_forearm
│       └── left_hand
├── right_upper_arm
│   └── right_forearm
│       └── right_hand
├── left_thigh
│   └── left_shin
│       └── left_foot
└── right_thigh
    └── right_shin
        └── right_foot
```

## Joints: The Connections

Joints define how links move relative to each other. They are the "joints" in the robot's skeleton.

### Joint Types for Humanoid Robots

#### 1. Revolute Joints
Rotate around a single axis with limited range (like elbows, knees):

```xml
<joint name="left_elbow" type="revolute">
  <parent link="left_upper_arm"/>
  <child link="left_forearm"/>
  <origin xyz="0 0 -0.3" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-2.0" upper="0.0" effort="100" velocity="2"/>
  <dynamics damping="0.5" friction="0.1"/>
</joint>
```

#### 2. Continuous Joints
Rotate continuously around an axis (like head rotation):

```xml
<joint name="head_yaw" type="continuous">
  <parent link="torso"/>
  <child link="head"/>
  <origin xyz="0 0 0.9" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <dynamics damping="0.2"/>
</joint>
```

#### 3. Fixed Joints
Connect links without allowing movement (for attaching sensors):

```xml
<joint name="head_to_camera" type="fixed">
  <parent link="head"/>
  <child link="camera_frame"/>
  <origin xyz="0.05 0 0.05" rpy="0 0 0"/>
</joint>
```

### Joint Parameters

#### Origin
- `xyz`: Position offset from parent link
- `rpy`: Rotation offset (roll, pitch, yaw) from parent link

#### Axis
- Defines the rotation axis as a unit vector
- `xyz="0 1 0"` means rotation around the Y-axis

#### Limits (for revolute joints)
- `lower/upper`: Angle limits in radians
- `effort`: Maximum torque (N-m)
- `velocity`: Maximum speed (rad/s)

#### Dynamics
- `damping`: Resistance to motion
- `friction`: Static friction coefficient

### Humanoid Joint Configurations

#### Arm Joints
```xml
<!-- Shoulder: 3 DOF for full arm movement -->
<joint name="left_shoulder_yaw" type="revolute">
  <parent link="torso"/>
  <child link="left_upper_arm"/>
  <origin xyz="0.2 0 0.6" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.0" upper="1.0" effort="50" velocity="2"/>
</joint>

<joint name="left_shoulder_pitch" type="revolute">
  <parent link="left_upper_arm"/>
  <child link="left_upper_arm"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>
  <limit lower="-2.0" upper="1.0" effort="50" velocity="2"/>
</joint>

<joint name="left_shoulder_roll" type="revolute">
  <parent link="left_upper_arm"/>
  <child link="left_upper_arm"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-3.0" upper="1.0" effort="50" velocity="2"/>
</joint>
```

#### Leg Joints
```xml
<!-- Hip: 3 DOF for leg movement -->
<joint name="left_hip_yaw" type="revolute">
  <parent link="torso"/>
  <child link="left_thigh"/>
  <origin xyz="0 -0.1 -0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-0.5" upper="0.5" effort="100" velocity="1"/>
</joint>

<joint name="left_hip_pitch" type="revolute">
  <parent link="left_thigh"/>
  <child link="left_thigh"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>
  <limit lower="-2.0" upper="0.5" effort="100" velocity="1"/>
</joint>

<joint name="left_knee" type="revolute">
  <parent link="left_thigh"/>
  <child link="left_shin"/>
  <origin xyz="0 0 -0.4" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>
  <limit lower="0.0" upper="2.5" effort="100" velocity="1"/>
</joint>
```

## Sensors in Robot Descriptions

Sensors are typically represented as additional links connected to the main robot structure via fixed joints.

### Common Robot Sensors

#### Camera Sensors
```xml
<!-- Camera attached to head -->
<joint name="head_to_camera" type="fixed">
  <parent link="head"/>
  <child link="camera_frame"/>
  <origin xyz="0.05 0 0.05" rpy="0 0 0"/>
</joint>

<link name="camera_frame">
  <visual>
    <geometry>
      <box size="0.02 0.03 0.01"/>
    </geometry>
    <material name="black">
      <color rgba="0 0 0 1"/>
    </material>
  </visual>
</link>

<!-- Gazebo plugin for camera simulation -->
<gazebo reference="camera_frame">
  <sensor type="camera" name="head_camera">
    <update_rate>30</update_rate>
    <camera name="head_camera">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <frame_name>camera_frame</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

#### IMU (Inertial Measurement Unit)
```xml
<!-- IMU in torso -->
<joint name="torso_to_imu" type="fixed">
  <parent link="torso"/>
  <child link="imu_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>

<link name="imu_link">
  <!-- No visual representation needed -->
</link>

<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
  </sensor>
</gazebo>
```

#### LIDAR Sensors
```xml
<!-- LIDAR on head -->
<joint name="head_to_lidar" type="fixed">
  <parent link="head"/>
  <child link="lidar_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
</joint>

<link name="lidar_link">
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.05"/>
    </geometry>
    <material name="gray">
      <color rgba="0.5 0.5 0.5 1"/>
    </material>
  </visual>
</link>

<gazebo reference="lidar_link">
  <sensor type="ray" name="head_lidar">
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>30</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="lidar_controller" filename="libgazebo_ros_laser.so">
      <topic_name>scan</topic_name>
      <frame_name>lidar_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

## Complete Humanoid Robot Example

Here's a more complete example showing how joints, links, and sensors work together:

```xml
<?xml version="1.0"?>
<robot name="complete_humanoid">
  <!-- Base torso with sensors -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="0.5" ixy="0.0" ixz="0.0"
               iyy="0.6" iyz="0.0" izz="0.3"/>
    </inertial>
  </link>

  <!-- IMU sensor in torso -->
  <joint name="torso_to_imu" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>

  <link name="imu_link"/>

  <!-- Head with camera -->
  <joint name="neck_joint" type="continuous">
    <parent link="base_link"/>
    <child link="head_link"/>
    <origin xyz="0 0 0.4" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <link name="head_link">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="skin">
        <color rgba="0.8 0.6 0.4 1"/>
      </material>
    </visual>
  </link>

  <!-- Camera on head -->
  <joint name="head_to_camera" type="fixed">
    <parent link="head_link"/>
    <child link="camera_link"/>
    <origin xyz="0.05 0 0.05" rpy="0 0 0"/>
  </joint>

  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.02 0.03 0.01"/>
      </geometry>
    </visual>
  </link>

  <!-- Left arm -->
  <joint name="left_shoulder" type="revolute">
    <parent link="base_link"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.2 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="2"/>
  </joint>

  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </visual>
  </link>

  <joint name="left_elbow" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_forearm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="0.0" effort="30" velocity="2"/>
  </joint>

  <link name="left_forearm">
    <visual>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
    </visual>
  </link>
</robot>
```

## Working with Joints, Links, and Sensors in ROS 2

### Accessing Joint Information
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
import math

class JointSensorNode(Node):
    def __init__(self):
        super().__init__('joint_sensor_node')

        # Subscribe to joint states
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)

        # Create transform broadcaster for TF tree
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer to publish transforms
        self.timer = self.create_timer(0.05, self.publish_transforms)

    def joint_callback(self, msg):
        # Process joint state messages
        self.joint_positions = dict(zip(msg.name, msg.position))

    def publish_transforms(self):
        # Publish transforms based on joint positions
        # This enables visualization and navigation
        pass
```

### Sensor Integration
```python
class SensorIntegrationNode(Node):
    def __init__(self):
        super().__init__('sensor_integration_node')

        # Subscribe to various sensors
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)

        self.camera_sub = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, 10)

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

    def imu_callback(self, msg):
        # Process IMU data for balance control
        self.process_imu_for_balance(msg)

    def camera_callback(self, msg):
        # Process camera data for perception
        self.process_camera_for_perception(msg)

    def scan_callback(self, msg):
        # Process LIDAR data for navigation
        self.process_scan_for_navigation(msg)
```

## Best Practices

### 1. Consistent Naming
- Use descriptive names that indicate function
- Follow a consistent convention (e.g., `left_elbow_joint`, `right_camera_link`)

### 2. Proper Scaling
- Use meters for all measurements
- Ensure realistic proportions
- Match physical robot dimensions

### 3. Realistic Joint Limits
- Set limits based on physical capabilities
- Include safety margins
- Consider wear and tear

### 4. Sensor Placement
- Place sensors where they'll be physically mounted
- Consider field of view and sensing range
- Account for sensor-to-sensor interference

### 5. Mass and Inertia Properties
- Use realistic values for simulation
- Consider actual component weights
- Account for payloads and attachments

## Common Mistakes to Avoid

### 1. Kinematic Loops
- URDF must form a tree structure
- No closed loops in the kinematic chain
- Use fixed joints for parallel mechanisms

### 2. Incorrect Joint Axes
- Ensure axis directions match physical joints
- Use right-hand rule for rotation axes
- Verify joint directions with visualization

### 3. Unrealistic Parameters
- Avoid zero masses or infinite limits
- Use realistic damping and friction values
- Ensure sensor parameters match hardware

## Key Takeaways

- Links define the rigid structure of the robot
- Joints define how parts move relative to each other
- Sensors are represented as additional links with special properties
- Proper URDF structure is essential for simulation and control
- Joint limits and physical properties affect robot behavior
- Sensors enable perception and environmental awareness
- Realistic parameters improve simulation accuracy

## Exercise

Design a simplified humanoid robot with the following requirements:
1. A torso with IMU sensor
2. A head with camera
3. Two arms with shoulder and elbow joints
4. Two legs with hip and knee joints
5. Proper joint limits based on human-like movement

Create the URDF structure showing:
- The link hierarchy
- Joint types and limits
- Sensor placements
- Basic visual properties

Explain how each component contributes to the robot's overall functionality.