---
sidebar_position: 3
---

# URDF Examples

This tutorial provides practical examples of creating robot descriptions using URDF (Unified Robot Description Format). These examples demonstrate common patterns and best practices for describing robots in ROS 2.

## Prerequisites

Before working through these examples, you should:
- Understand basic XML syntax
- Have ROS 2 installed with URDF support
- Know basic robotics concepts (links, joints, coordinate frames)

## What You'll Learn

In this tutorial, you'll learn how to:
1. Create basic robot models with links and joints
2. Define visual, collision, and inertial properties
3. Use different joint types for various movements
4. Add sensors to robot models
5. Validate and visualize your URDF models

## Example 1: Simple Mobile Robot

Let's start with a basic wheeled robot model.

**File: `simple_robot.urdf`**

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base link (main body of the robot) -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="5.0"/>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <inertia ixx="0.2" ixy="0.0" ixz="0.0"
               iyy="0.3" iyz="0.0"
               izz="0.4"/>
    </inertial>
  </link>

  <!-- Left wheel -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.15 -0.1" rpy="-1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0"
               iyy="0.001" iyz="0.0"
               izz="0.002"/>
    </inertial>
  </link>

  <!-- Right wheel -->
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.15 -0.1" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black"/>
    </visual>

    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0"
               iyy="0.001" iyz="0.0"
               izz="0.002"/>
    </inertial>
  </link>

  <!-- Caster wheel (fixed) -->
  <joint name="caster_joint" type="fixed">
    <parent link="base_link"/>
    <child link="caster_wheel"/>
    <origin xyz="0.2 0 -0.1" rpy="0 0 0"/>
  </joint>

  <link name="caster_wheel">
    <visual>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0"
               iyy="0.0001" iyz="0.0"
               izz="0.0001"/>
    </inertial>
  </link>
</robot>
```

## Example 2: Robot Arm

This example shows how to create a simple robot arm with multiple revolute joints.

**File: `robot_arm.urdf`**

```xml
<?xml version="1.0"?>
<robot name="robot_arm">
  <!-- Base of the arm -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.1" length="0.2"/>
      </geometry>
      <material name="green">
        <color rgba="0.2 0.8 0.2 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.1" length="0.2"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0"
               iyy="0.01" iyz="0.0"
               izz="0.005"/>
    </inertial>
  </link>

  <!-- Shoulder joint -->
  <joint name="shoulder_pan_joint" type="revolute">
    <parent link="base_link"/>
    <child link="shoulder_link"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="shoulder_link">
    <visual>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0.2 0.2 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.8"/>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <inertia ixx="0.002" ixy="0.0" ixz="0.0"
               iyy="0.002" iyz="0.0"
               izz="0.0001"/>
    </inertial>
  </link>

  <!-- Elbow joint -->
  <joint name="elbow_joint" type="revolute">
    <parent link="shoulder_link"/>
    <child link="elbow_link"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="0.5" effort="80" velocity="1"/>
  </joint>

  <link name="elbow_link">
    <visual>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <material name="blue">
        <color rgba="0.2 0.2 0.8 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.6"/>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0"
               iyy="0.001" iyz="0.0"
               izz="0.0001"/>
    </inertial>
  </link>

  <!-- Wrist joint -->
  <joint name="wrist_joint" type="revolute">
    <parent link="elbow_link"/>
    <child link="wrist_link"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1"/>
  </joint>

  <link name="wrist_link">
    <visual>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
      <material name="yellow">
        <color rgba="0.8 0.8 0.2 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0"
               iyy="0.0001" iyz="0.0"
               izz="0.0001"/>
    </inertial>
  </link>

  <!-- Gripper (end effector) -->
  <joint name="gripper_joint" type="fixed">
    <parent link="wrist_link"/>
    <child link="gripper_link"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
  </joint>

  <link name="gripper_link">
    <visual>
      <geometry>
        <box size="0.02 0.02 0.05"/>
      </geometry>
      <material name="white">
        <color rgba="0.9 0.9 0.9 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <box size="0.02 0.02 0.05"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.05"/>
      <inertia ixx="0.00001" ixy="0.0" ixz="0.0"
               iyy="0.00001" iyz="0.0"
               izz="0.00001"/>
    </inertial>
  </link>
</robot>
```

## Example 3: Humanoid Robot Torso

This example demonstrates a simplified humanoid torso with head and arms.

**File: `humanoid_torso.urdf`**

```xml
<?xml version="1.0"?>
<robot name="humanoid_torso">
  <!-- Main body/torso -->
  <link name="torso">
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
      <mass value="10.0"/>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <inertia ixx="0.5" ixy="0.0" ixz="0.0"
               iyy="0.6" iyz="0.0"
               izz="0.3"/>
    </inertial>
  </link>

  <!-- Neck joint -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 1.0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.5" upper="0.5" effort="20" velocity="2"/>
  </joint>

  <link name="head">
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
      <mass value="2.0"/>
      <inertia ixx="0.004" ixy="0.0" ixz="0.0"
               iyy="0.004" iyz="0.0"
               izz="0.004"/>
    </inertial>
  </link>

  <!-- Left shoulder (3 DOF for full arm movement) -->
  <joint name="left_shoulder_yaw" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.2 0 0.6" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.0" upper="1.0" effort="50" velocity="2"/>
  </joint>

  <link name="left_upper_arm">
    <visual>
      <origin xyz="0 0 -0.15" rpy="1.5708 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <material name="light_gray"/>
    </visual>

    <collision>
      <origin xyz="0 0 -0.15" rpy="1.5708 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.15" rpy="1.5708 0 0"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0"
               iyy="0.005" iyz="0.0"
               izz="0.0001"/>
    </inertial>
  </link>

  <!-- Left elbow -->
  <joint name="left_elbow" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_forearm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="0.0" effort="30" velocity="2"/>
  </joint>

  <link name="left_forearm">
    <visual>
      <origin xyz="0 0 -0.125" rpy="1.5708 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.25"/>
      </geometry>
      <material name="light_gray"/>
    </visual>

    <collision>
      <origin xyz="0 0 -0.125" rpy="1.5708 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.25"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.8"/>
      <origin xyz="0 0 -0.125" rpy="1.5708 0 0"/>
      <inertia ixx="0.003" ixy="0.0" ixz="0.0"
               iyy="0.003" iyz="0.0"
               izz="0.0001"/>
    </inertial>
  </link>

  <!-- Right shoulder (mirrored) -->
  <joint name="right_shoulder_yaw" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_arm"/>
    <origin xyz="-0.2 0 0.6" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.0" upper="1.0" effort="50" velocity="2"/>
  </joint>

  <link name="right_upper_arm">
    <visual>
      <origin xyz="0 0 -0.15" rpy="1.5708 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <material name="light_gray"/>
    </visual>

    <collision>
      <origin xyz="0 0 -0.15" rpy="1.5708 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.15" rpy="1.5708 0 0"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0"
               iyy="0.005" iyz="0.0"
               izz="0.0001"/>
    </inertial>
  </link>

  <!-- Right elbow -->
  <joint name="right_elbow" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_forearm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="0.0" effort="30" velocity="2"/>
  </joint>

  <link name="right_forearm">
    <visual>
      <origin xyz="0 0 -0.125" rpy="1.5708 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.25"/>
      </geometry>
      <material name="light_gray"/>
    </visual>

    <collision>
      <origin xyz="0 0 -0.125" rpy="1.5708 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.25"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.8"/>
      <origin xyz="0 0 -0.125" rpy="1.5708 0 0"/>
      <inertia ixx="0.003" ixy="0.0" ixz="0.0"
               iyy="0.003" iyz="0.0"
               izz="0.0001"/>
    </inertial>
  </link>
</robot>
```

## Example 4: Adding Sensors to Robots

This example shows how to add sensors to a robot model.

**File: `robot_with_sensors.urdf`**

```xml
<?xml version="1.0"?>
<robot name="robot_with_sensors">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.2" length="0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <cylinder radius="0.2" length="0.2"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0"
               iyy="0.1" iyz="0.0"
               izz="0.05"/>
    </inertial>
  </link>

  <!-- Camera sensor -->
  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.15 0 0.1" rpy="0 0 0"/>
  </joint>

  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.02 0.03 0.01"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
  </link>

  <!-- IMU sensor -->
  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
  </joint>

  <link name="imu_link">
    <!-- No visual representation needed for IMU -->
  </link>

  <!-- LIDAR sensor -->
  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
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

  <!-- Gazebo plugins for simulation -->
  <gazebo reference="camera_link">
    <sensor type="camera" name="camera_sensor">
      <update_rate>30</update_rate>
      <camera name="head_camera">
        <horizontal_fov>1.047</horizontal_fov>
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
        <frame_name>camera_link</frame_name>
        <topic_name>camera/image_raw</topic_name>
      </plugin>
    </sensor>
  </gazebo>

  <gazebo reference="imu_link">
    <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <plugin name="imu_controller" filename="libgazebo_ros_imu.so">
        <topic_name>imu/data</topic_name>
        <frame_name>imu_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>

  <gazebo reference="lidar_link">
    <sensor type="ray" name="lidar_sensor">
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
</robot>
```

## Validating URDF Models

To check if your URDF is valid, use the following command:

```bash
# Check URDF syntax
check_urdf /path/to/your/robot.urdf

# Visualize the URDF structure
urdf_to_graphiz /path/to/your/robot.urdf
```

This will generate a `.gv` file that you can view to see the link-joint structure of your robot.

## Visualizing URDF Models

To visualize your URDF in RViz:

```bash
# Publish the robot description
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="`cat /path/to/your/robot.urdf`"

# Or use a launch file to load the URDF
# Then run RViz to visualize
rviz2
```

## Best Practices

### 1. Consistent Naming
Use descriptive names that indicate the function of each link and joint:

```xml
<!-- Good naming -->
<link name="left_wheel"/>
<joint name="left_wheel_rotation_joint"/>
<link name="camera_link"/>
<joint name="head_to_camera_joint"/>

<!-- Avoid generic names -->
<link name="link1"/>
<joint name="joint1"/>
```

### 2. Proper Scaling
Always use meters for all measurements:

```xml
<!-- Correct: using meters -->
<box size="0.5 0.3 0.2"/>  <!-- 50cm x 30cm x 20cm -->

<!-- Incorrect: using centimeters or other units -->
<box size="50 30 20"/>     <!-- This is 50m x 30m x 20m! -->
```

### 3. Realistic Physical Properties
Use realistic mass and inertia values:

```xml
<!-- Realistic values for a small robot component -->
<inertial>
  <mass value="0.5"/>  <!-- 500g -->
  <inertia ixx="0.001" ixy="0.0" ixz="0.0"
           iyy="0.001" iyz="0.0"
           izz="0.002"/>
</inertial>
```

### 4. Hierarchical Structure
Maintain a proper tree structure (no loops):

```xml
<!-- Correct: tree structure -->
base_link
├── left_wheel
├── right_wheel
└── sensor_mount
    └── camera

<!-- Incorrect: this would create a loop -->
<!-- Don't connect two child links directly -->
```

### 5. Appropriate Joint Limits
Set realistic joint limits:

```xml
<!-- Realistic human-like arm limits -->
<joint name="elbow_joint" type="revolute">
  <limit lower="-2.0" upper="0.5" effort="80" velocity="1"/>
  <!-- Elbow can't rotate fully, only bend in one direction -->
</joint>
```

## Common Mistakes to Avoid

### 1. Invalid XML Syntax
```xml
<!-- Wrong: missing closing tag -->
<link name="example">
  <visual>
    <geometry>
      <box size="1 1 1"/>

<!-- Correct -->
<link name="example">
  <visual>
    <geometry>
      <box size="1 1 1"/>
    </geometry>
  </visual>
</link>
```

### 2. Zero Mass Values
```xml
<!-- Wrong: zero mass causes simulation problems -->
<inertial>
  <mass value="0"/>
  <!-- ... -->
</inertial>

<!-- Correct -->
<inertial>
  <mass value="0.1"/>  <!-- Small but non-zero -->
  <!-- ... -->
</inertial>
```

### 3. Inconsistent Units
```xml
<!-- Wrong: mixing units -->
<box size="50 30 20"/>  <!-- If these are cm, convert to m -->
<origin xyz="0.5 0.3 0.2"/>  <!-- These should be in m -->

<!-- Correct: consistent units -->
<box size="0.5 0.3 0.2"/>  <!-- All in meters -->
<origin xyz="0.5 0.3 0.2"/>
```

## Calculating Inertial Properties

For common shapes, here are the formulas for moment of inertia:

### Box (rectangular prism)
- `ixx = mass/12 * (height² + depth²)`
- `iyy = mass/12 * (width² + depth²)`
- `izz = mass/12 * (width² + height²)`

### Cylinder
- `ixx = mass/12 * (3*radius² + length²)` (around diameter)
- `izz = mass/2 * radius²` (around axis)

### Sphere
- `ixx = iyy = izz = 2/5 * mass * radius²`

## Using Xacro for Complex Models

For more complex robots, use Xacro (XML Macros) to simplify URDF:

**File: `robot_with_xacro.xacro`**
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="robot_with_xacro">

  <!-- Properties -->
  <xacro:property name="wheel_radius" value="0.1"/>
  <xacro:property name="wheel_width" value="0.05"/>
  <xacro:property name="base_length" value="0.5"/>

  <!-- Macro for creating wheels -->
  <xacro:macro name="wheel" params="prefix parent x_pos y_pos color:=black">
    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="${parent}"/>
      <child link="${prefix}_wheel"/>
      <origin xyz="${x_pos} ${y_pos} 0" rpy="0 0 0"/>
      <axis xyz="0 0 1"/>
    </joint>

    <link name="${prefix}_wheel">
      <visual>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
        <material name="${color}">
          <color rgba="0.1 0.1 0.1 1"/>
        </material>
      </visual>
    </link>
  </xacro:macro>

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="${base_length} 0.3 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
  </link>

  <!-- Use the macro to create wheels -->
  <xacro:wheel prefix="front_left" parent="base_link" x_pos="0.2" y_pos="0.15"/>
  <xacro:wheel prefix="front_right" parent="base_link" x_pos="0.2" y_pos="-0.15"/>
  <xacro:wheel prefix="rear_left" parent="base_link" x_pos="-0.2" y_pos="0.15"/>
  <xacro:wheel prefix="rear_right" parent="base_link" x_pos="-0.2" y_pos="-0.15"/>

</robot>
```

## Summary

These examples demonstrate essential URDF concepts:

1. **Basic structure**: Links connected by joints
2. **Visual, collision, and inertial properties**: For simulation and visualization
3. **Different joint types**: For various movement patterns
4. **Sensor integration**: Adding sensors to robot models
5. **Simulation plugins**: Connecting to Gazebo
6. **Best practices**: Proper naming, scaling, and structure

URDF is fundamental to robotics simulation and visualization in ROS 2. These patterns provide a solid foundation for describing robots of varying complexity, from simple mobile robots to complex humanoid systems.