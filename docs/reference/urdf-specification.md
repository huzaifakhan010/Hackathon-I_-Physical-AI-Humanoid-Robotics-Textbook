---
sidebar_position: 2
---

# URDF Specification Reference

This reference guide provides an overview of the Unified Robot Description Format (URDF) specification for beginners. It serves as a quick reference for creating robot descriptions.

## URDF Structure Overview

URDF is an XML-based format for representing robot models. Every URDF file has this basic structure:

```xml
<?xml version="1.0"?>
<robot name="robot_name">
  <!-- Links define rigid bodies -->
  <link name="link_name">
    <!-- Visual properties for rendering -->
    <visual>
      <!-- Visual geometry definition -->
    </visual>

    <!-- Collision properties for physics -->
    <collision>
      <!-- Collision geometry definition -->
    </collision>

    <!-- Inertial properties for physics simulation -->
    <inertial>
      <!-- Mass and inertia definition -->
    </inertial>
  </link>

  <!-- Joints define connections between links -->
  <joint name="joint_name" type="joint_type">
    <parent link="parent_link_name"/>
    <child link="child_link_name"/>
    <!-- Joint origin and properties -->
  </joint>
</robot>
```

## Links: Rigid Bodies

Links represent the rigid, non-moving parts of a robot. Each link can have three property sets:

### Visual Properties
Defines how the link appears in visualization tools:

```xml
<link name="link_name">
  <visual>
    <!-- Position and orientation offset -->
    <origin xyz="0 0 0" rpy="0 0 0"/>

    <!-- Geometry definition -->
    <geometry>
      <!-- Choose one geometry type -->
      <box size="0.1 0.1 0.1"/>           <!-- Cuboid -->
      <cylinder radius="0.05" length="0.2"/> <!-- Cylinder -->
      <sphere radius="0.1"/>               <!-- Sphere -->
      <mesh filename="package://meshes/link.dae"/> <!-- Mesh file -->
    </geometry>

    <!-- Material definition -->
    <material name="material_name">
      <color rgba="0.8 0.2 0.2 1.0"/>  <!-- Red color with full opacity -->
      <texture filename="package://textures/texture.png"/> <!-- Optional texture -->
    </material>
  </visual>
</link>
```

### Collision Properties
Defines how the link interacts with physics simulation:

```xml
<link name="link_name">
  <collision>
    <!-- Position and orientation offset -->
    <origin xyz="0 0 0" rpy="0 0 0"/>

    <!-- Geometry definition (same as visual but can be different) -->
    <geometry>
      <box size="0.1 0.1 0.1"/>
      <!-- Often uses simpler geometry than visual for performance -->
    </geometry>
  </collision>
</link>
```

### Inertial Properties
Defines the physical properties for simulation:

```xml
<link name="link_name">
  <inertial>
    <!-- Mass in kilograms -->
    <mass value="1.0"/>

    <!-- Position and orientation offset -->
    <origin xyz="0 0 0" rpy="0 0 0"/>

    <!-- Inertia matrix (3x3 symmetric matrix) -->
    <inertia
      ixx="0.01" ixy="0.0" ixz="0.0"
      iyy="0.01" iyz="0.0"
      izz="0.01"/>
  </inertial>
</link>
```

## Joints: Connections Between Links

Joints define how links move relative to each other. The joint type determines the allowed motion:

### Joint Attributes
```xml
<joint name="joint_name" type="joint_type">
  <!-- Required: parent and child links -->
  <parent link="parent_link_name"/>
  <child link="child_link_name"/>

  <!-- Position and orientation of joint relative to parent -->
  <origin xyz="0 0 0" rpy="0 0 0"/>

  <!-- Axis of rotation or translation (for revolute/prismatic joints) -->
  <axis xyz="1 0 0"/>  <!-- Rotate/translate along X-axis -->
</joint>
```

### Joint Types

#### 1. Revolute Joint
Rotates around a single axis with limited range:

```xml
<joint name="elbow_joint" type="revolute">
  <parent link="upper_arm"/>
  <child link="forearm"/>
  <origin xyz="0 0 -0.3" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>  <!-- Rotate around Y-axis -->

  <!-- Joint limits -->
  <limit
    lower="-2.0"      <!-- Lower limit in radians -->
    upper="0.5"       <!-- Upper limit in radians -->
    effort="50"       <!-- Maximum torque (N-m) -->
    velocity="2"/>    <!-- Maximum velocity (rad/s) -->

  <!-- Optional dynamics properties -->
  <dynamics damping="0.5" friction="0.1"/>
</joint>
```

#### 2. Continuous Joint
Rotates continuously around a single axis:

```xml
<joint name="continuous_joint" type="continuous">
  <parent link="base"/>
  <child link="rotating_part"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>  <!-- Rotate around Z-axis -->

  <!-- Only effort and velocity limits (no position limits) -->
  <limit effort="100" velocity="5"/>
</joint>
```

#### 3. Prismatic Joint
Linear sliding motion along an axis:

```xml
<joint name="slider_joint" type="prismatic">
  <parent link="base"/>
  <child link="slider"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>  <!-- Move along Z-axis -->

  <!-- Linear limits -->
  <limit
    lower="0.0"       <!-- Lower position limit (m) -->
    upper="0.5"       <!-- Upper position limit (m) -->
    effort="200"      <!-- Maximum force (N) -->
    velocity="1"/>    <!-- Maximum velocity (m/s) -->
</joint>
```

#### 4. Fixed Joint
No movement between links (rigid connection):

```xml
<joint name="fixed_joint" type="fixed">
  <parent link="base"/>
  <child link="sensor_mount"/>
  <origin xyz="0.1 0 0.05" rpy="0 0 0"/>
  <!-- No limits or axis needed for fixed joints -->
</joint>
```

#### 5. Floating Joint
6 degrees of freedom (rarely used):

```xml
<joint name="floating_joint" type="floating">
  <parent link="base"/>
  <child link="floating_part"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <!-- Allows all 6 DOF: 3 translation + 3 rotation -->
</joint>
```

#### 6. Planar Joint
Motion constrained to a plane (rarely used):

```xml
<joint name="planar_joint" type="planar">
  <parent link="base"/>
  <child link="planar_part"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>  <!-- Normal to the plane -->
</joint>
```

## Common URDF Elements

### Materials
Define colors and textures for visualization:

```xml
<!-- Define material once -->
<material name="red">
  <color rgba="1 0 0 1"/>
</material>

<!-- Use material in visual elements -->
<visual>
  <material name="red"/>
  <geometry>
    <sphere radius="0.1"/>
  </geometry>
</visual>
```

### Gazebo Plugins
For simulation integration:

```xml
<!-- Apply material in Gazebo -->
<gazebo reference="link_name">
  <material>Gazebo/Red</material>
</gazebo>

<!-- Add controller plugins -->
<gazebo>
  <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>0.4</wheel_separation>
    <wheel_diameter>0.2</wheel_diameter>
  </plugin>
</gazebo>
```

## Complete Example: Simple Robot

Here's a complete example of a simple differential drive robot:

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base link -->
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
      <inertia
        ixx="0.2" ixy="0.0" ixz="0.0"
        iyy="0.3" iyz="0.0"
        izz="0.4"/>
    </inertial>
  </link>

  <!-- Left wheel -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.2 -0.1" rpy="-1.5708 0 0"/>
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
      <inertia
        ixx="0.001" ixy="0.0" ixz="0.0"
        iyy="0.001" iyz="0.0"
        izz="0.002"/>
    </inertial>
  </link>

  <!-- Right wheel -->
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.2 -0.1" rpy="1.5708 0 0"/>
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
      <inertia
        ixx="0.001" ixy="0.0" ixz="0.0"
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
      <inertia
        ixx="0.0001" ixy="0.0" ixz="0.0"
        iyy="0.0001" iyz="0.0"
        izz="0.0001"/>
    </inertial>
  </link>
</robot>
```

## URDF Best Practices

### 1. Consistent Naming
- Use descriptive names that indicate function
- Follow a consistent convention (e.g., `left_wheel_joint`, `camera_link`)
- Use underscores to separate words

### 2. Proper Scaling
- Use meters for all length measurements
- Ensure realistic proportions
- Match physical robot dimensions

### 3. Realistic Physical Properties
- Use realistic mass values
- Calculate inertia tensors properly
- Set appropriate joint limits

### 4. Efficient Geometry
- Use simple collision geometries for performance
- Use detailed visual meshes for appearance
- Consider using primitive shapes when possible

### 5. Hierarchical Structure
- Create a tree structure (no kinematic loops)
- Start with a base link
- Connect all parts in a logical hierarchy

## Common URDF Validation

### Check URDF Syntax
```bash
# Check if URDF is valid
check_urdf /path/to/robot.urdf

# Visualize URDF structure
urdf_to_graphiz /path/to/robot.urdf
```

### Common Issues to Avoid
- Invalid XML syntax
- Missing parent links
- Joint limits that don't make physical sense
- Zero masses or infinite limits
- Kinematic loops (URDF must be a tree structure)

## Xacro for Complex Robots

For complex robots, use Xacro (XML Macros) to simplify URDF:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="robot_with_xacro">

  <!-- Define properties -->
  <xacro:property name="wheel_radius" value="0.1"/>
  <xacro:property name="wheel_width" value="0.05"/>

  <!-- Macro for creating wheels -->
  <xacro:macro name="wheel" params="prefix parent x_pos y_pos">
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
      </visual>
    </link>
  </xacro:macro>

  <!-- Use the macro -->
  <xacro:wheel prefix="front_left" parent="base_link" x_pos="0.2" y_pos="0.15"/>
  <xacro:wheel prefix="front_right" parent="base_link" x_pos="0.2" y_pos="-0.15"/>

</robot>
```

## Key Takeaways

- URDF describes robot structure using XML
- Links are rigid bodies, joints define connections and motion
- Each link has visual, collision, and inertial properties
- Joint types determine allowed motion (revolute, continuous, prismatic, etc.)
- Proper physical properties are essential for simulation
- Use macros (Xacro) for complex or repetitive structures
- Always validate URDF before using in simulation
- Follow consistent naming and scaling conventions