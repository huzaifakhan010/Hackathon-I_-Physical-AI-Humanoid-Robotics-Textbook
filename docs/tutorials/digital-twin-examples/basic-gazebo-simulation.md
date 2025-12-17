---
sidebar_position: 17
title: "Basic Gazebo Simulation Tutorial"
---

# Basic Gazebo Simulation Tutorial

## Overview

This tutorial will guide you through creating a basic Gazebo simulation environment for a humanoid robot. You'll learn how to set up a simple world, spawn a robot, and interact with the simulation.

## Prerequisites

Before starting this tutorial, you should:
- Have Gazebo Classic installed (version 11 or later)
- Understand basic ROS concepts (topics, services, messages)
- Have a basic humanoid robot model (URDF or SDF format)
- Be familiar with terminal/command line operations

## Setting Up a Basic World

### Creating a Simple World File

Let's start by creating a basic world file. Create a new file called `basic_world.world`:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="basic_world">
    <!-- Physics Engine Configuration -->
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Include Ground Plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Include Sun Light -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Add a Simple Box Obstacle -->
    <model name="box_obstacle">
      <pose>2 0 0.5 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.083</ixx>
            <iyy>0.083</iyy>
            <izz>0.083</izz>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyz>0</iyz>
          </inertia>
        </inertial>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.2 0.1 1</ambient>
            <diffuse>0.8 0.2 0.1 1</diffuse>
            <specular>0.8 0.2 0.1 1</specular>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>
```

### World Configuration Explained

- **`<gravity>`**: Sets gravitational acceleration (9.8 m/s² downward)
- **`<max_step_size>`**: Maximum time step for physics simulation (0.001 seconds)
- **`<real_time_factor>`**: Target simulation speed (1x real-time)
- **`<real_time_update_rate>`**: Update frequency (1000 Hz)

## Spawning a Robot

### Using a Pre-existing Model

For this tutorial, we'll use a simple humanoid model. If you don't have one, you can start with a simple model like this:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="simple_humanoid">
    <pose>0 0 1 0 0 0</pose>

    <!-- Torso -->
    <link name="torso">
      <inertial>
        <mass>10.0</mass>
        <inertia>
          <ixx>0.5</ixx>
          <iyy>0.5</iyy>
          <izz>0.5</izz>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyz>0</iyz>
        </inertia>
      </inertial>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.5 0.3 0.8</size>
          </box>
        </geometry>
        <material>
          <ambient>0.5 0.5 1 1</ambient>
          <diffuse>0.5 0.5 1 1</diffuse>
          <specular>0.5 0.5 1 1</specular>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.5 0.3 0.8</size>
          </box>
        </geometry>
      </collision>
    </link>

    <!-- Head -->
    <link name="head">
      <pose>0 0 0.6 0 0 0</pose>
      <inertial>
        <mass>2.0</mass>
        <inertia>
          <ixx>0.02</ixx>
          <iyy>0.02</iyy>
          <izz>0.02</izz>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyz>0</iyz>
        </inertia>
      </inertial>
      <visual name="visual">
        <geometry>
          <sphere>
            <radius>0.15</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>0.8 0.8 0.8 1</ambient>
          <diffuse>0.8 0.8 0.8 1</diffuse>
          <specular>0.8 0.8 0.8 1</specular>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <sphere>
            <radius>0.15</radius>
          </sphere>
        </geometry>
      </collision>
    </link>

    <!-- Joint connecting head to torso -->
    <joint name="neck_joint" type="revolute">
      <parent>torso</parent>
      <child>head</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <lower>-0.5</lower>
          <upper>0.5</upper>
          <effort>100</effort>
          <velocity>1</velocity>
        </limit>
      </axis>
    </joint>
  </model>
</sdf>
```

### Running the Simulation

1. **Start Gazebo with your world**:
   ```bash
   gazebo basic_world.world
   ```

2. **Spawn your robot** (in a new terminal):
   ```bash
   # If using ROS, you can use the spawn service:
   rosservice call /gazebo/spawn_sdf_model "model_name: 'simple_humanoid'
   model_xml: '`cat simple_humanoid.sdf`'
   robot_namespace: 'simple_humanoid'
   initial_pose:
     position: {x: 0, y: 0, z: 1}
     orientation: {x: 0, y: 0, z: 0, w: 1}
   reference_frame: 'world'"
   ```

## Adding Sensors

### Camera Sensor Example

Let's add a simple RGB camera to the robot's head:

```xml
<link name="head">
  <!-- existing head properties -->

  <!-- Camera sensor -->
  <sensor name="camera" type="camera">
    <camera>
      <horizontal_fov>1.047</horizontal_fov> <!-- 60 degrees -->
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>10</far>
      </clip>
    </camera>
    <always_on>true</always_on>
    <update_rate>30</update_rate>
    <visualize>true</visualize>
  </sensor>
</link>
```

### IMU Sensor Example

Add an IMU to the torso for balance information:

```xml
<link name="torso">
  <!-- existing torso properties -->

  <!-- IMU sensor -->
  <sensor name="imu" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.001</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.001</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.001</stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
  </sensor>
</link>
```

## Controlling the Robot

### Using Joint Control

You can control the robot's joints using ROS topics. For the neck joint we created:

```bash
# Set the joint position to 0.2 radians
rostopic pub /simple_humanoid/neck_joint_position_controller/command std_msgs/Float64 "data: 0.2"
```

### Setting up ROS Controllers

To make joint control work, you'll need controller configuration. Create a `controllers.yaml` file:

```yaml
simple_humanoid:
  # Position controllers
  neck_joint_position_controller:
    type: position_controllers/JointPositionController
    joint: neck_joint
    pid: {p: 100.0, i: 0.01, d: 10.0}
```

## Interacting with the Simulation

### Using Gazebo GUI

- **Move Model**: Click the move tool and drag models in the world
- **Reset Simulation**: Use the reset button to restart the simulation
- **Pause/Play**: Control simulation time with the transport controls
- **Step Forward**: Step through simulation one time step at a time

### Command Line Tools

- **`gz model`**: Control models in the simulation
- **`gz topic`**: List and inspect topics
- **`gz service`**: Call services in the simulation
- **`gz stats`**: View simulation statistics

## Validation and Testing

### Checking Model Behavior

1. **Visual Inspection**: Does the robot behave as expected?
2. **Physics Validation**: Does the robot respond to gravity and collisions appropriately?
3. **Sensor Output**: Are sensors providing expected data?
4. **Control Response**: Do control commands produce expected movements?

### Performance Monitoring

- **Real-time Factor**: Monitor if simulation is keeping up with real-time (should be close to 1.0)
- **Frame Rate**: Ensure GUI is running smoothly
- **CPU Usage**: Monitor system resource usage
- **Physics Accuracy**: Verify physics behavior remains stable

## Troubleshooting Common Issues

### Simulation Instability

**Problem**: Robot oscillates or behaves erratically
**Solutions**:
- Reduce physics time step size
- Adjust solver parameters
- Verify mass and inertia properties
- Check joint limits and constraints

### Performance Issues

**Problem**: Simulation runs slowly
**Solutions**:
- Simplify collision geometry
- Reduce model complexity
- Lower sensor update rates
- Use simpler physics parameters

### Joint Control Problems

**Problem**: Joints don't respond to commands
**Solutions**:
- Verify controller configuration
- Check ROS topic connections
- Ensure proper controller loading
- Validate joint type and limits

## Extending the Tutorial

### Adding More Complex Environments

- Create rooms with walls and furniture
- Add multiple robots to the same simulation
- Include dynamic obstacles and moving objects
- Implement complex terrain with ramps and stairs

### Advanced Sensor Integration

- Add LiDAR sensors for navigation
- Include force/torque sensors for manipulation
- Integrate GPS sensors for outdoor navigation
- Add multiple cameras for stereo vision

### Multi-Robot Scenarios

- Simulate robot teams working together
- Implement communication between robots
- Test collision avoidance algorithms
- Coordinate complex multi-robot tasks

## Summary

This tutorial has covered the basics of creating a Gazebo simulation for a humanoid robot:
- Setting up a basic world with physics properties
- Creating a simple humanoid robot model
- Adding sensors to the robot
- Controlling robot joints
- Interacting with and validating the simulation

These fundamentals provide a solid foundation for more complex Digital Twin simulations. As you advance, consider exploring more sophisticated models, complex environments, and integration with AI systems for perception and control.

## Next Steps

After completing this tutorial, consider exploring:
- More complex robot models with complete humanoid kinematics
- Integration with ROS control frameworks
- Advanced sensor simulation for AI training
- Physics validation and sim-to-real transfer techniques