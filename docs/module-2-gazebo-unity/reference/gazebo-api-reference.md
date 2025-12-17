---
sidebar_position: 14
title: "Gazebo API Reference"
---

# Gazebo API Reference

## Overview

This reference provides an overview of key Gazebo concepts and APIs that are relevant for Digital Twin development. Understanding these concepts helps in creating effective simulation environments for humanoid robots.

## Core Concepts

### World Files (SDF Format)
Simulation Description Format (SDF) is the XML-based format used to describe simulation worlds in Gazebo.

#### Basic World Structure
```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="default">
    <!-- World elements go here -->
  </world>
</sdf>
```

#### Common World Elements
- **`<physics>`**: Physics engine configuration
- **`<model>`**: Robot and object definitions
- **`<light>`**: Lighting configuration
- **`<plugin>`**: Custom plugin definitions

### Models
Models represent objects, robots, or environmental elements in the simulation.

#### Model Structure
```xml
<model name="my_robot">
  <pose>0 0 0 0 0 0</pose>  <!-- x, y, z, roll, pitch, yaw -->
  <link name="link_name">
    <!-- Link properties -->
  </link>
  <joint name="joint_name" type="revolute">
    <!-- Joint properties -->
  </joint>
</model>
```

#### Link Properties
- **`<inertial>`**: Mass, center of mass, and inertia properties
- **`<visual>`**: Visual appearance (color, mesh, etc.)
- **`<collision>`**: Collision geometry for physics simulation
- **`<sensor>`**: Sensor definitions attached to the link

### Physics Engine Configuration

#### Common Physics Parameters
- **`<gravity>`**: Gravitational acceleration (typically 0 0 -9.8)
- **`<max_step_size>`**: Maximum time step for simulation
- **`<real_time_factor>`**: Target simulation speed relative to real time
- **`<real_time_update_rate>`**: Update rate in Hz

## Key APIs and Tools

### Gazebo Client (gzclient)
The GUI client for visualizing and interacting with simulations.

### Gazebo Server (gzserver)
The backend server that runs the physics simulation.

### Model Database (Fuel)
Gazebo's online repository of pre-built models and worlds.

## ROS Integration

### Common Message Types
- **`gazebo_msgs/ModelState`**: Control model positions and velocities
- **`gazebo_msgs/ModelStates`**: Get positions of all models
- **`sensor_msgs/JointState`**: Joint position, velocity, and effort
- **`geometry_msgs/Twist`**: Robot velocity commands

### Common Services
- **`/gazebo/spawn_sdf_model`**: Spawn new models in simulation
- **`/gazebo/delete_model`**: Remove models from simulation
- **`/gazebo/set_model_state`**: Set model position and velocity
- **`/gazebo/get_model_state`**: Get current model state

## Physics Parameters

### Damping and Friction
- **`<damping>`**: Linear and angular damping coefficients
- **`<friction>`**: Static and dynamic friction coefficients
- **`<kp>` and `<kd>`**: Spring stiffness and damping coefficients

### Contact Properties
- **`<max_contacts>`**: Maximum number of contacts for a collision
- **`<surface>`**: Contact surface parameters
- **`<ode>`**: Open Dynamics Engine specific parameters

## Sensor Simulation

### Common Sensor Types
- **`<camera>`**: RGB camera sensors
- **`<gpu_lidar>`**: GPU-accelerated LiDAR
- **`<imu>`**: Inertial measurement unit
- **`<contact>`**: Contact sensors for touch detection

### Sensor Parameters
- **`<update_rate>`**: How often sensor data is updated
- **`<always_on>`**: Whether sensor is always active
- **`<visualize>`**: Whether to visualize sensor in GUI
- **`<topic>`**: ROS topic for sensor data output

## Environment Setup

### Creating a Basic World
```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="simple_world">
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>

    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>
  </world>
</sdf>
```

## Common Use Cases

### Spawning Robots
Use the spawn service to add robots to a running simulation:
```bash
rosservice call /gazebo/spawn_sdf_model "model_name: 'my_robot'
model_xml: '<robot description here>'
robot_namespace: 'my_robot'
initial_pose:
  position: {x: 0, y: 0, z: 0}
  orientation: {x: 0, y: 0, z: 0, w: 1}
reference_frame: 'world'"
```

### Controlling Robot Motion
Send commands to robot joints using ROS topics:
- Joint position controllers: `/robot/joint_position_controller/command`
- Velocity controllers: `/robot/joint_velocity_controller/command`
- Effort controllers: `/robot/joint_effort_controller/command`

## Troubleshooting Common Issues

### Simulation Instability
- Increase physics update rate
- Reduce time step size
- Check mass and inertia properties
- Verify joint limits and constraints

### Performance Issues
- Simplify collision geometry
- Reduce model complexity
- Lower sensor update rates
- Use simpler physics engine settings

## Best Practices

### Model Design
- Use appropriate collision geometry (simpler than visual)
- Set realistic mass and inertia properties
- Use joint limits to prevent self-collision
- Validate models in isolation before complex scenarios

### Simulation Setup
- Start with simple worlds and add complexity gradually
- Use appropriate physics parameters for your application
- Validate simulation behavior against real-world expectations
- Document simulation parameters for reproducibility

## Resources

### Official Documentation
- [Gazebo Classic Documentation](http://gazebosim.org/tutorials)
- [SDF Specification](http://sdformat.org/spec)
- [Gazebo Tutorials](http://gazebosim.org/tutorials)

### Community Resources
- Gazebo Answers forum
- ROS Answers (for ROS integration)
- GitHub repositories with example models and worlds

## Summary

This reference provides the essential Gazebo concepts needed for Digital Twin development. Understanding these elements enables the creation of effective simulation environments that accurately represent physical systems for humanoid robot development.