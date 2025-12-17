# ROS 2 Terminology Guide

This document provides beginner-friendly definitions for key ROS 2 concepts used throughout the module.

## Key Terms

### ROS 2 Node
A software component in the ROS 2 system that performs specific tasks, like a brain cell (neuron) in a robotic nervous system. Examples include sensor nodes, controller nodes, or AI agent nodes.

### ROS 2 Topic
A communication channel where nodes can send and receive continuous streams of information, like a radio frequency where multiple listeners can hear the same broadcast. Used for publisher/subscriber communication patterns.

### ROS 2 Service
A request-response communication method where one node asks another for specific information or action, like asking a question and getting an answer. Used for synchronous communication.

### rclpy
Python library that allows Python programs to connect to and communicate with ROS 2 systems, enabling Python AI agents to control robots.

### URDF (Unified Robot Description Format)
An XML-based format used to describe robot structure including joints, links, and sensors, like a blueprint for a robot.

### Humanoid Robot
A robot designed with human-like structure including joints (like elbows and knees), links (like arms and legs), and sensors for movement and interaction.

## Analogies Used

Throughout this module, we use these analogies to explain complex concepts:
- **ROS 2 as Nervous System**: ROS 2 connects robot components like the nervous system connects parts of the human body
- **Nodes as Organs**: Each node performs a specific function like organs in a biological system
- **Topics as Nerve Pathways**: Topics carry continuous information like nerve pathways in the nervous system
- **Services as Phone Calls**: Services provide specific request-response like phone calls between people
- **URDF as Blueprint**: URDF describes robot structure like a blueprint describes a building