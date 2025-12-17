---
sidebar_position: 3
---

# ROS 2 Architecture Overview

This section provides an overview of the ROS 2 architecture, focusing on nodes, messages, and executors - the fundamental building blocks of the robotic nervous system.

## The ROS 2 Architecture

ROS 2 uses a distributed architecture where different software components (called "nodes") run independently and communicate with each other. Think of it like a city where different departments work independently but coordinate through official channels.

### Key Components

#### Nodes
Nodes are like individual workers or departments in a company:
- Each node performs a specific task (like perception, decision-making, or actuation)
- Nodes run independently of each other
- Nodes communicate through messages
- Examples: camera sensor node, motor controller node, path planner node

#### Messages
Messages are like emails or memos that nodes send to each other:
- They carry information between nodes
- They follow standard formats
- They can contain sensor data, commands, or status updates
- Examples: camera images, motor commands, position data

#### Communication Patterns
ROS 2 supports different ways for nodes to communicate:
- **Topics**: For continuous data streams (like radio broadcasts)
- **Services**: For request-response interactions (like phone calls)
- **Actions**: For long-running tasks with feedback (like project management)

#### Executors
Executors are like managers that run and coordinate nodes:
- They schedule when nodes perform their tasks
- They handle communication between nodes
- They ensure nodes run efficiently
- They can run multiple nodes simultaneously

## The Human Body Comparison

To understand ROS 2 architecture, think of the human body:

| ROS 2 Component | Human Body Equivalent | Function |
|-----------------|----------------------|----------|
| Node | Organ (heart, lung, brain) | Performs specific function |
| Message | Hormone or nerve signal | Carries information |
| Topic | Hormone in bloodstream | Broadcasts to multiple targets |
| Service | Direct conversation | One-to-one communication |
| Executor | Brain or nervous system | Coordinates all parts |

## How ROS 2 Fits into a Humanoid Robot System

In a humanoid robot, ROS 2 architecture connects:

1. **Perception Nodes**: Sensors like cameras, LIDAR, IMU that gather information
2. **Decision Nodes**: AI agents that process information and plan actions
3. **Action Nodes**: Motor controllers that execute physical movements
4. **Support Nodes**: System services like logging, visualization, and debugging

## The Communication Flow

Here's how information flows in a ROS 2 humanoid robot:

1. **Sensors publish data** to topics (camera sees obstacle)
2. **Perception nodes process** the raw data (identifies it as a wall)
3. **Decision nodes plan** appropriate action (decide to move around)
4. **Action nodes execute** the plan (send motor commands)
5. **Feedback flows back** to confirm success (encoders confirm movement)

## Key Architectural Principles

- **Decentralized**: No single point of failure
- **Modular**: Components can be added, removed, or replaced
- **Scalable**: New nodes can be added without disrupting existing ones
- **Language-independent**: Nodes can be written in different programming languages

## Summary

ROS 2 architecture provides a robust, flexible framework for building complex robot systems. By understanding nodes, messages, and communication patterns, you can begin to see how all the parts of a humanoid robot work together as a unified system.

## Exercise

Draw a simple diagram showing how a humanoid robot might use ROS 2 architecture to walk forward. Include at least 3 nodes (like camera, path planner, motor controller) and show how they would communicate using topics and services.