---
sidebar_position: 6
---

# Chapter 2 Exercises

Complete these exercises to reinforce your understanding of ROS 2 communication patterns: nodes, topics, and services.

## Conceptual Exercises

### Exercise 1: Communication Pattern Identification
For each scenario, identify which ROS 2 communication pattern would be most appropriate and explain why:

1. A camera continuously sending image data to multiple processing nodes
2. Requesting the current position of a robot
3. Commanding a robot to move to a specific location with feedback during the journey
4. Broadcasting battery level to all monitoring nodes
5. Saving a map file to disk
6. Requesting the robot to switch to autonomous mode

### Exercise 2: System Design - Delivery Robot
Design a ROS 2 system for a delivery robot that moves packages within a building:

1. List 6 different nodes this robot would need
2. For each node, describe what it would do
3. Identify which communication pattern (topic or service) each node would use
4. Draw (or describe) how information would flow between nodes

### Exercise 3: Topic vs Service Decision Matrix
Create a decision matrix for choosing between topics and services. For each factor below, indicate whether topics or services are more appropriate:

- Data that changes continuously
- One-time configuration changes
- Sensor data sharing
- Requesting specific information
- Broadcasting status to multiple nodes
- Actions that must be confirmed

## Practical Exercises

### Exercise 4: Message Flow Analysis
Analyze the following robot system:
- Camera node publishing images to `/camera/image_raw`
- Object detection node subscribing to `/camera/image_raw` and publishing results to `/detected_objects`
- Navigation node subscribing to `/detected_objects` and publishing velocity commands to `/cmd_vel`
- Motor controller node subscribing to `/cmd_vel`

Answer these questions:
1. How many topics are in this system?
2. Which nodes are publishers? Which are subscribers?
3. What would happen if the object detection node failed?
4. How would the information flow change if the navigation node also needed raw camera data?

### Exercise 5: Humanoid Robot Communication Design
Design the communication system for a humanoid robot that can walk, recognize faces, and respond to voice commands:

1. Identify at least 8 nodes needed for this robot
2. For each node, specify whether it would primarily be a publisher, subscriber, or service provider
3. Design the topics and services needed for coordination
4. Explain how the robot would handle a scenario where it sees a familiar face and walks toward it

### Exercise 6: QoS Considerations
For each scenario, explain what QoS settings would be most appropriate and why:

1. Publishing high-resolution camera images
2. Sending critical emergency stop commands
3. Broadcasting robot pose for visualization
4. Requesting robot configuration parameters
5. Publishing joint state information for control

## Application Exercises

### Exercise 7: Topic Design Challenge
Design a topic-based system for a security robot that patrols a building:

1. Create a list of 5-7 topics the system would need
2. For each topic, specify the message type and frequency
3. Identify which nodes would publish to each topic
4. Identify which nodes would subscribe to each topic
5. Explain how the system would detect and respond to an intruder

### Exercise 8: Service Design Challenge
Design service-based interactions for the same security robot:

1. Create 4-5 services the robot would need
2. For each service, specify the request and response message types
3. Identify which nodes would provide each service
4. Identify which nodes would call each service
5. Explain how the services would work with the topics from Exercise 7

### Exercise 9: Error Handling Design
Design how your robot system would handle these failure scenarios:

1. A sensor node stops publishing data
2. A service call times out
3. Network connectivity is temporarily lost
4. A node crashes and restarts
5. Too many messages are being published, causing delays

For each scenario, describe:
- How other nodes would detect the problem
- What fallback behaviors should occur
- How the system would recover

## Hands-On Scenarios

### Exercise 10: Robot Coordination Problem
A mobile robot has these nodes:
- IMU sensor node publishing orientation data
- LIDAR sensor node publishing obstacle distances
- Path planner node creating navigation plans
- Motor controller node executing movement commands
- UI node displaying robot status

The robot needs to navigate around obstacles while maintaining balance. Design the complete communication system:

1. Create all necessary topics and services
2. Show the message flow between all nodes
3. Explain how each node contributes to the overall behavior
4. Identify potential communication bottlenecks and how to address them

### Exercise 11: System Expansion
Starting with your system from Exercise 10, add these new capabilities:
- Object recognition
- Speech output
- Remote control
- Data logging

1. Add the new nodes needed for these capabilities
2. Integrate them into your existing communication system
3. Explain how the new nodes interact with existing ones
4. Identify any new topics or services needed

## Self-Assessment Questions

### Question 1
Which statement best describes the difference between topics and services in ROS 2?
A) Topics are faster than services
B) Topics provide continuous data flow, services provide request-response interaction
C) Services can have multiple clients, topics cannot
D) Topics use TCP, services use UDP

### Question 2
When would you choose to use a service instead of a topic?
A) When broadcasting sensor data continuously
B) When requesting specific information or action on demand
C) When sharing status between multiple nodes
D) When sending large amounts of data

### Question 3
What happens if a subscriber connects to a topic after a publisher has been publishing for a while?
A) The subscriber receives all previously published messages
B) The subscriber only receives new messages published after connection
C) The subscriber receives the last message published before connection
D) The connection fails automatically

### Question 4
Which is a good use case for a ROS 2 service?
A) Publishing camera images at 30 Hz
B) Requesting the current robot position
C) Broadcasting battery level every second
D) Sending continuous velocity commands

### Question 5
What is the main advantage of the publish/subscribe pattern?
A) Guaranteed message delivery
B) Direct node-to-node communication
C) Loose coupling between publishers and subscribers
D) Synchronous communication

## Answers to Self-Assessment Questions

### Question 1: B) Topics provide continuous data flow, services provide request-response interaction
Topics are for continuous data streams, while services are for on-demand request-response communication.

### Question 2: B) When requesting specific information or action on demand
Services are ideal for specific requests that need immediate responses.

### Question 3: B) The subscriber only receives new messages published after connection
With default settings, subscribers only receive messages published after they connect.

### Question 4: B) Requesting the current robot position
Services are perfect for requesting specific information on demand.

### Question 5: C) Loose coupling between publishers and subscribers
Publishers don't need to know about subscribers, and vice versa, enabling flexible system design.

## Challenge Exercise: Complete System Design

Design a complete ROS 2 system for a humanoid robot assistant that can:
- Navigate around a home environment
- Recognize and respond to family members
- Fetch small objects
- Provide information through speech
- Report status to a smartphone app

Create a detailed design document that includes:
1. Complete list of all nodes needed
2. All topics and services with message types
3. Message flow diagrams
4. Error handling strategies
5. QoS settings for each communication channel
6. Expected message rates and data sizes

## Learning Objectives Check

After completing these exercises, you should be able to:
- [ ] Distinguish between topics and services in ROS 2
- [ ] Design appropriate communication patterns for robot systems
- [ ] Identify when to use topics vs services
- [ ] Create message flow diagrams for multi-node systems
- [ ] Apply best practices for ROS 2 communication
- [ ] Troubleshoot common communication issues
- [ ] Design systems with proper error handling
- [ ] Consider QoS requirements for different applications

## Extension Activities

1. **Research**: Investigate ROS 2 Actions and explain how they differ from topics and services
2. **Implementation**: Create simple publisher and subscriber nodes to test your understanding
3. **Analysis**: Study real ROS 2 packages to see how they implement communication patterns
4. **Comparison**: Compare ROS 2 communication patterns to other middleware systems

## Next Steps

Continue to [Chapter 3: Python Control and Robot Description](../chapter-3-python-control-urdf/) to learn how to connect Python-based AI agents to ROS 2 and understand robot description with URDF, or return to the [Module Overview](../../module-1-ros2-arch/index).