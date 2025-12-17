---
sidebar_position: 4
---

# Chapter 1 Exercises

Complete these exercises to reinforce your understanding of ROS 2 architecture fundamentals.

## Conceptual Exercises

### Exercise 1: Real-World Analogies
Think of a complex system in your daily life (like a hospital, airport, or school) and identify:
1. What are the "nodes" in this system?
2. How do they communicate?
3. What serves as the "middleware" that coordinates everything?

### Exercise 2: Robot Component Mapping
Imagine a delivery robot that navigates a building to deliver packages:
1. List 5 different nodes this robot would need
2. For each node, describe what it would do
3. How would these nodes communicate using ROS 2 patterns?

### Exercise 3: Architecture Understanding
Explain in your own words:
1. What is a ROS 2 node and how is it like a brain cell?
2. Why do robots need middleware like ROS 2?
3. How is ROS 2 architecture similar to a human nervous system?

## Application Exercises

### Exercise 4: Communication Pattern Identification
For each scenario, identify which ROS 2 communication pattern would be most appropriate:
1. A camera sending live video feed to multiple processing nodes
2. Requesting the current position of a robot
3. Commanding a robot to move to a specific location with feedback during the journey
4. Broadcasting battery level to all nodes

### Exercise 5: System Design
Design a simple ROS 2 system for a security robot that patrols a building:
1. Draw (or describe) the main nodes needed
2. Identify the communication patterns between nodes
3. Explain how information would flow in your system

## Assessment Questions

### Question 1
Which of the following best describes ROS 2's role in a robot system?
A) The main processing unit that makes all decisions
B) The communication backbone that connects different components
C) A type of sensor that helps robots perceive their environment
D) A programming language for robotics

### Question 2
In the nervous system analogy, what would ROS 2 nodes be most similar to?
A) The blood that flows through the body
B) Individual organs or body parts that perform specific functions
C) The skin that protects the body
D) The food that provides energy

### Question 3
Which statement about ROS 2 architecture is TRUE?
A) All nodes must run on the same computer
B) Nodes must be written in the same programming language
C) Nodes communicate through standardized messages
D) There must be one central controller for all nodes

### Question 4
What happens if one node in a ROS 2 system fails?
A) The entire robot system stops working
B) Only that specific function is affected (other nodes continue working)
C) All sensor nodes stop working
D) The robot reboots automatically

## Answers

### Conceptual Exercises
- Exercise 1: Answers will vary based on the system chosen
- Exercise 2: Example nodes might include: camera node, navigation node, motor controller node, battery monitor node, obstacle detection node
- Exercise 3: Key concepts: nodes as functional units, middleware as communication coordinator, nervous system as distributed control

### Application Exercises
- Exercise 4: 1-Topic, 2-Service, 3-Action, 4-Topic
- Exercise 5: Should include perception, decision-making, and actuation nodes with appropriate communication

### Assessment Questions
- Question 1: B) The communication backbone that connects different components
- Question 2: B) Individual organs or body parts that perform specific functions
- Question 3: C) Nodes communicate through standardized messages
- Question 4: B) Only that specific function is affected (other nodes continue working)

## Self-Evaluation

After completing these exercises, you should be able to:
- [ ] Explain ROS 2 architecture using real-world analogies
- [ ] Identify the key components of ROS 2 architecture
- [ ] Describe how ROS 2 connects different parts of a humanoid robot
- [ ] Understand the role of middleware in robot systems

## Next Steps

Continue to [Chapter 2: Communication in ROS 2](../chapter-2-communication-patterns/) to learn about how nodes communicate using different patterns, or return to the [Module Overview](../../module-1-ros2-arch/index).