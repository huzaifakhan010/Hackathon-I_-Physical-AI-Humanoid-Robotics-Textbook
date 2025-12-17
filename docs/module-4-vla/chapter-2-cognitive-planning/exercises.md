# Chapter 2 Exercises: Cognitive Planning and ROS 2 Action Sequencing

## Practical Exercises for Cognitive Planning Setup

### Exercise 1: LLM Integration
Implement a basic cognitive planning system that takes a natural language command and generates a sequence of ROS 2 actions.

**Requirements:**
- Use OpenAI API for cognitive planning
- Create a simple prompt engineering approach
- Generate at least 3 different types of action sequences
- Include error handling for API failures

### Exercise 2: Action Sequence Generation
Design and implement a system that converts complex commands like "navigate to the kitchen and bring me a cup" into a sequence of ROS 2 actions.

**Requirements:**
- Break down the command into sub-tasks
- Generate navigation actions
- Generate object recognition actions
- Generate manipulation actions
- Consider the order dependencies between actions

### Exercise 3: Constraint Handling
Implement a cognitive planning system that considers environmental and safety constraints.

**Requirements:**
- Define at least 3 different types of constraints
- Modify action sequences based on constraints
- Implement fallback plans when constraints prevent primary actions
- Validate that generated sequences respect all constraints

### Exercise 4: Multi-step Planning
Create a planning system that handles multi-step commands with dependencies.

**Requirements:**
- Handle commands with temporal dependencies
- Generate action sequences with proper ordering
- Implement feedback loops for plan adjustment
- Test with at least 5 different multi-step scenarios

## Evaluation Rubric for Cognitive Planning Implementation

| Criteria | Excellent (4) | Good (3) | Satisfactory (2) | Needs Improvement (1) |
|----------|---------------|----------|------------------|----------------------|
| LLM Integration | Seamless integration with sophisticated prompt engineering | Good integration with basic prompt engineering | Basic integration with simple prompts | Poor integration or frequent failures |
| Planning Quality | Generates optimal action sequences with proper dependencies | Generates good sequences with minor inefficiencies | Basic sequences that accomplish tasks | Sequences with logical errors or conflicts |
| Constraint Handling | Robust constraint checking and handling with fallbacks | Good constraint handling with some limitations | Basic constraint handling | Minimal or incorrect constraint handling |
| Multi-step Planning | Handles complex multi-step commands with dependencies | Handles most multi-step commands well | Basic multi-step planning | Poor multi-step planning capabilities |
| Error Recovery | Comprehensive error handling and recovery | Good error handling | Basic error handling | Minimal error handling |