# Quickstart Guide: Vision-Language-Action (VLA) Integration

## Overview
This guide provides a rapid introduction to implementing the Vision-Language-Action (VLA) pipeline for humanoid robots, combining voice recognition, LLM-based cognitive planning, and ROS 2 action execution.

## Prerequisites
- ROS 2 Humble Hawksbill or later installed
- OpenAI API access for Whisper and LLM services
- Humanoid robot with ROS 2 interface
- Python 3.8+ environment
- Basic knowledge of ROS 2 concepts (from Module 1)

## Setup Steps

### 1. Environment Setup
```bash
# Create virtual environment
python -m venv vla_env
source vla_env/bin/activate  # On Windows: vla_env\Scripts\activate

# Install required packages
pip install openai speech-recognition rospy
```

### 2. Voice Recognition Configuration
```python
import openai
import speech_recognition as sr

# Configure OpenAI API key
openai.api_key = "your-api-key-here"

# Initialize speech recognizer
recognizer = sr.Recognizer()
```

### 3. Basic VLA Pipeline Implementation
```python
# Example voice-to-action pipeline
def voice_to_action_pipeline():
    # 1. Capture voice input
    with sr.Microphone() as source:
        print("Listening for command...")
        audio = recognizer.listen(source)

    # 2. Transcribe using Whisper
    transcription = openai.Audio.transcribe("whisper-1", audio)
    command_text = transcription.text

    # 3. Generate cognitive plan with LLM
    plan = generate_cognitive_plan(command_text)

    # 4. Convert to ROS 2 action sequence
    action_sequence = convert_to_ros_actions(plan)

    # 5. Execute on humanoid robot
    execute_on_robot(action_sequence)
```

## Running the Example
1. Ensure your ROS 2 environment is sourced
2. Set up your OpenAI API key in environment variables
3. Connect to your humanoid robot's ROS 2 network
4. Run the voice_to_action_pipeline() function
5. Speak a simple command like "move forward 1 meter"

## Expected Output
- Voice input should be captured and transcribed
- LLM should generate an appropriate action plan
- ROS 2 actions should be sent to the robot
- Robot should execute the requested movement

## Next Steps
- Explore Chapter 1 for detailed voice-to-action implementation
- Review Chapter 2 for cognitive planning techniques
- Work through the capstone project in Chapter 3