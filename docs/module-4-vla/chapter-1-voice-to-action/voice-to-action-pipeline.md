# Voice-to-Action Pipeline

## Overview

The voice-to-action pipeline is the core system that transforms spoken commands into executable robot actions. This pipeline connects voice recognition capabilities with the robot's action execution system, forming the foundation of voice-controlled robotics.

## Pipeline Architecture

The voice-to-action pipeline consists of several interconnected stages:

1. **Audio Capture**: Collecting voice input from the environment
2. **Voice Recognition**: Converting speech to text using Whisper
3. **Command Parsing**: Interpreting the transcribed text
4. **Action Mapping**: Converting commands to ROS 2 action sequences
5. **Execution**: Executing the mapped actions on the robot

## Complete Pipeline Implementation

### Basic Voice-to-Action Pipeline

```python
import openai
import speech_recognition as sr
import rospy
from std_msgs.msg import String
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class VoiceToActionPipeline:
    def __init__(self):
        # Initialize speech recognition
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # Initialize ROS components
        rospy.init_node('voice_to_action_node')
        self.command_publisher = rospy.Publisher('voice_commands', String, queue_size=10)

        # Setup for ROS 2 action clients (using rclpy in ROS 2)
        # Note: This is a conceptual example; actual ROS 2 implementation would differ
        self.move_client = SimpleActionClient('move_base', MoveBaseAction)

        # Configuration
        self.openai_api_key = rospy.get_param('~openai_api_key', '')
        openai.api_key = self.openai_api_key

    def capture_voice_input(self):
        """
        Capture voice input from microphone
        """
        with self.microphone as source:
            print("Listening for command...")
            # Adjust for ambient noise
            self.recognizer.adjust_for_ambient_noise(source)
            # Listen for audio
            audio = self.recognizer.listen(source)
        return audio

    def transcribe_with_whisper(self, audio):
        """
        Transcribe audio using OpenAI Whisper
        """
        # Convert audio to WAV format for Whisper
        wav_data = audio.get_wav_data()
        audio_file = io.BytesIO(wav_data)
        audio_file.name = "temp_audio.wav"

        try:
            transcript = openai.Audio.transcribe(
                model="whisper-1",
                file=audio_file,
                response_format="verbose_json"
            )
            return transcript.text, transcript.confidence
        except Exception as e:
            rospy.logerr(f"Whisper transcription failed: {e}")
            return None, 0.0

    def parse_command(self, text):
        """
        Parse natural language command into structured action
        """
        # Simple command parsing - in practice, this would use more sophisticated NLP
        text = text.lower().strip()

        if "move forward" in text or "go forward" in text:
            distance = self.extract_distance(text)
            return "move", {"direction": "forward", "distance": distance}
        elif "move backward" in text or "go backward" in text:
            distance = self.extract_distance(text)
            return "move", {"direction": "backward", "distance": distance}
        elif "turn left" in text:
            return "turn", {"direction": "left"}
        elif "turn right" in text:
            return "turn", {"direction": "right"}
        elif "stop" in text or "halt" in text:
            return "stop", {}
        else:
            return "unknown", {"text": text}

    def extract_distance(self, text):
        """
        Extract distance from command text
        """
        import re
        # Look for patterns like "2 meters", "1.5 meters", etc.
        match = re.search(r'(\d+\.?\d*)\s*(meter|m|cm|centimeter)', text)
        if match:
            distance = float(match.group(1))
            unit = match.group(2)
            # Convert to meters if needed
            if unit in ['cm', 'centimeter']:
                distance = distance / 100.0
            return distance
        return 1.0  # Default distance

    def map_to_ros_action(self, command_type, params):
        """
        Map parsed command to ROS 2 action
        """
        if command_type == "move":
            return self.create_move_action(params)
        elif command_type == "turn":
            return self.create_turn_action(params)
        elif command_type == "stop":
            return self.create_stop_action(params)
        else:
            return None

    def create_move_action(self, params):
        """
        Create move action for ROS 2
        """
        # This is a conceptual example - actual implementation would depend on robot capabilities
        direction = params.get("direction", "forward")
        distance = params.get("distance", 1.0)

        # Create goal for move_base action
        goal = MoveBaseGoal()
        # Set goal position based on current position and direction/distance
        # (Implementation would require current robot pose)

        return goal

    def execute_action(self, action_goal):
        """
        Execute the ROS 2 action
        """
        if action_goal:
            self.move_client.send_goal(action_goal)
            # Wait for result with timeout
            finished_within_time = self.move_client.wait_for_result(rospy.Duration(30.0))
            if not finished_within_time:
                rospy.logwarn("Action did not finish before the time limit")
                return False
            return True
        return False

    def run_pipeline(self):
        """
        Run the complete voice-to-action pipeline
        """
        try:
            # 1. Capture voice input
            audio = self.capture_voice_input()

            # 2. Transcribe with Whisper
            text, confidence = self.transcribe_with_whisper(audio)

            if text is None:
                rospy.logerr("Transcription failed")
                return False

            if confidence < 0.7:  # Confidence threshold
                rospy.logwarn(f"Low confidence transcription: {confidence}")
                # Optionally ask for repetition
                return False

            # 3. Parse command
            command_type, params = self.parse_command(text)

            if command_type == "unknown":
                rospy.logwarn(f"Unknown command: {text}")
                return False

            # 4. Map to ROS action
            action_goal = self.map_to_ros_action(command_type, params)

            # 5. Execute action
            success = self.execute_action(action_goal)

            if success:
                rospy.loginfo(f"Successfully executed command: {text}")
                self.command_publisher.publish(text)
            else:
                rospy.logerr(f"Failed to execute command: {text}")

            return success

        except Exception as e:
            rospy.logerr(f"Pipeline execution failed: {e}")
            return False

# Usage example
if __name__ == "__main__":
    pipeline = VoiceToActionPipeline()

    # Run continuously
    rate = rospy.Rate(1)  # Check for commands at 1 Hz
    while not rospy.is_shutdown():
        pipeline.run_pipeline()
        rate.sleep()
```

## Error Handling Patterns

### Graceful Degradation
```python
def robust_pipeline_execution(self):
    """
    Execute pipeline with comprehensive error handling
    """
    try:
        audio = self.capture_voice_input()
        text, confidence = self.transcribe_with_whisper(audio)

        if text is None:
            self.handle_transcription_failure()
            return False

        if confidence < 0.7:
            return self.request_command_repetition(text, confidence)

        command_type, params = self.parse_command(text)

        if command_type == "unknown":
            return self.handle_unknown_command(text)

        action_goal = self.map_to_ros_action(command_type, params)
        return self.execute_action(action_goal)

    except sr.WaitTimeoutError:
        rospy.logwarn("No audio detected within timeout")
        return False
    except sr.UnknownValueError:
        rospy.logwarn("Could not understand audio")
        return False
    except Exception as e:
        rospy.logerr(f"Pipeline error: {e}")
        return False

def handle_transcription_failure(self):
    """
    Handle cases where transcription fails
    """
    # Provide feedback to user
    rospy.loginfo("Could not transcribe your command. Please try again.")
    # Could also trigger audio feedback to user

def request_command_repetition(self, text, confidence):
    """
    Request user to repeat command when confidence is low
    """
    rospy.loginfo(f"Low confidence ({confidence:.2f}) for command: '{text}'")
    rospy.loginfo("Please repeat your command more clearly.")
    return False

def handle_unknown_command(self, text):
    """
    Handle unknown commands
    """
    rospy.loginfo(f"Unknown command: '{text}'")
    rospy.loginfo("Available commands: move forward, move backward, turn left, turn right, stop")
    return False
```

## Performance Considerations

### Optimization Strategies
1. **Audio Preprocessing**: Optimize audio quality before sending to Whisper
2. **Command Caching**: Cache results for frequently used commands
3. **Confidence Thresholding**: Set appropriate confidence thresholds for your application
4. **Asynchronous Processing**: Process audio in background threads when possible

### Resource Management
- **API Rate Limits**: Implement proper rate limiting for Whisper API calls
- **Audio Buffering**: Manage audio buffers efficiently to avoid memory issues
- **Connection Management**: Maintain persistent connections to Whisper API

## Text-described Diagram: Complete Voice-to-Action Pipeline

```
[User Speaks] → [Audio Capture] → [Whisper Transcription] → [Command Parsing] → [Action Mapping] → [ROS 2 Execution]
       ↓              ↓                    ↓                      ↓                 ↓                  ↓
   Raw Audio    Audio Buffer        Transcribed Text      Parsed Command    ROS Action Goal    Robot Action
       ↓              ↓                    ↓                      ↓                 ↓                  ↓
   Microphone    Speech Recognizer    OpenAI API           NLP Processing    Action Client     Physical Robot
```

## Testing the Pipeline

### Unit Testing Components
```python
import unittest
from unittest.mock import Mock, patch

class TestVoiceToActionPipeline(unittest.TestCase):
    def setUp(self):
        self.pipeline = VoiceToActionPipeline()

    @patch('openai.Audio.transcribe')
    def test_transcription(self, mock_transcribe):
        # Mock Whisper response
        mock_transcribe.return_value = Mock(text="move forward 2 meters", confidence=0.9)

        # Test transcription with mock audio
        text, confidence = self.pipeline.transcribe_with_whisper(Mock())

        self.assertEqual(text, "move forward 2 meters")
        self.assertEqual(confidence, 0.9)

    def test_command_parsing(self):
        command_type, params = self.pipeline.parse_command("move forward 2 meters")

        self.assertEqual(command_type, "move")
        self.assertEqual(params["direction"], "forward")
        self.assertEqual(params["distance"], 2.0)

    def test_distance_extraction(self):
        distance = self.pipeline.extract_distance("move forward 1.5 meters")
        self.assertEqual(distance, 1.5)

        distance_cm = self.pipeline.extract_distance("move forward 100 centimeters")
        self.assertEqual(distance_cm, 1.0)  # 100 cm = 1.0 m
```

## Summary

The voice-to-action pipeline represents the complete journey from spoken command to robot action. Success in this pipeline requires careful attention to each stage: reliable audio capture, accurate transcription, robust command parsing, appropriate action mapping, and reliable execution. The next section will explore practical exercises for implementing this pipeline.