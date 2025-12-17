# Cognitive Planning Implementation

## Overview

Cognitive planning implementation combines LLM integration and action sequencing to create a complete system that can interpret natural language commands and execute them as robot actions. This section provides a comprehensive implementation guide for building end-to-end cognitive planning systems.

## Complete Cognitive Planning Architecture

### System Components

The cognitive planning system consists of several interconnected components:

```
[Natural Language Command] → [LLM Interpretation] → [Plan Validation] → [Action Sequencing] → [Execution] → [Feedback]
         ↓                        ↓                    ↓                  ↓                ↓           ↓
    Voice Input           Cognitive Plan        Constraint Check    ROS 2 Actions   Robot State   Learning
```

### Main Implementation Class

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import openai
import json
from typing import Dict, Any, List, Optional
import time

class CognitivePlanningNode(Node):
    def __init__(self):
        super().__init__('cognitive_planning_node')

        # Initialize LLM client
        openai.api_key = self.get_parameter_or('openai_api_key', 'your-api-key')

        # Initialize action clients for different capabilities
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.manip_client = ActionClient(self, ManipulateObject, 'manipulate_object')
        self.detect_client = ActionClient(self, DetectObjects, 'detect_objects')

        # Publishers and subscribers
        self.status_pub = self.create_publisher(String, 'cognitive_planning_status', 10)
        self.command_sub = self.create_subscription(
            String, 'natural_language_command', self.command_callback, 10
        )

        # Service for direct planning requests
        self.plan_service = self.create_service(
            GenerateCognitivePlan,
            'generate_cognitive_plan',
            self.plan_service_callback
        )

        # System parameters
        self.max_retries = self.get_parameter_or('max_retries', 3)
        self.planning_timeout = self.get_parameter_or('planning_timeout', 30.0)

        self.get_logger().info("Cognitive Planning Node initialized")

    def command_callback(self, msg: String):
        """
        Handle incoming natural language commands
        """
        command = msg.data
        self.get_logger().info(f"Received command: {command}")

        # Publish planning status
        status_msg = String()
        status_msg.data = f"Processing command: {command}"
        self.status_pub.publish(status_msg)

        # Execute the complete cognitive planning pipeline
        success = self.execute_cognitive_plan(command)

        if success:
            self.get_logger().info("Command executed successfully")
            status_msg.data = "Command executed successfully"
        else:
            self.get_logger().error("Command execution failed")
            status_msg.data = "Command execution failed"

        self.status_pub.publish(status_msg)

    def execute_cognitive_plan(self, command: str) -> bool:
        """
        Execute the complete cognitive planning pipeline
        """
        try:
            # 1. Get environment context
            context = self.get_environment_context()

            # 2. Generate cognitive plan using LLM
            cognitive_plan = self.generate_plan_with_llm(command, context)

            if not cognitive_plan or not cognitive_plan.get("plan"):
                self.get_logger().error("No valid plan generated")
                return False

            # 3. Validate the plan
            if not self.validate_plan(cognitive_plan, context):
                self.get_logger().error("Plan validation failed")
                return False

            # 4. Convert plan to action sequence
            action_sequence = self.convert_plan_to_actions(cognitive_plan)

            # 5. Execute the action sequence
            execution_success = self.execute_action_sequence(action_sequence)

            return execution_success

        except Exception as e:
            self.get_logger().error(f"Cognitive planning pipeline failed: {e}")
            return False

    def generate_plan_with_llm(self, command: str, context: Dict[str, Any]) -> Dict[str, Any]:
        """
        Generate cognitive plan using LLM
        """
        system_prompt = """
        You are a cognitive planning assistant for a humanoid robot. Your role is to interpret natural language commands and convert them into structured action sequences for the robot.

        Each action should be one of the following types:
        - NAVIGATE: Move to a specific location
        - DETECT_OBJECT: Identify and locate objects
        - GRASP_OBJECT: Pick up an object
        - PLACE_OBJECT: Place an object at a location
        - SAY: Speak a message
        - WAIT: Wait for a specified duration

        Output your response as a JSON object with the following structure:
        {
            "plan": [
                {
                    "action": "action_type",
                    "parameters": {"param1": "value1", ...},
                    "description": "Brief description of the action",
                    "dependencies": [list of action indices that must complete first]
                }
            ],
            "reasoning": "Brief explanation of your planning process",
            "estimated_duration": "Estimated time to complete the plan in seconds"
        }

        Always consider safety constraints and environmental limitations.
        """

        user_message = f"Command: {command}\n\nEnvironment context: {json.dumps(context)}"

        try:
            response = openai.ChatCompletion.create(
                model="gpt-4",
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_message}
                ],
                temperature=0.3,
                response_format={"type": "json_object"}
            )

            plan_data = json.loads(response.choices[0].message.content)
            return plan_data
        except Exception as e:
            self.get_logger().error(f"LLM plan generation failed: {e}")
            return {}

    def validate_plan(self, plan: Dict[str, Any], context: Dict[str, Any]) -> bool:
        """
        Validate the cognitive plan before execution
        """
        # Check if plan has required structure
        if "plan" not in plan:
            return False

        plan_actions = plan["plan"]

        # Validate each action type
        valid_action_types = {"NAVIGATE", "DETECT_OBJECT", "GRASP_OBJECT",
                             "PLACE_OBJECT", "SAY", "WAIT"}

        for action in plan_actions:
            if action.get("action") not in valid_action_types:
                self.get_logger().error(f"Invalid action type: {action.get('action')}")
                return False

        # Check for circular dependencies
        if self.has_circular_dependencies(plan_actions):
            self.get_logger().error("Plan has circular dependencies")
            return False

        # Validate action parameters
        for action in plan_actions:
            if not self.validate_action_parameters(action, context):
                self.get_logger().error(f"Invalid parameters for action: {action}")
                return False

        return True

    def has_circular_dependencies(self, actions: List[Dict[str, Any]]) -> bool:
        """
        Check if the action sequence has circular dependencies
        """
        # Create dependency graph
        graph = {i: set() for i in range(len(actions))}

        for i, action in enumerate(actions):
            deps = action.get("dependencies", [])
            for dep_idx in deps:
                if 0 <= dep_idx < len(actions):
                    graph[i].add(dep_idx)

        # Check for cycles using DFS
        visited = set()
        rec_stack = set()

        def has_cycle_util(node):
            visited.add(node)
            rec_stack.add(node)

            for neighbor in graph[node]:
                if neighbor not in visited:
                    if has_cycle_util(neighbor):
                        return True
                elif neighbor in rec_stack:
                    return True

            rec_stack.remove(node)
            return False

        for node in range(len(actions)):
            if node not in visited:
                if has_cycle_util(node):
                    return True

        return False

    def validate_action_parameters(self, action: Dict[str, Any], context: Dict[str, Any]) -> bool:
        """
        Validate action parameters against environment context
        """
        action_type = action.get("action")
        params = action.get("parameters", {})

        if action_type == "NAVIGATE":
            # Validate navigation parameters
            target_pose = params.get("target_pose")
            if not target_pose:
                return False

            # Check if target is in known map (simplified check)
            # In practice, this would involve more sophisticated validation

        elif action_type == "GRASP_OBJECT":
            # Validate grasp parameters
            object_pose = params.get("object_pose")
            if not object_pose:
                return False

        # Add more validation rules for other action types

        return True

    def convert_plan_to_actions(self, cognitive_plan: Dict[str, Any]) -> List[Dict[str, Any]]:
        """
        Convert cognitive plan to executable action sequence
        """
        plan_actions = cognitive_plan.get("plan", [])
        action_sequence = []

        for plan_item in plan_actions:
            action_type = plan_item.get("action")
            params = plan_item.get("parameters", {})
            description = plan_item.get("description", "")

            # Convert to ROS 2 action format
            ros_action = {
                "action_type": action_type,
                "parameters": params,
                "description": description,
                "dependencies": plan_item.get("dependencies", []),
                "timeout": 30.0  # Default timeout
            }

            # Set specific timeouts based on action type
            if action_type in ["NAVIGATE", "DETECT_OBJECT"]:
                ros_action["timeout"] = 60.0
            elif action_type in ["GRASP_OBJECT", "PLACE_OBJECT"]:
                ros_action["timeout"] = 45.0
            elif action_type == "SAY":
                ros_action["timeout"] = 10.0

            action_sequence.append(ros_action)

        return action_sequence

    def execute_action_sequence(self, action_sequence: List[Dict[str, Any]]) -> bool:
        """
        Execute the sequence of actions
        """
        completed_actions = set()
        max_attempts = self.max_retries

        for i, action in enumerate(action_sequence):
            # Check dependencies
            deps = set(action.get("dependencies", []))
            if not deps.issubset(completed_actions):
                self.get_logger().error(f"Dependencies not met for action {i}")
                return False

            # Execute action with retry logic
            success = False
            for attempt in range(max_attempts):
                try:
                    action_success = self.execute_single_action(action)
                    if action_success:
                        success = True
                        completed_actions.add(i)
                        break
                    else:
                        self.get_logger().warn(f"Action {i} failed on attempt {attempt + 1}")
                except Exception as e:
                    self.get_logger().error(f"Action {i} error on attempt {attempt + 1}: {e}")

                if attempt < max_attempts - 1:
                    time.sleep(1.0)  # Wait before retry

            if not success:
                self.get_logger().error(f"Action {i} failed after {max_attempts} attempts")
                return False

        return True

    def execute_single_action(self, action: Dict[str, Any]) -> bool:
        """
        Execute a single action based on its type
        """
        action_type = action["action_type"]
        params = action["parameters"]
        timeout = action["timeout"]

        if action_type == "NAVIGATE":
            return self.execute_navigation_action(params, timeout)
        elif action_type == "DETECT_OBJECT":
            return self.execute_detection_action(params, timeout)
        elif action_type == "GRASP_OBJECT":
            return self.execute_grasp_action(params, timeout)
        elif action_type == "PLACE_OBJECT":
            return self.execute_place_action(params, timeout)
        elif action_type == "SAY":
            return self.execute_speak_action(params, timeout)
        elif action_type == "WAIT":
            return self.execute_wait_action(params, timeout)
        else:
            self.get_logger().error(f"Unknown action type: {action_type}")
            return False

    def execute_navigation_action(self, params: Dict[str, Any], timeout: float) -> bool:
        """
        Execute navigation action
        """
        target_pose = params.get("target_pose")
        if not target_pose:
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = Pose()  # Convert target_pose to ROS Pose
        # Set pose parameters from target_pose

        self.nav_client.wait_for_server(timeout_sec=1.0)
        future = self.nav_client.send_goal_async(goal_msg)

        # Wait for result
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)

        if future.result() is not None:
            return future.result().result.success
        else:
            return False

    def execute_detection_action(self, params: Dict[str, Any], timeout: float) -> bool:
        """
        Execute object detection action
        """
        object_type = params.get("object_type")
        search_area = params.get("search_area")

        goal_msg = DetectObjects.Goal()
        goal_msg.object_type = object_type or ""
        # Set other parameters

        self.detect_client.wait_for_server(timeout_sec=1.0)
        future = self.detect_client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)

        if future.result() is not None:
            return future.result().result.found_objects
        else:
            return False

    def execute_grasp_action(self, params: Dict[str, Any], timeout: float) -> bool:
        """
        Execute object grasping action
        """
        # Implementation would depend on specific manipulation capabilities
        # This is a simplified example
        return True

    def execute_place_action(self, params: Dict[str, Any], timeout: float) -> bool:
        """
        Execute object placement action
        """
        # Implementation would depend on specific manipulation capabilities
        return True

    def execute_speak_action(self, params: Dict[str, Any], timeout: float) -> bool:
        """
        Execute speech action
        """
        text = params.get("text", "")
        if not text:
            return False

        # Publish to speech system
        # Implementation would depend on specific speech system
        return True

    def execute_wait_action(self, params: Dict[str, Any], timeout: float) -> bool:
        """
        Execute wait action
        """
        duration = params.get("duration", 1.0)
        time.sleep(min(duration, timeout))
        return True

    def get_environment_context(self) -> Dict[str, Any]:
        """
        Get current environment context for planning
        """
        # This would typically gather information from various sensors and systems
        context = {
            "robot_pose": self.get_robot_pose(),
            "map_data": self.get_map_data(),
            "object_locations": self.get_known_objects(),
            "robot_capabilities": self.get_robot_capabilities(),
            "current_time": time.time()
        }
        return context

    def get_robot_pose(self) -> Dict[str, Any]:
        """
        Get current robot pose
        """
        # Implementation would get robot pose from localization system
        return {"x": 0.0, "y": 0.0, "theta": 0.0}

    def get_map_data(self) -> Dict[str, Any]:
        """
        Get current map data
        """
        # Implementation would get map from mapping system
        return {"known_areas": [], "obstacles": []}

    def get_known_objects(self) -> List[Dict[str, Any]]:
        """
        Get known object locations
        """
        # Implementation would get object data from perception system
        return []

    def get_robot_capabilities(self) -> List[str]:
        """
        Get robot capabilities
        """
        return [
            "navigation",
            "object_detection",
            "manipulation",
            "speech_output"
        ]

    def plan_service_callback(self, request, response):
        """
        Service callback for direct plan generation
        """
        try:
            context = self.get_environment_context()
            plan = self.generate_plan_with_llm(request.command, context)

            if plan:
                response.plan = json.dumps(plan)
                response.success = True
                response.message = "Plan generated successfully"
            else:
                response.success = False
                response.message = "Failed to generate plan"
        except Exception as e:
            response.success = False
            response.message = f"Error: {str(e)}"

        return response
```

## Advanced Cognitive Planning Features

### Learning and Adaptation

```python
class AdaptiveCognitivePlanner(CognitivePlanningNode):
    def __init__(self):
        super().__init__()
        self.execution_history = []
        self.performance_metrics = {}

    def execute_cognitive_plan(self, command: str) -> bool:
        """
        Execute cognitive plan with learning from past experiences
        """
        # Check for similar past commands
        similar_plan = self.find_similar_plan(command)

        if similar_plan:
            # Adapt the similar plan instead of generating a new one
            adapted_plan = self.adapt_plan(similar_plan, command)
        else:
            # Generate new plan as usual
            context = self.get_environment_context()
            adapted_plan = self.generate_plan_with_llm(command, context)

        # Execute the plan
        success = self.execute_plan_with_monitoring(adapted_plan, command)

        # Record execution for future learning
        self.record_execution(command, adapted_plan, success)

        return success

    def find_similar_plan(self, command: str) -> Optional[Dict[str, Any]]:
        """
        Find a similar plan from execution history
        """
        # Use semantic similarity to find similar commands
        # Implementation would use embedding comparison or other similarity measures
        pass

    def adapt_plan(self, existing_plan: Dict[str, Any], new_command: str) -> Dict[str, Any]:
        """
        Adapt an existing plan for a new command
        """
        # Use LLM to adapt the plan based on the new command
        adaptation_prompt = f"""
        Adapt the following plan for this new command:

        Existing Plan: {json.dumps(existing_plan)}
        New Command: {new_command}

        Provide an adapted plan that addresses the new command while maintaining the successful elements of the original plan.
        """

        # Implementation would call LLM to adapt the plan
        pass

    def execute_plan_with_monitoring(self, plan: Dict[str, Any], command: str) -> bool:
        """
        Execute plan with real-time monitoring and adjustment
        """
        # Monitor execution and adjust if needed
        # Implementation would include feedback loops
        pass

    def record_execution(self, command: str, plan: Dict[str, Any], success: bool):
        """
        Record execution for future learning
        """
        execution_record = {
            "command": command,
            "plan": plan,
            "success": success,
            "timestamp": time.time(),
            "environment_context": self.get_environment_context()
        }
        self.execution_history.append(execution_record)
```

### Multi-Modal Cognitive Planning

```python
class MultiModalCognitivePlanner(CognitivePlanningNode):
    def __init__(self):
        super().__init__()
        # Additional components for multi-modal processing
        self.vision_processor = VisionProcessor()
        self.speech_processor = SpeechProcessor()

    def generate_plan_with_multimodal_input(self, command: str,
                                          visual_context: str = None,
                                          audio_context: str = None) -> Dict[str, Any]:
        """
        Generate cognitive plan using multiple input modalities
        """
        system_prompt = """
        You are a multi-modal cognitive planning assistant for a humanoid robot.
        Consider both the verbal command and additional sensory information when creating plans.

        Verbal Command: {command}
        Visual Context: {visual_context}
        Audio Context: {audio_context}

        Generate a plan that leverages all available information.
        """

        # Implementation would combine all modalities for plan generation
        pass
```

## Performance Optimization

### Caching and Precomputation

```python
import functools
import hashlib
from typing import Tuple

class OptimizedCognitivePlanner(CognitivePlanningNode):
    def __init__(self):
        super().__init__()
        self.plan_cache = {}
        self.max_cache_size = 100
        self.cache_access_count = 0
        self.cache_hit_count = 0

    @functools.lru_cache(maxsize=50)
    def generate_plan_cached(self, command_hash: str, context_hash: str) -> Dict[str, Any]:
        """
        Generate plan with LRU caching
        """
        # This method will automatically cache results
        # Implementation would retrieve from actual generation method
        pass

    def get_command_context_hash(self, command: str, context: Dict[str, Any]) -> str:
        """
        Generate hash for command and context combination
        """
        combined = f"{command}_{json.dumps(context, sort_keys=True)}"
        return hashlib.md5(combined.encode()).hexdigest()

    def execute_cognitive_plan(self, command: str) -> bool:
        """
        Execute with caching consideration
        """
        context = self.get_environment_context()
        cache_key = self.get_command_context_hash(command, context)

        # Check cache first
        if cache_key in self.plan_cache:
            self.cache_hit_count += 1
            cached_plan = self.plan_cache[cache_key]
            self.get_logger().info("Using cached plan")
        else:
            # Generate new plan
            cached_plan = self.generate_plan_with_llm(command, context)

            # Add to cache
            if len(self.plan_cache) < self.max_cache_size:
                self.plan_cache[cache_key] = cached_plan

        self.cache_access_count += 1

        # Execute the plan (cached or new)
        return self.execute_plan_with_context(cached_plan, context)
```

## Safety and Constraint Handling

### Comprehensive Safety Framework

```python
class SafeCognitivePlanner(CognitivePlanningNode):
    def __init__(self):
        super().__init__()
        self.safety_constraints = self.load_safety_constraints()
        self.emergency_stop = False

    def validate_plan(self, plan: Dict[str, Any], context: Dict[str, Any]) -> bool:
        """
        Enhanced validation with safety constraints
        """
        # First run standard validation
        if not super().validate_plan(plan, context):
            return False

        # Check safety constraints
        for action in plan.get("plan", []):
            if not self.check_action_safety(action, context):
                return False

        # Check overall plan safety
        if not self.check_plan_safety(plan, context):
            return False

        return True

    def check_action_safety(self, action: Dict[str, Any], context: Dict[str, Any]) -> bool:
        """
        Check if individual action is safe
        """
        action_type = action.get("action")
        params = action.get("parameters", {})

        if action_type == "NAVIGATE":
            target = params.get("target_pose")
            if target and not self.is_safe_navigation_target(target, context):
                return False

        elif action_type == "GRASP_OBJECT":
            obj_pose = params.get("object_pose")
            if obj_pose and not self.is_safe_to_grasp(obj_pose, context):
                return False

        return True

    def is_safe_navigation_target(self, target: Dict[str, Any], context: Dict[str, Any]) -> bool:
        """
        Check if navigation target is safe
        """
        # Check if target is in safe area
        # Check for obstacles
        # Check for humans in path
        return True  # Simplified implementation

    def is_safe_to_grasp(self, obj_pose: Dict[str, Any], context: Dict[str, Any]) -> bool:
        """
        Check if it's safe to grasp an object
        """
        # Check if object is in safe zone
        # Check if humans are nearby
        # Check if object is stable
        return True  # Simplified implementation

    def check_plan_safety(self, plan: Dict[str, Any], context: Dict[str, Any]) -> bool:
        """
        Check if the overall plan is safe
        """
        # Check for safety violations across the entire plan
        # Ensure plan doesn't violate safety constraints
        return True

    def execute_single_action(self, action: Dict[str, Any]) -> bool:
        """
        Execute action with safety monitoring
        """
        # Check emergency stop
        if self.emergency_stop:
            return False

        # Pre-execution safety check
        context = self.get_environment_context()
        if not self.check_action_safety(action, context):
            self.get_logger().error("Action failed safety check")
            return False

        # Execute the action
        success = super().execute_single_action(action)

        # Post-execution safety check
        if success:
            new_context = self.get_environment_context()
            if not self.check_post_execution_safety(new_context):
                self.get_logger().warn("Safety violation detected after action execution")
                # Optionally revert or take corrective action

        return success

    def check_post_execution_safety(self, context: Dict[str, Any]) -> bool:
        """
        Check safety after action execution
        """
        # Check for safety violations after action
        return True
```

## Summary

Cognitive planning implementation requires integrating multiple components: LLM-based plan generation, plan validation, action sequencing, execution, and safety monitoring. A well-implemented system should be robust, safe, and capable of learning from experience. The implementation should handle errors gracefully, validate plans before execution, and incorporate safety constraints throughout the process. The next section will explore practical exercises for implementing these concepts.