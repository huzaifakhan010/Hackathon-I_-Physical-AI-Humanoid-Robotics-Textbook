# LLM Integration for Cognitive Planning

## Introduction

Large Language Models (LLMs) serve as the cognitive engine in the Vision-Language-Action (VLA) pipeline, transforming natural language commands into structured plans and action sequences. This section covers the integration of LLMs for cognitive planning in robotic systems.

## Role of LLMs in Cognitive Planning

### Understanding Natural Language Commands

LLMs excel at interpreting the nuances of natural language, making them ideal for cognitive planning in robotics:

- **Context Understanding**: LLMs can infer context from commands that might be ambiguous to rule-based systems
- **Multi-step Reasoning**: They can decompose complex commands into sequences of simpler actions
- **Constraint Handling**: LLMs can consider environmental and safety constraints when generating plans
- **Adaptability**: They can handle variations in command phrasing and adapt to new scenarios

### Cognitive Planning Process

The cognitive planning process using LLMs involves several key steps:

1. **Command Interpretation**: Understanding the user's intent from natural language
2. **Task Decomposition**: Breaking complex commands into manageable sub-tasks
3. **Action Mapping**: Converting sub-tasks into executable robot actions
4. **Sequence Optimization**: Ordering actions efficiently while respecting dependencies
5. **Constraint Validation**: Ensuring the plan respects environmental and safety constraints

## Integration Patterns

### Direct API Integration

The most common approach for integrating LLMs with robotic systems is through API calls:

```python
import openai
import json
from typing import List, Dict, Any

class LLMCognitivePlanner:
    def __init__(self, api_key: str, model: str = "gpt-4"):
        openai.api_key = api_key
        self.model = model
        self.system_prompt = self._create_system_prompt()

    def _create_system_prompt(self) -> str:
        """
        Create a system prompt that guides the LLM's behavior
        """
        return """
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
                    "description": "Brief description of the action"
                }
            ],
            "reasoning": "Brief explanation of your planning process"
        }

        Always consider safety constraints and environmental limitations.
        """

    def generate_plan(self, command: str, environment_context: Dict[str, Any] = None) -> Dict[str, Any]:
        """
        Generate a cognitive plan for the given command
        """
        user_message = f"Command: {command}"

        if environment_context:
            user_message += f"\nEnvironment context: {json.dumps(environment_context)}"

        try:
            response = openai.ChatCompletion.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": user_message}
                ],
                temperature=0.3,  # Lower temperature for more consistent planning
                response_format={"type": "json_object"}
            )

            plan_data = json.loads(response.choices[0].message.content)
            return plan_data
        except Exception as e:
            print(f"Error generating plan: {e}")
            return {"plan": [], "reasoning": f"Error: {str(e)}"}
```

### Prompt Engineering for Robotics

Effective prompt engineering is crucial for reliable cognitive planning:

```python
class RoboticPromptEngineer:
    @staticmethod
    def create_task_decomposition_prompt(command: str, robot_capabilities: List[str],
                                       environment: Dict[str, Any]) -> str:
        """
        Create a prompt that guides the LLM in decomposing tasks
        """
        capabilities_str = ", ".join(robot_capabilities)
        env_str = json.dumps(environment, indent=2)

        return f"""
        You are a task decomposition expert for a humanoid robot. Decompose the following command into a sequence of executable actions.

        Command: {command}

        Robot Capabilities: {capabilities_str}

        Environment: {env_str}

        Decompose this command into a sequence of actions that the robot can execute. Consider:
        1. The spatial relationships mentioned in the command
        2. The objects that need to be manipulated
        3. The sequence of operations required
        4. Any safety constraints that apply

        Provide your response as a JSON object with the action sequence.
        """

    @staticmethod
    def create_constraint_handling_prompt(base_plan: List[Dict], constraints: List[str]) -> str:
        """
        Create a prompt to modify a plan based on constraints
        """
        constraints_str = "\n".join([f"- {c}" for c in constraints])

        return f"""
        You have generated an initial action plan. Now, incorporate the following constraints:

        Constraints:
        {constraints_str}

        Base Plan:
        {json.dumps(base_plan, indent=2)}

        Modify the plan to respect all constraints. If a constraint prevents an action, suggest an alternative or skip the action with justification.
        """
```

## Advanced Integration Techniques

### Context-Aware Planning

LLMs can leverage contextual information to make better planning decisions:

```python
def context_aware_planning(self, command: str, context: Dict[str, Any]) -> Dict[str, Any]:
    """
    Generate a plan considering contextual information
    """
    # Include robot state, environment map, object locations, etc.
    context_str = json.dumps(context, indent=2)

    prompt = f"""
    Command: {command}

    Current Context:
    {context_str}

    Generate a plan considering the current state of the robot and environment.
    """

    # Use the prompt with the LLM
    # ... implementation details
```

### Multi-Modal Integration

For VLA systems, LLMs can be combined with vision systems for enhanced planning:

```python
def multimodal_planning(self, command: str, visual_context: str) -> Dict[str, Any]:
    """
    Generate a plan using both text command and visual context
    """
    prompt = f"""
    Command: {command}

    Visual Context: {visual_context}

    Generate a plan that incorporates both the verbal command and visual information.
    """

    # Implementation would involve multimodal models like GPT-4V
    # or separate processing of visual and text inputs
```

## Performance and Reliability Considerations

### Caching and Optimization

```python
import functools
import hashlib

class OptimizedLLMPlanner:
    def __init__(self, api_key: str):
        openai.api_key = api_key
        self.cache = {}
        self.max_cache_size = 1000

    @functools.lru_cache(maxsize=100)
    def generate_plan_cached(self, command: str, context_hash: str) -> Dict[str, Any]:
        """
        Generate plan with caching to reduce API calls for similar commands
        """
        # Generate plan using LLM
        # This method will automatically cache results based on input parameters
        pass

    def _get_context_hash(self, context: Dict[str, Any]) -> str:
        """
        Create a hash of the context for caching purposes
        """
        context_str = json.dumps(context, sort_keys=True)
        return hashlib.md5(context_str.encode()).hexdigest()
```

### Error Handling and Fallbacks

```python
class RobustLLMPlanner:
    def __init__(self, api_key: str):
        openai.api_key = api_key
        self.fallback_planner = RuleBasedPlanner()  # Simple rule-based fallback

    def generate_plan_with_fallback(self, command: str) -> Dict[str, Any]:
        """
        Generate plan with fallback to rule-based system if LLM fails
        """
        try:
            # Try LLM-based planning
            result = self.generate_plan(command)
            if result and len(result.get("plan", [])) > 0:
                return result
        except Exception as e:
            print(f"LLM planning failed: {e}, using fallback")

        # Use fallback planner
        return self.fallback_planner.generate_plan(command)
```

## Integration with ROS 2

### ROS 2 Service for LLM Planning

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from your_msgs.srv import CognitivePlan

class LLMPlanningService(Node):
    def __init__(self):
        super().__init__('llm_planning_service')
        self.planner = LLMCognitivePlanner(api_key="your-api-key")
        self.srv = self.create_service(CognitivePlan, 'generate_cognitive_plan', self.plan_callback)

    def plan_callback(self, request, response):
        """
        Service callback to generate cognitive plans
        """
        try:
            environment_context = self.get_environment_context()
            plan_data = self.planner.generate_plan(request.command, environment_context)

            # Convert plan_data to response format
            response.plan = json.dumps(plan_data)
            response.success = True
            response.message = "Plan generated successfully"
        except Exception as e:
            response.success = False
            response.message = f"Error generating plan: {str(e)}"

        return response

    def get_environment_context(self) -> Dict[str, Any]:
        """
        Get current environment context for planning
        """
        # Implementation would gather current robot state, environment map, etc.
        pass
```

## Summary

LLM integration provides powerful cognitive planning capabilities for robotic systems, enabling natural language interaction and complex task decomposition. Success in LLM integration requires careful prompt engineering, proper error handling, and thoughtful consideration of the robot's capabilities and constraints. The next section will explore how to convert cognitive plans into executable ROS 2 action sequences.