---
sidebar_position: 2
title: "Cognitive Planning: Translating Natural Language to Robot Actions"
---

import ChapterTranslator from '@site/src/components/Translation/ChapterTranslator';

<ChapterTranslator>

# Lesson 4.2: Cognitive Planning: Translating Natural Language to Robot Actions

## Overview
This lesson explores cognitive planning using Large Language Models (LLMs) to translate natural language instructions into executable robot action sequences. Learn to implement LLM-driven planning for humanoid robots, bridging high-level commands with low-level robot controls.

## Learning Objectives
By the end of this lesson, you should be able to:
- Integrate LLMs with ROS 2 for cognitive planning
- Design prompt engineering strategies for robot action translation
- Implement natural language to action sequence conversion
- Handle ambiguity and context in natural language commands
- Create fallback and error recovery mechanisms for LLM planning
- Optimize LLM inference for real-time robotics applications

## Understanding Cognitive Planning with LLMs

### The Cognitive Planning Pipeline
```
                    Cognitive Planning Pipeline

   Natural           LLM-Based        Action
   Language          Cognitive        Sequence
   Instruction  -->  Planning     -->  Execution
   ("Clean room")    (GPT-4/LLaMA)    (ROS 2 cmds)

                              |
         "Go to kitchen,      |  1. Parse high-level intent
          find trash bin,     |  2. Identify required actions
          pick it up,         |  3. Sequence actions temporally
          dispose contents"   |  4. Validate action feasibility
                             |  5. Generate ROS 2 command sequence

                    Robot Action Execution
```

### LLM-Driven Planning Architecture
```python
# cognitive_planning_architecture.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from action_msgs.msg import GoalStatus
import openai
import json
import asyncio
from typing import Dict, List, Optional

class LLMBasedCognitivePlanner(Node):
    def __init__(self):
        super().__init__('llm_cognitive_planner')

        # LLM configuration
        self.llm_model = "gpt-4-turbo-preview"  # Or use local model like Llama 3
        self.llm_temperature = 0.3
        self.llm_max_tokens = 500
        self.llm_timeout = 30.0  # seconds

        # Robot context and capabilities
        self.robot_capabilities = {
            'navigation': True,
            'manipulation': True,
            'vision': True,
            'speech': True,
            'available_actions': [
                'move_to', 'pick_up', 'place_down', 'grasp', 'release',
                'look_at', 'find_object', 'navigate_to', 'inspect'
            ]
        }

        # Subscriptions
        self.natural_language_sub = self.create_subscription(
            String,
            '/natural_language/command',
            self.language_command_callback,
            10
        )

        self.robot_state_sub = self.create_subscription(
            String,  # In practice, this would be a more specific message type
            '/robot/state',
            self.robot_state_callback,
            10
        )

        # Publishers
        self.action_sequence_pub = self.create_publisher(
            String,
            '/cognitive_planner/action_sequence',
            10
        )

        self.planning_feedback_pub = self.create_publisher(
            String,
            '/cognitive_planner/feedback',
            10
        )

        # Internal state
        self.current_robot_state = {}
        self.planning_history = []

        self.get_logger().info('LLM Cognitive Planner initialized')

    def language_command_callback(self, msg):
        """Process natural language command using LLM"""
        command_text = msg.data
        self.get_logger().info(f'Received natural language command: {command_text}')

        # Get robot context
        robot_context = self.get_current_robot_context()

        # Plan actions using LLM
        action_sequence = self.plan_actions_with_llm(command_text, robot_context)

        if action_sequence:
            # Publish action sequence
            action_msg = String()
            action_msg.data = json.dumps(action_sequence)
            self.action_sequence_pub.publish(action_msg)

            # Log the planning result
            self.planning_history.append({
                'command': command_text,
                'context': robot_context,
                'plan': action_sequence,
                'timestamp': self.get_clock().now().to_msg()
            })

            self.get_logger().info(f'Published action sequence with {len(action_sequence)} actions')
        else:
            self.get_logger().error('Failed to generate action sequence from LLM')

    def get_current_robot_context(self):
        """Get current robot state and environment context"""
        return {
            'capabilities': self.robot_capabilities,
            'current_position': self.current_robot_state.get('position', [0, 0, 0]),
            'current_orientation': self.current_robot_state.get('orientation', [0, 0, 0, 1]),
            'detected_objects': self.current_robot_state.get('detected_objects', []),
            'available_tools': self.current_robot_state.get('available_tools', []),
            'battery_level': self.current_robot_state.get('battery_level', 100.0),
            'current_task': self.current_robot_state.get('current_task', ''),
            'recent_actions': self.current_robot_state.get('recent_actions', [])[-5:]  # Last 5 actions
        }

    def plan_actions_with_llm(self, command: str, context: Dict) -> Optional[List[Dict]]:
        """Use LLM to plan actions from natural language command"""
        try:
            # Create a detailed prompt for the LLM
            prompt = self.create_planning_prompt(command, context)

            # Call the LLM
            response = self.call_llm(prompt)

            # Parse the response
            action_sequence = self.parse_llm_response(response)

            # Validate the action sequence
            validated_sequence = self.validate_action_sequence(action_sequence, context)

            return validated_sequence

        except Exception as e:
            self.get_logger().error(f'Error in LLM planning: {str(e)}')
            return None

    def create_planning_prompt(self, command: str, context: Dict) -> str:
        """Create a detailed prompt for LLM-based action planning"""
        prompt = f"""You are an intelligent cognitive planner for a humanoid robot. Your task is to translate high-level natural language commands into a sequence of specific, executable robot actions.

Current Robot Context:
- Capabilities: {{json.dumps(context['capabilities'])}}
- Current Position: {{context['current_position']}}
- Detected Objects: {{json.dumps(context['detected_objects'])}}
- Available Tools: {{json.dumps(context['available_tools'])}}
- Battery Level: {{context['battery_level']}}%

Command: "{command}"

Please respond with a JSON array of robot actions to execute. Each action should be a dictionary with:
- "action_type": The type of action (e.g., "move_to", "pick_up", "navigate_to", "inspect")
- "parameters": A dictionary of parameters needed for the action
- "priority": Integer priority (lower numbers execute first)
- "estimated_duration": Estimated time in seconds

Example response format:
[
    {{
        "action_type": "find_object",
        "parameters": {{
            "object_type": "trash_bin",
            "search_area": "kitchen"
        }},
        "priority": 1,
        "estimated_duration": 10.0
    }},
    {{
        "action_type": "navigate_to",
        "parameters": {{
            "target_position": [1.5, 2.0, 0.0],
            "approach_direction": "front"
        }},
        "priority": 2,
        "estimated_duration": 15.0
    }},
    {{
        "action_type": "inspect",
        "parameters": {{
            "target_object": "trash_bin",
            "inspection_type": "visual"
        }},
        "priority": 3,
        "estimated_duration": 5.0
    }}
]

Ensure all actions are physically possible for the robot and logically sequenced to achieve the goal."""

        return prompt

    def call_llm(self, prompt: str) -> str:
        """Call the LLM with the given prompt"""
        # In a real implementation, this would call the actual LLM API
        # For this example, we'll simulate the response
        import time
        start_time = time.time()

        # Simulate API call (in practice, use OpenAI, Anthropic, or local model)
        # response = openai.ChatCompletion.create(
        #     model=self.llm_model,
        #     messages=[{"role": "user", "content": prompt}],
        #     temperature=self.llm_temperature,
        #     max_tokens=self.llm_max_tokens
        # )
        # return response.choices[0].message.content

        # For this example, return a mock response based on the command
        if "clean room" in prompt.lower():
            return json.dumps([
                {
                    "action_type": "find_object",
                    "parameters": {
                        "object_type": "trash_bin",
                        "search_area": "kitchen"
                    },
                    "priority": 1,
                    "estimated_duration": 8.0
                },
                {
                    "action_type": "navigate_to",
                    "parameters": {
                        "target_position": [2.0, 1.5, 0.0],
                        "approach_direction": "front"
                    },
                    "priority": 2,
                    "estimated_duration": 12.0
                },
                {
                    "action_type": "find_object",
                    "parameters": {
                        "object_type": "trash",
                        "search_area": "current_location"
                    },
                    "priority": 3,
                    "estimated_duration": 10.0
                },
                {
                    "action_type": "pick_up",
                    "parameters": {
                        "target_object": "trash",
                        "grasp_type": "top_grasp"
                    },
                    "priority": 4,
                    "estimated_duration": 5.0
                },
                {
                    "action_type": "navigate_to",
                    "parameters": {
                        "target_position": [0.0, 0.0, 0.0],
                        "approach_direction": "front"
                    },
                    "priority": 5,
                    "estimated_duration": 15.0
                },
                {
                    "action_type": "place_down",
                    "parameters": {
                        "target_position": [0.5, 0.5, 0.1],
                        "placement_type": "precise"
                    },
                    "priority": 6,
                    "estimated_duration": 4.0
                }
            ])
        elif "go to kitchen" in prompt.lower():
            return json.dumps([
                {
                    "action_type": "navigate_to",
                    "parameters": {
                        "target_position": [3.0, 2.0, 0.0],
                        "approach_direction": "front"
                    },
                    "priority": 1,
                    "estimated_duration": 20.0
                }
            ])
        else:
            # Default response for unrecognized commands
            return json.dumps([
                {
                    "action_type": "find_object",
                    "parameters": {
                        "object_type": "unknown_object",
                        "search_area": "current_location"
                    },
                    "priority": 1,
                    "estimated_duration": 5.0
                }
            ])

    def parse_llm_response(self, response_text: str) -> List[Dict]:
        """Parse LLM response into action sequence"""
        try:
            # Try to parse as JSON
            action_sequence = json.loads(response_text)
            return action_sequence
        except json.JSONDecodeError:
            self.get_logger().error(f'LLM response is not valid JSON: {response_text[:100]}...')

            # Try to extract JSON from the response if it contains other text
            import re
            json_match = re.search(r'\[(.*?)\]', response_text, re.DOTALL)
            if json_match:
                try:
                    action_sequence = json.loads(json_match.group(0))
                    return action_sequence
                except json.JSONDecodeError:
                    pass

            return []

    def validate_action_sequence(self, action_sequence: List[Dict], context: Dict) -> List[Dict]:
        """Validate and refine the action sequence"""
        validated_sequence = []

        for action in action_sequence:
            # Validate action type
            if action.get('action_type') not in self.robot_capabilities['available_actions']:
                self.get_logger().warn(f'Invalid action type: {action.get("action_type")}')
                continue

            # Validate parameters
            if not self.validate_action_parameters(action, context):
                self.get_logger().warn(f'Invalid parameters for action: {action}')
                continue

            # Add validated action to sequence
            validated_sequence.append(action)

        return validated_sequence

    def validate_action_parameters(self, action: Dict, context: Dict) -> bool:
        """Validate action-specific parameters"""
        action_type = action.get('action_type', '')
        parameters = action.get('parameters', {})

        if action_type == 'navigate_to':
            if 'target_position' not in parameters:
                return False
            if not isinstance(parameters['target_position'], list) or len(parameters['target_position']) < 2:
                return False

        elif action_type == 'pick_up':
            if 'target_object' not in parameters and 'object_type' not in parameters:
                return False

        elif action_type == 'move_to':
            if 'target_position' not in parameters:
                return False

        elif action_type == 'find_object':
            if 'object_type' not in parameters:
                return False

        # Additional validation could go here
        return True

    def robot_state_callback(self, msg):
        """Update robot state context"""
        try:
            state_data = json.loads(msg.data)
            self.current_robot_state.update(state_data)
        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON in robot state message')

def main(args=None):
    rclpy.init(args=args)
    node = LLMBasedCognitivePlanner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down LLM Cognitive Planner')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Prompt Engineering for Robotics

### Effective Prompt Design Patterns
```python
# prompt_engineering_robots.py
import json
from typing import Dict, List, Any

class PromptEngineeringForRobots:
    """Collection of effective prompt engineering techniques for robotics applications"""

    @staticmethod
    def create_structured_instruction_prompt(command: str, context: Dict) -> str:
        """Create a structured prompt for LLM-based instruction following"""
        prompt = f"""## ROBOT COGNITIVE PLANNING TASK

You are an AI cognitive planner for a humanoid robot. Your job is to translate natural language commands into executable robot action sequences.

### INPUT CONTEXT
- Robot Type: Humanoid
- Current Position: {{context.get('position', 'unknown')}}
- Available Actions: {{', '.join(context.get('available_actions', []))}}
- Detected Objects: [objects list]
- Battery Level: {{context.get('battery_level', 100)}}%

### NATURAL LANGUAGE COMMAND
"{command}"

### OUTPUT REQUIREMENTS
1. Return ONLY a valid JSON array of actions
2. Each action must have: "action_type", "parameters", "priority"
3. Action types must be from: {{', '.join(context.get('available_actions', []))}}
4. Parameters must be specific and actionable
5. Priority: lower numbers execute first (1 is highest priority)

### ACTION SEQUENCE GENERATION RULES
- Actions must be physically possible for a humanoid robot
- Actions must be logically sequenced to achieve the goal
- Include necessary intermediate actions (e.g., navigation before manipulation)
- Consider robot's current state when planning

### EXAMPLE OUTPUT FORMAT
[
    {{
        "action_type": "navigate_to",
        "parameters": {{
            "target_position": [2.5, 1.8, 0.0],
            "approach_direction": "front",
            "safety_margin": 0.5
        }},
        "priority": 1,
        "estimated_duration": 15.0
    }},
    {{
        "action_type": "find_object",
        "parameters": {{
            "object_type": "red_cube",
            "search_method": "systematic_scan",
            "max_search_time": 10.0
        }},
        "priority": 2,
        "estimated_duration": 8.0
    }},
    {{
        "action_type": "pick_up",
        "parameters": {{
            "target_object": "red_cube",
            "grasp_type": "pinch_grasp",
            "grasp_position": [2.5, 1.8, 0.2]
        }},
        "priority": 3,
        "estimated_duration": 6.0
    }}
]

### NOW GENERATE THE ACTION SEQUENCE FOR THE COMMAND ABOVE:"""

        return prompt

    @staticmethod
    def create_few_shot_learning_prompt(command: str, context: Dict) -> str:
        """Create a prompt using few-shot learning examples"""
        examples = [
            {
                "command": "Pick up the blue ball from the table",
                "plan": [
                    {
                        "action_type": "navigate_to",
                        "parameters": {"target_position": [1.0, 1.0, 0.0]},
                        "priority": 1
                    },
                    {
                        "action_type": "find_object",
                        "parameters": {"object_type": "blue_ball"},
                        "priority": 2
                    },
                    {
                        "action_type": "pick_up",
                        "parameters": {"target_object": "blue_ball"},
                        "priority": 3
                    }
                ]
            },
            {
                "command": "Go to the kitchen and bring me a bottle of water",
                "plan": [
                    {
                        "action_type": "navigate_to",
                        "parameters": {"target_position": [3.0, 2.0, 0.0]},
                        "priority": 1
                    },
                    {
                        "action_type": "find_object",
                        "parameters": {"object_type": "water_bottle"},
                        "priority": 2
                    },
                    {
                        "action_type": "pick_up",
                        "parameters": {"target_object": "water_bottle"},
                        "priority": 3
                    },
                    {
                        "action_type": "navigate_to",
                        "parameters": {"target_position": [0.0, 0.0, 0.0]},
                        "priority": 4
                    }
                ]
            }
        ]

        prompt = f"""## FEW-SHOT LEARNING EXAMPLES

Here are examples of natural language commands mapped to robot action sequences:

### EXAMPLE 1:
Command: "{{examples[0]['command']}}"
Plan: {{json.dumps(examples[0]['plan'], indent=2)}}

### EXAMPLE 2:
Command: "{{examples[1]['command']}}"
Plan: {{json.dumps(examples[1]['plan'], indent=2)}}

### ROBOT CAPABILITIES
- Navigation: move to locations
- Manipulation: pick up, place down, grasp objects
- Perception: find objects, inspect items
- Communication: speak, listen

### CURRENT CONTEXT
- Current Location: {{context.get('position', 'starting position')}}
- Available Objects: [objects list]

### NEW COMMAND TO PLAN
"{command}"

### OUTPUT (JSON ONLY):
"""

        return prompt

    @staticmethod
    def create_chain_of_thought_prompt(command: str, context: Dict) -> str:
        """Create a prompt that encourages chain-of-thought reasoning"""
        prompt = f"""## CHAIN-OF-THOUGHT ROBOT PLANNING

You are an AI cognitive planner for a humanoid robot. Think step-by-step to create an action plan for the given command.

### NATURAL LANGUAGE COMMAND
"{command}"

### STEP-BY-STEP REASONING PROCESS
1. **Goal Analysis**: What is the ultimate goal?
2. **Current State Assessment**: What does the robot know about its environment?
3. **Required Actions**: What specific actions are needed to achieve the goal?
4. **Action Sequencing**: In what order should the actions be executed?
5. **Feasibility Check**: Are all actions possible given robot capabilities?

### ROBOT CONTEXT
- Capabilities: {{', '.join(context.get('available_actions', []))}}
- Current Position: {{context.get('position', 'unknown')}}
- Known Objects: [objects list]

### REASONING:
First, I need to understand the goal of the command. The user wants the robot to...

### ACTION SEQUENCE (JSON FORMAT):
Return only the final JSON action sequence after your reasoning."""

        return prompt

    @staticmethod
    def create_self_consistency_prompt(command: str, context: Dict) -> List[str]:
        """Create multiple prompt variants for self-consistency"""
        prompts = []

        # Original structured prompt
        prompts.append(PromptEngineeringForRobots.create_structured_instruction_prompt(command, context))

        # Chain of thought variant
        prompts.append(PromptEngineeringForRobots.create_chain_of_thought_prompt(command, context))

        # Few-shot learning variant
        prompts.append(PromptEngineeringForRobots.create_few_shot_learning_prompt(command, context))

        # Simplified direct prompt
        simple_prompt = f"""Robot command: {{command}}
Context: Robot capabilities include navigation, manipulation, and perception.
Output: JSON array of actions to execute the command."""
        prompts.append(simple_prompt)

        return prompts

    @staticmethod
    def aggregate_self_consistency_responses(responses: List[str]) -> List[Dict]:
        """Aggregate responses from self-consistency prompting"""
        import ast
        from collections import Counter

        # Parse all responses
        parsed_responses = []
        for response in responses:
            try:
                parsed = json.loads(response)
                if isinstance(parsed, list) and len(parsed) > 0:
                    parsed_responses.append(parsed)
            except json.JSONDecodeError:
                # Try to extract JSON from response
                import re
                json_match = re.search(r'\[(.*?)\]', response, re.DOTALL)
                if json_match:
                    try:
                        parsed = json.loads(json_match.group(0))
                        if isinstance(parsed, list) and len(parsed) > 0:
                            parsed_responses.append(parsed)
                    except json.JSONDecodeError:
                        continue

        if not parsed_responses:
            return []

        # Find most common action sequence pattern
        # This is a simplified approach - in practice, you'd want more sophisticated aggregation
        return parsed_responses[0]  # Return the first valid response for now
```

## LLM Integration with Isaac ROS

### Isaac ROS LLM Bridge
```python
# isaac_ros_llm_bridge.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
import threading
import queue
import time
from concurrent.futures import ThreadPoolExecutor
import asyncio

class IsaacROSLlmBridge(Node):
    def __init__(self):
        super().__init__('isaac_ros_llm_bridge')

        # LLM configuration
        self.llm_model = "gpt-4-turbo"
        self.max_workers = 2  # Limit concurrent LLM calls
        self.response_timeout = 30.0  # seconds

        # Command processing queue
        self.command_queue = queue.Queue()
        self.executor = ThreadPoolExecutor(max_workers=self.max_workers)

        # Subscriptions
        self.voice_command_sub = self.create_subscription(
            String,
            '/voice_command/parsed',
            self.voice_command_callback,
            10
        )

        self.vision_input_sub = self.create_subscription(
            Image,
            '/camera/color/image_rect_color',
            self.vision_input_callback,
            10
        )

        self.robot_state_sub = self.create_subscription(
            String,  # In practice, this would be a robot state message
            '/robot/state',
            self.robot_state_callback,
            10
        )

        # Publishers
        self.action_sequence_pub = self.create_publisher(
            String,
            '/llm_planner/action_sequence',
            10
        )

        self.visualization_pub = self.create_publisher(
            MarkerArray,
            '/llm_planner/visualization',
            10
        )

        self.feedback_pub = self.create_publisher(
            String,
            '/llm_planner/feedback',
            10
        )

        # Internal state
        self.current_robot_state = {}
        self.vision_context = {}
        self.llm_processing_active = False

        # Start command processing thread
        self.processing_thread = threading.Thread(target=self.process_commands, daemon=True)
        self.processing_thread.start()

        self.get_logger().info('Isaac ROS LLM Bridge initialized')

    def voice_command_callback(self, msg):
        """Process incoming voice commands"""
        try:
            command_data = json.loads(msg.data)
            self.command_queue.put(command_data)
            self.get_logger().info(f'Queued command: {command_data.get("original_command", "unknown")}')
        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON in voice command message')

    def vision_input_callback(self, msg):
        """Process incoming vision data for context"""
        # In a real implementation, this would extract visual context
        # For now, we'll just store the timestamp
        self.vision_context['last_image_timestamp'] = msg.header.stamp

    def robot_state_callback(self, msg):
        """Update robot state context"""
        try:
            state_data = json.loads(msg.data)
            self.current_robot_state.update(state_data)
        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON in robot state message')

    def process_commands(self):
        """Process commands in a separate thread"""
        while rclpy.ok():
            try:
                # Get command from queue with timeout
                command_data = self.command_queue.get(timeout=1.0)

                # Check if LLM is currently busy
                if self.llm_processing_active:
                    self.get_logger().warn('LLM is busy, queuing command')
                    # Add command back to queue briefly
                    time.sleep(0.5)
                    self.command_queue.put(command_data)
                    continue

                # Process command with LLM
                self.process_command_with_llm(command_data)

                self.command_queue.task_done()

            except queue.Empty:
                continue  # No commands to process, continue loop
            except Exception as e:
                self.get_logger().error(f'Error processing command: {str(e)}')

    def process_command_with_llm(self, command_data):
        """Process a single command using LLM"""
        self.llm_processing_active = True

        try:
            # Get current context
            context = self.get_current_context()

            # Create LLM prompt
            prompt = self.create_enhanced_prompt(
                command_data.get('original_command', ''),
                context
            )

            # Call LLM in executor to prevent blocking
            future = self.executor.submit(self.call_llm_sync, prompt)

            # Wait for result with timeout
            try:
                response = future.result(timeout=self.response_timeout)

                # Parse and validate response
                action_sequence = self.parse_and_validate_response(response)

                if action_sequence:
                    # Publish action sequence
                    self.publish_action_sequence(action_sequence, command_data)

                    # Provide feedback
                    feedback = {
                        'status': 'success',
                        'command': command_data.get('original_command', ''),
                        'actions_planned': len(action_sequence),
                        'processing_time': time.time() - command_data.get('timestamp', time.time())
                    }
                    self.publish_feedback(feedback)
                else:
                    self.get_logger().error('Could not parse valid action sequence from LLM response')
                    self.publish_feedback({
                        'status': 'failure',
                        'command': command_data.get('original_command', ''),
                        'error': 'Could not parse valid action sequence'
                    })

            except TimeoutError:
                self.get_logger().error('LLM call timed out')
                self.publish_feedback({
                    'status': 'timeout',
                    'command': command_data.get('original_command', ''),
                    'error': 'LLM call timed out'
                })

        except Exception as e:
            self.get_logger().error(f'Error in LLM processing: {str(e)}')
            self.publish_feedback({
                'status': 'error',
                'command': command_data.get('original_command', ''),
                'error': str(e)
            })

        finally:
            self.llm_processing_active = False

    def get_current_context(self):
        """Get comprehensive robot context"""
        return {
            'robot_state': self.current_robot_state,
            'vision_context': self.vision_context,
            'environment_map': self.get_environment_map(),
            'robot_capabilities': self.get_robot_capabilities(),
            'current_task': self.get_current_task(),
            'recent_actions': self.get_recent_actions(),
            'battery_level': self.get_battery_level()
        }

    def get_environment_map(self):
        """Get current environment map (simplified)"""
        # In practice, this would interface with SLAM or mapping systems
        return {
            'known_locations': ['kitchen', 'bedroom', 'living_room', 'office'],
            'obstacles': [],
            'free_spaces': []
        }

    def get_robot_capabilities(self):
        """Get robot capabilities"""
        return {
            'navigation': True,
            'manipulation': True,
            'perception': True,
            'locomotion_type': 'bipedal',
            'max_payload': 2.0,  # kg
            'reach_distance': 1.2  # meters
        }

    def create_enhanced_prompt(self, command: str, context: Dict) -> str:
        """Create an enhanced prompt with Isaac ROS-specific context"""
        prompt = f"""## ISAAC ROS COGNITIVE PLANNER

You are the cognitive planning module for an Isaac ROS humanoid robot. Translate the user's natural language command into a detailed action sequence.

### ROBOT SPECIFICATIONS (Isaac ROS)
- Platform: Humanoid (bipedal locomotion)
- Navigation: Enabled with obstacle avoidance
- Manipulation: Enabled with dexterous hands
- Perception: RGB-D camera, IMU, encoders
- Max Payload: {{context['robot_capabilities']['max_payload']}}kg
- Reach Distance: {{context['robot_capabilities']['reach_distance']}}m

### ENVIRONMENT CONTEXT
- Known Locations: {{', '.join(context['environment_map']['known_locations'])}}
- Current Position: {{context['robot_state'].get('position', 'unknown')}}
- Battery Level: {{context['battery_level']}}%
- Detected Objects: {{context['vision_context'].get('detected_objects', 'none')}}

### NATURAL LANGUAGE COMMAND
"{command}"

### PLANNING CONSTRAINTS
1. Consider bipedal locomotion limitations (stability, balance)
2. Account for humanoid manipulation constraints (DOFs, dexterity)
3. Ensure actions are feasible in simulation environment
4. Include safety checks for navigation and manipulation
5. Plan for potential contingencies (object not found, path blocked)

### OUTPUT FORMAT
Return ONLY a valid JSON array with the following structure:
[
    {{
        "action_type": "navigate_to|pick_up|place_down|find_object|inspect|speak|listen",
        "parameters": {{
            "target_position": [x, y, z] (for navigation),
            "target_object": "object_name" (for manipulation),
            "approach_direction": "front|side|top",
            "grasp_type": "pinch|power|precision",
            "text_to_speak": "message" (for speak action)
        }},
        "priority": 1 (lower = higher priority),
        "estimated_duration": 10.0 (in seconds),
        "feasibility_score": 0.9 (0.0-1.0, higher = more feasible)
    }}
]

### ACTION SEQUENCE:"""

        return prompt

    def call_llm_sync(self, prompt: str) -> str:
        """Synchronous call to LLM (in practice, use actual API)"""
        # This is a mock implementation
        # In practice, use openai.OpenAI().chat.completions.create() or similar

        # Simulate LLM call delay
        time.sleep(2.0)

        # Mock response based on command
        command_lower = prompt.lower()

        if "pick up" in command_lower or "grasp" in command_lower:
            return json.dumps([
                {
                    "action_type": "find_object",
                    "parameters": {"object_type": "unknown_object"},
                    "priority": 1,
                    "estimated_duration": 5.0,
                    "feasibility_score": 0.8
                },
                {
                    "action_type": "navigate_to",
                    "parameters": {"target_position": [1.0, 1.0, 0.0]},
                    "priority": 2,
                    "estimated_duration": 10.0,
                    "feasibility_score": 0.9
                },
                {
                    "action_type": "pick_up",
                    "parameters": {"target_object": "unknown_object"},
                    "priority": 3,
                    "estimated_duration": 8.0,
                    "feasibility_score": 0.7
                }
            ])
        elif "go to" in command_lower or "navigate" in command_lower:
            return json.dumps([
                {
                    "action_type": "navigate_to",
                    "parameters": {"target_position": [2.0, 2.0, 0.0]},
                    "priority": 1,
                    "estimated_duration": 15.0,
                    "feasibility_score": 0.95
                }
            ])
        else:
            return json.dumps([
                {
                    "action_type": "find_object",
                    "parameters": {"object_type": "relevant_object"},
                    "priority": 1,
                    "estimated_duration": 8.0,
                    "feasibility_score": 0.6
                }
            ])

    def parse_and_validate_response(self, response_text: str) -> List[Dict]:
        """Parse and validate LLM response"""
        try:
            # Attempt to parse as JSON
            action_sequence = json.loads(response_text)

            # Validate structure
            if not isinstance(action_sequence, list):
                return []

            # Validate each action
            validated_sequence = []
            for action in action_sequence:
                if self.validate_action_structure(action):
                    validated_sequence.append(action)

            return validated_sequence

        except json.JSONDecodeError:
            # Try to extract JSON from response
            import re
            json_match = re.search(r'\[(.*?)\]', response_text, re.DOTALL)
            if json_match:
                try:
                    extracted_json = json_match.group(0)
                    action_sequence = json.loads(extracted_json)

                    # Validate extracted sequence
                    validated_sequence = []
                    for action in action_sequence:
                        if self.validate_action_structure(action):
                            validated_sequence.append(action)

                    return validated_sequence
                except json.JSONDecodeError:
                    pass

        return []

    def validate_action_structure(self, action: Dict) -> bool:
        """Validate individual action structure"""
        required_keys = ['action_type', 'parameters', 'priority', 'estimated_duration', 'feasibility_score']

        for key in required_keys:
            if key not in action:
                return False

        # Validate types
        if not isinstance(action['action_type'], str):
            return False
        if not isinstance(action['parameters'], dict):
            return False
        if not isinstance(action['priority'], int):
            return False
        if not isinstance(action['estimated_duration'], (int, float)):
            return False
        if not isinstance(action['feasibility_score'], (int, float)):
            return False

        # Validate feasibility score range
        if not 0.0 <= action['feasibility_score'] <= 1.0:
            return False

        return True

    def publish_action_sequence(self, action_sequence: List[Dict], original_command: Dict):
        """Publish the validated action sequence"""
        sequence_msg = String()
        sequence_msg.data = json.dumps({
            'original_command': original_command,
            'action_sequence': action_sequence,
            'timestamp': time.time()
        })

        self.action_sequence_pub.publish(sequence_msg)

    def publish_feedback(self, feedback_data: Dict):
        """Publish processing feedback"""
        feedback_msg = String()
        feedback_msg.data = json.dumps(feedback_data)

        self.feedback_pub.publish(feedback_msg)

def main(args=None):
    rclpy.init(args=args)
    node = IsaacRosLlmBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Isaac ROS LLM Bridge')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Handling Ambiguity and Context

### Context-Aware Command Resolution
```python
# context_aware_resolver.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from typing import Dict, List, Optional, Tuple

class ContextAwareCommandResolver(Node):
    def __init__(self):
        super().__init__('context_aware_resolver')

        # Subscriptions
        self.parsed_command_sub = self.create_subscription(
            String,
            '/llm_planner/action_sequence',
            self.parsed_command_callback,
            10
        )

        self.environment_context_sub = self.create_subscription(
            String,
            '/environment/context',
            self.environment_context_callback,
            10
        )

        self.robot_state_sub = self.create_subscription(
            String,
            '/robot/state',
            self.robot_state_callback,
            10
        )

        # Publishers
        self.resolved_command_pub = self.create_publisher(
            String,
            '/context_aware_planner/resolved_commands',
            10
        )

        # Internal state
        self.environment_context = {}
        self.robot_state = {}
        self.object_database = {}  # Store known objects and their properties
        self.location_database = {}  # Store known locations and their properties

        self.get_logger().info('Context-Aware Command Resolver initialized')

    def parsed_command_callback(self, msg):
        """Process parsed commands and resolve ambiguities with context"""
        try:
            command_data = json.loads(msg.data)

            # Resolve ambiguous references using context
            resolved_command = self.resolve_command_ambiguities(command_data)

            # Publish resolved command
            resolved_msg = String()
            resolved_msg.data = json.dumps(resolved_command)
            self.resolved_command_pub.publish(resolved_msg)

            self.get_logger().info(f'Resolved command with {len(resolved_command.get("action_sequence", []))} actions')

        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON in parsed command message')

    def environment_context_callback(self, msg):
        """Update environment context"""
        try:
            context_data = json.loads(msg.data)
            self.environment_context.update(context_data)
        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON in environment context message')

    def robot_state_callback(self, msg):
        """Update robot state"""
        try:
            state_data = json.loads(msg.data)
            self.robot_state.update(state_data)
        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON in robot state message')

    def resolve_command_ambiguities(self, command_data: Dict) -> Dict:
        """Resolve ambiguities in command using context"""
        resolved_data = command_data.copy()
        action_sequence = command_data.get('action_sequence', [])

        resolved_actions = []
        for action in action_sequence:
            resolved_action = self.resolve_action_ambiguities(action)
            resolved_actions.append(resolved_action)

        resolved_data['action_sequence'] = resolved_actions
        resolved_data['resolution_log'] = self.generate_resolution_log(action_sequence, resolved_actions)

        return resolved_data

    def resolve_action_ambiguities(self, action: Dict) -> Dict:
        """Resolve ambiguities in a single action"""
        resolved_action = action.copy()
        params = action.get('parameters', {})

        # Resolve ambiguous object references
        if 'target_object' in params and params['target_object'] in ['it', 'that', 'the one', 'this']:
            # Use context to determine what "it" or "that" refers to
            resolved_object = self.resolve_ambiguous_object_reference(params['target_object'])
            if resolved_object:
                resolved_action['parameters']['target_object'] = resolved_object
                resolved_action['resolution_notes'] = f"Resolved '{params['target_object']}' to '{resolved_object}'"

        # Resolve ambiguous location references
        if 'target_position' in params and params['target_position'] == 'there':
            # Use context to determine what "there" refers to
            resolved_position = self.resolve_ambiguous_location_reference('there')
            if resolved_position:
                resolved_action['parameters']['target_position'] = resolved_position
                resolved_action['resolution_notes'] = resolved_action.get('resolution_notes', '') + f"; Resolved 'there' to {resolved_position}"

        # Resolve color/size/shape ambiguities
        if 'object_descriptor' in params:
            resolved_descriptor = self.resolve_descriptor_ambiguity(params['object_descriptor'])
            if resolved_descriptor != params['object_descriptor']:
                resolved_action['parameters']['object_descriptor'] = resolved_descriptor
                resolved_action['resolution_notes'] = resolved_action.get('resolution_notes', '') + f"; Clarified descriptor to {resolved_descriptor}"

        # Resolve action parameter ambiguities
        if action['action_type'] == 'navigate_to':
            resolved_action = self.resolve_navigation_ambiguities(resolved_action)
        elif action['action_type'] in ['pick_up', 'grasp', 'take']:
            resolved_action = self.resolve_manipulation_ambiguities(resolved_action)

        return resolved_action

    def resolve_ambiguous_object_reference(self, reference: str) -> Optional[str]:
        """Resolve ambiguous object references like 'it', 'that', 'the red one'"""
        if not self.environment_context.get('detected_objects'):
            return None

        # Get recently detected objects
        recent_objects = self.environment_context['detected_objects'][-5:]  # Last 5 detected objects

        if reference == 'it' or reference == 'that':
            # Use the most recently detected object
            if recent_objects:
                return recent_objects[-1].get('name', 'unknown_object')
        elif 'the' in reference and any(color in reference for color in ['red', 'blue', 'green', 'yellow', 'white', 'black']):
            # Try to match by color
            target_color = None
            for color in ['red', 'blue', 'green', 'yellow', 'white', 'black']:
                if color in reference:
                    target_color = color
                    break

            for obj in reversed(recent_objects):  # Search most recent first
                if obj.get('color') == target_color:
                    return obj.get('name', 'unknown_object')

        return None

    def resolve_ambiguous_location_reference(self, reference: str) -> Optional[List[float]]:
        """Resolve ambiguous location references like 'there', 'over there'"""
        if reference == 'there':
            # In a real system, this would use spatial reasoning
            # For now, return a position relative to robot's current location
            current_pos = self.robot_state.get('position', [0, 0, 0])
            if current_pos:
                # Example: "there" is 1m in front of robot
                return [current_pos[0] + 1.0, current_pos[1], current_pos[2]]

        return None

    def resolve_descriptor_ambiguity(self, descriptor: str) -> str:
        """Resolve ambiguous descriptors like 'big', 'small', 'left', 'right'"""
        # In a real system, this would use spatial relationships and object properties
        # For now, return the descriptor as-is
        return descriptor

    def resolve_navigation_ambiguities(self, action: Dict) -> Dict:
        """Resolve ambiguities specific to navigation actions"""
        resolved_action = action.copy()
        params = action.get('parameters', {})

        # If target is ambiguous, try to resolve using context
        if 'target_position' in params and isinstance(params['target_position'], str):
            resolved_position = self.lookup_known_location(params['target_position'])
            if resolved_position:
                resolved_action['parameters']['target_position'] = resolved_position
                resolved_action['resolution_notes'] = resolved_action.get('resolution_notes', '') + f"; Looked up location '{params['target_position']}'"

        return resolved_action

    def resolve_manipulation_ambiguities(self, action: Dict) -> Dict:
        """Resolve ambiguities specific to manipulation actions"""
        resolved_action = action.copy()
        params = action.get('parameters', {})

        # If target object is ambiguous, try to resolve using context
        if 'target_object' in params and isinstance(params['target_object'], str):
            resolved_object = self.lookup_known_object(params['target_object'])
            if resolved_object:
                resolved_action['parameters']['target_object'] = resolved_object
                resolved_action['resolution_notes'] = resolved_action.get('resolution_notes', '') + f"; Looked up object '{params['target_object']}'"

        return resolved_action

    def lookup_known_location(self, location_name: str) -> Optional[List[float]]:
        """Look up known location by name"""
        if location_name in self.location_database:
            return self.location_database[location_name].get('coordinates')

        # Try fuzzy matching
        import difflib
        close_matches = difflib.get_close_matches(
            location_name,
            self.location_database.keys(),
            n=1,
            cutoff=0.6
        )

        if close_matches:
            return self.location_database[close_matches[0]].get('coordinates')

        return None

    def lookup_known_object(self, object_name: str) -> Optional[str]:
        """Look up known object by name"""
        if object_name in self.object_database:
            return object_name  # Return the confirmed object name

        # Try fuzzy matching
        import difflib
        close_matches = difflib.get_close_matches(
            object_name,
            self.object_database.keys(),
            n=1,
            cutoff=0.6
        )

        if close_matches:
            return close_matches[0]

        return None

    def generate_resolution_log(self, original_actions: List[Dict], resolved_actions: List[Dict]) -> List[str]:
        """Generate a log of resolutions made"""
        log_entries = []

        for orig, res in zip(original_actions, resolved_actions):
            if 'resolution_notes' in res:
                log_entries.append(f"Action {orig.get('action_type')}: {res['resolution_notes']}")

        return log_entries

def main(args=None):
    rclpy.init(args=args)
    node = ContextAwareCommandResolver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Context-Aware Command Resolver')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Performance Optimization

### Optimizing LLM Inference for Real-Time Robotics
```python
# llm_optimization.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import asyncio
import threading
import queue
import time
from functools import lru_cache
import torch

class OptimizedLLMPlanner(Node):
    def __init__(self):
        super().__init__('optimized_llm_planner')

        # Performance optimization parameters
        self.cache_size = 128  # Number of command-action pairs to cache
        self.batch_size = 4    # Number of commands to process in batch
        self.inference_timeout = 15.0  # Timeout for LLM inference
        self.max_retries = 3   # Number of retries for failed inference

        # Command batching
        self.command_batch = []
        self.batch_timer = self.create_timer(0.5, self.process_batch)  # Process every 500ms

        # Performance metrics
        self.metrics = {
            'total_commands': 0,
            'cached_responses': 0,
            'avg_inference_time': 0.0,
            'success_rate': 0.0
        }

        # Subscriptions and publishers
        self.command_sub = self.create_subscription(
            String,
            '/natural_language/command',
            self.command_callback,
            10
        )

        self.optimized_action_pub = self.create_publisher(
            String,
            '/optimized_planner/action_sequence',
            10
        )

        # Cache for frequently used command-action pairs
        self.command_cache = {}

        self.get_logger().info('Optimized LLM Planner initialized')

    @lru_cache(maxsize=128)
    def get_cached_action_sequence(self, command_hash: str) -> Optional[tuple]:
        """Cache frequently used action sequences"""
        # This would return a cached action sequence if available
        return None

    def command_callback(self, msg):
        """Process incoming commands with optimization"""
        command_text = msg.data
        command_hash = hash(command_text)

        # Check cache first
        cached_result = self.get_cached_action_sequence(command_hash)
        if cached_result:
            self.metrics['cached_responses'] += 1
            action_sequence, timestamp = cached_result

            # Publish cached result
            self.publish_action_sequence(action_sequence)
            self.get_logger().info(f'Used cached response for command: {command_text[:50]}...')
            return

        # Add to batch for processing
        self.command_batch.append({
            'command': command_text,
            'hash': command_hash,
            'timestamp': time.time()
        })

        # Process immediately if batch is full
        if len(self.command_batch) >= self.batch_size:
            self.process_batch()

    def process_batch(self):
        """Process command batch with optimized LLM inference"""
        if not self.command_batch:
            return

        # Process all commands in batch
        for cmd_data in self.command_batch:
            start_time = time.time()

            try:
                # Plan actions with error handling and retries
                action_sequence = self.plan_with_retry(cmd_data['command'])

                if action_sequence:
                    # Cache the result
                    self.command_cache[cmd_data['hash']] = (action_sequence, time.time())

                    # Publish action sequence
                    self.publish_action_sequence(action_sequence)

                    # Update metrics
                    inference_time = time.time() - start_time
                    self.update_performance_metrics(True, inference_time)

                    self.get_logger().info(f'Processed command in {inference_time:.2f}s: {cmd_data["command"][:30]}...')
                else:
                    self.update_performance_metrics(False, time.time() - start_time)
                    self.get_logger().error(f'Failed to process command: {cmd_data["command"][:30]}...')

            except Exception as e:
                self.update_performance_metrics(False, time.time() - start_time)
                self.get_logger().error(f'Error processing command {cmd_data["command"][:30]}...: {str(e)}')

        # Clear batch
        self.command_batch.clear()

    def plan_with_retry(self, command: str, max_retries: int = None) -> Optional[List[Dict]]:
        """Plan actions with retry mechanism"""
        if max_retries is None:
            max_retries = self.max_retries

        last_error = None
        for attempt in range(max_retries):
            try:
                # Use a more focused prompt for better performance
                action_sequence = self.plan_with_optimized_prompt(command)
                return action_sequence
            except Exception as e:
                last_error = e
                self.get_logger().warn(f'Attempt {attempt + 1} failed: {str(e)}. Retrying...')
                time.sleep(0.5 * (attempt + 1))  # Exponential backoff

        self.get_logger().error(f'All {max_retries} attempts failed. Last error: {str(last_error)}')
        return None

    def plan_with_optimized_prompt(self, command: str) -> List[Dict]:
        """Plan actions using an optimized prompt for faster inference"""
        # Use a more concise, focused prompt
        optimized_prompt = f"""CMD: {{command}}
        ROBOT: Humanoid with navigation, manipulation, perception
        OUTPUT: JSON [{{"action_type": "...", "params": {{}}, "priority": 1}}]
        LIMIT: 3-5 actions max, simple but complete plan"""

        # Call LLM with optimized parameters
        response = self.call_optimized_llm(optimized_prompt)

        # Parse response
        try:
            import json
            return json.loads(response)
        except json.JSONDecodeError:
            # Try to extract JSON from response
            import re
            json_match = re.search(r'\[(.*?)\]', response, re.DOTALL)
            if json_match:
                try:
                    return json.loads(json_match.group(0))
                except json.JSONDecodeError:
                    pass

        return []

    def call_optimized_llm(self, prompt: str) -> str:
        """Call LLM with optimized parameters for speed"""
        # In practice, use streaming API, lower temperature, fewer tokens
        # For this example, using a mock implementation
        import time
        time.sleep(1.0)  # Simulate processing time

        # Mock response
        return '[{"action_type": "navigate_to", "params": {"target_position": [1.0, 1.0, 0.0]}, "priority": 1}]'

    def update_performance_metrics(self, success: bool, inference_time: float):
        """Update performance metrics"""
        self.metrics['total_commands'] += 1

        if success:
            # Update average inference time
            old_avg = self.metrics['avg_inference_time']
            total_cmds = self.metrics['total_commands']
            new_avg = ((old_avg * (total_cmds - 1)) + inference_time) / total_cmds
            self.metrics['avg_inference_time'] = new_avg

            # Update success rate
            successful_cmds = sum(1 for _ in range(total_cmds) if success)  # This is simplified
            self.metrics['success_rate'] = successful_cmds / total_cmds

    def get_performance_report(self):
        """Get current performance metrics"""
        return {
            'total_commands_processed': self.metrics['total_commands'],
            'cached_responses_used': self.metrics['cached_responses'],
            'average_inference_time': self.metrics['avg_inference_time'],
            'current_success_rate': self.metrics['success_rate'],
            'cache_hit_rate': self.metrics['cached_responses'] / max(self.metrics['total_commands'], 1)
        }

    def publish_action_sequence(self, action_sequence: List[Dict]):
        """Publish the action sequence"""
        import json
        msg = String()
        msg.data = json.dumps(action_sequence)
        self.optimized_action_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OptimizedLLMPlanner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Print final performance report
        report = node.get_performance_report()
        node.get_logger().info(f'Performance Report: {report}')
        node.get_logger().info('Shutting down Optimized LLM Planner')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Hands-On Exercise
1. Set up OpenAI API credentials (or local LLM) for the cognitive planner
2. Implement the LLM-based cognitive planner node with context awareness
3. Test with various natural language commands
4. Implement prompt engineering techniques for better accuracy
5. Integrate with Isaac ROS for simulation-based testing
6. Optimize LLM inference for real-time performance
7. Evaluate the system's ability to handle ambiguous commands

Example commands to test:
```bash
# Test the cognitive planner
ros2 run your_cognitive_planning_package llm_cognitive_planner_node

# Send a natural language command
ros2 topic pub /natural_language/command std_msgs/String "data: 'go to the kitchen and pick up the red cup'"

# Monitor the planned actions
ros2 topic echo /cognitive_planner/action_sequence
```

## Summary
This lesson covered cognitive planning using LLMs to translate natural language commands into robot action sequences. You learned about prompt engineering, context-aware command resolution, Isaac ROS integration, and performance optimization techniques. The next lesson will explore object recognition and manipulation for VLA systems.
</ChapterTranslator>