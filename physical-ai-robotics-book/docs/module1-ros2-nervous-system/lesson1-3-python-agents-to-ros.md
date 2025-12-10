---
sidebar_position: 3
title: "Bridging Python Agents to ROS Controllers with rclpy"
---

import ChapterTranslator from '@site/src/components/Translation/ChapterTranslator';

<ChapterTranslator>


# Bridging Python Agents to ROS Controllers with rclpy

## Overview
This lesson covers how to integrate Python-based AI agents with ROS 2 robot controllers using the `rclpy` client library. This lesson will cover writing Python code to interact with ROS 2 systems, enabling intelligent control.

## Learning Objectives
By the end of this lesson, you should be able to:
- Understand how to use rclpy to interface Python agents with ROS 2
- Create ROS 2 nodes that act as bridges between AI agents and robot controllers
- Implement communication patterns between Python agents and ROS 2 systems
- Design architectures for AI-driven robot control

## Introduction to rclpy
`rclpy` is the Python client library for ROS 2. It provides Python bindings for the ROS 2 client library (rcl) and the ROS 2 middleware layer (rmw). This allows Python programs to interact with ROS 2 systems.

## Architecture of AI-Agent-to-ROS Bridge
The bridge between AI agents and ROS 2 controllers typically involves:
1. An AI agent running as a Python process
2. A ROS 2 bridge node that translates between the AI agent and ROS 2
3. ROS 2 controllers that execute commands on the robot

## Example: Simple AI Agent Communicating with ROS 2
Let's create a simple example where a Python AI agent makes decisions and sends commands to a robot:

### AI Agent Node
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import random

class AIAgentNode(Node):
    def __init__(self):
        super().__init__('ai_agent_node')

        # Publisher to send commands to robot
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber to receive sensor data
        self.sensor_subscriber = self.create_subscription(
            String, '/sensor_data', self.sensor_callback, 10)

        # Timer for decision making
        self.timer = self.create_timer(1.0, self.make_decision)

        self.sensor_data = None
        self.get_logger().info('AI Agent Node initialized')

    def sensor_callback(self, msg):
        self.sensor_data = msg.data
        self.get_logger().info(f'Received sensor data: {self.sensor_data}')

    def make_decision(self):
        # Simple AI logic - in real applications, this could involve complex ML models
        twist_msg = Twist()

        if self.sensor_data and 'obstacle' in self.sensor_data:
            # If obstacle detected, turn
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.5  # Turn right
        else:
            # Otherwise, move forward
            twist_msg.linear.x = 0.5  # Move forward
            twist_msg.angular.z = 0.0

        self.cmd_vel_publisher.publish(twist_msg)
        self.get_logger().info(f'Published command: linear.x={twist_msg.linear.x}, angular.z={twist_msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    ai_agent_node = AIAgentNode()

    try:
        rclpy.spin(ai_agent_node)
    except KeyboardInterrupt:
        pass
    finally:
        ai_agent_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced: Using External AI Libraries
You can integrate more sophisticated AI libraries like TensorFlow, PyTorch, or scikit-learn:

### Example with Decision Logic
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
from collections import deque

class AdvancedAIAgentNode(Node):
    def __init__(self):
        super().__init__('advanced_ai_agent_node')

        # Publishers and subscribers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_subscriber = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # Timer for decision making
        self.timer = self.create_timer(0.1, self.make_decision)  # 10Hz

        # State management
        self.scan_data = None
        self.command_history = deque(maxlen=10)  # Keep last 10 commands
        self.obstacle_threshold = 1.0  # meters

        self.get_logger().info('Advanced AI Agent Node initialized')

    def scan_callback(self, msg):
        # Store the latest scan data
        self.scan_data = np.array(msg.ranges)
        # Filter out invalid readings (inf, nan)
        self.scan_data = np.where((self.scan_data == float('inf')) |
                                  np.isnan(self.scan_data),
                                  msg.range_max,
                                  self.scan_data)

    def simple_navigation_logic(self):
        """Simple navigation algorithm"""
        if self.scan_data is None:
            return 0.0, 0.0  # Stop if no data

        # Find the minimum distance in front (�30 degrees)
        front_indices = slice(len(self.scan_data)//2 - 30, len(self.scan_data)//2 + 30)
        front_distances = self.scan_data[front_indices]

        min_distance = np.min(front_distances)

        if min_distance < self.obstacle_threshold:
            # Obstacle detected, turn
            linear_vel = 0.0
            angular_vel = 0.5
        else:
            # Clear path, move forward
            linear_vel = 0.3
            angular_vel = 0.0

        return linear_vel, angular_vel

    def make_decision(self):
        if self.scan_data is None:
            return

        linear_vel, angular_vel = self.simple_navigation_logic()

        # Create and publish twist message
        twist_msg = Twist()
        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel

        self.cmd_vel_publisher.publish(twist_msg)

        # Store command in history
        self.command_history.append((linear_vel, angular_vel))

        self.get_logger().info(f'Command: linear.x={linear_vel:.2f}, angular.z={angular_vel:.2f}')

def main(args=None):
    rclpy.init(args=args)
    ai_agent_node = AdvancedAIAgentNode()

    try:
        rclpy.spin(ai_agent_node)
    except KeyboardInterrupt:
        pass
    finally:
        ai_agent_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Integration Patterns
### 1. Direct Integration
The AI agent runs as a ROS 2 node directly, using rclpy for all communication.

### 2. Bridge Pattern
The AI agent runs separately and communicates with a bridge node via inter-process communication (IPC) like shared memory, TCP sockets, or message queues.

### 3. Service-Based Integration
The AI agent calls ROS 2 services to request specific actions from the robot.

## Hands-On Exercise
1. Create a new ROS 2 package for this lesson:
   ```bash
   cd ~/ros2_ws/src
   ros2 pkg create --build-type ament_python ai_ros_bridge
   cd ai_ros_bridge
   ```

2. Create the AI agent node in `ai_ros_bridge/ai_ros_bridge/simple_ai_agent.py` with the first example code above.

3. Update the `setup.py` file to include your new node as an executable.

4. Build and source your workspace:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select ai_ros_bridge
   source install/setup.bash
   ```

5. In a simulation environment (like Gazebo), run your AI agent:
   ```bash
   ros2 run ai_ros_bridge simple_ai_agent
   ```

## Troubleshooting Tips
- Ensure proper ROS 2 environment setup before running nodes
- Check topic names match between publisher and subscriber
- Verify that the robot simulation or hardware is properly connected
- Use `ros2 topic list` and `ros2 node list` to verify communication

## Summary
This lesson demonstrated how to bridge Python-based AI agents with ROS 2 robot controllers using rclpy. This integration enables intelligent control of robots using AI algorithms. The next lesson will cover understanding URDF (Unified Robot Description Format) for humanoids.
</ChapterTranslator>