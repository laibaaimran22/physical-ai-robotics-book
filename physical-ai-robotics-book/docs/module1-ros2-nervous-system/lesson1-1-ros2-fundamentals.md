---
sidebar_position: 1
title: "Introduction to ROS 2 Fundamentals"
---

import ChapterTranslator from '@site/src/components/Translation/ChapterTranslator';

<ChapterTranslator>

 # Introduction to ROS 2 Fundamentals


## Overview
This lesson introduces the core concepts of ROS 2, including its architecture, communication mechanisms (nodes, topics, services), and command-line tools. This lesson will establish the foundational knowledge for building robot applications.

## Learning Objectives
By the end of this lesson, you should be able to:
- Explain the basic architecture of ROS 2
- Understand the core communication concepts: nodes, topics, services
- Use basic ROS 2 command-line tools
- Set up a simple ROS 2 workspace

## What is ROS 2?
Robot Operating System 2 (ROS 2) is an open-source framework for building robot applications. It provides libraries, tools, and conventions to help software developers create robot applications. Despite its name, ROS 2 is not an operating system but rather a middleware that provides services designed for a heterogeneous computer cluster.

## Key Concepts
### Nodes
A node is a process that performs computation. Nodes are the fundamental building blocks of a ROS 2 program. Each node is designed to perform a specific task and can communicate with other nodes to achieve complex behaviors.

### Topics and Messages
Topics are named buses over which nodes exchange messages. Messages are the data packets sent over topics. Topics enable asynchronous communication between nodes using a publish-subscribe model.

### Services
Services provide synchronous request/response communication between nodes. When a node needs specific information or wants another node to perform an action, it can call a service.

## Basic Command-Line Tools
ROS 2 provides several command-line tools to interact with the system:
- `ros2 node list` - List active nodes
- `ros2 topic list` - List active topics
- `ros2 service list` - List active services
- `ros2 run <package> <executable>` - Run a node
- `ros2 launch <package> <launch_file>` - Launch a configuration of nodes

## Hands-On Exercise
1. Create a new ROS 2 workspace:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws
   ```

2. Source your ROS 2 installation:
   ```bash
   source /opt/ros/humble/setup.bash  # Replace 'humble' with your ROS 2 distribution
   ```

3. Build the workspace:
   ```bash
   colcon build
   source install/setup.bash
   ```

4. Verify that your environment is set up correctly:
   ```bash
   ros2 topic list
   ros2 node list
   ```

## Summary
This lesson introduced the fundamental concepts of ROS 2. Understanding these core concepts is essential for building more complex robotic applications. In the next lesson, we'll dive deeper into creating and managing ROS 2 nodes, topics, and services in practice.

</ChapterTranslator>