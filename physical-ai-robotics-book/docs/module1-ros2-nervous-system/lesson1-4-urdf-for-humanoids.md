---
sidebar_position: 4
title: "Understanding URDF for Humanoid Robotics"
---

import ChapterTranslator from '@site/src/components/Translation/ChapterTranslator';

<ChapterTranslator>

# Understanding URDF for Humanoid Robotics

## Overview
This lesson delves into the Unified Robot Description Format (URDF) for modeling humanoid robots. Understand how to define robot kinematics, visualize robot models, and prepare them for simulation and control.

## Learning Objectives
By the end of this lesson, you should be able to:
- Understand the structure and components of URDF files
- Create URDF models for humanoid robots
- Define joints, links, and kinematic chains
- Visualize URDF models in RViz
- Prepare URDF models for simulation and control

## What is URDF?
URDF (Unified Robot Description Format) is an XML format used in ROS to describe robot models. It defines the physical and kinematic properties of a robot, including:
- Links: Rigid parts of the robot
- Joints: Connections between links
- Visual and collision properties
- Inertial properties

## URDF Structure for Humanoid Robots
A humanoid robot URDF typically includes:
- Torso/Body
- Head
- Arms (upper arm, lower arm, hand)
- Legs (upper leg, lower leg, foot)
- Joint connections between body parts

## Basic URDF Example
Here's a simplified URDF for a basic humanoid robot:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Joint connecting base to torso -->
  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
  </joint>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="100" velocity="1"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.15 0.1 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1"/>
  </joint>

  <link name="left_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.003" ixy="0.0" ixz="0.0" iyy="0.003" iyz="0.0" izz="0.00008"/>
    </inertial>
  </link>

  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1"/>
  </joint>

  <!-- Right Arm -->
  <link name="right_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="right_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_arm"/>
    <origin xyz="0.15 -0.1 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1"/>
  </joint>

  <link name="right_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.003" ixy="0.0" ixz="0.0" iyy="0.003" iyz="0.0" izz="0.00008"/>
    </inertial>
  </link>

  <joint name="right_elbow_joint" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_lower_arm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1"/>
  </joint>

</robot>
```

## URDF Components Explained

### Links
Links represent rigid bodies in the robot. Each link has:
- Visual properties: How the link appears in simulation
- Collision properties: How the link interacts with other objects
- Inertial properties: Mass and moment of inertia for physics simulation

### Joints
Joints connect links and define their relative motion. Joint types include:
- `fixed`: No movement allowed
- `revolute`: Rotational movement around an axis
- `prismatic`: Linear sliding movement
- `continuous`: Continuous rotation (like revolute but unlimited)
- `floating`: 6DOF movement (for base of floating robots)

### Joint Limits
For revolute and prismatic joints, specify:
- `lower` and `upper`: Position limits
- `effort`: Maximum torque/force
- `velocity`: Maximum velocity

## Visualization in RViz
To visualize your URDF model in RViz:

1. Create a launch file to publish the robot state:
```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare(package='your_robot_description').find('your_robot_description')
    default_model_path = os.path.join(pkg_share, 'urdf/your_robot.urdf')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_gui': True}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='model', default_value=default_model_path,
                              description='Absolute path to robot urdf file'),
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ])
```

2. Launch the visualization:
```bash
ros2 launch your_robot_description display.launch.py
```

## Xacro for Complex Models
For complex humanoid robots, use Xacro (XML Macros) to make URDF more maintainable:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_with_xacro">

  <!-- Define properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="arm_length" value="0.3" />
  <xacro:property name="arm_radius" value="0.05" />

  <!-- Macro for arm -->
  <xacro:macro name="arm" params="side parent_link position">
    <link name="${side}_upper_arm">
      <visual>
        <geometry>
          <cylinder length="${arm_length}" radius="${arm_radius}"/>
        </geometry>
        <origin xyz="0 0 ${-arm_length/2}" rpy="0 0 0"/>
      </visual>
      <collision>
        <geometry>
          <cylinder length="${arm_length}" radius="${arm_radius}"/>
        </geometry>
        <origin xyz="0 0 ${-arm_length/2}" rpy="0 0 0"/>
      </collision>
      <inertial>
        <mass value="0.5"/>
        <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.0001"/>
      </inertial>
    </link>

    <joint name="${side}_shoulder_joint" type="revolute">
      <parent link="${parent_link}"/>
      <child link="${side}_upper_arm"/>
      <origin xyz="${position}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="50" velocity="1"/>
    </joint>
  </xacro:macro>

  <!-- Use the macro -->
  <xacro:arm side="left" parent_link="torso" position="0.15 0.1 0.1"/>
  <xacro:arm side="right" parent_link="torso" position="0.15 -0.1 0.1"/>

</robot>
```

## Hands-On Exercise
1. Create a URDF package:
   ```bash
   cd ~/ros2_ws/src
   ros2 pkg create --build-type ament_python robot_description
   cd robot_description
   ```

2. Create a URDF directory and add the basic humanoid URDF:
   ```bash
   mkdir urdf
   # Add the URDF code above to urdf/humanoid.urdf
   ```

3. Create a launch directory and add a display launch file:
   ```bash
   mkdir launch
   # Create launch files to visualize the robot
   ```

4. Update package.xml and setup.py appropriately.

5. Build and run:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select robot_description
   source install/setup.bash
   ros2 launch robot_description display.launch.py
   ```

## Best Practices for Humanoid URDF
- Use consistent naming conventions
- Define proper inertial properties for stable simulation
- Use Xacro for complex, repetitive structures
- Organize URDF in separate files for different body parts
- Include proper joint limits to reflect real robot constraints
- Test the kinematic chain for proper connectivity

## Summary
This lesson covered the fundamentals of URDF for humanoid robotics. Understanding URDF is essential for robot simulation, visualization, and control. In the next module, we'll explore physics simulation and environment building with Gazebo and Unity.
</ChapterTranslator>