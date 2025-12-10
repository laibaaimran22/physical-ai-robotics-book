---
sidebar_position: 1
title: "Introduction to Gazebo for Robot Simulation"
---

import ChapterTranslator from '@site/src/components/Translation/ChapterTranslator';

<ChapterTranslator>


# Introduction to Gazebo for Robot Simulation

## Overview
This lesson gets you started with Gazebo, a powerful 3D robotics simulator. Learn to create virtual environments, import robot models, and understand the basics of physics simulation, gravity, and collision detection.

## Learning Objectives
By the end of this lesson, you should be able to:
- Understand the core concepts and architecture of Gazebo
- Launch and navigate the Gazebo simulation environment
- Create and customize simple virtual environments
- Import and spawn robot models in Gazebo
- Understand basic physics concepts in simulation

## What is Gazebo?
Gazebo is a 3D dynamic simulator with the ability to accurately and efficiently simulate populations of robots in complex indoor and outdoor environments. It provides:
- High-fidelity physics simulation using ODE, Bullet, DART, and Simbody engines
- Realistic rendering with support for multiple graphics engines
- A wide variety of sensors (LIDAR, cameras, IMUs, etc.)
- Integration with ROS/ROS 2 through gazebo_ros_pkgs

## Gazebo Architecture
Gazebo consists of three main components:
1. **Server (gzserver)**: Handles physics simulation, sensor updates, and plugin execution
2. **Client (gzclient)**: Provides the graphical user interface
3. **Plugin Interface**: Allows custom functionality to be added to the simulation

## Getting Started with Gazebo
### Launching Gazebo
To launch Gazebo with an empty world:
```bash
gz sim
# or if using ROS 2 integration:
ros2 launch gazebo_ros gazebo.launch.py
```

### Basic Interface
- **Main Menu**: Access to simulation controls, plugins, and settings
- **Toolbar**: Common actions like inserting models, setting poses, etc.
- **3D View**: The main simulation environment
- **Scene Tree**: Shows all objects in the simulation
- **Layers**: Control visibility of different elements

## Creating Virtual Environments
### Adding Models from Database
Gazebo provides a database of pre-made models:
1. Click the "Insert" tab in the top toolbar
2. Browse the online database or local models
3. Click and drag models into the simulation

### Creating Custom Worlds
World files are SDF (Simulation Description Format) files that define the entire simulation environment:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="my_world">
    <!-- Include a ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Include a sun for lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Define a custom box -->
    <model name="my_box">
      <pose>0 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </visual>
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>1.0</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>1.0</iyy>
            <iyz>0.0</iyz>
            <izz>1.0</izz>
          </inertia>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
```

### Running a Custom World
To run a simulation with a custom world file:
```bash
gz sim -r my_world.sdf
# or with ROS 2:
ros2 launch gazebo_ros empty_world.launch.py world:=/path/to/my_world.sdf
```

## Physics Simulation Concepts
### Gravity
By default, gravity is enabled and set to Earth's gravity (-9.8 m/s� in the Z direction). You can modify it in your world file:

```xml
<world name="my_world">
  <gravity>0 0 -9.8</gravity>
  <!-- rest of the world definition -->
</world>
```

### Collision Detection
Gazebo uses collision meshes to determine when objects interact. For best performance:
- Use simpler collision meshes than visual meshes
- Ensure collision meshes are properly aligned with visual meshes
- Use appropriate physics properties (mass, friction, restitution)

### Physics Engine Parameters
You can configure the physics engine in your world file:

```xml
<world name="my_world">
  <physics type="ode">
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1.0</real_time_factor>
    <real_time_update_rate>1000</real_time_update_rate>
  </physics>
</world>
```

## Importing Robot Models
### Using URDF with Gazebo
To use a URDF robot model in Gazebo, you need to add Gazebo-specific tags to your URDF or create a companion SDF file.

Example URDF with Gazebo extensions:
```xml
<robot name="my_robot">
  <!-- Your URDF content -->

  <!-- Gazebo-specific extensions -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
  </gazebo>

  <!-- Spawn the robot in simulation -->
  <gazebo>
    <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
      <ros>
        <namespace>my_robot</namespace>
      </ros>
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.4</wheel_separation>
      <wheel_diameter>0.2</wheel_diameter>
    </plugin>
  </gazebo>
</robot>
```

### Spawning Robots
To spawn a robot model in Gazebo:
```bash
# Using ROS 2
ros2 run gazebo_ros spawn_entity.py -entity my_robot -file /path/to/robot.urdf -x 0 -y 0 -z 1

# Using Gazebo directly
gz model -f /path/to/robot.sdf -m my_robot
```

## Simulation Controls
### Play/Pause
- Use the play/pause buttons in the GUI or command line to control simulation time
- `gz sim -r` runs the simulation automatically

### Time Management
- Real Time Factor (RTF) indicates how fast the simulation runs relative to real time
- Adjust physics parameters to achieve desired RTF

## Hands-On Exercise
1. Launch Gazebo:
   ```bash
   gz sim
   ```

2. Add a few models from the database (e.g., a ground plane, a box, and a robot)

3. Create a simple world file named `simple_room.sdf` with:
   - A ground plane
   - A box obstacle
   - A custom light source

4. Run your custom world:
   ```bash
   gz sim -r simple_room.sdf
   ```

5. Try modifying the physics parameters in your world file and observe the differences.

## Troubleshooting Tips
- Ensure your robot models have proper inertial properties to avoid simulation instability
- Use appropriate collision meshes for performance
- Check that joint limits and types match between URDF and Gazebo plugins
- Monitor simulation performance and adjust physics parameters as needed

## Summary
This lesson introduced the fundamentals of Gazebo for robot simulation. You learned how to create virtual environments, import robot models, and understand basic physics concepts. In the next lesson, we'll explore advanced physics and sensor simulation in Gazebo.
</ChapterTranslator>