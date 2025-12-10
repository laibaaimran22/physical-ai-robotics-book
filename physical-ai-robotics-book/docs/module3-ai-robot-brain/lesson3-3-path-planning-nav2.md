---
sidebar_label: 'Lesson 3.3: Path Planning for Bipedal Humanoid Movement with Nav2'
---

import ChapterTranslator from '@site/src/components/Translation/ChapterTranslator';

<ChapterTranslator>

# Lesson 3.3: Path Planning for Bipedal Humanoid Movement with Nav2

## Overview
This lesson covers the Nav2 stack for path planning specifically tailored for bipedal humanoid robots. You'll learn to configure global and local planners for the unique challenges of humanoid locomotion.

## Learning Objectives
By the end of this lesson, you should be able to:
- Understand Nav2's architecture and components for humanoid navigation
- Configure global and local planners for bipedal movement
- Implement footstep planning for stable humanoid locomotion
- Integrate Nav2 with Isaac Sim and Isaac ROS for humanoid navigation

## Introduction to Nav2 for Humanoids
Navigation2 (Nav2) is ROS 2's state-of-the-art navigation framework. For humanoid robots, navigation presents unique challenges:
- **Stability**: Maintaining balance during locomotion
- **Footstep Planning**: Planning stable foot placements
- **Dynamic Balance**: Adjusting to changing center of mass

### Key Nav2 Components for Humanoids
- **Global Planner**: Computes optimal path considering stability constraints
- **Local Planner**: Adjusts trajectory for immediate obstacles and balance
- **Controller**: Executes footstep commands for bipedal locomotion
- **Behavior Trees**: Manages navigation recovery behaviors

## Setting Up Nav2 for Humanoid Robots

### Nav2 Configuration for Humanoid Locomotion
```yaml
# config/humanoid_nav2_params.yaml
bt_navigator:
  ros__parameters:
    use_sim_time: true
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_xml_filename: "navigate_w_replanning_and_recovery.xml"
    default_server_timeout: 20
    # Footstep planning specific parameters
    goal_checker:
      plugin: "nav2_behavior_tree::GoalChecker"
      xy_goal_tolerance: 0.3  # Larger tolerance for humanoid precision
      yaw_goal_tolerance: 0.5 # Allow more rotational tolerance

controller_server:
  ros__parameters:
    use_sim_time: true
    controller_frequency: 10.0  # Lower frequency for humanoid stability
    min_x_velocity_threshold: 0.05
    min_y_velocity_threshold: 0.05
    min_theta_velocity_threshold: 0.05
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["HumanoidMppiController"]

    # Humanoid-specific MPPI Controller
    HumanoidMppiController:
      plugin: "nav2_mppi_controller::MppiController"
      time_steps: 15              # Increased horizon for stability
      control_horizon: 8          # Control horizon for humanoid
      time_delta: 0.2             # Slower control for balance
      discretization: 0.2         # Larger steps for humanoid
      # Cost weights for humanoid-specific considerations
      cost_obstacles: 2.0         # Higher weight for obstacle avoidance
      cost_goal_dist: 1.0
      cost_path_align: 0.5
      cost_goal_angle: 0.2
      cost_nonholonomic: 0.1
      # Humanoid-specific costs
      cost_balance: 3.0           # Penalty for balance violations
      cost_foot_placement: 2.5    # Penalty for unstable foot placement

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: true
      rolling_window: true
      width: 6  # Larger window for humanoid awareness
      height: 6
      resolution: 0.05
      robot_radius: 0.4  # Larger radius for humanoid footprint
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 5.0  # Higher inflation for humanoid safety
        inflation_radius: 0.8     # Larger inflation for stability

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 0.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: true
      robot_radius: 0.4
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"
          raytrace_max_range: 5.0  # Longer range for humanoid planning
          obstacle_max_range: 4.0
```


</ChapterTranslator>