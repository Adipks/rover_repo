# Move Base Package Overview

The `move_base` package in ROS is a key component of the navigation stack, providing a high-level interface that allows a robot to autonomously navigate to a goal position while avoiding obstacles. It integrates global and local planners, costmaps, and controllers to perform efficient path planning and execution.

## What the Move Base Package Does

### 1. Goal Navigation
`move_base` enables a robot to receive a navigation goal (a specific pose in the map) and then handles the planning and movement required to reach the goal, avoiding obstacles along the way.

### 2. Path Planning
- **Global Planner**: Calculates a path from the robot's current position to the goal based on the static map, accounting for known obstacles.
- **Local Planner**: Generates real-time, short-term paths that adjust to dynamic obstacles, ensuring safe navigation using sensor data.

### 3. Costmaps
- **Global Costmap**: A static map representing the known environment and obstacles.
- **Local Costmap**: Dynamically updated based on real-time sensor data (e.g., laser or depth cameras), allowing the robot to avoid dynamic obstacles.

### 4. Obstacle Avoidance
The robot continuously updates its local costmap with sensor data to avoid obstacles while moving toward the goal.

### 5. Controller
The controller translates the planned path into velocity commands that are sent to the robot's base, allowing it to move along the desired path.

## How `move_base` Works

1. The robot receives a goal pose.
2. The **global planner** calculates a path to the goal.
3. The **local planner** refines the path in real-time, adjusting for obstacles and executing velocity commands to move the robot safely.
4. The robot’s movement is continuously monitored and adjusted based on sensor input, avoiding obstacles while following the global path.

## How to Configure Move Base

### 1. Move Base Parameters

The main configuration file, typically `move_base_params.yaml`, sets the core parameters for `move_base`:

```yaml
base_global_planner: "navfn/NavfnROS"            # Global planner plugin
base_local_planner: "dwa_local_planner/DWAPlannerROS"  # Local planner plugin
controller_frequency: 10.0                       # Frequency of velocity command updates
planner_frequency: 5.0                           # Frequency of re-planning
```

### 2. Costmap Configuration

Separate parameter files configure the global and local costmaps, such as `global_costmap_params.yaml` and `local_costmap_params.yaml`.

#### Example Global Costmap Configuration:
```yaml
global_costmap:
  global_frame: "map"
  robot_base_frame: "base_link"
  update_frequency: 5.0
  static_map: true
```

#### Example Local Costmap Configuration:
```yaml
local_costmap:
  global_frame: "odom"
  robot_base_frame: "base_link"
  update_frequency: 5.0
  rolling_window: true
  width: 3.0
  height: 3.0
```

### 3. Global Planner Configuration

Example `NavfnROS` configuration for global planning:

```yaml
NavfnROS:
  allow_unknown: true               # Enable path planning through unknown areas
  planner_window: 0.5
  default_tolerance: 0.5
```

### 4. Local Planner Configuration (DWA or TEB)

#### Example DWA Local Planner Configuration:
```yaml
DWAPlannerROS:
  max_vel_x: 0.5                    # Maximum forward velocity
  min_vel_x: 0.1
  max_vel_theta: 1.0                # Maximum rotational velocity
  min_vel_theta: 0.1
  acc_lim_x: 2.5                    # Acceleration limit in X direction
  acc_lim_theta: 3.2                # Acceleration limit in rotational direction
```

### 5. Launching Move Base

Create a launch file to run `move_base`, like this:

```xml
<launch>
  <node pkg="move_base" type="move_base" name="move_base" output="screen">
    <param name="base_global_planner" value="navfn/NavfnROS"/>
    <param name="base_local_planner" value="dwa_local_planner/DWAPlannerROS"/>
    <rosparam file="$(find my_robot_navigation)/config/move_base_params.yaml" command="load"/>
    <rosparam file="$(find my_robot_navigation)/config/global_costmap_params.yaml" command="load"/>
    <rosparam file="$(find my_robot_navigation)/config/local_costmap_params.yaml" command="load"/>
  </node>
</launch>
```

## Documentation

For more detailed information and configuration options, refer to the official ROS Wiki page:
- [move_base on ROS Wiki](https://wiki.ros.org/move_base)

---
The `move_base` package is essential for autonomous robot navigation, integrating global and local path planning with obstacle avoidance to achieve smooth, efficient motion to goal positions in both known and unknown environments.
