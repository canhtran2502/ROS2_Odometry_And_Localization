# 10 - Autonomous Navigation & Nav2 Integration Roadmap

## A. Objective

This document analyzes the status of autonomous navigation within the BumperBot repository. It clearly distinguishes the perception and localization systems currently implemented from full autonomous path planning, and provides an end-to-end engineering blueprint for integrating the standard ROS 2 Navigation stack (**Nav2**).

---

## B. Repository Implementation Status: Verified Findings

A static and dynamic code inspection confirms the following boundary:

### Implemented in Repository:
1. **Perception**: 2D LiDAR range scanning (`/scan`) bridged from Gazebo.
2. **Reactive Collision Avoidance**: Autonomous 2-tier obstacle safety stop (`safety_stop_node`).
3. **Map Serving**: `nav2_map_server` integrated via lifecycle management.
4. **Global Localization**: `nav2_amcl` particle filter localization maintaining `map -> odom`.
5. **Simultaneous Localization and Mapping**: `slam_toolbox` graph-based SLAM.
6. **Actuation Execution**: `ros2_control` velocity tracking via `diff_drive_controller`.

### NOT FOUND in Repository:
- **Global Path Planners**: NavFn, Smac Planner ($A^*$, Hybrid $A^*$), or Dijkstra algorithms.
- **Local Trajectory Controllers**: DWB (Dynamic Window Approach), TEB, or Regulated Pure Pursuit.
- **Costmap Generation**: Nav2 2D Costmaps (Global Costmap, Local Costmap, inflation layers, obstacle voxel layers).
- **Behavior Trees**: Nav2 BT Navigator (`bt_navigator`), XML navigation behavior trees, or recovery servers (spin, backup, wait).
- **Navigation Launch Infrastructure**: No `nav2_bringup` or navigation execution launch files exist in the workspace.

---

## C. Fundamental Theory: The Autonomous AMR Navigation Pipeline

For an embedded engineer transitioning into AMRs/AGVs, the complete autonomous navigation stack operates as follows:

```
[Goal Pose: /goal_pose]
         |
         v
+-----------------------------------------------------------------------------------+
|                            BT NAVIGATOR (Behavior Tree)                           |
|       Supervises execution, handles recovery behaviors, retries on failure        |
+-----------------------------------------------------------------------------------+
         |                                                 |
         | (1. Requests Path)                              | (2. Dispatches Path)
         v                                                 v
+-----------------------------------+             +---------------------------------+
|       GLOBAL PLANNER SERVER       |             |     CONTROLLER SERVER (Local)   |
|   (e.g., NavFn, Smac Planner A*)  |             |  (e.g., DWB Local Planner, RPP) |
+-----------------------------------+             +---------------------------------+
         |                                                         |
         v (Samples from)                                          v (Samples from)
+-----------------------------------+             +---------------------------------+
|          GLOBAL COSTMAP           |             |          LOCAL COSTMAP          |
|  Static Map + Inflation Layer     |             |  Rolling 3x3m Window + /scan    |
+-----------------------------------+             +---------------------------------+
                                                                   |
                                                                   v
                                                     /bumperbot_controller/cmd_vel
                                                                   |
                                                                   v
                                                        [diff_drive_controller]
```

### 1. Costmap 2D Representation & Inflation
A costmap discretizes the environment into cost values $[0, 255]$:
- **`254` (Lethal Obstacle)**: Cells intersecting physical obstacles detected by sensors or the map.
- **`253` (Inscribed Obstacle)**: Cells within the robot's inscribed radius $r_{\text{inscribed}} = 0.10\text{ m}$. Collision is guaranteed if the robot origin enters this cell.
- **`1` to `252` (Inflation Decay Zone)**: Exponential cost decay modeled as:
  $$\text{cost}(d) = 253 \cdot \exp\left(-\zeta \cdot (d - r_{\text{inscribed}})\right)$$
  where $\zeta$ is the inflation scaling factor.
- **`0` (Free Space)**: Completely traversable terrain.
- **`255` (Unknown Space)**: Unobserved areas.

### 2. Global Path Planning
Computes a collision-free geometric path from current pose $(x_0, y_0)$ to target goal $(x_g, y_g)$ using Dijkstra or $A^*$ heuristic graph search across the Global Costmap.

### 3. Local Trajectory Controller (DWB / Pure Pursuit)
Runs at high frequency (typically 20 Hz) on a rolling Local Costmap ($3\text{m} \times 3\text{m}$ window). It evaluates candidate velocity pairs $(v, \omega)$ within acceleration limits, simulates forward trajectories over short horizons ($1\text{–}2\text{ s}$), scores each candidate against path alignment, goal distance, and obstacle clearance, and outputs the optimal `/cmd_vel`.

---

## D. Step-by-Step Blueprint: Integrating Nav2 with BumperBot

To extend BumperBot into a fully autonomous AMR, follow these integration steps:

### Step 1: Install Nav2 Packages
```bash
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

### Step 2: Create `nav2_params.yaml`
Create `src/bumperbot_bringup/config/nav2_params.yaml` configuring the robot footprint:
```yaml
amcl:
  ros__parameters:
    use_sim_time: True

bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_footprint
    odom_topic: /bumperbot_controller/odom

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      max_vel_x: 0.7
      min_vel_x: -0.7
      max_vel_theta: 5.0
      min_speed_xy: 0.0
      max_speed_xy: 0.7
      acc_lim_x: 0.7
      acc_lim_theta: 5.0
      decel_lim_x: -0.7
      decel_lim_theta: -5.0

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_footprint
      use_sim_time: True
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.10
      plugins: ["obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.35

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_footprint
      use_sim_time: True
      robot_radius: 0.10
      resolution: 0.05
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        observation_sources: scan
        scan:
          topic: /scan
          clearing: True
          marking: True
          data_type: "LaserScan"
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.35

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: true
      allow_unknown: true
```

### Step 3: Launch Navigation Bringup
Create a launch action incorporating Nav2 bringup in `simulated_robot.launch.py`:
```python
nav2_bringup = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory("nav2_bringup"), "launch", "navigation_launch.py")
    ),
    launch_arguments={
        "use_sim_time": "True",
        "params_file": os.path.join(get_package_share_directory("bumperbot_bringup"), "config", "nav2_params.yaml"),
    }.items(),
)
```

With this integration, developers can click the **"Nav2 Goal"** tool in RViz2, prompting BumperBot to plan global trajectories around walls and autonomously navigate to the goal while dynamically circumnavigating furniture.

