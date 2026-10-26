# 10 - Autonomous Navigation & Nav2 Integration Roadmap

## A. Objective

This document describes autonomous navigation in the BumperBot repository: the Nav2 stack configured in the `bumperbot_navigation` package, the AGV waypoint **mission manager** built on top of it (sections A2 to A7), and the Nav2 background and original integration blueprint (sections B to D).

---

## A2. Nav2 and the Mission Manager (`bumperbot_navigation`)

The `bumperbot_navigation` package (`ament_cmake` + `ament_cmake_python`) contains:

| Item | Purpose |
|---|---|
| `mission_manager` (C++) / `mission_manager.py` (Python) | `/execute_mission` action server, Nav2 `navigate_to_pose` client, `/mission/status` publisher. Same parameters and behavior in both languages |
| `mission_logic` (C++ library, no rclcpp) / `mission_logic.py` (no rclpy) | Station loading and the `MissionTracker` state machine; both languages are checked against the shared `test/data/mission_cases.yaml` |
| `config/stations.yaml` | Named stations on the saved `small_house` map, including the `dock` |
| `config/mission_manager.yaml` | Node parameters |
| `config/nav2_params.yaml` | Nav2 parameters for BumperBot |
| `launch/navigation.launch.py` | `controller_server`, `planner_server`, `behavior_server`, `bt_navigator` and `lifecycle_manager_navigation` |

Nav2 uses NavFn (A*) as planner, Regulated Pure Pursuit as controller (0.2 m/s, rotate to heading), the default `navigate_to_pose` behavior tree with the spin/backup/wait recoveries, `robot_base_frame: base_footprint`, `odom_topic: /bumperbot_controller/odom`, `robot_radius: 0.12` and an inflation radius of 0.35 m. The goal checker tolerance (`xy_goal_tolerance: 0.10`) is below the 0.2 m mission tolerance, so AMCL noise has margin. `map_server` and `amcl` are not configured here; they come from `bumperbot_localization` (`global_localization.launch.py`).

`controller_server` and `behavior_server` publish `cmd_vel` remapped to `nav_vel`, a `twist_mux` input (`navigation`, priority 20). Priorities: joystick 99 > keyboard 90 > navigation 20 > exploration 10; the `safety_stop` lock (255) wins over all. On the real robot nothing publishes on `nav_vel`; `navigation.launch.py` is not wired into `real_robot.launch.py`.

```
ros2 action send_goal /execute_mission {stations: [pickup, dropoff]}
        |
        v
 stations.yaml --> mission_manager (C++ | Python, MissionTracker) --> /mission/status, feedback, result
        | NavigateToPose goal            ^ TF map -> base_footprint (tolerance check)
        v                                |
 bt_navigator -> planner_server / controller_server / behavior_server
        | nav_vel (priority 20)
        v
 twist_mux (+ safety_stop lock 255) -> /bumperbot_controller/cmd_vel_unstamped -> diff_drive_controller
```

## A3. Running a Mission

```bash
ros2 launch bumperbot_bringup mission_sim.launch.py simulator:=mujoco world_name:=small_house
ros2 action send_goal /execute_mission bumperbot_msgs/action/ExecuteMission "{stations: [pickup, dropoff]}" --feedback
ros2 topic echo /mission/status
```

`mission_sim.launch.py` (arguments `simulator`, `world_name`, `use_rviz`, `viewer`, `use_python`, `stations_file`, `mission_config`) includes `simulated_robot.launch.py` with `use_slam:=False` (map_server + AMCL on `small_house`), `navigation.launch.py` with `use_sim_time:=true` and one `mission_manager` node (C++ unless `use_python:=true`). The robot spawns at (0, 0, 0) and AMCL starts there, so the dock is the spawn pose. The real-robot launch path is not modified.

## A4. `stations.yaml` Format

```yaml
frame_id: map            # frame of the station poses
map: small_house         # informative: map the stations were chosen on
dock_station: dock       # name of the dock station (must exist)
stations:
  dock:    {x: 0.0,  y: 0.0,  yaw: 0.0,  dwell: 0.0}
  pickup:  {x: -1.5, y: 1.5,  yaw: 1.57, dwell: 3.0}
```

`x`, `y` in m, `yaw` in rad, `dwell` in s. Loading fails (`ValueError` / `std::invalid_argument`) when the file is missing or not a mapping, `stations` is missing or empty, the dock is not defined, a field is missing, not a number or not finite, `dwell < 0`, a station name or any other mapping key is duplicated, or `frame_id` / `dock_station` is present but not a non-null string. Numeric fields must be plain YAML numbers (int or float, exponent forms such as `1e3` included); quoted strings like `"1.5"` are rejected in both implementations. `mission_manager` also refuses to start when `frame_id` differs from its `global_frame` parameter, because the tolerance check compares the station coordinates with the TF in `global_frame`. Every station of the shipped file has at least 0.30 m clearance on the `small_house` map and lies in the free area connected to the dock.

## A5. Mission Rules

- Goal: an ordered list of station names. An empty list, an unknown station name or a second goal while a mission is active is rejected.
- For each station: send a `NavigateToPose` goal (frame `map`, yaw to quaternion), wait for the Nav2 result, then check with TF `map -> base_footprint` that the planar distance to the station is at most `goal_tolerance` (0.2 m). A Nav2 SUCCEEDED with a larger distance (or an unavailable TF) counts as a failed attempt. After the check the robot dwells `dwell` seconds.
- Nav2 not ready: a goal REJECTED by the `NavigateToPose` server is treated as "server not ready" (for example the lifecycle is not active yet). The goal is re-sent every `retry_delay` seconds without counting a failed attempt; only after `nav_server_timeout` of continuous rejection does it count as a failure.
- Dead Nav2 mid-goal: while waiting for the result, if the `NavigateToPose` server stays unavailable for longer than `nav_server_timeout` (wall) the attempt fails at once (no cancel is possible) instead of waiting for `nav_goal_timeout`; it counts as a failed attempt.
- Shutdown: on Ctrl+C / SIGTERM (or node destruction) while a Nav2 goal is active, a best-effort cancel of that goal is sent (it cannot throw) and the C++ mission thread (a joinable `std::thread`) is joined before the node exits, so it never outlives the node and `std::terminate` is not reached. The mission result is ABORTED with message `node shutting down`.
- Retries: a failed attempt is repeated after `retry_delay` seconds, up to `max_retries` extra attempts; then the station is skipped and the mission goes on.
- Abort: `max_consecutive_failures` consecutive skipped stations abort the mission (the remaining stations are not tried; 0 disables this).
- Dock: after the last station (`return_to_dock`), after an abort and after a cancel the robot navigates to the dock station (skipped only when the robot is currently at the dock: it reached the dock and has not started driving to another station since), with up to `dock_max_retries` extra attempts. If the dock cannot be reached the outcome is `OUTCOME_DOCK_FAILED` and `docked` is false.
- Cancel: the active Nav2 goal is canceled, the dwell is stopped and the robot returns to the dock; the action ends CANCELED. Terminal status precedence: a cancel that is in progress yields action status CANCELED even if the dock return then fails (the `outcome` field still carries `DOCK_FAILED`). The action stays EXECUTING while the robot drives back (an AGV must not stop in the aisle); a second cancel is ignored.
- Result: `outcome` (COMPLETED 0, COMPLETED_WITH_SKIPS 1, ABORTED 2, CANCELED 3, DOCK_FAILED 4), `docked`, `visited`, `skipped`, `message`. Final action status: SUCCEEDED for outcomes 0 and 1, ABORTED for 2 and 4, CANCELED for 3.
- `/mission/status` (`bumperbot_msgs/msg/MissionStatus`, reliable + transient_local): `IDLE`, `NAVIGATING`, `DWELLING`, `RETURNING_TO_DOCK`, `COMPLETED`, `ABORTED`, `CANCELED`, `DOCK_FAILED`, with the current station, index, attempt, progress and the visited/skipped lists; published at start, on every change and at `status_rate` Hz. Action feedback carries the same content plus `distance_remaining`.

Parameters (`config/mission_manager.yaml`): `stations_file` (empty = package default), `nav_action_name`, `global_frame`, `robot_frame`, `goal_tolerance`, `max_retries`, `retry_delay`, `max_consecutive_failures`, `dock_max_retries`, `return_to_dock`, `nav_server_timeout`, `nav_goal_timeout`, `tf_timeout`, `clock_stall_timeout` (30.0 s wall; node-clock waits end after this long without the clock advancing; 0 disables), `status_rate`, `behavior_tree`.

## A6. Simulator Independence

The mission manager depends only on the `navigate_to_pose` action, TF and its own interfaces; it does not reference Gazebo or MuJoCo. Clocks, exactly:
  - Node clock (sim time when `use_sim_time=true`): the dwell, `retry_delay` (also the retry delay while Nav2 rejects goals and the docking retry delay) and the `/mission/status` timer (`create_timer` in both C++ and Python, so the status stops and starts with sim time identically). These waits are interruptible by a cancel (docking waits excepted: the dock return must finish) and bounded by a wall-clock fallback: if the node clock does not advance for `clock_stall_timeout` wall seconds (for example `/clock` stalled) the wait counts as elapsed and a warning is logged.
  - Wall (steady) time: `nav_goal_timeout`, `nav_server_timeout`, `tf_timeout` and `clock_stall_timeout` itself.

## A7. Tests

`colcon test --packages-select bumperbot_navigation bumperbot_bringup`: unit tests for the station loading and the `MissionTracker` (pytest and gtest, shared case table), a map check of the stations, launch tests of both node implementations against a mock `NavigateToPose` server, and a headless MuJoCo smoke test. The full-mission end-to-end test is opt-in (`BUMPERBOT_MISSION_E2E=1`).

---

---

## B. Repository Implementation Status: Verified Findings

The following subsystems are implemented in the repository:

1. **Perception**: 2D LiDAR range scanning (`/scan`) bridged from Gazebo.
2. **Reactive Collision Avoidance**: Autonomous 2-tier obstacle safety stop (`safety_stop_node`).
3. **Map Serving**: `nav2_map_server` integrated via lifecycle management.
4. **Global Localization**: `nav2_amcl` particle filter localization maintaining `map -> odom`.
5. **Simultaneous Localization and Mapping**: `slam_toolbox` graph-based SLAM.
6. **Actuation Execution**: `ros2_control` velocity tracking via `diff_drive_controller`.

7. **Autonomous Exploration (without Nav2)**: `frontier_explorer` (`bumperbot_mapping`) uses its own grid A* planner on an inflated costmap and a pure-pursuit follower to explore an unknown map with `slam_toolbox`; launched with `auto_explore.launch.py`. It does not depend on Nav2. See [08_mapping_and_slam.md](08_mapping_and_slam.md).

8. **Nav2 Navigation and Waypoint Missions**: `bumperbot_navigation` configures the Nav2 planner (NavFn), controller (Regulated Pure Pursuit), behavior server and `bt_navigator`, and adds the `mission_manager` (section A2). The exploration feature above still does not use Nav2.

> Note: the sections C and D below are the original Nav2 background and integration blueprint (DWB, a `nav2_params.yaml` in `bumperbot_bringup`, `nav2_bringup` included from `simulated_robot.launch.py`). The implemented configuration differs: it lives in `bumperbot_navigation` (section A2), uses Regulated Pure Pursuit, and is started by `navigation.launch.py` / `mission_sim.launch.py`, not by `simulated_robot.launch.py`.

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

