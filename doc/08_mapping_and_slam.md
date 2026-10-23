# 08 - Mapping & SLAM (Simultaneous Localization and Mapping)

## A. Objective

This document covers the theory, algorithms, and practical operations of 2D Occupancy Grid Mapping and Graph-Based SLAM implemented in BumperBot. It compares the custom known-pose ray mapper with the production-grade `slam_toolbox` system.

---

## B. 2D Occupancy Grid Mapping Theory

### 1. Discrete Spatial Representation
An occupancy grid represents continuous 2D space as a regular lattice of square cells:
- **Map Parameters (`nav_msgs/msg/MapMetaData`)**:
  - `resolution`: Physical width of each cell side (e.g., $0.05\text{ m/pixel}$ or $0.10\text{ m/pixel}$).
  - `width` / `height`: Grid dimensions in integer cells ($W, H$).
  - `origin`: Continuous $(x, y, z)$ position of the bottom-left corner cell $(0, 0)$ relative to the map frame.

### 2. World-to-Cell Coordinate Conversions
Given continuous coordinates $(p_x, p_y)$ in the `map` or `odom` frame:
$$x_{\text{cell}} = \text{round}\left(\frac{p_x - \text{origin.position.x}}{\text{resolution}}\right)$$
$$y_{\text{cell}} = \text{round}\left(\frac{p_y - \text{origin.position.y}}{\text{resolution}}\right)$$

Cell boundary validity check:
$$0 \le x_{\text{cell}} < W \quad \text{and} \quad 0 \le y_{\text{cell}} < H$$

Linear 1D index conversion in row-major order:
$$\text{index} = y_{\text{cell}} \cdot W + x_{\text{cell}}$$

### 3. Log-Odds Formulation
In standard probabilistic mapping, cell states are maintained in log-odds form:
$$l(m_i) = \log \frac{P(m_i = 1)}{1 - P(m_i = 1)}$$
Updates given measurement $z_t$ additively accumulate:
$$l_t(m_i) = l_{t-1}(m_i) + \text{inv\_sensor\_model}(m_i, z_t) - l_0$$
In ROS 2 `OccupancyGrid`:
- `-1`: Unknown space (unexplored).
- `0`: Free space (cleared by ray tracing).
- `100`: Occupied space (obstacle detected).

---

## C. Known-Pose Mapping Implementation (`mapping_with_known_poses`)

Implemented in `src/bumperbot_mapping/src/mapping_with_known_poses.cpp` and `mapping_with_known_poses.py`:

```
               [LiDAR Scan (/scan)]
                        |
                        v
        [lookup_transform("odom", "laser_link", scan.header.stamp)]
                        |
                        v (Exact robot pose: x, y, yaw at scan time)
     +------------------------------------------------------+
     | For each beam i in 0..359:                           |
     |   Validate r in [range_min, range_max]               |
     |   angle = angle_min + i * angle_increment + yaw     |
     |   px = r * cos(angle) + translation.x                |
     |   py = r * sin(angle) + translation.y                |
     |   cell = coordinatesToPose(px, py)                   |
     |   map.data[cell] = 100                               |
     +------------------------------------------------------+
                        |
                        v (1.0 Hz Timer)
          [Publish /map (nav_msgs/OccupancyGrid)]
```

### Key Engineering Details:
1. **Timestamp Synchronization**:
   Uses `Time.from_msg(scan.header.stamp)` with a 50ms timeout to interpolate the exact historical robot pose from `tf_buffer`.
2. **Multi-Threaded Concurrency**:
   Uses `MultiThreadedExecutor` so that waiting on TF lookups does not starve the `/tf` subscriber callback queue.
3. **Boundary Checking**:
   Validates `poseOnMap(beam_p)` to eliminate buffer overflow exceptions when beams hit beyond the $50\text{m} \times 50\text{m}$ grid bounds.

---

## D. Graph-Based SLAM (`slam_toolbox`)

For unknown environments where odometry drifts, BumperBot uses `slam_toolbox` (configured in `src/bumperbot_mapping/config/slam_toolbox.yaml`).

### 1. Mathematical Formulation
Graph SLAM constructs a pose graph consisting of:
- **Nodes $\mathbf{x}_i$**: Robot poses at keyframes.
- **Edges $\mathbf{z}_{ij}$**: Spatial transformation constraints between nodes derived from odometry or laser scan-matching.

The error function minimized across all constraints is:
$$\min_{\mathbf{x}} \sum_{ij} \mathbf{e}_{ij}(\mathbf{x}_i, \mathbf{x}_j)^T \mathbf{\Omega}_{ij} \mathbf{e}_{ij}(\mathbf{x}_i, \mathbf{x}_j)$$
where $\mathbf{\Omega}_{ij}$ is the information matrix (inverse covariance).

### 2. Configuration Highlights (`slam_toolbox.yaml`)
- **Nonlinear Solver**: Google Ceres Solver (`solver_plugins::CeresSolver`).
- **Matrix Factorization**: Sparse Normal Cholesky (`SPARSE_NORMAL_CHOLESKY`).
- **Optimization Strategy**: Levenberg-Marquardt (`LEVENBERG_MARQUARDT`).
- **Map Resolution**: $0.05\text{ m}$ per cell.
- **Maximum Laser Range**: $12.0\text{ m}$.
- **Loop Closure**:
  - `do_loop_closing: true`
  - `loop_search_maximum_distance: 3.0 m`
  - `loop_match_minimum_chain_size: 10`
  - `loop_match_minimum_response_fine: 0.45`
- **Frames**: `odom_frame: odom`, `map_frame: map`, `base_frame: base_footprint`, `scan_topic: /scan`.

---

## E. End-to-End Mapping & Map Saving Workflow

### Step 1: Launch Simulation with SLAM Enabled
In Terminal 1:
```bash
ros2 launch bumperbot_bringup simulated_robot.launch.py world_name:=small_house use_slam:=True
```
*Expected Behavior*: Ignition Gazebo loads `small_house.world`, spawns BumperBot, starts `slam_toolbox`, and opens RViz2 configured with the SLAM display layout.

### Step 2: Teleoperate the Robot to Explore
In Terminal 2:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/key_vel
```
Drive the robot smoothly through the rooms of the house to close loops and populate the occupancy grid.

### Step 3: Save the Generated Map
In Terminal 3:
```bash
# Save using Nav2 map_saver CLI utility
ros2 run nav2_map_server map_saver_cli -f ~/my_bumperbot_map --ros-args -p use_sim_time:=True
```
This writes:
1. `~/my_bumperbot_map.yaml` (Metadata, thresholds, resolution)
2. `~/my_bumperbot_map.pgm` (Portable Graymap raster image)

---

## E2. Autonomous Frontier Exploration (`frontier_explorer`)

The robot can explore an unknown, closed environment by itself while `slam_toolbox` builds the map. No Nav2 planner, controller or behavior tree is used.

```bash
ros2 launch bumperbot_bringup auto_explore.launch.py simulator:=mujoco world_name:=small_house
# args: simulator (gazebo|mujoco), world_name, use_rviz, viewer, map_save_path, explorer_config
```

`auto_explore.launch.py` includes `simulated_robot.launch.py` with `use_slam:=True` and starts `frontier_explorer` with `config/frontier_explorer.yaml`.

### Algorithm
1. **Inputs**: `/map` (slam_toolbox), the pose from TF `map -> base_footprint`, and `/scan`. The laser yaw offset is read from TF (`base_footprint -> laser_link`); the lidar is mounted backwards (yaw ~ pi), so it is never hard-coded.
2. **Frontiers**: free cells with an unknown 4-neighbour, clustered 8-connected; clusters smaller than `min_frontier_size` cells are dropped.
3. **Costmap**: cells closer than `inflation_radius` (0.30 m) to an occupied cell, and unknown cells, are not traversable.
4. **Goal selection**: for each cluster, the reachable traversable cell closest to the centroid; clusters are ranked by `distance - size_weight * size`; grid A* (8-connected, no corner cutting) runs on the best 5 until one gives a path. Reached or failed goals are blacklisted (`goal_blacklist_radius`).
5. **Following**: pure pursuit (`lookahead_distance`), rotate in place above `rotate_in_place_angle`.
6. **Safety layer**: on `/scan`, stricter than `safety_stop_node` (front stop at 0.25 m vs. its 0.2 m danger zone, speed scaled down below 0.45 m, recovery by reversing or turning to the freer side). The `safety_stop` lock (priority 255) still overrides everything.
7. **Start**: one 360 degree turn (`initial_spin`) seeds the map.

### Termination and map saving
Exploration is complete when no reachable frontier of meaningful size remains for `completion_hysteresis` consecutive replans AND `completion_timeout` seconds (or `max_exploration_time` is reached). The node then publishes zero velocity for 1 s, calls `/map_saver/save_map` (`nav2_map_server`, already started by `slam.launch.py`) with `map_save_path` (default `~/bumperbot_maps/explored_map`, directory created), falls back to `/slam_toolbox/save_map`, and reports the result.

| Topic | Type | Meaning |
|---|---|---|
| `explore_vel` | `geometry_msgs/Twist` | Velocity command for `twist_mux` (priority 10, timeout 0.5 s) |
| `/explore/status` | `std_msgs/String` (latched) | `WAITING_FOR_MAP`, `EXPLORING`, `COMPLETE`, `MAP_SAVED`, `MAP_SAVE_FAILED` |
| `/explore/path` | `nav_msgs/Path` | Current A* path (RViz) |
| `/explore/frontiers` | `visualization_msgs/MarkerArray` | Frontier centroids and the selected goal |

### Taking over manually
The joystick (`joy_vel`, 99) and the keyboard (`key_vel`, 90) have a higher `twist_mux` priority than the explorer, so moving the stick or pressing a key takes over immediately; the explorer resumes after the 0.5 s timeout once the manual input stops.

### Implementation notes
The algorithms are in `bumperbot_mapping/frontier_exploration.py` (numpy + scipy, no rclpy), and `frontier_explorer.py` is a thin ROS wrapper. The node is Python-only: it is new, and the pure module is shared with the headless pytest and MuJoCo tests, which a C++ port could not run. A C++ port would be a separate feature. `slam_toolbox.yaml` is unchanged (`map_update_interval: 5.0` makes frontier updates slow, so the explorer keeps following its path between map updates).

---

## F. Pre-Built Map Verification

The workspace includes a pre-mapped small house asset in `src/bumperbot_mapping/maps/small_house/`:
- **`map.yaml`**:
  ```yaml
  image: map.pgm
  resolution: 0.050000
  origin: [-12.500000, -12.500000, 0.000000]
  negate: 0
  occupied_thresh: 0.65
  free_thresh: 0.196
  ```
- **`map.pgm`**: $500 \times 500$ pixel raster corresponding to a $25\text{m} \times 25\text{m}$ physical coverage area.

