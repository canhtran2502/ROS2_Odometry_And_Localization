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

