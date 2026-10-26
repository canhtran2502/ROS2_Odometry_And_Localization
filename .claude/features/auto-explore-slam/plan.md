# Autonomous frontier exploration with SLAM

## Goal

The robot explores an unknown, closed environment on its own while `slam_toolbox` builds a new map.
A self-contained `frontier_explorer` node (no Nav2 planners, controller server or bt_navigator):
1. reads `/map` (slam_toolbox), the robot pose from TF `map -> base_footprint` and `/scan`;
2. detects and clusters frontiers (free cells next to unknown cells), picks the best reachable one,
   plans a grid A* path on an inflated costmap, and follows it with a pure-pursuit controller;
3. keeps a reactive safety layer on `/scan`, independent of (and stricter than) `safety_stop_node`;
4. publishes `geometry_msgs/Twist` on `explore_vel`, a new low-priority `twist_mux` input, so the
   joystick, the keyboard and the `safety_stop` lock can always override it;
5. when no reachable frontier of meaningful size is left (with hysteresis), it publishes zero
   velocity, saves the map through `nav2_map_server`'s `map_saver_server` (already started by
   `slam.launch.py`), falls back to `slam_toolbox/save_map`, and reports completion on a latched
   status topic and in the log.

It runs in simulation on Gazebo and on MuJoCo through one launch file. The real-robot launch path is
not modified (the only shared change is one extra, unused `twist_mux` input).

The algorithms live in a pure Python module (numpy + scipy, no rclpy), so pytest can test them
headless. The ROS node is a thin wrapper around this module.

### Python-only justification (C++/Python parity rule)

The CLAUDE.md rule "keep C++ and Python versions of the same node in sync" covers nodes that
already exist in both languages. This is a new node, and the repo already has Python-only nodes
(`mpu6050_driver.py`, `mujoco_sim_node`). The exploration logic is shared on purpose with the headless
pytest and MuJoCo tests, which cannot run a C++ port. A second implementation would double the
maintenance and could not be tested by the same suite. So the node is Python-only. This is written
down in `doc/03` and in the module docstring. A C++ port would be a separate feature, with the pure
module as its reference.

### Amendment (iteration 3): SLAM map quality

AC1-AC8 pass, but a manual run of `auto_explore.launch.py simulator:=mujoco` produced a warped
SLAM map (copy in `.claude/features/auto-explore-slam/explored_map.pgm`): the walls fan out by about
+-8-15 deg and there is a seam near x = -2.3 m. No AC checked SLAM map quality. This amendment:
1. adds an end-to-end SLAM map quality test (AC9) with a reusable pure-Python map comparison helper;
2. fixes the root cause found in the investigation, a MuJoCo contact-model bug that gives the wheel
   odometry a large heading scale error, and pins it with a sim-level drift test (AC10).

## Architecture

### Nodes

| Package | Language | Executable / node name | Role |
|---|---|---|---|
| `bumperbot_mapping` | Python (new) | `frontier_explorer.py` / `frontier_explorer` | ROS wrapper: subscriptions, TF, timers, publishers, map-save client |
| `bumperbot_mapping` | Python module (new, no rclpy) | `bumperbot_mapping/frontier_exploration.py` | Core algorithms + `ExplorerCore` state machine |
| `slam_toolbox` | existing | `slam_toolbox` | builds `/map`, publishes `map -> odom` |
| `nav2_map_server` | existing | `map_saver_server` | `/map_saver/save_map` service (already in `slam.launch.py`) |
| `twist_mux` | existing | `twist_mux` | new input `explore` (`explore_vel`, priority 10) |
| `bumperbot_utils` | existing | `safety_stop_node` | unchanged; its lock (priority 255) still overrides everything |

### Interfaces of `frontier_explorer`

Subscriptions:
- `/map` `nav_msgs/msg/OccupancyGrid`. QoS reliable + transient_local (matches slam_toolbox).
- `/scan` `sensor_msgs/msg/LaserScan`, sensor-data QoS.
- TF: `map -> base_footprint` (pose, looked up every control tick), and the static
  `base_footprint -> laser_link` (once) to get the laser yaw offset. In the URDF `laser_joint` has
  `rpy="0 0 3.14"`, so the lidar is mounted backwards and the offset must NOT be hard-coded.

Publications:
- `explore_vel` `geometry_msgs/msg/Twist` (param `cmd_vel_topic`) at `control_rate` (10 Hz) while
  exploring. After completion it publishes zeros for 1 s, then stops publishing so the mux input
  times out.
- `/explore/status` `std_msgs/msg/String`, transient_local, depth 1. Values: `WAITING_FOR_MAP`,
  `EXPLORING`, `COMPLETE`, `MAP_SAVED`, `MAP_SAVE_FAILED`.
- `/explore/path` `nav_msgs/msg/Path` (frame `map`), the current A* path, for RViz.
- `/explore/frontiers` `visualization_msgs/msg/MarkerArray`, cluster centroids and the selected goal.

Service clients:
- `/map_saver/save_map` `nav2_msgs/srv/SaveMap` (primary; `map_url` = expanded
  `map_save_path`, `map_topic` = `map`, `image_format` = `pgm`, `map_mode` = `trinary`,
  thresholds 0.65 / 0.196).
- `/slam_toolbox/save_map` `slam_toolbox/srv/SaveMap` (fallback if the primary is not available
  within `save_map_timeout`).

Parameters (in `bumperbot_mapping/config/frontier_explorer.yaml`, all also declared in code):

| Name | Default | Meaning |
|---|---|---|
| `global_frame` / `robot_frame` | `map` / `base_footprint` | TF frames |
| `cmd_vel_topic` | `explore_vel` | twist_mux input |
| `control_rate` | 10.0 | Hz, follower and safety loop |
| `replan_period` | 2.0 | s, replan on the latest map |
| `robot_radius` | 0.12 | m |
| `inflation_radius` | 0.30 | m; cells closer than this to an obstacle are not traversable |
| `unknown_is_obstacle` | true | the planner only crosses known free cells |
| `min_frontier_size` | 8 | cells (0.4 m at 0.05 m/cell) |
| `goal_blacklist_radius` | 0.5 | m |
| `progress_timeout` | 20.0 | s without 0.1 m of progress, then blacklist the goal |
| `max_linear_speed` / `max_angular_speed` | 0.15 / 0.8 | m/s, rad/s |
| `lookahead_distance` | 0.35 | m, pure pursuit |
| `rotate_in_place_angle` | 0.8 | rad; rotate without translating above this heading error |
| `goal_tolerance` | 0.25 | m |
| `stop_distance` | 0.25 | m, front-sector stop (larger than safety_stop's 0.2 m danger zone) |
| `slow_distance` | 0.45 | m, front-sector speed scaling |
| `front_sector_half_angle` | 0.6 | rad |
| `initial_spin` | true | one 360 deg turn at start to seed the map |
| `completion_hysteresis` | 3 | consecutive replans with no valid frontier |
| `completion_timeout` | 10.0 | s; minimum time with no valid frontier before COMPLETE |
| `max_exploration_time` | 0.0 | s, 0 = unlimited; when reached: stop, save, then COMPLETE |
| `save_map_on_complete` | true | |
| `map_save_path` | `~/bumperbot_maps/explored_map` | expanded with `os.path.expanduser` in the node; the directory is created |
| `save_map_timeout` | 5.0 | s |
| `shutdown_on_complete` | false | exit the node after saving |
| `use_sim_time` | true from launch | |

### Pure module `frontier_exploration.py` (no rclpy; numpy + scipy.ndimage)

- `GridInfo(resolution, origin_x, origin_y, width, height)`, `world_to_cell`, `cell_to_world`.
- `grid_from_msg_data(data, width, height) -> np.int8 (H, W)` (row-major, same as OccupancyGrid).
- `build_costmap(grid, inflation_cells, unknown_is_obstacle) -> bool traversable mask`
  (`scipy.ndimage.distance_transform_edt` on occupied cells, value >= 50 counts as occupied).
- `detect_frontiers(grid) -> bool mask`: free (0..49) cell with at least one unknown (-1)
  4-neighbour.
- `cluster_frontiers(mask, min_size) -> list[Frontier(cells, size, centroid_cell)]`
  (8-connected `scipy.ndimage.label`).
- `reachable_mask(traversable, start_cell)`: connected component of the start. If the start cell is
  inflated, first snap it to the nearest traversable cell within `inflation_radius + 1` cells
  (BFS), so a robot near a wall can still plan.
- `astar(traversable, start, goal) -> list[cell] | None`: 8-connected, octile heuristic, no corner
  cutting.
- `select_goal(frontiers, traversable, reachable, robot_cell, blacklist, ...) -> (goal_cell, path) | None`:
  for each cluster, the goal is the reachable traversable cell closest to its centroid (search
  within 1 m). Clusters are ranked by `distance - size_weight * size`, and A* runs on the top K=5
  until one gives a path.
- `simplify_path(path, grid_info, spacing) -> list[(x, y)]` in world coordinates.
- `PurePursuit.compute(pose, path) -> (v, w, done)`: lookahead point, rotate in place when the
  heading error is above `rotate_in_place_angle`, otherwise curvature control, v and w limits.
- `scan_safety(ranges, angle_min, angle_increment, laser_yaw_offset, ...) -> (scale, blocked, min_all, escape_dir)`:
  computes beam angles in the base frame (wrap-safe), the front-sector minimum, speed scaling,
  blocked flag, and a recovery direction (sign of the freer side, or reverse when the obstacle is in
  front and the rear is clear).
- `CompletionMonitor(hysteresis, timeout).update(has_frontier, t) -> bool`; resets as soon as a
  valid frontier comes back.
- `ExplorerCore(params)`: state machine `WAITING_FOR_MAP -> INITIAL_SPIN -> EXPLORING -> COMPLETE`.
  - `update_map(grid, info, t)` stores the grid and marks a replan as due;
  - `step(pose, scan, t) -> Command(v, w, state, path, frontiers)` replans when due or when the
    goal is reached or blocked, handles blacklisting and progress timeout, applies `scan_safety`
    to the follower output, and returns zero velocity in `COMPLETE`.
  The core uses whatever time base it is given (ROS time in the node, sim time in tests).

### TF

`map -(slam_toolbox)-> odom -(diff_drive_controller)-> base_footprint -(robot_state_publisher)-> base_link -> laser_link`.
No new TF is published.

### Launch

`bumperbot_bringup/launch/auto_explore.launch.py` (new):
- args: `simulator` (`gazebo`|`mujoco`, default `gazebo`), `world_name` (default `small_house`),
  `use_rviz` (default `true`), `viewer` (MuJoCo viewer, default `true`), `map_save_path`
  (default empty = use the YAML value), `explorer_config` (default
  `get_package_share_directory("bumperbot_mapping")/config/frontier_explorer.yaml`).
- includes `simulated_robot.launch.py` with `use_slam:=True`, `simulator`, `world_name`,
  `use_rviz`, `viewer`;
- starts `frontier_explorer.py` with `explorer_config`, `{"use_sim_time": True}`, and
  `map_save_path` only when it is not empty (OpaqueFunction).
- No absolute paths; everything goes through `get_package_share_directory`.

`simulated_robot.launch.py` (modify): add `use_rviz` (default `true`, gates both RViz nodes) and
`viewer` (default `true`, forwarded to `mujoco.launch.py`). Existing behavior does not change.

`twist_mux_topics.yaml` (modify): add input `explore: {topic: explore_vel, timeout: 0.5, priority: 10}`.
The priorities become joystick 99 > keyboard 90 > explore 10, and the `safety_stop` lock 255 still
wins. On the real robot nothing publishes on `explore_vel`, so the extra input never becomes active.

### Data flow

```
 Gazebo / MuJoCo ---> /scan ----------------------------+------------------+
        |                                               |                  |
   ros2_control (diff_drive) --> /odom, TF odom->base   v                  v
        ^                                         slam_toolbox       safety_stop_node
        |                                          |  /map, TF map->odom   | /safety_stop (lock 255)
        |                                          v                       |
        |                      +--------------------------------+          |
        |                      | frontier_explorer (rclpy)       |          |
        |                      |   ExplorerCore (pure Python):   |          |
        |                      |   frontiers -> goal -> A* ->    |          |
        |                      |   pure pursuit -> scan safety   |          |
        |                      +--------------------------------+          |
        |                         | explore_vel (Twist, prio 10)           |
        |                         | /explore/status, /explore/path         |
        |                         | on COMPLETE: /map_saver/save_map       |
        |                         v                                        v
        |   joy_vel(99), key_vel(90) ---> twist_mux <----------------------+
        |                                    | /bumperbot_controller/cmd_vel_unstamped
        +--------------- twist_relay <-------+  (stamps -> /bumperbot_controller/cmd_vel)
```

### SLAM map quality (amendment)

#### Investigation results (read-only, headless `BumperbotSim`)

Encoder dead reckoning with the diff_drive_controller parameters (r = 0.033 m, L = 0.17 m, which
match the MJCF: wheel contact points at y = +-0.0851 m, sphere radius 0.033 m), 100 Hz, commands
ramped with the controller limits (0.7 m/s^2, 5 rad/s^2), `empty.xml`:

| Run | heading error now | position error now | with fix below |
|---|---|---|---|
| 2 m square, v 0.15, w 0.8 (explorer limits), 4 left turns | 14.3 deg | 0.356 m | 0.68 deg / 0.017 m |
| 4 x 360 deg in place, w 0.8 | about 58 deg (29 deg per 2 turns), always odom > truth | 0 | 2.7 deg |
| straight 3 m, v 0.15 | 0.1 deg | 0.156 m (odom 5 % too long) | 0.001 m |

Root cause (high confidence): the casters are meant to be frictionless (`condim="1"` set by
`tools/generate_robot.py`), but MuJoCo uses the LARGER condim of the two geoms in a contact unless
one geom has a higher `priority`. The floor plane has condim 3, so each caster contact has condim 3
and friction 1.0 (checked: `data.contact[i].dim == 3`, friction [1, 1, ...]). The casters carry
about 1.8 N of the 9.9 N robot weight, drag sideways during every turn, and the wheels slip
(odometry over-reports rotation by about 3.5-4 %, and distance by 5 %). The explorer rotates in
place a lot (`rotate_in_place_angle` 0.8, initial spin), so the odometry heading drifts by about
7 deg per 180 deg turned. slam_toolbox only adds a scan every 0.5 m / 0.5 rad and weighs the odometry
prior (`angle_variance_penalty`, `minimum_angle_penalty`), so it does not fully correct the drift.
The result is the fanned map with a seam where a loop closure or scan match snapped.
A second, smaller error remains with the caster fix alone (about 1 % heading scale, 3.4 deg per turn):
the soft default wheel contact (`solref 0.02 1`, 0.55 mm penetration) shrinks the effective wheel
radius. A stiffer wheel contact (`solref="0.004 1"`, still >= 2 x timestep 0.002) removes it.

Fix (sim-only, `bumperbot_mujoco/mjcf/robot.xml` + its generator):
- caster collision geoms: add `priority="1"` so their `condim="1"` (frictionless) wins;
- wheel collision geoms: add `priority="1" solref="0.004 1"` (their own friction 1.5 is kept).
Gazebo, the URDF, ros2_control params and the real robot are not touched.

Other candidates checked:
- Stale second MuJoCo stack on the same `ROS_DOMAIN_ID` (two `/clock`, `/scan`, odom TF publishers
  and two twist_mux nodes that both forward `explore_vel`): a real hazard that cannot be confirmed
  after the fact. It would rather give ghost walls, doubled geometry and TF time jumps than a smooth
  fan. AC9 runs on a dedicated `ROS_DOMAIN_ID` with `ROS_LOCALHOST_ONLY=1`, and asserts exactly one
  `/clock` and one `/scan` publisher. doc/12 gets a troubleshooting entry.
- use_sim_time: `simulated_robot.launch.py` passes `use_sim_time: True` to slam, the controller,
  RSP and the explorer. No mismatch found.
- Lidar mount: MJCF `laser_link` quat (yaw pi) and position match the URDF `laser_joint`
  (yaw 3.14, 0.0016 rad apart, negligible). `/scan` angle_increment 6.28/359 is used for both the
  ray-cast and the message, so it is consistent.
- Wheel radius and separation: they match the MJCF (see above).
- slam_toolbox config (`map_update_interval 5.0`, `minimum_travel_heading 0.5`): it amplifies the
  odometry error but does not cause it. It is unchanged (shared with manual SLAM). It is a fallback
  only if AC9 still fails after AC10 passes (see Risks).

#### Map comparison helper `bumperbot_mapping/map_compare.py` (pure Python, numpy + scipy, no rclpy, no cv2)

cv2 is not usable here (built against numpy 1.x, numpy is 2.2.6), so do not import it.
- `load_map(yaml_path) -> MapImage(grid, resolution, origin_x, origin_y)`: reads the map_server YAML
  and the P5 PGM (own reader, flipped so row 0 = min y), occupied = pixel < 50 (trinary 0),
  free = 254, unknown = 205.
- `occupied_points(map) -> (N, 2)` world coordinates of the occupied cell centres.
- `reference_distance_field(ref_map) -> callable(points) -> distances` (EDT on the reference occupied
  cells, 5.0 m outside the grid).
- `rigid_align(points, dist_fn, max_angle_deg=20, max_shift=1.5) -> (theta, tx, ty, cost)`:
  minimises the truncated chamfer `mean(min(d, 0.5))` over rotation about the point centroid and
  translation. Coarse-to-fine: 1 deg / 0.2 m, then 0.1 deg / 0.02 m around the best, global search
  on at most 2000 subsampled points. Budget: <= 30 s for a 3000-point map.
- `dominant_direction_deg(points) -> float` in [-45, 45): the angle that maximises the sharpness
  (sum of squared 0.05 m bin counts) of the projections on that axis and its normal.
- `tile_residuals(aligned_points, dist_fn, tile=4.0, stride=2.0, min_points=150)`: for each tile
  with >= `min_points` cells, a local rigid search (+-15 deg at 0.25 deg, +-0.4 m at 0.05 m) gives
  the residual rotation, the residual shift and the inlier fraction (d <= 0.15 m) of that tile.
- `compare_maps(explored, reference) -> MapQuality` dataclass: `n_occupied`, `extent_m`,
  `global_rotation_deg`, `inlier_fraction` (d <= 0.15 m after alignment), `median_distance`,
  `dominant_residual_deg` (aligned explored vs reference), `max_tile_rotation_deg`,
  `max_tile_shift_m`, `min_tile_inlier_fraction`, `n_tiles`; plus `passes(thresholds) -> (bool, reasons)`.

Thresholds (`DEFAULT_THRESHOLDS`), measured with a prototype of this algorithm on real data:

| Metric | Threshold | user's warped map | reference, rotated 7 deg + shifted + 30 % dropout + 3 cm noise |
|---|---|---|---|
| `inlier_fraction` (d <= 0.15 m = 3 cells) | >= 0.80 | 0.62 | 1.00 |
| `dominant_residual_deg` | <= 2.0 | 0.0 (not discriminative alone for a fan) | 0.25 |
| `max_tile_rotation_deg` (residual wall rotation) | <= 2.0 | 15 (many tiles 6-15) | 1.25 |
| `max_tile_shift_m` (seam) | <= 0.20 | 0.53 | 0.00 |
| `min_tile_inlier_fraction` (seam) | >= 0.50 | 0.07 | 1.00 |
| `n_occupied` / extent | >= 2000 cells, >= 15 x 9 m | 3214, 18.6 x 11 m | - |

0.80 leaves room for real differences between the reference (made with slam_toolbox in Gazebo:
furniture outlines, glass, wall thickness) and a correct MuJoCo SLAM map. 0.15 m is 3 cells, which
covers 1-cell rasterisation and wall-thickness offsets. 2 deg is the requested rotation bound, and
it is above the 1.25 deg noise floor of the tile estimator. The per-tile rotation, shift and inlier
checks are the "no seam" checks: a seam always gives at least one tile with a large local rotation
or shift, or with a low inlier fraction.

#### AC9 test flow

```
 ROS_DOMAIN_ID=87, ROS_LOCALHOST_ONLY=1 (dedicated, isolated)
 launch_testing: auto_explore.launch.py simulator:=mujoco viewer:=false use_rviz:=false
                 map_save_path:=<tmp>/explored explorer_config:=<tmp override yaml>
   mujoco_sim_node -> /clock,/scan -> slam_toolbox -> /map -> frontier_explorer
        ^                                                      | explore_vel
        +-- ros2_control <- twist_mux <------------------------+
   frontier_explorer: COMPLETE or max_exploration_time -> /map_saver/save_map -> <tmp>/explored.{pgm,yaml}
 test process: wait for /explore/status == MAP_SAVED
   -> map_compare.compare_maps(<tmp>/explored.yaml, small_house/map.yaml) -> assert thresholds
```

## Files

| Path | create/modify | Purpose |
|---|---|---|
| `src/bumperbot_mapping/bumperbot_mapping/frontier_exploration.py` | create | Pure-Python core (no rclpy): grid utils, costmap, frontiers, clustering, A*, goal selection, pure pursuit, scan safety, completion monitor, `ExplorerCore` |
| `src/bumperbot_mapping/bumperbot_mapping/frontier_explorer.py` | create | Thin rclpy node wrapper (shebang, `main()`), map saving, status |
| `src/bumperbot_mapping/config/frontier_explorer.yaml` | create | Default parameters (English comments) |
| `src/bumperbot_mapping/CMakeLists.txt` | modify | Install `frontier_explorer.py` as a program; add `rviz` to the installed directories (`slam.rviz` is referenced by bringup but not installed today); `ament_add_pytest_test` for the unit, synthetic and node tests; `add_launch_test` for the node test |
| `src/bumperbot_mapping/package.xml` | modify | exec_depend `std_msgs`, `visualization_msgs`, `nav2_msgs`, `slam_toolbox`, `tf2_ros`, `python3-numpy`, `python3-scipy`; test_depend `ament_cmake_pytest`, `launch_testing_ament_cmake`, `launch_testing_ros`, `python3-pytest` |
| `src/bumperbot_mapping/test/test_frontier_exploration.py` | create | Unit tests for AC1-AC3 |
| `src/bumperbot_mapping/test/test_exploration_synthetic.py` | create | Closed-loop kinematic test on synthetic grids (AC4), plus a small ray-cast helper |
| `src/bumperbot_mapping/test/test_frontier_explorer_node.launch.py` | create | launch_testing test of the node with synthetic inputs and a fake save_map server (AC6) |
| `src/bumperbot_mujoco/test/test_auto_explore_sim.py` | create | Headless MuJoCo E2E on `small_house` with a ground-truth-pose ray-cast mapper (AC5) |
| `src/bumperbot_mujoco/package.xml` | modify | test_depend `bumperbot_mapping`, `python3-scipy` |
| `src/bumperbot_bringup/launch/auto_explore.launch.py` | create | SLAM + explorer for gazebo or mujoco |
| `src/bumperbot_bringup/launch/simulated_robot.launch.py` | modify | Add `use_rviz` and `viewer` args (defaults keep the current behavior) |
| `src/bumperbot_bringup/package.xml` | modify | exec_depend `bumperbot_mapping`, `bumperbot_localization`, `bumperbot_utils` (used but not declared); test_depend `launch_testing_ament_cmake`, `launch_testing_ros`, `ament_cmake_pytest` |
| `src/bumperbot_bringup/CMakeLists.txt` | modify | Register the bringup tests under `BUILD_TESTING` |
| `src/bumperbot_bringup/test/test_launch_config.py` | create | Static checks for AC7: launch files load, no absolute paths, twist_mux priorities, docs mention the feature |
| `src/bumperbot_bringup/test/test_auto_explore_mujoco.launch.py` | create | Full-stack headless MuJoCo smoke test (AC8) |
| `src/bumperbot_controller/config/twist_mux_topics.yaml` | modify | Add the `explore` input, priority 10 |
| `README.md` | modify | Key capabilities, Quick Start / Feature guide "Autonomous exploration", package reference, command reference |
| `doc/08_mapping_and_slam.md` | modify | New section: frontier exploration algorithm, termination, map saving, launch usage |
| `doc/03_ros2_nodes_topics_services.md` | modify | `frontier_explorer` node entry (Python-only justification), new topics in the catalog, `explore` twist_mux input |
| `doc/10_navigation.md` | modify | Implementation status: built-in grid A* + pure pursuit used for exploration; correct the claim that Nav2 is not installed (the Nav2 packages are installed now, but this feature does not use them) |
| `doc/11_build_run_and_command_reference.md` | modify | Run and test commands for auto exploration |
| `doc/01_repository_structure.md` | modify | List the new files |
| `src/bumperbot_mapping/bumperbot_mapping/map_compare.py` | create | AC9 helper: PGM/YAML loader, rigid 2D alignment, dominant wall direction, tile residuals, `compare_maps`, `DEFAULT_THRESHOLDS` (pure Python) |
| `src/bumperbot_mapping/test/test_map_compare.py` | create | AC9 helper unit tests: perturbed reference passes, synthetic seam fails, the user's warped map fails |
| `src/bumperbot_mapping/test/data/warped_small_house.pgm`, `.yaml` | create | Regression fixture: copy of `.claude/features/auto-explore-slam/explored_map.{pgm,yaml}` (image name inside the YAML updated) |
| `src/bumperbot_mapping/CMakeLists.txt` | modify | Register `test_map_compare.py` (`ament_add_pytest_test`, TIMEOUT 300) |
| `src/bumperbot_bringup/test/test_slam_map_quality_mujoco.launch.py` | create | AC9 full-stack SLAM map quality test (opt-in with `BUMPERBOT_SLAM_E2E=1`) |
| `src/bumperbot_bringup/CMakeLists.txt` | modify | Register the AC9 launch test (TIMEOUT 2100); it skips itself unless `BUMPERBOT_SLAM_E2E=1` |
| `src/bumperbot_bringup/package.xml` | modify | test_depend `python3-scipy`, `python3-yaml` (if not already present) |
| `src/bumperbot_mujoco/mjcf/robot.xml` | modify | Root-cause fix: caster geoms `priority="1"`; wheel collision geoms `priority="1" solref="0.004 1"` (English XML comment) |
| `src/bumperbot_mujoco/tools/generate_robot.py` | modify | Emit the same caster priority and wheel solref/priority, so regenerating keeps the fix |
| `src/bumperbot_mujoco/test/test_odometry_drift.py` | create | AC10: encoder odometry drift and caster contact checks in `BumperbotSim` |
| `doc/05_robot_description_and_simulation.md` | modify | MuJoCo contact model: condim/priority rule, caster and wheel contact params, measured odometry accuracy |
| `doc/12_troubleshooting.md` | modify | New entry "Warped / fanned SLAM map": duplicate stacks on the same ROS_DOMAIN_ID (`ros2 topic info /clock` publisher count), MuJoCo odometry slip, how to run AC9 |
| `doc/08_mapping_and_slam.md`, `doc/11_build_run_and_command_reference.md`, `README.md` | modify | Map quality test command and what it checks; advise a dedicated `ROS_DOMAIN_ID` for sim runs |

Not touched: `real_robot.launch.py`, `bumperbot_firmware/*`, `hardware_interface.launch.py`,
`slam_toolbox.yaml`, `sim_core.py`, `mujoco_sim_node.py`, the URDF/xacro, `bumperbot_gazebo.xacro`,
`bumperbot_controllers.yaml`. The amendment only changes the MuJoCo robot model (sim-only path).

## Acceptance criteria

- AC1: `frontier_exploration` imports without rclpy. A fresh interpreter that imports it has no
  `rclpy` in `sys.modules`. On synthetic grids, `detect_frontiers` marks exactly the free cells
  4-adjacent to unknown cells, and `cluster_frontiers` returns the expected number of 8-connected
  clusters with the right sizes and centroids. Clusters smaller than `min_frontier_size` are dropped.
- AC2: `build_costmap` marks every cell within `inflation_radius` of an occupied cell as not
  traversable, and unknown cells too when `unknown_is_obstacle` is set. `astar` returns a
  collision-free 8-connected path (no inflated cell, no corner cutting) between two rooms joined by
  a doorway, and `None` for a goal in a closed room. A start cell inside the inflation zone is
  snapped to a traversable cell. `select_goal` never returns a blacklisted or unreachable frontier,
  and returns `None` when only unreachable frontiers exist.
- AC3: `PurePursuit` always respects `max_linear_speed` / `max_angular_speed`. It rotates in place
  (v == 0, w != 0) when the heading error is above `rotate_in_place_angle`, and reports `done` within
  `goal_tolerance`. `scan_safety` with `laser_yaw_offset = pi` treats a beam at laser angle pi as
  the robot's front: v is 0 when that range is below `stop_distance`, and scaled when it is between
  stop and slow distance. A close obstacle behind the robot does not stop forward motion.
  `CompletionMonitor` returns true only after `completion_hysteresis` consecutive empty updates AND
  `completion_timeout` seconds, and resets when a frontier comes back.
- AC4: Synthetic closed loop (no MuJoCo, no ROS). A unicycle robot with a ray-cast lidar
  (360 beams, 12 m, laser offset pi) runs `ExplorerCore` in a closed 2-room plus corridor grid
  (about 8 m x 6 m, 0.05 m cells, doorways 0.8 m wide). It reaches `COMPLETE` within 900 s of
  simulated time, maps >= 95 % of the free cells reachable from the start, never enters an occupied
  cell, and the final command is exactly (0, 0).
- AC5: Headless MuJoCo E2E. `BumperbotSim` runs `small_house.xml` from `bumperbot_mujoco/mjcf`
  (closed world; Twist is converted to wheel speeds with r = 0.033 m, L = 0.17 m). The map comes
  from a log-odds ray-cast mapper fed with `sim.scan()` and the ground-truth `sim.base_pose()`,
  updated every 1 s like slam_toolbox. The `safety_stop` lock is emulated: zero command whenever a
  beam is below 0.2 m. `ExplorerCore` reaches `COMPLETE` within 1800 s of simulated time and
  180 s of wall-clock time. Coverage is >= 80 % of the reference free cells
  (`bumperbot_mapping/maps/small_house/map.pgm`, flood-filled from the start cell, 2-cell alignment
  tolerance). The minimum lidar range seen during the run is >= 0.15 m (no collision), and the
  emulated lock is never active for more than 5 s in a row (no deadlock).
- AC6: Node wrapper (launch_testing, no simulator). Inputs are a latched synthetic `/map` with
  frontiers, a static TF `map -> base_footprint` plus `base_footprint -> laser_link` (yaw pi), and an
  open `/scan`. Within 15 s, `/explore/status` reports `EXPLORING`, `explore_vel` carries a non-zero
  Twist, and `/explore/path` is not empty. Then a fully known closed map is published (no
  frontiers). Within `completion_timeout + 10` s, status reaches `COMPLETE`, a zero Twist is
  published on `explore_vel`, a fake `/map_saver/save_map` server receives a request whose `map_url`
  equals the `map_save_path` parameter, and status ends at `MAP_SAVED`.
- AC7: Integration and safety of other code paths:
  - `ros2 launch bumperbot_bringup auto_explore.launch.py --show-args` exits 0 and lists
    `simulator`, `world_name`, `use_rviz`, `viewer`, `map_save_path`;
  - `generate_launch_description()` of `auto_explore.launch.py` loads and contains a
    `frontier_explorer.py` node;
  - `ros2 launch bumperbot_bringup real_robot.launch.py --show-args` and
    `simulated_robot.launch.py --show-args` exit 0;
  - no launch file under `src/*/launch` contains a string literal that starts with `/home` or `/opt`;
  - `twist_mux_topics.yaml` has explore priority 10 < keyboard 90 < joystick 99, and
    `twist_mux_locks.yaml` keeps `safety_stop` at 255;
  - `git diff main --stat` shows no change to `real_robot.launch.py` or `src/bumperbot_firmware`;
  - `README.md`, `doc/08`, `doc/03` and `doc/10` mention `auto_explore.launch.py` and
    `frontier_explorer`.
- AC8: Full-stack headless MuJoCo smoke test. `auto_explore.launch.py simulator:=mujoco
  world_name:=small_house viewer:=false use_rviz:=false` brings the stack up. Within 120 s: `/map`
  is received, `/explore/status` reports `EXPLORING`, and the TF `map -> base_footprint` translation
  has moved >= 0.3 m from its first value. (This is the closest automated check of the ROS wiring
  with slam_toolbox. Gazebo shares the same launch path and is covered by AC7.)

- AC9: SLAM map quality, end to end, headless MuJoCo, isolated. With `ROS_DOMAIN_ID=87` and
  `ROS_LOCALHOST_ONLY=1`, `auto_explore.launch.py simulator:=mujoco world_name:=small_house
  viewer:=false use_rviz:=false` runs sim + controller + slam_toolbox + explorer (with
  `max_exploration_time: 1500.0` from a test override YAML). Exactly one node publishes `/clock`
  and exactly one publishes `/scan` (checked 20 s after start). `/explore/status` reaches
  `MAP_SAVED` within 1800 s of wall time, and the saved map is compared by
  `map_compare.compare_maps` with `src/bumperbot_mapping/maps/small_house/map.pgm` after a rigid
  2D alignment. It must have: `n_occupied >= 2000` and an extent >= 15 m x 9 m;
  `inlier_fraction >= 0.80` (occupied cells within 0.15 m of a reference wall);
  `dominant_residual_deg <= 2.0` and `max_tile_rotation_deg <= 2.0` (residual wall rotation);
  no seam: `max_tile_shift_m <= 0.20` and `min_tile_inlier_fraction >= 0.50`.
  The helper itself is unit-tested: the reference map under a random rigid transform
  (+-10 deg, +-1 m), 30 % dropout and 3 cm noise passes and the recovered rotation is within 0.3 deg
  of the applied one; a synthetic seam (reference cells with x > -2.3 m rotated by 8 deg about
  (-2.3, 0)) fails with `max_tile_rotation_deg > 4`; the fixture `warped_small_house` (the user's
  map) fails on `inlier_fraction`, `max_tile_rotation_deg` and the seam checks; one
  `compare_maps` call takes <= 30 s.
- AC10: MuJoCo wheel odometry is consistent with the ground truth (root-cause pin). In
  `BumperbotSim(empty.xml)`, encoder dead reckoning with r = 0.033 m, L = 0.17 m at 100 Hz, and
  commands ramped with the controller limits (0.7 m/s^2, 5 rad/s^2):
  - 2 m square, v = 0.15 m/s, w = 0.8 rad/s, four 90 deg left turns, 1 s pauses: final
    |odom yaw - true yaw| <= 1.5 deg and position error <= 0.05 m (today: 14.3 deg, 0.36 m);
  - 4 full turns in place at +0.8 rad/s and at -0.8 rad/s: |heading error| <= 5 deg each
    (today: about 58 deg);
  - 3 m straight at 0.15 m/s: position error <= 0.03 m (today: 0.156 m);
  - after settling, every active contact that involves `caster_front_link_collision` or
    `caster_rear_link_collision` has `contact.dim == 1` (frictionless; today: 3).
  `test_sim_core.py` and AC5 still pass with the modified model.

- AC11: The MuJoCo `small_house` collision geometry matches the house (root cause of the warped
  maps, found in iteration 4). `tools/sdf_to_mjcf.py` decomposes the whole outer house wall
  (`residential_HouseWallB_01`) into at most `MAX_HULLS = 8` convex hulls. MuJoCo collides
  with each hull, so hulls such as `c0_0` (30 m2), `c0_5` (36 m2) and `c0_1` (21 m2) fill
  whole strips of the house and produce diagonal walls that the lidar and SLAM see faithfully.
  Pass conditions:
  - a ground-truth occupancy grid of the MuJoCo world at lidar height (downward `mj_ray` over
    the collision group, 0.05 m cells) compared with `maps/small_house/map.pgm` through
    `map_compare.compare_maps` gives inlier fraction >= 0.85, max tile rotation <= 2 deg and
    max tile shift <= 0.20 m;
  - no collision hull of the house walls covers an XY hull area larger than 4 m2 while being
    wider than 0.6 m in both directions (a thin wall segment, not a slab);
  - `test_sim_core.py`, AC5, AC8 and AC10 still pass on the regenerated world.

## Test plan

All commands run from the workspace root
`/home/canhtran/Working/04_Investigating/udemy/ros2_map_localization/ws/bumperbot_ws`, after
`source /opt/ros/humble/setup.bash` (and `source install/setup.bash` for colcon/launch tests).

- AC1, AC2, AC3: pytest unit tests.
  - File: `src/bumperbot_mapping/test/test_frontier_exploration.py`. The AC1 import check uses
    `subprocess.run([sys.executable, "-c", "import bumperbot_mapping.frontier_exploration, sys; assert 'rclpy' not in sys.modules"])`.
  - Command: `PYTHONPATH=src/bumperbot_mapping:$PYTHONPATH python3 -m pytest src/bumperbot_mapping/test/test_frontier_exploration.py -q`
  - colcon: `colcon build --packages-select bumperbot_mapping && colcon test --packages-select bumperbot_mapping --ctest-args -R frontier_exploration && colcon test-result --verbose`
  - Pass: all tests pass, 0 failures.
- AC4: pytest closed-loop synthetic test.
  - File: `src/bumperbot_mapping/test/test_exploration_synthetic.py`
  - Command: `PYTHONPATH=src/bumperbot_mapping:$PYTHONPATH python3 -m pytest src/bumperbot_mapping/test/test_exploration_synthetic.py -q`
  - Pass: test passes. The asserted coverage, sim time, collision and final command are printed
    with `-s`.
- AC5: headless MuJoCo sim test (`pytest.importorskip("mujoco")`; skip if the reference map is not
  found).
  - File: `src/bumperbot_mujoco/test/test_auto_explore_sim.py`. It finds the reference map with
    `ament_index_python` and falls back to `../../bumperbot_mapping/maps/small_house/map.pgm`
    relative to the test file. It includes a small P5 PGM reader.
  - Command: `PYTHONPATH=src/bumperbot_mapping:src/bumperbot_mujoco:$PYTHONPATH python3 -m pytest src/bumperbot_mujoco/test/test_auto_explore_sim.py -q -s`
  - Pass: test passes (not skipped). Coverage >= 0.80, `COMPLETE` reached, min range >= 0.15,
    wall time < 180 s (a sanity check found 10 s of sim plus 50 scans takes about 0.25 s of wall time).
  - Regression: `python3 -m pytest src/bumperbot_mujoco/test/test_sim_core.py -q` still passes.
- AC6: launch_testing test of the node.
  - File: `src/bumperbot_mapping/test/test_frontier_explorer_node.launch.py`, registered with
    `add_launch_test(... TIMEOUT 90)`. `use_sim_time:=false`, short `completion_timeout` (2 s),
    `map_save_path` set to a tmp dir.
  - Command: `colcon build --packages-select bumperbot_mapping && source install/setup.bash && launch_test src/bumperbot_mapping/test/test_frontier_explorer_node.launch.py`
    (or `colcon test --packages-select bumperbot_mapping --ctest-args -R frontier_explorer_node`)
  - Pass: exit code 0, all active and post-shutdown tests pass.
- AC7: pytest static checks plus a CLI smoke test.
  - File: `src/bumperbot_bringup/test/test_launch_config.py`. It loads launch files with
    `importlib`, parses YAML, scans launch files for absolute-path literals, and greps the docs.
  - Commands:
    - `colcon build --packages-select bumperbot_controller bumperbot_mapping bumperbot_bringup && source install/setup.bash`
    - `ros2 launch bumperbot_bringup auto_explore.launch.py --show-args` (exit 0, the 5 args are listed)
    - `ros2 launch bumperbot_bringup real_robot.launch.py --show-args && ros2 launch bumperbot_bringup simulated_robot.launch.py --show-args` (exit 0)
    - `python3 -m pytest src/bumperbot_bringup/test/test_launch_config.py -q`
    - `git diff main --stat -- src/bumperbot_bringup/launch/real_robot.launch.py src/bumperbot_firmware` (empty output)
  - Pass: every command exits 0 and the git diff output is empty.
- AC8: launch smoke test, full stack, headless MuJoCo.
  - File: `src/bumperbot_bringup/test/test_auto_explore_mujoco.launch.py`. It includes
    `auto_explore.launch.py` with `simulator:=mujoco viewer:=false use_rviz:=false`, uses
    `save_map_on_complete:=false` through the YAML override, and skips if `import mujoco` fails.
  - Command: `colcon build && source install/setup.bash && timeout 180 launch_test src/bumperbot_bringup/test/test_auto_explore_mujoco.launch.py`
  - Pass: exit code 0 within 180 s.

- AC9: helper unit tests plus the opt-in full-stack launch test.
  - Helper: `src/bumperbot_mapping/test/test_map_compare.py` (uses
    `src/bumperbot_mapping/maps/small_house/map.{pgm,yaml}` and `test/data/warped_small_house.*`).
    - Command: `PYTHONPATH=src/bumperbot_mapping:$PYTHONPATH python3 -m pytest src/bumperbot_mapping/test/test_map_compare.py -q -s`
    - Pass: all tests pass; the printed `MapQuality` of the warped fixture shows the failing checks.
  - E2E: `src/bumperbot_bringup/test/test_slam_map_quality_mujoco.launch.py` (launch_testing). In
    `generate_test_description` it writes a tmp override YAML (installed `frontier_explorer.yaml`
    with `max_exploration_time: 1500.0`), and passes `map_save_path:=<tmp>/explored`. It skips if
    `BUMPERBOT_SLAM_E2E != 1` or `import mujoco` fails. It fails at once if `ROS_DOMAIN_ID` is unset
    or `0`. Active tests: the publisher counts of `/clock` and `/scan` are 1 after 20 s; wait for
    `MAP_SAVED` (timeout 1800 s, print the status history and whether COMPLETE or the time limit
    ended the run); load `<tmp>/explored.yaml`, run `compare_maps` and assert every threshold. Print
    the full `MapQuality`, and copy the map to `$BUMPERBOT_TEST_ARTIFACTS` if it is set.
  - Command: `colcon build --packages-select bumperbot_mujoco bumperbot_mapping bumperbot_bringup && source install/setup.bash && BUMPERBOT_SLAM_E2E=1 ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1 timeout 2100 launch_test src/bumperbot_bringup/test/test_slam_map_quality_mujoco.launch.py`
  - Before running: `pgrep -af "mujoco_sim_node|ros2_control_node|slam_toolbox"` must print nothing
    (kill stale stacks; see doc/12).
  - Pass: exit code 0. Runtime is about 15-30 min, because `mujoco_sim_node` steps on a wall-clock
    timer (at most 1x real time).
- AC10: headless MuJoCo sim test.
  - File: `src/bumperbot_mujoco/test/test_odometry_drift.py` (`pytest.importorskip("mujoco")`).
  - Command: `PYTHONPATH=src/bumperbot_mujoco:$PYTHONPATH python3 -m pytest src/bumperbot_mujoco/test/test_odometry_drift.py src/bumperbot_mujoco/test/test_sim_core.py -q -s`
  - Pass: all pass, and the measured errors are printed. Before the robot.xml fix the drift tests
    and the caster contact test must FAIL (the coder checks this once, by running the test first).
  - Regression: rerun AC5 (`test_auto_explore_sim.py`) and AC8; both still pass.

- AC11: world fidelity test.
  - File: `src/bumperbot_mujoco/test/test_world_fidelity.py` (`pytest.importorskip("mujoco")`).
  - Command: `PYTHONPATH=src/bumperbot_mapping:src/bumperbot_mujoco:$PYTHONPATH python3 -m pytest -q src/bumperbot_mujoco/test/test_world_fidelity.py`
  - Pass: both checks of AC11 hold.

## Implementation steps

1. Add the `explore` input to `twist_mux_topics.yaml` (priority 10, timeout 0.5, English comment).
2. Create `frontier_exploration.py` with grid utils, `build_costmap`, `detect_frontiers`,
   `cluster_frontiers`. Write the matching tests in `test_frontier_exploration.py` (AC1) and make
   them pass.
3. Add `reachable_mask` (with start snapping), `astar`, `select_goal` with a blacklist, and
   `simplify_path`, plus their tests (AC2).
4. Add `PurePursuit`, `scan_safety` (wrap-safe base-frame angles from `laser_yaw_offset`, recovery
   direction) and `CompletionMonitor`, plus their tests (AC3).
5. Add `ExplorerCore`: states, initial spin, replan scheduling, goal reached/blocked handling,
   progress timeout and blacklisting, `max_exploration_time`, zero output in `COMPLETE`. Return a
   `Command` dataclass.
6. Write `test_exploration_synthetic.py`: a synthetic world grid, a ray caster (DDA/Bresenham)
   from the true pose, a unicycle integration at 10 Hz, map snapshots every 1 s. Tune the defaults
   until AC4 passes, and keep the tuned values in the YAML.
7. Write `test_auto_explore_sim.py` (AC5). Get the laser yaw offset from the MuJoCo model
   (`laser_site` xmat relative to the `base_footprint` body), do not assume it. Use a log-odds
   mapper at 0.05 m with a grid of the same size and origin as `map.pgm` (500 x 500, origin -12.5).
   Convert Twist to wheel speeds and emulate the safety lock. Tune until it passes. If the 1800 s
   budget is not enough, report the real numbers to the architect instead of lowering thresholds
   on your own.
8. Write the node `frontier_explorer.py`:
   - declare all parameters;
   - set up the subscriptions (map with transient_local, scan with sensor QoS), TF buffer and
     listener, publishers (status transient_local) and the control timer;
   - convert messages to numpy and call `ExplorerCore.step`;
   - publish Twist, Path and markers;
   - on `COMPLETE`, publish zeros for 1 s, then call the `SaveMap` service asynchronously (never
     block in a callback), fall back to slam_toolbox, publish the final status and log it;
   - apply `shutdown_on_complete`;
   - if the TF lookup fails, publish zero velocity and throttle the warning log.
9. Create `config/frontier_explorer.yaml`. Update `bumperbot_mapping/CMakeLists.txt` (install the
   program and the `rviz` dir, register the tests) and `package.xml`.
10. Write `test_frontier_explorer_node.launch.py` (AC6) with a fake `/map_saver/save_map` server in
    the test process.
11. Add the `use_rviz` and `viewer` args to `simulated_robot.launch.py` (defaults unchanged). Create
    `auto_explore.launch.py`. Update the bringup `package.xml` and `CMakeLists.txt`.
12. Write `test_launch_config.py` (AC7) and `test_auto_explore_mujoco.launch.py` (AC8). Add the
    `bumperbot_mujoco` test_depend on `bumperbot_mapping`.
13. Update `README.md` and `doc/08`, `doc/03`, `doc/10`, `doc/11`, `doc/01`. Cover: usage
    (`ros2 launch bumperbot_bringup auto_explore.launch.py simulator:=mujoco world_name:=small_house`),
    the status topic, where the map is saved, the twist_mux priority, how to take over with the
    joystick or keyboard, and the Python-only justification.
14. Run every Test plan command, and `colcon build` of the full workspace, to confirm nothing else
    broke.

15. (AC10, test first) Write `src/bumperbot_mujoco/test/test_odometry_drift.py`: a helper that
    drives `BumperbotSim` with ramped (v, w) segments at 100 Hz, integrates encoder odometry
    (midpoint, r 0.033, L 0.17) and compares it with `sim.base_pose()`; the three motion cases and
    the caster `contact.dim` check. Run it and confirm it fails on the current model.
16. Fix `src/bumperbot_mujoco/mjcf/robot.xml`: add `priority="1"` to `caster_front_link_collision`
    and `caster_rear_link_collision`; add `priority="1" solref="0.004 1"` to
    `wheel_left_link_collision` and `wheel_right_link_collision`. Add an English XML comment that
    explains the MuJoCo condim/priority rule. Do not touch the URDF or Gazebo files.
17. Make the same change in `tools/generate_robot.py` (a `WHEEL_SOLREF = "0.004 1"` constant, set
    `priority` on caster and wheel collision geoms, and fix the caster comment). If xacro + mujoco
    are available, regenerate into the scratch dir and diff it with the edited `robot.xml`
    (only the expected attributes may differ). Do not overwrite meshes.
18. Run AC10, `test_sim_core.py` and AC5. If a `test_sim_core.py` bound fails only because the
    robot no longer slips (for example `test_drives_straight_forward` x range), report the measured
    value to the architect instead of widening the bound on your own.
19. (AC9 helper) Create `bumperbot_mapping/map_compare.py` as in Architecture (no rclpy, no cv2), and
    copy the fixture to `src/bumperbot_mapping/test/data/` (set `image: warped_small_house.pgm`).
    Write `test_map_compare.py`. Use fixed RNG seeds, and keep each `compare_maps` call <= 30 s
    (vectorised distance lookups, coarse-to-fine search, subsampling). Register it in CMake.
20. (AC9 E2E) Write `test_slam_map_quality_mujoco.launch.py`, reusing the structure of
    `test_auto_explore_mujoco.launch.py` (include `auto_explore.launch.py` with launch args, no
    absolute paths: locate files with `get_package_share_directory`, and use `tempfile` for the
    output). Register it in the bringup CMake with TIMEOUT 2100.
21. Run AC9 once on an isolated domain. Report the full `MapQuality`, the end reason (COMPLETE or
    time limit) and the sim/wall durations. If AC9 fails while AC10 passes, stop and report the
    numbers (see the slam_toolbox fallback in Risks); do not loosen thresholds.
22. Docs: `doc/05` (MuJoCo contact parameters and why), `doc/12` (warped map: duplicate stacks /
    `ROS_DOMAIN_ID`, MuJoCo slip), `doc/08` + `doc/11` + `README.md` (the map quality test, its
    command and opt-in env var, and the advice to use a dedicated `ROS_DOMAIN_ID`).
23. Rerun the whole Test plan (AC1-AC10).

24. (AC11) Fix the collision geometry of the house walls in `tools/sdf_to_mjcf.py`. Large, thin,
    concave structures must not be covered by a handful of hulls. Options, in order of preference:
    (a) build wall collision from the wall mesh's 2D footprint as thin boxes (deterministic,
    no CoACD); (b) adaptive decomposition: raise the hull budget per model until the summed
    hull volume is within 1.5x of the mesh volume. Install the generator's deps only in a
    venv (`python3 -m venv --system-site-packages`) under the session scratchpad, never into
    the user's Python. Regenerate `mjcf/small_house.xml` and only the affected collision meshes,
    then rerun AC5, AC8, AC9, AC10 and `test_sim_core.py`.

## Risks and open questions

- **Safety-stop deadlock.** `safety_stop_node` locks twist_mux (priority 255) whenever ANY beam is
  at or below 0.2 m, in any direction, and the lock blocks the explorer's own escape commands. The
  mitigations are an inflation of 0.30 m, a front stop at 0.25 m and recovery logic that steers
  away from the closest obstacle before reaching 0.2 m. AC5 emulates the lock to catch this. If it
  still happens, an option is to make the danger distance configurable per launch. Do not weaken
  safety_stop in general.
- **Lidar mounted with yaw pi** (`laser_joint rpy="0 0 3.14"`). It must come from TF or the sim
  model. Hard-coding a zero offset would make the robot "see" walls behind it as walls in front.
- **slam_toolbox `map_update_interval: 5.0`** slows down how fast frontiers get updated. The
  explorer keeps following its current path between updates and uses hysteresis for completion.
  `slam_toolbox.yaml` stays unchanged on purpose (manual SLAM shares it). If exploration is too slow
  in practice, add an explore-specific SLAM config through the existing `slam_config` arg in a
  follow-up.
- **AC5 coverage vs the reference map.** `map.pgm` was made with slam_toolbox, so drift may shift
  it from the MuJoCo world, and some free cells (under furniture, behind small gaps) are visible in
  the reference but not reachable or visible for the explorer. The 2-cell tolerance and the 80 %
  threshold cover this. The coder should report the measured value. Also check that the MuJoCo
  `small_house` geometry matches the Gazebo world used to make the map (it was converted by
  `tools/sdf_to_mjcf.py`).
- **Ground-truth-pose mapper in AC5** checks exploration logic, not SLAM. The SLAM wiring is only
  smoke-tested (AC8). Full exploration of Gazebo is not tested automatically, because there is no
  practical headless Gazebo run in CI here.
- **AC8 is heavy** (controller_manager, topic_based_ros2_control, slam_toolbox, twist_mux and
  `joy_node` with no joystick, which only warns). It may be slower or flaky on a loaded machine. Its
  timeout is 180 s.
- **Performance.** A* in pure Python on the full slam map can be slow. Mitigations: crop to the
  known bounding box, use scipy for inflation, labeling and reachability, run A* only on the top-5
  candidates, and replan every 2 s. If a replan takes more than 0.5 s, the coder should report it.
- **Dependencies.** `python3-scipy` becomes a runtime dependency of `bumperbot_mapping`. It is
  available here (scipy 1.15.3, numpy 2.2.6). Any numpy/scipy ABI mismatch with the system ROS
  Python should be reported.
- **Nav2 status.** Nav2 is actually installed in this environment (`nav2_planner`,
  `nav2_controller`, `nav2_bt_navigator`, ... are present), which contradicts `doc/10`. As requested,
  the feature does not depend on them. `doc/10` must be corrected.
- **Pre-existing packaging gaps** fixed here: `bumperbot_mapping` does not install `rviz/`
  (`simulated_robot.launch.py` points to `slam.rviz`), and `bumperbot_bringup/package.xml` does not
  declare `bumperbot_mapping`, `bumperbot_localization` or `bumperbot_utils`.
- **Open question:** should `map_save_path` default to the workspace (`src/bumperbot_mapping/maps/<world>/`)
  instead of `~/bumperbot_maps/`? The plan uses the home directory, so it never overwrites the
  committed small_house map.
- **Open question:** after `COMPLETE`, should the explorer restart on a service call (for example
  `/explore/restart`, `std_srvs/Trigger`)? It is not in scope now and is easy to add later.
- **AC9 runtime (amendment).** `mujoco_sim_node` advances 0.01 s of sim time per 100 Hz wall-clock
  timer tick, so the stack runs at most 1x real time, headless or not (`BumperbotSim` alone runs
  about 15x faster, but the ROS stack cannot). AC5 needed 736 s of sim time with a perfect map, and
  with SLAM (5 s map updates) 900-1500 s is expected. So AC9 is bounded by
  `max_exploration_time: 1500` (the explorer then saves the map anyway) and a 1800 s wait, and it is
  opt-in (`BUMPERBOT_SLAM_E2E=1`), so the default `colcon test` stays fast. A follow-up could add a
  `real_time_factor` parameter to `mujoco_sim_node` (several physics ticks per timer tick, default
  1.0). It is left out because the controller_manager loop is wall-clock paced, and running faster
  than real time would change the control rate in sim time.
- **AC9 thresholds vs the reference map.** The reference is a Gazebo slam_toolbox map, not ground
  truth. If a visually correct MuJoCo map gets an inlier fraction between 0.70 and 0.80 (for
  example, furniture outlines differ), the coder reports the numbers and the per-tile table; the
  architect then decides whether to change the threshold or to build a ground-truth reference by
  ray-casting the MJCF (AC5 mapper with the true pose). Tiles with < 150 occupied cells are ignored,
  because their rotation estimate is noisy (up to about 4 deg on the perfect map with 80 cells).
- **Fix does not fully remove the SLAM warp.** About 0.7 deg of heading error per square remains
  (the real robot is worse). If AC9 still fails with AC10 green, the next lever is an
  exploration-specific slam_toolbox config passed through the existing `slam_config` path:
  `minimum_travel_heading` 0.2, `minimum_travel_distance` 0.2, `map_update_interval` 2.0,
  `angle_variance_penalty` 0.5. Alternatively, lower `max_angular_speed` in the explorer to 0.5.
  The shared `slam_toolbox.yaml` stays unchanged.
- **Stale stacks.** Any process from an earlier launch on the same domain corrupts `/clock`, `/scan`
  and TF, and its twist_mux also forwards `explore_vel`. AC9 enforces isolation and checks the
  publisher counts. The docs tell users to kill old stacks (`pkill -f mujoco_sim_node`, see
  doc/12 section 4) or to use a dedicated `ROS_DOMAIN_ID`. The user's run cannot be replayed, so
  its contribution stays unknown. The measured odometry bug is enough to explain the fan shape.
- **Model change side effects.** Stiffer wheel contact and frictionless casters change the
  dynamics slightly (faster turning response, less slip). `test_sim_core.py`, AC5 and AC8 must be
  rerun. `small_house.xml` and `small_warehouse.xml` include `robot.xml`, so all worlds get the fix.
