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

Not touched: `real_robot.launch.py`, `bumperbot_firmware/*`, `hardware_interface.launch.py`,
`slam_toolbox.yaml`, `sim_core.py`, `mujoco_sim_node.py`.

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
