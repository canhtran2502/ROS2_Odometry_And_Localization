# AGV waypoint mission manager (ExecuteMission over Nav2 NavigateToPose)

## Goal

The robot runs AGV-style missions on a saved map:
1. `stations.yaml` defines named stations (`x`, `y`, `yaw`, `dwell`) in the `map` frame of the saved
   `small_house` map. One of them must be the `dock` station.
2. A `mission_manager` node (C++ and Python versions, same behavior, same parameters, same
   interfaces) provides the `ExecuteMission` action server (`/execute_mission`). The goal is an
   ordered list of station names. For each station the node:
   - sends a Nav2 `nav2_msgs/action/NavigateToPose` goal;
   - checks that the robot really is within `goal_tolerance` (0.2 m) of the station, using TF
     `map -> base_footprint`, after Nav2 reports SUCCEEDED;
   - dwells `dwell` seconds at the station;
   - retries a failed station up to `max_retries` times, then skips it and goes on;
   - returns to the dock at the end of the mission, and also on error (too many consecutive
     failures) and on cancel.
3. It publishes `/mission/status` (`bumperbot_msgs/msg/MissionStatus`: state, current station,
   index, progress, visited and skipped lists) and action feedback.
4. A Nav2 navigation stack (planner, controller, behaviors, bt_navigator) is configured for
   BumperBot for the first time. It reuses the existing `map_server` + AMCL
   (`global_localization.launch.py`). Nav2 velocity goes into a new low-priority `twist_mux` input.
5. A separate simulation launch file `bumperbot_bringup/launch/mission_sim.launch.py` starts
   simulator (Gazebo or MuJoCo) + localization + Nav2 + mission manager.

The mission manager depends only on the `navigate_to_pose` action, TF and its own interfaces. It
never references Gazebo or MuJoCo. The real-robot launch path (`real_robot.launch.py`) is not
modified. The only shared change is an extra, unused-on-the-real-robot `twist_mux` input
(`nav_vel`), the same pattern as `explore_vel`.

All tests run headless and need no simulator, except one MuJoCo smoke test and one opt-in E2E test.
The node tests use a mock `NavigateToPose` action server that also publishes the TF
`map -> base_footprint` at the "reached" pose.

### Package placement

There is no navigation package in `src/` (doc/10 says "no Nav2 navigation stack is configured").
The mission manager and the Nav2 configuration do not fit into `bumperbot_mapping` (SLAM),
`bumperbot_localization` (EKF/AMCL) or `bumperbot_utils` (safety). A new package
`bumperbot_navigation` (ament_cmake + ament_cmake_python, same layout as `bumperbot_utils` /
`bumperbot_localization`: C++ in `src/` + `include/`, Python in `bumperbot_navigation/`) holds:
mission manager (both languages), pure mission logic (both languages), `stations.yaml`,
`nav2_params.yaml`, `navigation.launch.py`, and the tests. Interfaces go into the existing
`bumperbot_msgs`. The simulation launch goes into the existing `bumperbot_bringup`.

### C++ / Python parity strategy

- The mission state machine lives in a pure library in each language, with no ROS dependency:
  `bumperbot_navigation/mission_logic.py` and `include/bumperbot_navigation/mission_logic.hpp` +
  `src/mission_logic.cpp`. The same public API (names in snake_case in both languages).
- A shared, language-neutral case table `test/data/mission_cases.yaml` (scripted Nav2 outcomes and
  expected step sequences) is executed by both the pytest and the gtest suites, so the two state
  machines are checked against the same expectations.
- The ROS node test (`launch_testing`) is parametrized with `impl in [cpp, py]` and runs the same
  test methods against both executables.

## Architecture

### Nodes

| Package | Language | Executable / node name | Role |
|---|---|---|---|
| `bumperbot_navigation` | C++ (new) | `mission_manager` / `mission_manager` | ExecuteMission server, Nav2 client, status publisher |
| `bumperbot_navigation` | Python (new) | `mission_manager.py` / `mission_manager` | Same as C++ (selected by `use_python`) |
| `bumperbot_navigation` | C++ lib (new, no rclcpp) | `mission_logic` (static/shared lib) | Station loading (yaml-cpp), validation, `MissionTracker` |
| `bumperbot_navigation` | Python module (new, no rclpy) | `bumperbot_navigation/mission_logic.py` | Same as C++ (PyYAML) |
| `nav2_planner` | existing | `planner_server` | NavFn A* on global costmap |
| `nav2_controller` | existing | `controller_server` | Regulated Pure Pursuit, `cmd_vel` remapped to `nav_vel` |
| `nav2_behaviors` | existing | `behavior_server` | spin / backup / wait, `cmd_vel` remapped to `nav_vel` |
| `nav2_bt_navigator` | existing | `bt_navigator` | `navigate_to_pose` action server (default BT XML) |
| `nav2_lifecycle_manager` | existing | `lifecycle_manager_navigation` | autostart of the 4 Nav2 nodes above |
| `nav2_map_server` / `nav2_amcl` | existing | `map_server`, `amcl` | unchanged, from `global_localization.launch.py` |
| `twist_mux` | existing | `twist_mux` | new input `navigation` (`nav_vel`, priority 20) |
| `bumperbot_utils` | existing | `safety_stop_node` | unchanged, its lock (255) still wins |

### Interfaces (new, in `bumperbot_msgs`)

`action/ExecuteMission.action`:
```
# Goal: ordered list of station names from stations.yaml
string[] stations
---
# Result
uint8 OUTCOME_COMPLETED=0              # every station visited, docked
uint8 OUTCOME_COMPLETED_WITH_SKIPS=1   # some stations skipped, docked
uint8 OUTCOME_ABORTED=2                # mission stopped on error, docked
uint8 OUTCOME_CANCELED=3               # canceled by the client, docked
uint8 OUTCOME_DOCK_FAILED=4            # the dock could not be reached
uint8 outcome
bool docked
string[] visited
string[] skipped
string message
---
# Feedback
string state
string current_station
int32 current_index          # index in the goal list, -1 while docking / idle
uint32 total_stations
uint32 attempt               # 1-based attempt of the current navigation
float32 progress             # (visited + skipped) / total_stations, 0..1
float32 distance_remaining   # from NavigateToPose feedback, -1 if unknown
```

`msg/MissionStatus.msg`:
```
string STATE_IDLE=IDLE
string STATE_NAVIGATING=NAVIGATING
string STATE_DWELLING=DWELLING
string STATE_RETURNING_TO_DOCK=RETURNING_TO_DOCK
string STATE_COMPLETED=COMPLETED
string STATE_ABORTED=ABORTED
string STATE_CANCELED=CANCELED
string STATE_DOCK_FAILED=DOCK_FAILED
builtin_interfaces/Time stamp
string state
string current_station
int32 current_index
uint32 total_stations
uint32 attempt
float32 progress
string[] stations
string[] visited
string[] skipped
string message
```
`bumperbot_msgs/CMakeLists.txt`: add both files, `DEPENDENCIES geometry_msgs builtin_interfaces`.
`package.xml`: add `<depend>builtin_interfaces</depend>`.

### Interfaces of `mission_manager` (identical in C++ and Python)

- Action server `/execute_mission` `bumperbot_msgs/action/ExecuteMission`.
  - Goal callback REJECTS: an empty list, any unknown station name (log the name), and a new goal
    while a mission is active (single mission at a time).
  - Cancel callback ACCEPTS. On cancel: cancel the active Nav2 goal, stop dwelling, return to dock,
    then finish with status CANCELED, `outcome = OUTCOME_CANCELED`.
  - Final action status: SUCCEEDED for COMPLETED / COMPLETED_WITH_SKIPS, ABORTED for ABORTED /
    DOCK_FAILED, CANCELED for CANCELED.
- Action client `navigate_to_pose` `nav2_msgs/action/NavigateToPose` (param `nav_action_name`).
  Goal pose: `header.frame_id = frame_id` of stations.yaml (`map`), `header.stamp = now`,
  position `(x, y, 0)`, orientation from yaw (`z = sin(yaw/2)`, `w = cos(yaw/2)`),
  `behavior_tree = ""` (param `behavior_tree`).
- TF lookup `global_frame -> robot_frame` (`map -> base_footprint`), latest available
  (time 0), timeout `tf_timeout`, after each SUCCEEDED Nav2 result. Planar distance to the station
  > `goal_tolerance`, or the lookup fails, counts as a failed attempt.
- Publisher `/mission/status` `bumperbot_msgs/msg/MissionStatus`, reliable, transient_local,
  depth 1. Published at startup (`IDLE`), on every state change, and at `status_rate` Hz.
- Action feedback with the same content on every state change and on NavigateToPose feedback.

Parameters (all declared in code, same defaults in both languages, also in
`bumperbot_navigation/config/mission_manager.yaml`):

| Name | Default | Meaning |
|---|---|---|
| `stations_file` | `""` -> `<share bumperbot_navigation>/config/stations.yaml` | resolved with `ament_index` (no absolute path in code) |
| `nav_action_name` | `navigate_to_pose` | Nav2 action |
| `global_frame` / `robot_frame` | `map` / `base_footprint` | TF frames for the tolerance check |
| `goal_tolerance` | 0.2 | m, planar distance accepted as "reached" |
| `max_retries` | 2 | extra attempts per station (3 attempts total) |
| `retry_delay` | 1.0 | s between attempts |
| `max_consecutive_failures` | 2 | consecutive skipped stations that abort the mission (0 = never) |
| `dock_max_retries` | 3 | extra attempts for the dock |
| `return_to_dock` | true | return to dock at the end (always true on error / cancel) |
| `nav_server_timeout` | 10.0 | s to wait for the Nav2 server per attempt |
| `nav_goal_timeout` | 300.0 | s per attempt; cancel the Nav2 goal and count a failure after it |
| `tf_timeout` | 1.0 | s |
| `status_rate` | 2.0 | Hz |
| `behavior_tree` | `""` | forwarded to NavigateToPose |
| `use_sim_time` | false (true from the sim launch) | |

Waiting/dwell use the node clock (works with sim time). Python: `MultiThreadedExecutor` +
`ReentrantCallbackGroup`, the execute callback waits on futures with `threading.Event` and
timeouts (never `spin_until_future_complete` inside a callback). C++: execute in a detached
`std::thread` (same pattern as `simple_action_server.cpp`), `MultiThreadedExecutor`, futures with
`wait_for`.

### `stations.yaml` format (`bumperbot_navigation/config/stations.yaml`)

```yaml
# Stations on the saved small_house map. x, y in m, yaw in rad, dwell in s.
frame_id: map
map: small_house
dock_station: dock
stations:
  dock:    {x: 0.0,  y: 0.0,  yaw: 0.0,  dwell: 0.0}
  pickup:  {x: -1.5, y: 1.5,  yaw: 1.57, dwell: 3.0}
  dropoff: {x: 3.0,  y: -2.5, yaw: 0.0,  dwell: 3.0}
  storage: {x: -5.0, y: -3.0, yaw: 3.14, dwell: 3.0}
```
The candidates come from a read-only scan of `maps/small_house/map.pgm`: each one has >= 0.5 m
clearance and is in the free component of the spawn pose (0, 0). AC3 enforces this, so the coder
moves a station if the test fails. The robot spawns at (0, 0, 0) in MuJoCo and AMCL starts at
(0, 0, 0), so the dock is the spawn pose.

### Pure mission logic (same API in both languages)

- `Station{name, x, y, yaw, dwell}`; `StationSet{frame_id, dock_station, stations}`.
- `load_stations(path) -> StationSet`. Raises `ValueError` (Python) / throws
  `std::invalid_argument` (C++) when: the file is missing or not a mapping, `stations` is missing
  or empty, the dock station is missing, a field is missing or not a number, a value is not finite,
  `dwell < 0`. Default `frame_id = map`, `dock_station = dock`.
- `validate_mission(names, station_set) -> str` (empty = valid; otherwise the reason).
- `yaw_to_quaternion(yaw) -> (z, w)`, `planar_distance(ax, ay, bx, by)`.
- `MissionTracker(station_set, names, params)` where params =
  `{max_retries, max_consecutive_failures, dock_max_retries, return_to_dock, goal_tolerance}`:
  - `next_step() -> Step{kind: NAVIGATE | DWELL | FINISHED, station, attempt, docking, dwell}`;
  - `report_navigation(succeeded: bool, distance_error: float)` (NaN = TF unknown = failure; reached
    only if `succeeded and distance_error <= goal_tolerance`);
  - `report_dwell_done()`, `request_cancel()`, `abort(reason)`;
  - read-only: `state`, `current_station`, `current_index`, `attempt`, `progress`, `visited`,
    `skipped`, `outcome`, `docked`, `message`.
  - Rules: stations in goal order; after `max_retries + 1` failed attempts a station is skipped;
    `max_consecutive_failures` consecutive skips abort the mission (remaining stations are not
    tried); after the last station, after abort and after cancel the tracker goes to docking
    (unless the last reached goal was the dock and nothing moved after it); dock failure after
    `dock_max_retries + 1` attempts gives `OUTCOME_DOCK_FAILED`, `docked = false`.
  - The tracker has no clock and no threads; the node drives it.

### TF

`map -(amcl)-> odom -(diff_drive_controller)-> base_footprint -> base_link -> ...`. The mission
manager only reads `map -> base_footprint`. No new TF is published (the mock in the tests publishes
`map -> base_footprint` itself; it never runs together with AMCL).

### Nav2 configuration (`bumperbot_navigation/config/nav2_params.yaml`)

Start from `/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml` (copy the bt_navigator
`plugin_lib_names` list from there) and change: `robot_base_frame: base_footprint`,
`odom_topic: /bumperbot_controller/odom`, `robot_radius: 0.12`, inflation radius 0.35,
controller `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`
(`desired_linear_vel: 0.2`, `use_rotate_to_heading: true`), goal checker
`nav2_controller::SimpleGoalChecker` with `xy_goal_tolerance: 0.10`, `yaw_goal_tolerance: 0.25`
(below the 0.2 m mission tolerance, so AMCL noise has margin), planner
`nav2_navfn_planner/NavfnPlanner` (`use_astar: true`, `allow_unknown: false`), global costmap
static + obstacle (`/scan`) + inflation, local rolling 3x3 m obstacle + inflation. No AMCL / map
server sections (they stay in `bumperbot_localization`).

### Launch

`bumperbot_navigation/launch/navigation.launch.py` (new): args `use_sim_time` (default `false`),
`nav2_params` (default `<share bumperbot_navigation>/config/nav2_params.yaml`). Starts
`controller_server` (remap `cmd_vel -> nav_vel`), `planner_server`, `behavior_server`
(remap `cmd_vel -> nav_vel`), `bt_navigator`, `lifecycle_manager_navigation`
(`node_names` = those 4, `autostart: true`). Simulator independent; usable on the real robot later
with `use_sim_time:=false` (not wired into `real_robot.launch.py` in this feature).

`bumperbot_bringup/launch/mission_sim.launch.py` (new): args `simulator` (`gazebo`|`mujoco`,
default `gazebo`), `world_name` (default `small_house`), `use_rviz` (default `true`), `viewer`
(default `true`), `use_python` (default `false`), `stations_file` (default
`<share bumperbot_navigation>/config/stations.yaml`), `mission_config` (default
`<share bumperbot_navigation>/config/mission_manager.yaml`). It includes
`simulated_robot.launch.py` with `use_slam:=False` (AMCL + map_server on `small_house`),
`navigation.launch.py` with `use_sim_time:=true`, and one `mission_manager` node (C++ unless
`use_python`) with `use_sim_time: True`. All paths via `get_package_share_directory`.

`bumperbot_controller/config/twist_mux_topics.yaml` (modify): add
`navigation: {topic: nav_vel, timeout: 0.5, priority: 20}`. Order: joystick 99 > keyboard 90 >
navigation 20 > explore 10; `safety_stop` lock 255 wins. On the real robot nothing publishes on
`nav_vel`.

### Data flow

```
                      ros2 action send_goal /execute_mission {stations: [pickup, dropoff]}
                                         |
                                         v
 stations.yaml --> +--------------------------------------+ --> /mission/status (MissionStatus)
                   | mission_manager (C++ | Python)       | --> ExecuteMission feedback/result
                   |   MissionTracker (pure logic)        |
                   +--------------------------------------+
                      | NavigateToPose goal    ^ TF map->base_footprint (tolerance check)
                      v                        |
   bt_navigator -> planner_server / controller_server / behavior_server
                      | nav_vel (Twist, prio 20)        ^ /map, /scan, TF
                      v                                 |
  joy_vel(99), key_vel(90) -> twist_mux <- safety_stop lock(255)
                      | /bumperbot_controller/cmd_vel_unstamped -> twist_relay -> diff_drive
                      v
            Gazebo / MuJoCo / real robot (ros2_control) -> /scan, /odom, TF odom->base_footprint
                                                   map_server + amcl -> /map, TF map->odom
```

In the tests the whole Nav2 + simulator part is replaced by `MockNav2` (one rclpy node in the test
process: a `navigate_to_pose` action server with scripted outcomes + a TF broadcaster).

## Files

| Path | create/modify | Purpose |
|---|---|---|
| `src/bumperbot_msgs/action/ExecuteMission.action` | create | Mission action |
| `src/bumperbot_msgs/msg/MissionStatus.msg` | create | `/mission/status` message |
| `src/bumperbot_msgs/CMakeLists.txt` | modify | Generate the two interfaces, add `builtin_interfaces` |
| `src/bumperbot_msgs/package.xml` | modify | `builtin_interfaces` depend |
| `src/bumperbot_navigation/package.xml` | create | New package manifest (deps below) |
| `src/bumperbot_navigation/CMakeLists.txt` | create | Build lib + C++ node, install Python node/module, config, launch; register tests |
| `src/bumperbot_navigation/include/bumperbot_navigation/mission_logic.hpp` | create | C++ pure logic API |
| `src/bumperbot_navigation/src/mission_logic.cpp` | create | C++ pure logic (yaml-cpp) |
| `src/bumperbot_navigation/include/bumperbot_navigation/mission_manager.hpp` | create | C++ node class |
| `src/bumperbot_navigation/src/mission_manager.cpp` | create | C++ node |
| `src/bumperbot_navigation/bumperbot_navigation/__init__.py` | create | Python package |
| `src/bumperbot_navigation/bumperbot_navigation/mission_logic.py` | create | Python pure logic (PyYAML) |
| `src/bumperbot_navigation/bumperbot_navigation/mission_manager.py` | create | Python node (executable `mission_manager.py`) |
| `src/bumperbot_navigation/config/stations.yaml` | create | Stations on `small_house`, incl. `dock` |
| `src/bumperbot_navigation/config/mission_manager.yaml` | create | Node parameters |
| `src/bumperbot_navigation/config/nav2_params.yaml` | create | Nav2 parameters for BumperBot |
| `src/bumperbot_navigation/launch/navigation.launch.py` | create | Nav2 servers + lifecycle manager |
| `src/bumperbot_navigation/test/data/stations_test.yaml` | create | Test stations (`dock`, `a`, `b`, `c`, `u1`, `u2`; dwell 1.0 for `a`, 0.2 for others) |
| `src/bumperbot_navigation/test/data/invalid_*.yaml` | create | Invalid fixtures (no dock, missing field, negative dwell, NaN, empty, not a mapping) |
| `src/bumperbot_navigation/test/data/mission_cases.yaml` | create | Shared parity cases for pytest + gtest |
| `src/bumperbot_navigation/test/test_mission_logic.py` | create | AC2, AC4 (Python) |
| `src/bumperbot_navigation/test/test_mission_logic.cpp` | create | AC2, AC4 (C++, gtest) |
| `src/bumperbot_navigation/test/test_stations_on_map.py` | create | AC3 |
| `src/bumperbot_navigation/test/mock_nav2.py` | create | `MockNav2` helper (action server + TF), used by the launch test |
| `src/bumperbot_navigation/test/test_mission_manager.launch.py` | create | AC5-AC11, parametrized cpp/py |
| `src/bumperbot_navigation/test/test_interfaces_and_static.py` | create | AC1, AC13 |
| `src/bumperbot_navigation/test/test_mission_docs.py` | create | AC14 |
| `src/bumperbot_bringup/launch/mission_sim.launch.py` | create | Simulation launch |
| `src/bumperbot_bringup/test/test_mission_launch_config.py` | create | AC12 |
| `src/bumperbot_bringup/test/test_mission_sim_mujoco.launch.py` | create | AC15 smoke + AC16 opt-in E2E |
| `src/bumperbot_bringup/CMakeLists.txt` | modify | Register the two new tests |
| `src/bumperbot_bringup/package.xml` | modify | `exec_depend bumperbot_navigation`, `nav2_msgs` test depend |
| `src/bumperbot_controller/config/twist_mux_topics.yaml` | modify | `navigation` input `nav_vel`, priority 20 |
| `README.md` | modify | Capabilities, package table, "AGV missions" quick start, commands |
| `doc/10_navigation.md` | modify | Nav2 now configured; mission manager design, stations format, retries/dock rules |
| `doc/03_ros2_nodes_topics_services.md` | modify | `mission_manager` node, `/mission/status`, `/execute_mission`, `nav_vel`, new interfaces |
| `doc/01_repository_structure.md` | modify | `bumperbot_navigation` in tree and dependency table |
| `doc/02_system_architecture.md` | modify | Navigation layer + twist_mux priorities |
| `doc/11_build_run_and_command_reference.md` | modify | Build/run/test commands |

`bumperbot_navigation/package.xml` deps: buildtool `ament_cmake`, `ament_cmake_python`; depend
`rclcpp`, `rclcpp_action`, `rclpy`, `bumperbot_msgs`, `nav2_msgs`, `geometry_msgs`, `tf2`,
`tf2_ros`, `tf2_geometry_msgs`, `yaml_cpp_vendor`, `ament_index_cpp`; exec `ament_index_python`,
`python3-yaml`, `nav2_controller`, `nav2_planner`, `nav2_behaviors`, `nav2_bt_navigator`,
`nav2_lifecycle_manager`, `nav2_regulated_pure_pursuit_controller`, `nav2_navfn_planner`;
test `ament_cmake_gtest`, `ament_cmake_pytest`, `launch_testing_ament_cmake`,
`launch_testing_ros`, `python3-pytest`, `python3-numpy`, `python3-scipy`.

## Acceptance criteria

- AC1: `bumperbot_msgs` builds `ExecuteMission.action` and `MissionStatus.msg` with exactly the
  fields and constants listed above; `ros2 interface show bumperbot_msgs/action/ExecuteMission`
  and `.../msg/MissionStatus` succeed; the existing interfaces (`Fibonacci`, `AddTwoInts`,
  `GetTransform`) are still generated.
- AC2: `load_stations` in Python and C++ accepts `config/stations.yaml` and
  `test/data/stations_test.yaml` with identical parsed values, and rejects every
  `test/data/invalid_*.yaml` fixture (error, no crash); `validate_mission` rejects an empty list
  and unknown names in both languages.
- AC3: the shipped `config/stations.yaml` has `dock` plus >= 3 other stations; every station lies
  on a free cell of `bumperbot_mapping/maps/small_house/map.pgm` with >= 0.30 m clearance to the
  nearest occupied cell and in the same free connected component as the dock (cells with
  clearance > 0.15 m).
- AC4: Python and C++ `MissionTracker` produce identical step sequences and final
  (`outcome`, `docked`, `visited`, `skipped`) for every case in `test/data/mission_cases.yaml`,
  which covers at least: ordered success of 3 stations then dock; retry then success; skip after
  `max_retries + 1` failures; success reported with distance error 0.35 m counted as failure and
  0.15 m accepted; abort after `max_consecutive_failures` consecutive skips then dock; cancel
  during navigation and during dwell then dock; dock unreachable -> `OUTCOME_DOCK_FAILED`;
  mission ending at `dock` does not send a second dock goal; progress is monotonic and 1.0 at the
  end.
- AC5: (node, both `impl=cpp` and `impl=py`, against `MockNav2`) mission `[a, b, c]` sends exactly
  4 NavigateToPose goals in the order a, b, c, dock, each with `frame_id == map`, position equal
  to the station (1e-6) and quaternion equal to `yaw_to_quaternion(yaw)` (1e-6); the result is
  SUCCEEDED, `outcome == OUTCOME_COMPLETED`, `docked == true`, `visited == [a, b, c]`; at each
  "reached" event the TF `map -> base_footprint` is within 0.2 m of the station.
- AC6: (node, both impls) after station `a` (dwell 1.0 s) succeeds, the next goal arrives no
  earlier than 0.95 s and no later than `1.0 + retry_delay + 1.0` s later; the status shows
  `DWELLING` with `current_station == a` during that time.
- AC7: (node, both impls) with `max_retries = 2`: `b` aborted twice then succeeds -> 3 goals for
  `b`, `b` in `visited`; `u1` always aborted -> exactly 3 goals for `u1`, `u1` in `skipped`, the
  mission continues with the next station, `outcome == OUTCOME_COMPLETED_WITH_SKIPS`,
  `docked == true`, action status SUCCEEDED.
- AC8: (node, both impls) the mock reports SUCCEEDED but puts the robot 0.35 m away from `b` on
  the first attempt -> the node sends a second goal for `b`; with an offset of 0.15 m the first
  attempt is accepted (no second goal).
- AC9: (node, both impls) error and cancel return to the dock: (a) mission `[u1, u2, c]` with
  `max_consecutive_failures = 2` -> `c` is never sent, the last goal is `dock`, action status
  ABORTED, `outcome == OUTCOME_ABORTED`, `docked == true`; (b) a cancel request during the
  navigation to `b` -> the mock receives a cancel for that Nav2 goal, then a `dock` goal, and the
  action ends CANCELED with `outcome == OUTCOME_CANCELED`, `docked == true`; (c) dock always
  aborted -> `outcome == OUTCOME_DOCK_FAILED`, `docked == false`, action status ABORTED.
- AC10: (node, both impls) goals with an empty list or an unknown name (`[a, nowhere]`) are
  rejected, and a second goal sent while a mission is active is rejected while the first mission
  still completes.
- AC11: (node, both impls) `/mission/status` is transient_local: a subscriber created after node
  start receives `IDLE`; during AC5 the sequence of distinct `current_station` values is
  a, b, c, dock, `progress` never decreases and is 1.0 in the final message, the final state is
  `COMPLETED`; at least one action feedback message is received per station.
- AC12: launch and configuration: `ros2 launch bumperbot_bringup mission_sim.launch.py --show-args`
  and `ros2 launch bumperbot_navigation navigation.launch.py --show-args` succeed and list the
  arguments above; no string literal in the two launch files starts with `/` (except topic
  remaps) and both use `get_package_share_directory`; `mission_sim.launch.py` includes
  `simulated_robot.launch.py` with `use_slam` false and creates exactly one `mission_manager`
  node (C++ by default, Python with `use_python:=true`); `nav2_params.yaml` uses
  `base_footprint`, `/bumperbot_controller/odom`, `xy_goal_tolerance <= 0.15`; controller and
  behavior servers remap `cmd_vel` to `nav_vel`; `twist_mux_topics.yaml` has `nav_vel` with a
  priority above `explore_vel` and below `key_vel`; `real_robot.launch.py` and
  `simulated_robot.launch.py` are unchanged versus `main` and their `--show-args` still succeed.
- AC13: simulator independence and parity of surface: the mission manager sources (C++, Python,
  both logic files) contain no `mujoco`, `gazebo`, `gz` or `/clock` references; both executables
  declare the same parameter names with the same defaults (checked by starting each one and
  calling `ros2 param dump`-equivalent `list_parameters`/`get_parameters` in the launch test, or by
  parsing both sources); all comments are English (ASCII-only check on the new source files).
- AC14: `README.md`, `doc/10_navigation.md`, `doc/03_ros2_nodes_topics_services.md`,
  `doc/01_repository_structure.md` and `doc/11_build_run_and_command_reference.md` mention
  `bumperbot_navigation`, `mission_sim.launch.py`, `/execute_mission`, `/mission/status` and
  `stations.yaml`; doc/10 no longer says that no Nav2 stack is configured.
- AC15: headless MuJoCo smoke test: `mission_sim.launch.py simulator:=mujoco viewer:=false
  use_rviz:=false` brings up, within 120 s, an available `/navigate_to_pose` action server
  (bt_navigator active), an available `/execute_mission` action server, and a `/mission/status`
  message with state `IDLE`.
- AC16 (opt-in, `BUMPERBOT_MISSION_E2E=1`): on the same headless MuJoCo stack, mission
  `[pickup, dropoff]` finishes within 600 s with `outcome == OUTCOME_COMPLETED`, `docked == true`,
  goals in order, and the TF `map -> base_footprint` (AMCL) within 0.2 m of each station when it
  is reported reached and of the dock at the end.

## Test plan

Build once:
```
cd <ws> && colcon build --packages-select bumperbot_msgs bumperbot_controller bumperbot_navigation bumperbot_bringup && source install/setup.bash
```
Launch tests run with `ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1` to isolate them.

| AC | Type | File | Command | Pass condition |
|---|---|---|---|---|
| AC1 | pytest (CLI) | `src/bumperbot_navigation/test/test_interfaces_and_static.py::test_interfaces*` | `python3 -m pytest src/bumperbot_navigation/test/test_interfaces_and_static.py -k interfaces -v` | `ros2 interface show` returns 0 for the 2 new + 3 old interfaces, all fields/constants present |
| AC2 | pytest unit + gtest | `test/test_mission_logic.py` (`-k stations`), `test/test_mission_logic.cpp` (`StationsTest.*`) | `python3 -m pytest src/bumperbot_navigation/test/test_mission_logic.py -k "stations or validate" -v` and `./build/bumperbot_navigation/test_mission_logic_cpp --gtest_filter='Stations*:Validate*'` | all pass; both print the same parsed values for the two valid files (compared against the expected values in the test) |
| AC3 | pytest unit (numpy/scipy) | `test/test_stations_on_map.py` | `python3 -m pytest src/bumperbot_navigation/test/test_stations_on_map.py -v` | every station passes clearance + connectivity checks |
| AC4 | pytest unit + gtest | `test/test_mission_logic.py` (`-k cases`), `test/test_mission_logic.cpp` (`MissionCases.*`) | `python3 -m pytest src/bumperbot_navigation/test/test_mission_logic.py -k cases -v` and `./build/bumperbot_navigation/test_mission_logic_cpp --gtest_filter='MissionCases*'` | every case in `mission_cases.yaml` passes in both languages |
| AC5-AC11 | launch_testing (mock Nav2, no simulator), `@launch_testing.parametrize("impl", ["cpp", "py"])` | `test/test_mission_manager.launch.py` | `ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 timeout 300 launch_test src/bumperbot_navigation/test/test_mission_manager.launch.py` | all test methods pass for both impls (`test_ordered_visit`, `test_dwell`, `test_retry_and_skip`, `test_tolerance_check`, `test_abort_returns_to_dock`, `test_cancel_returns_to_dock`, `test_dock_failure`, `test_goal_rejection`, `test_status_topic`) |
| AC12 | pytest (static + `--show-args`) | `src/bumperbot_bringup/test/test_mission_launch_config.py` | `python3 -m pytest src/bumperbot_bringup/test/test_mission_launch_config.py -v` | all checks pass; `git diff --quiet main -- src/bumperbot_bringup/launch/real_robot.launch.py src/bumperbot_bringup/launch/simulated_robot.launch.py` returns 0 |
| AC13 | pytest static (+ param check inside the launch test) | `test/test_interfaces_and_static.py::test_sim_independence*`, `::test_parameter_parity`, `::test_ascii_comments` | `python3 -m pytest src/bumperbot_navigation/test/test_interfaces_and_static.py -v` | no forbidden tokens; same parameter name/default set extracted from both node sources (and, in the launch test, `get_parameters` on both running nodes returns the same values) |
| AC14 | pytest (grep) | `test/test_mission_docs.py` | `python3 -m pytest src/bumperbot_navigation/test/test_mission_docs.py -v` | all required strings found, the old "no Nav2 navigation stack is configured" sentence is gone |
| AC15 | launch_testing, headless MuJoCo | `src/bumperbot_bringup/test/test_mission_sim_mujoco.launch.py::test_stack_up` | `ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1 timeout 240 launch_test src/bumperbot_bringup/test/test_mission_sim_mujoco.launch.py` | action servers available and `IDLE` received within 120 s |
| AC16 | launch_testing, headless MuJoCo, opt-in | same file, `test_full_mission` (`unittest.skipUnless(BUMPERBOT_MISSION_E2E=1)`) | `BUMPERBOT_MISSION_E2E=1 ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1 timeout 900 launch_test src/bumperbot_bringup/test/test_mission_sim_mujoco.launch.py` | COMPLETED, docked, every reached pose within 0.2 m |

Full suite: `colcon test --packages-select bumperbot_navigation bumperbot_bringup --event-handlers console_direct+ && colcon test-result --verbose`
(must show 0 failures; AC16 reports as skipped unless enabled).

`MockNav2` (in `test/mock_nav2.py`, runs in the test process on its own executor thread):
- `navigate_to_pose` action server; for each goal records `(station name resolved by pose, pose,
  receive time)`; behavior per station from a dict set by each test method:
  `{"abort_first": n, "always_abort": bool, "offset": meters, "duration": s}` (default: succeed after
  0.2 s, offset 0); publishes NavigateToPose feedback; honors cancel requests (records them).
- Before returning SUCCEEDED it moves its robot pose to station + offset (along +x) and keeps
  broadcasting `map -> base_footprint` at 20 Hz; it waits 0.1 s after the move so TF propagates.
- The node under test is launched with `stations_file = test/data/stations_test.yaml`,
  `retry_delay = 0.1`, `nav_server_timeout = 5.0`, `nav_goal_timeout = 10.0`, `use_sim_time = false`.

## Implementation steps

1. `bumperbot_msgs`: add `action/ExecuteMission.action`, `msg/MissionStatus.msg`, update CMake
   and package.xml; build and run `ros2 interface show` (AC1).
2. Create `bumperbot_navigation` skeleton (package.xml, CMakeLists with `ament_cmake_python`,
   `__init__.py`, empty config/launch dirs installed to share). Build.
3. Python `mission_logic.py`: `Station`, `StationSet`, `load_stations`, `validate_mission`,
   `yaw_to_quaternion`, `planar_distance`, `MissionTracker` per the rules above.
4. Test data: `stations_test.yaml`, `invalid_*.yaml`, `mission_cases.yaml` (format: `params`,
   `stations`, list of scripted events `nav: {ok, err}` / `dwell_done` / `cancel` and the expected
   steps and final fields). Write `test_mission_logic.py`; make it pass (AC2, AC4 Python).
5. C++ `mission_logic.hpp/.cpp` with the same API (yaml-cpp, `std::optional`/exceptions), gtest
   `test_mission_logic.cpp` reading the same YAML files (path via a compile definition
   `TEST_DATA_DIR` set in CMake, not hard-coded); make it pass (AC2, AC4 C++).
6. `config/stations.yaml` + `test_stations_on_map.py` (map path via `ament_index` or relative to
   the test file); adjust station coordinates until AC3 passes.
7. Python node `mission_manager.py`: parameters, station loading (ament_index default), action
   server with goal/cancel callbacks, Nav2 client, TF tolerance check, dwell loop, status
   publisher + timer, driven by `MissionTracker`. Install as `mission_manager.py`.
8. `test/mock_nav2.py` + `test/test_mission_manager.launch.py` (parametrized; start with
   `impl=py` only); make AC5-AC11 pass for Python.
9. C++ node `mission_manager.hpp/.cpp` mirroring step 7 (same parameter names/defaults, same log
   messages where practical); enable `impl=cpp` in the launch test; make AC5-AC11 pass for C++.
10. `config/mission_manager.yaml`, `config/nav2_params.yaml`, `launch/navigation.launch.py`.
11. `twist_mux_topics.yaml`: add `navigation` input (`nav_vel`, 20) with an English comment.
12. `bumperbot_bringup/launch/mission_sim.launch.py`; package.xml / CMake updates;
    `test_mission_launch_config.py` (AC12) and `test_interfaces_and_static.py` (AC1, AC13).
13. `test_mission_sim_mujoco.launch.py`: AC15 smoke (default on) and AC16 (opt-in). Run AC15;
    run AC16 once with `BUMPERBOT_MISSION_E2E=1` and tune `nav2_params.yaml` if needed.
14. Register all tests in both CMakeLists (only when files exist, like the existing ones):
    `ament_add_gtest(test_mission_logic_cpp ...)`, `ament_add_pytest_test(...)` with
    `APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:${CMAKE_CURRENT_SOURCE_DIR}/test`,
    `add_launch_test(... TIMEOUT 300)`.
15. Docs: README (capabilities, package table, "AGV missions" section with the launch and
    `ros2 action send_goal /execute_mission bumperbot_msgs/action/ExecuteMission "{stations: [pickup, dropoff]}" --feedback`
    and `ros2 topic echo /mission/status`), doc/10 (status update + mission manager section:
    stations format, retry/skip/abort/cancel/dock rules, tolerance), doc/03, doc/01, doc/02,
    doc/11. Run `test_mission_docs.py` (AC14).
16. Full `colcon test` for `bumperbot_msgs bumperbot_navigation bumperbot_bringup bumperbot_mapping`
    to confirm nothing else regressed (existing `test_launch_config.py` still passes with the new
    twist_mux input).

## Risks and open questions

- Nav2 on MuJoCo/AMCL may be slow or fail through narrow doors in `small_house`; AC16 is opt-in
  for this reason. If it fails, first tune `nav2_params.yaml` (inflation, RPP speeds,
  progress checker) and move stations to wider areas; the mock-based ACs are the contract.
- `safety_stop` locks `twist_mux` near obstacles (0.2 m); Nav2 then stalls and its progress
  checker aborts. The mission manager counts that as a failed attempt and retries, which is the
  intended behavior, but RPP must keep enough clearance (inflation 0.35 m) to avoid lock loops.
- TF race in the tolerance check: the latest `map -> base_footprint` may lag the Nav2 SUCCEEDED
  result. The mock waits 0.1 s; in the real stack AMCL updates only after motion
  (`update_min_d 0.25`), so the check uses the latest transform with `tf_timeout`. If AC16 shows
  false failures, add a short settle delay (param `verify_delay`, default 0.2 s) in both nodes.
- `launch_testing.parametrize` exists in Humble (`launch_testing/parametrize.py`); if it causes
  trouble, fall back to two `add_launch_test` targets with `ARGS "impl:=cpp"` / `"impl:=py"`.
- Humble `bt_navigator` needs `plugin_lib_names`; copy the list from the installed nav2_bringup
  params, not from newer Nav2 docs (Iron+ removed it).
- Cancel semantics: the action stays EXECUTING while the robot drives back to the dock after a
  cancel request (it can take minutes). This is a deliberate choice (AGV must not stop in the
  aisle); documented in doc/10. A second cancel is ignored.
- `nav_vel` input is added to the shared `twist_mux_topics.yaml`, so it exists on the real robot
  too; harmless while nothing publishes on it. Wiring Nav2 into `real_robot.launch.py` is out of
  scope.
- Open: should the mission stop when `return_to_dock` is false at the end? Implemented as a param
  (default true); on error and cancel the robot always docks.
