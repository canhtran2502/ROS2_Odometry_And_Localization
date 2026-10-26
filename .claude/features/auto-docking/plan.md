# AGV auto-docking with simulated battery and battery monitor

## Goal

The robot docks to a charger on its own, charges, and undocks again:
1. A `dock_manager` node (C++ and Python versions with the same behavior, parameters and interfaces)
   provides the `DockRobot` action server (`/dock_robot`, command DOCK or UNDOCK). For DOCK it:
   - drives to the dock **staging pose** with Nav2 `NavigateToPose`. The staging pose is the `dock`
     station of `bumperbot_navigation/config/stations.yaml` (the same pose `mission_manager` returns
     to). The Nav2 step is skipped when the robot is already within `staging_tolerance`;
   - computes the **charger pose** (the `base_footprint` pose when docked). It lies
     `staging_distance` (0.5 m) behind the staging pose, on the staging heading, so the robot backs
     straight in;
   - aligns on a pre-dock line and backs in, closed loop on odometry (`odom -> base_footprint`). The
     dock target comes from the known map pose and is refined by an abstract dock detection topic
     (`/dock/detected_pose`, `geometry_msgs/PoseStamped`);
   - checks the final pose error. On a failed attempt it drives back to the pre-dock pose and
     retries. Timeouts, stall detection and the `safety_stop` lock are respected. Every failure,
     cancel or shutdown path ends with a zero velocity command and a defined state;
   - publishes `/dock/status` (`bumperbot_msgs/msg/DockStatus`).
   For UNDOCK it drives straight out from the charger to the staging pose.
2. A simulation-only `battery_sim` node publishes `/battery_state` (`sensor_msgs/BatteryState`). It
   drains while the robot is idle and drains faster while it moves (from odometry). It charges while
   `/dock/status` reports `docked`.
3. A simulator-independent `battery_monitor` node watches `/battery_state`:
   - When the charge drops below `low_threshold` it starts an automatic dock sequence. If an
     `ExecuteMission` mission is active, it first cancels it. `mission_manager` then drives back to
     the `dock` station (the staging pose), as it already does on cancel. After that it sends
     `DockRobot` DOCK.
   - It stays in CHARGING until the charge reaches `full_threshold`, then sends `DockRobot` UNDOCK.
4. There is a separate simulation launch file `bumperbot_bringup/launch/dock_sim.launch.py`. It runs
   the mission stack, docking, the battery sim and the MuJoCo dock marker emulator.
5. The acceptance target in headless MuJoCo is a final docked pose error of <= 0.05 m and a yaw error
   of <= 0.1 rad. Both are measured against the simulator ground truth.

### Dock-detection source: decision and justification

Chosen: **known dock pose from the map/TF, refined by an abstract fiducial-style pose topic**
`/dock/detected_pose` (`geometry_msgs/PoseStamped`, any TF-resolvable frame). It contains the
charger pose (the docked `base_footprint` pose), optionally after a fixed marker-to-dock offset
(`detection_offset_*`).

The LiDAR-based option is rejected:
- No world has a dock model (checked: `small_house.xml` has no dock/charger body). Adding geometry
  changes the world that the saved `small_house` map, `test_world_fidelity.py` and the AMCL E2E tests
  depend on.
- A LiDAR-visible dock right behind the robot is closer than `safety_stop`'s `danger_distance`
  (0.2 m). The `twist_mux` lock (255) would then block the final insertion. The only way around that
  would be to bypass or tune safety, which the request forbids for the real robot.
- A pose topic keeps `dock_manager` detector agnostic. On the real robot an AprilTag/ArUco node (or a
  laser dock detector) publishes the same message. In simulation a sim-only emulator does it.

How it works:
- **Map-only fallback** (`detection_mode: none`, or `optional` with no detection): the target is the
  charger pose in `map`, turned into `odom` once at the start of the final approach (snapshot of
  `map -> odom`). Precision is then bounded by the AMCL error, which is the reason for the refinement
  input.
- **Simulation**: a sim-only `dock_marker_sim` node (in `bumperbot_mujoco`) emulates a rear-facing
  detector. It reads a new `/mujoco/ground_truth` topic, applies range and field-of-view gating plus
  seeded Gaussian noise, and publishes `/dock/detected_pose` in `base_footprint`. With
  `simulator:=gazebo` it is not started, and `detection_mode: optional` falls back to map-only.

### Package placement

- `dock_manager` (C++/Python), pure `dock_logic` (C++/Python), `battery_monitor` + pure
  `battery_logic` (Python) and `battery_sim` (Python, simulation only, clearly marked) go into the
  existing `bumperbot_navigation`. That package already holds `mission_manager`, `stations.yaml`
  loading (`mission_logic.load_stations`, already C++/Python parity tested) and the Nav2 config.
  `docking.launch.py` (simulator independent) also goes there.
- Interfaces go into `bumperbot_msgs`.
- MuJoCo-specific pieces (`/mujoco/ground_truth`, `dock_marker_sim`) go into the sim-only
  `bumperbot_mujoco`.
- The simulation launch `dock_sim.launch.py` goes into `bumperbot_bringup`.
- `real_robot.launch.py`, `simulated_robot.launch.py` and `mission_sim.launch.py` are not modified.

### C++ / Python parity strategy

- Same pattern as `mission_logic`: a pure library with no ROS in each language
  (`bumperbot_navigation/dock_logic.py`, `include/bumperbot_navigation/dock_logic.hpp` +
  `src/dock_logic.cpp`), with the same snake_case API.
- A shared case table `test/data/dock_cases.yaml` is run by pytest and gtest. It covers geometry,
  controller outputs (1e-9), the detection filter, the phase timer and `DockTracker` step sequences.
- The staging pose is loaded with the existing `load_stations` in both languages, so YAML handling
  (duplicate keys, quoted numbers, null, exponent, NaN) is identical by construction. AC3 checks it
  again at node level.
- The node `launch_testing` tests are parametrized with `impl in [cpp, py]`.
- `battery_monitor` and `battery_sim` are Python only. CLAUDE.md requires sync only where both
  versions exist, and the request asks for parity only for `dock_manager` (see Risks).

## Architecture

### Nodes

| Package | Language | Executable / node name | Role |
|---|---|---|---|
| `bumperbot_navigation` | C++ (new) | `dock_manager` / `dock_manager` | DockRobot server, Nav2 client, dock controller, `/dock/status` |
| `bumperbot_navigation` | Python (new) | `dock_manager.py` / `dock_manager` | Same as C++ (selected by `use_python`) |
| `bumperbot_navigation` | C++ lib (new, no rclcpp) | `dock_logic` | Geometry, controller, detection filter, phase timer, `DockTracker` |
| `bumperbot_navigation` | Python module (new, no rclpy) | `bumperbot_navigation/dock_logic.py` | Same as C++ |
| `bumperbot_navigation` | Python (new) | `battery_monitor.py` / `battery_monitor` | Low-battery dock trigger, charging state, undock when full |
| `bumperbot_navigation` | Python module (new, no rclpy) | `bumperbot_navigation/battery_logic.py` | `BatteryModel` (sim) + `BatteryMonitorLogic` |
| `bumperbot_navigation` | Python (new, **SIM ONLY**) | `battery_sim.py` / `battery_sim` | Publishes `/battery_state`, charges while docked |
| `bumperbot_mujoco` | Python (new, **SIM ONLY**) | `dock_marker_sim` / `dock_marker_sim` | Emulated fiducial detector -> `/dock/detected_pose` |
| `bumperbot_mujoco` | Python (modify) | `mujoco_sim_node` | Adds `/mujoco/ground_truth` (nav_msgs/Odometry, frame `world`) |
| `bumperbot_navigation` | existing | `mission_manager` | Unchanged; canceled by `battery_monitor`, returns to `dock` station |
| `twist_mux` | existing | `twist_mux` | New input `docking` (`dock_vel`, priority 30) |
| `bumperbot_utils` | existing | `safety_stop_node` | Unchanged; its lock (255) still wins over docking |

### Interfaces (new, in `bumperbot_msgs`)

`action/DockRobot.action`:
```
# Goal
uint8 COMMAND_DOCK=0
uint8 COMMAND_UNDOCK=1
uint8 command
---
# Result
uint8 OUTCOME_DOCKED=0              # docked within tolerance
uint8 OUTCOME_UNDOCKED=1            # undocked, robot at the staging pose
uint8 OUTCOME_ALREADY_DOCKED=2      # DOCK while docked, no motion
uint8 OUTCOME_CANCELED=3            # canceled, robot stopped
uint8 OUTCOME_STAGING_FAILED=4      # Nav2 could not reach the staging pose
uint8 OUTCOME_DETECTION_FAILED=5    # detection_mode required and no valid detection
uint8 OUTCOME_DOCKING_FAILED=6      # alignment / insertion failed after all attempts
uint8 OUTCOME_UNDOCKING_FAILED=7
uint8 OUTCOME_ERROR=8               # TF lost, clock stalled, internal exception, shutdown
uint8 outcome
bool docked
uint32 attempts
float32 position_error              # m, final estimate, -1 if unknown
float32 yaw_error                   # rad, final estimate, -1 if unknown
string message
---
# Feedback
string state
uint32 attempt
float32 distance_to_dock            # m, -1 if unknown
float32 yaw_error
bool detection_used
```
Final action status: SUCCEEDED for DOCKED / UNDOCKED / ALREADY_DOCKED, CANCELED for CANCELED,
ABORTED otherwise.

`msg/DockStatus.msg` (`/dock/status`):
```
string STATE_UNDOCKED=UNDOCKED
string STATE_NAVIGATING_TO_STAGING=NAVIGATING_TO_STAGING
string STATE_WAITING_FOR_DETECTION=WAITING_FOR_DETECTION
string STATE_ALIGNING=ALIGNING
string STATE_DOCKING=DOCKING
string STATE_DOCKED=DOCKED
string STATE_CHARGING=CHARGING
string STATE_UNDOCKING=UNDOCKING
string STATE_FAILED=FAILED
builtin_interfaces/Time stamp
string state
bool docked
bool charging                       # docked and /battery_state reports CHARGING
float32 battery_percentage          # 0..1, -1 if unknown
uint32 attempt
float32 position_error              # m, -1 if unknown
float32 yaw_error                   # rad, -1 if unknown
bool detection_used
string frame_id                     # global frame of the two poses below
geometry_msgs/Pose2D staging_pose
geometry_msgs/Pose2D dock_pose      # charger pose (docked base_footprint pose)
string message
```
`bumperbot_msgs/CMakeLists.txt`: add both files. Dependencies are already `geometry_msgs` and
`builtin_interfaces`.

### `dock_manager` interfaces (identical in C++ and Python)

- Action server `/dock_robot` `bumperbot_msgs/action/DockRobot`.
  - The goal callback answers at once (non-blocking checks, < 1 s) and **rejects**:
    - an unknown `command`;
    - a second goal while one is active;
    - UNDOCK when the state is not DOCKED/CHARGING;
    - DOCK while `/mission/status` reports an active mission (NAVIGATING, DWELLING,
      RETURNING_TO_DOCK). No status received counts as no mission;
    - any goal while a dependency is not ready: `can_transform(global_frame, robot_frame)` and
      `can_transform(odom_frame, robot_frame)` false (timeout 0), or, for DOCK when staging is
      needed, the Nav2 action server not ready.
    A rejection never counts an attempt, never sends a Nav2 goal and never publishes velocity.
  - Cancel callback: ACCEPTS. It cancels the active Nav2 goal, publishes zero velocity within one
    control period and ends CANCELED with `docked = false`, state UNDOCKED.
  - DOCK while DOCKED/CHARGING (robot still within `undock_detect_distance` of the docked pose)
    returns SUCCEEDED, `OUTCOME_ALREADY_DOCKED`, with no motion.
- Action client `navigate_to_pose` `nav2_msgs/action/NavigateToPose` (param `nav_action_name`). The
  goal is the staging pose in `global_frame`, built the same way as in `mission_manager`.
- Publisher `cmd_vel_topic` (`dock_vel`, `geometry_msgs/Twist`), the `twist_mux` input. The node
  never publishes on `safety_stop` or on any lock topic.
- Subscriptions:
  - `detection_topic` (`dock/detected_pose`, PoseStamped);
  - `safety_stop_topic` (`safety_stop`, `std_msgs/Bool`), read only. While locked, the node commands
    zero, and the phase timers keep running;
  - `battery_topic` (`battery_state`, BatteryState), for `charging` / `battery_percentage` only;
  - `mission_status_topic` (`mission/status`, MissionStatus, transient_local).
- TF reads: `global_frame -> robot_frame`, `global_frame -> odom_frame`, `odom_frame -> robot_frame`.
  Detections are transformed into `odom_frame` at their stamp, with a fallback to the latest
  transform. The node publishes no TF.
- Publisher `/dock/status` (`DockStatus`), reliable, transient_local, depth 1. Published at startup
  (UNDOCKED, with `staging_pose` / `dock_pose` filled), on every state change and at `status_rate`.
  The status timer is a wall timer, so it keeps publishing when `/clock` stalls.
- Action feedback on every state change and at `status_rate` while active.

Parameters (declared in code, same defaults in both languages, also in
`config/dock_manager.yaml`):

| Name | Default | Meaning |
|---|---|---|
| `stations_file` | `""` -> `<share bumperbot_navigation>/config/stations.yaml` | via ament_index |
| `dock_station` | `""` -> `dock_station` of the stations file | staging station name |
| `nav_action_name` | `navigate_to_pose` | |
| `global_frame` / `odom_frame` / `robot_frame` | `map` / `odom` / `base_footprint` | must be 3 distinct names, and `global_frame` must equal the stations `frame_id` |
| `cmd_vel_topic` | `dock_vel` | twist_mux input |
| `detection_topic` / `safety_stop_topic` / `battery_topic` / `mission_status_topic` | `dock/detected_pose` / `safety_stop` / `battery_state` / `mission/status` | |
| `dock_backwards` | true | back in (false: drive in forward) |
| `staging_distance` | 0.5 | m, staging pose to charger pose |
| `staging_tolerance` / `staging_yaw_tolerance` | 0.25 / 0.6 | skip Nav2 when within (m / rad) |
| `nav_max_retries` | 2 | extra Nav2 attempts to staging |
| `nav_server_timeout` / `nav_goal_timeout` | 10.0 / 180.0 | s |
| `detection_mode` | `optional` | `none` / `optional` / `required` (other value: refuse to start) |
| `detection_timeout` | 3.0 | s to wait for a detection at the pre-dock pose |
| `detection_max_age` | 0.5 | s, older detections are ignored |
| `detection_max_jump` | 0.3 | m, detections farther from the current estimate are rejected |
| `detection_filter_alpha` | 0.3 | EMA weight of a new detection (0 < a <= 1) |
| `detection_offset_x` / `_y` / `_yaw` | 0.0 | marker pose to docked pose offset |
| `align_position_tolerance` / `align_yaw_tolerance` | 0.02 / 0.03 | pre-dock pose tolerance |
| `dock_position_tolerance` / `dock_yaw_tolerance` | 0.03 / 0.05 | final verify tolerance |
| `max_linear_speed` / `max_angular_speed` | 0.15 / 0.6 | m/s, rad/s (hard clamp) |
| `approach_speed` / `slow_down_distance` | 0.05 / 0.15 | final segment speed and length |
| `k_lateral` / `k_heading` | 2.0 / 2.0 | line-tracking gains |
| `max_insert_lateral_error` / `max_insert_yaw_error` | 0.08 / 0.3 | abort the attempt when exceeded during insertion |
| `max_retries` | 2 | extra docking attempts (3 total) |
| `align_timeout` / `docking_timeout` / `undocking_timeout` | 30.0 / 40.0 / 30.0 | s per phase |
| `stall_timeout` | 5.0 | s with a non-zero command and < 0.01 m / 0.02 rad of motion -> failed attempt |
| `safety_stop_timeout` | 10.0 | s locked inside one phase -> failed attempt |
| `settle_time` | 0.5 | s stopped before verify |
| `undock_detect_distance` | 0.10 | m from the docked pose -> state UNDOCKED |
| `tf_timeout` / `tf_stale_timeout` | 1.0 / 0.5 | s; stale odom TF commands zero, lost > tf_timeout -> ERROR |
| `control_rate` / `status_rate` | 20.0 / 2.0 | Hz |
| `clock_stall_timeout` | 5.0 | wall s without node-clock progress during a goal -> ERROR |
| `use_sim_time` | false (true from the sim launch) | |

Startup validation (both languages, the node logs the reason and exits with code != 0 within 5 s,
without publishing velocity):
- the stations file fails `load_stations`;
- the dock station is missing;
- the stations `frame_id` differs from `global_frame`;
- the frame names are not distinct;
- `detection_mode` is invalid;
- any tolerance, speed, rate or timeout is <= 0;
- `detection_filter_alpha` is outside (0, 1].

### Control (pure `dock_logic`, same in both languages)

- Types: `Pose2D{x, y, yaw}`, `normalize_angle`, `compose(a, b)`, `inverse(a)`,
  `relative(frame, pose)` (pose expressed in frame).
- `compute_dock_pose(staging, staging_distance, dock_backwards)`: with backwards docking it is
  `staging - d * (cos yaw, sin yaw)` with the same yaw, otherwise `+ d`. On the shipped config:
  staging (0, 0, 0) gives charger (-0.5, 0, 0). AC4 checks the corridor on the map.
- The controller works on the robot pose `p` expressed in the dock frame D (origin = charger pose,
  x axis = docked heading). `L = staging_distance`, pre-dock pose `P = (s*L, 0, 0)` with `s = +1`
  for backwards and `-1` for forwards. Every output is clamped to `max_linear_speed` /
  `max_angular_speed`.
  - `Aligner` (rotate, drive, rotate, with hysteresis) brings `p` to `P` within the align
    tolerances. `align_command(p) -> Command{v, w, done}`.
  - `insert_command(p)`:
    - `v = -s * (max_linear_speed if |x| > slow_down_distance else approach_speed)`, set to 0 when
      `s*x <= 0`;
    - `theta_des = -sign(v) * clamp(k_lateral * y, +-0.3)`;
    - `w = k_heading * normalize(theta_des - yaw)`;
    - `done` when `s*x <= 0`;
    - `failed` when `|y| > max_insert_lateral_error` or `|yaw| > max_insert_yaw_error`.
  - `undock_command(p)`: the same line tracking with `v` of the opposite sign, target `x = s*L`,
    `done` when `s*x >= L - 0.01`.
  - `pose_error(p) -> (hypot(x, y), |yaw|)`.
- `DetectionFilter(max_jump, alpha)`: `update(estimate_in_odom, detection_in_odom) -> (accepted,
  new_estimate)`. It rejects non-finite values and detections more than `max_jump` from the
  estimate. Otherwise it does an EMA on x, y and circular yaw.
- `PhaseTimer(timeout, clock_stall_timeout)`: `start(node_now, wall_now)`, `expired(node_now,
  wall_now)`. Time is node time. When node time has not advanced for `clock_stall_timeout` wall
  seconds, `stalled()` is true and the phase counts as expired.
- `DockTracker(params)` (no clock, no threads, the node drives it):
  - `start(command, docked, near_staging)`, which resets **every** per-goal field (attempt, docked,
    detection_used, errors, message);
  - `next_step() -> Step{kind: NAVIGATE_STAGING | WAIT_DETECTION | ALIGN | INSERT | SETTLE_VERIFY |
    UNDOCK | FINISHED, attempt}`;
  - `report_nav(ok)`, `report_detection(ok)`, `report_phase(result: OK | TIMEOUT | STALL |
    SAFETY_TIMEOUT | MISALIGNED)`, `report_verify(position_error, yaw_error)`, `request_cancel()`,
    `abort(reason)` (TF lost, clock stall, exception, shutdown);
  - read-only: `state`, `attempt`, `outcome`, `docked`, `message`.
  - Rules:
    - a failed ALIGN/INSERT/VERIFY goes back to ALIGN with attempt + 1, up to `max_retries + 1`
      attempts, then DOCKING_FAILED;
    - Nav2 gets `nav_max_retries + 1` attempts, then STAGING_FAILED;
    - `required` without a detection gives DETECTION_FAILED, and `optional` continues map-only;
    - cancel or abort gives FINISHED immediately (CANCELED / ERROR) with `docked = false`;
    - `docked = true` only after a successful verify, and it is cleared by the first motion of
      any later goal.

### Node runtime (both languages)

- Python: `MultiThreadedExecutor` with a `ReentrantCallbackGroup`. The goal work runs in a dedicated
  joinable `threading.Thread` started from the execute callback, which waits on it. C++: a
  joinable `std::thread`, joined before a new goal starts and in the destructor (no detached thread,
  the same pattern as the reviewed `mission_manager.cpp`).
- Worker body: `try { run } catch (everything) { abort(ERROR) } finally { publish zero 3x;
  cancel Nav2 goal if active; active = false; finish the goal handle exactly once (canceled if
  cancel requested, otherwise succeed/abort per outcome) }`.
- Cancel races are handled:
  - a cancel before the first control step, or between phases, is seen by the next poll (period
    1/control_rate);
  - a terminal result checks `is_cancel_requested` before succeed/abort;
  - a cancel after the terminal decision is ignored.
- Signals: custom SIGINT/SIGTERM handler (rclcpp default disabled, as in `mission_manager.cpp`;
  Python uses `signal.signal`). It sets `shutdown`. The worker stops the robot (zero velocity),
  cancels Nav2, aborts the goal with `OUTCOME_ERROR "shutdown"`, and the process exits 0 within 10 s.
- Control loop: wall-rate sleep at `control_rate`. Timeouts use `PhaseTimer` on the node clock with
  the wall fallback. A stale odom TF (> `tf_stale_timeout`) commands zero.
- Docked watch (status timer): in DOCKED/CHARGING, an `odom -> robot_frame` pose more than
  `undock_detect_distance` from the docked pose sets state UNDOCKED and `docked = false`.
  CHARGING <-> DOCKED follows `battery_state.power_supply_status == CHARGING`.

### `battery_monitor` (Python, simulator independent)

Interfaces:
- Subscribes `battery_state` (BatteryState), `dock/status` (DockStatus) and `mission/status`
  (MissionStatus).
- Action clients `dock_robot` (DockRobot) and `execute_mission` (ExecuteMission). It cancels a
  mission it did not start through the standard action protocol: the `action_msgs/srv/CancelGoal`
  service `execute_mission/_action/cancel_goal` with a zero `goal_info`, which means cancel all.
- Publishes `battery_monitor/state` (`std_msgs/String`, transient_local).
- Service `~/reset` (`std_srvs/Trigger`) leaves ERROR.

Parameters (`config/battery_monitor.yaml`):

| Name | Default |
|---|---|
| `low_threshold` | 0.20 |
| `full_threshold` | 0.95 |
| `rearm_hysteresis` | 0.05 |
| `battery_timeout` | 5.0 s |
| `mission_cancel_timeout` | 180.0 s |
| `dock_retry_delay` | 5.0 s |
| `dock_request_timeout` | 120.0 s |
| `max_dock_attempts` | 3 |
| `undock_when_full` | true |
| `update_rate` | 2.0 Hz |
| `clock_stall_timeout` | 5.0 s |

Pure `BatteryMonitorLogic` with the states WAITING_FOR_BATTERY, MONITORING, CANCELING_MISSION,
DOCKING, CHARGING, UNDOCKING, ERROR:
- No battery message, a NaN percentage, or data older than `battery_timeout` gives
  WAITING_FOR_BATTERY and never triggers.
- MONITORING with `p < low_threshold` and not CHARGING:
  - with an active mission it goes to CANCELING_MISSION (send cancel-all). When the mission status
    is terminal it goes to DOCKING. On `mission_cancel_timeout` it goes to DOCKING anyway, and the
    dock goal may be rejected while the mission is still active, which then follows the reject rule
    below;
  - otherwise it goes straight to DOCKING.
- DOCKING sends DOCK:
  - rejected: retry after `dock_retry_delay` without counting an attempt, ERROR after
    `dock_request_timeout`;
  - DOCKED / ALREADY_DOCKED: CHARGING;
  - any other result: attempt + 1 and retry after the delay, ERROR at `max_dock_attempts`.
- CHARGING:
  - `p >= full_threshold` and `undock_when_full`: UNDOCKING (send UNDOCK);
  - `dock/status.docked` false (pushed or driven away): MONITORING. The flag is cleared, so the
    monitor never stays CHARGING off the dock.
- UNDOCKING: UNDOCKED gives MONITORING, and the low trigger re-arms only when
  `p >= low_threshold + rearm_hysteresis`. A failure gives ERROR.
- Shutdown: cancel its own active DockRobot goal.

### `battery_sim` (Python, SIMULATION ONLY)

Pure `BatteryModel`:
- `percentage` stays in [0, 1].
- Drain (fraction/s) = `idle_drain_rate + linear_drain_rate * |v| + angular_drain_rate * |w|`,
  with v and w from `/bumperbot_controller/odom`.
- Charge = `charge_rate` while `/dock/status.docked`.
- `dt` is clamped to [0, 1] s. A paused simulation gives no change.
- `power_supply_status` is CHARGING (docked, < 1), FULL (docked, >= 1) or DISCHARGING.
- Voltage is linear between `empty_voltage` 10.5 V and `full_voltage` 12.6 V. `present = true`.

The node publishes `battery_state` at `publish_rate` (2 Hz). Parameter `percentage_override`
(default -1.0): when it is set to a value >= 0 at runtime the charge jumps to it once, for demos and
the E2E test. Config: `config/battery_sim.yaml`, with the header comment "simulation only". It is
started only by `dock_sim.launch.py`.

### `dock_marker_sim` and `/mujoco/ground_truth` (bumperbot_mujoco, SIMULATION ONLY)

- `mujoco_sim_node` publishes `/mujoco/ground_truth` (`nav_msgs/Odometry`, `frame_id world`,
  `child_frame_id base_footprint`, from `BumperbotSim.base_pose()`) at 50 Hz with the sim stamp. The
  MuJoCo world frame equals `map`, because the robot spawns at (0, 0, 0) and AMCL starts there.
- `dock_marker_sim` takes the parameters `dock_x/y/yaw` (charger pose in world, config
  `bumperbot_mujoco/config/dock_marker_sim.yaml` = (-0.5, 0, 0)), `max_range` 2.0, `fov` 2.1 rad,
  `facing` `rear`, `position_noise` 0.005, `yaw_noise` 0.01, `seed` 7 and `rate` 10 Hz.
- Pure function `emulate_detection(gt_pose, dock_pose, max_range, fov, facing) -> Pose2D | None`:
  the charger pose in `base_footprint`, or None outside range or field of view.
- It publishes PoseStamped in `base_footprint` with the ground-truth stamp.

### TF

```
map -(amcl)-> odom -(diff_drive_controller)-> base_footprint -> base_link -> laser_link ...
```
- `dock_manager` reads `map -> base_footprint`, `map -> odom`, `odom -> base_footprint`, and
  transforms detections from their `frame_id` (sim: `base_footprint`; real: e.g. a camera frame)
  into `odom`.
- No new TF frame is published. `/mujoco/ground_truth` uses a `world` frame id only inside that
  message and is not in TF.
- Frames are checked at startup (see validation). The config consistency test (AC19) also checks
  `mission_manager.yaml` and `dock_manager.yaml` use the same `global_frame` / `robot_frame`, and
  that Nav2 `robot_base_frame` is `base_footprint`.

### twist_mux and safety

- `bumperbot_controller/config/twist_mux_topics.yaml` gets a new input
  `docking: {topic: dock_vel, timeout: 0.5, priority: 30}`. The order becomes joystick 99 >
  keyboard 90 > docking 30 > navigation 20 > explore 10.
- Docking sits above Nav2, so a stale `nav_vel` cannot fight the insertion, and below human
  teleop. `twist_mux_locks.yaml` is unchanged: the `safety_stop` lock 255 overrides docking in sim
  and on the real robot.
- `dock_manager` treats a lock as a pause bounded by `safety_stop_timeout`, then a failed attempt.
- On the real robot nothing publishes `dock_vel` unless `docking.launch.py` is started (it is not
  wired into `real_robot.launch.py`).

### Launch

- `bumperbot_navigation/launch/docking.launch.py` (new, simulator independent).
  - Args: `use_sim_time` (`false`), `use_python` (`false`), `use_battery_monitor` (`true`),
    `stations_file`, `dock_config`, `battery_config` (defaults via `get_package_share_directory`).
  - Starts one `dock_manager` (C++ unless `use_python`) and, when enabled, `battery_monitor`.
- `bumperbot_bringup/launch/dock_sim.launch.py` (new).
  - Args: `simulator` (`mujoco`), `world_name` (`small_house`), `use_rviz` (`true`), `viewer`
    (`true`), `use_python` (`false`), `battery_initial` (`1.0`), `battery_sim_config`.
  - Includes `mission_sim.launch.py` (forwarding simulator, world, rviz, viewer, use_python) and
    `docking.launch.py` with `use_sim_time:=true`.
  - Starts `battery_sim` (`use_sim_time: True`, `initial_percentage: battery_initial`).
  - Starts `dock_marker_sim` only when `simulator == mujoco`.

### Data flow

```
 /battery_state (BatteryState) <-- battery_sim [SIM ONLY] <-- /bumperbot_controller/odom, /dock/status
        |
        v
 battery_monitor --cancel-all--> /execute_mission (mission_manager returns to `dock` = staging)
        |  ^ /mission/status
        | DockRobot DOCK / UNDOCK
        v
 +-------------------------------------------+ --> /dock/status (DockStatus, transient_local)
 | dock_manager (C++ | Python)               | --> DockRobot feedback / result
 |   DockTracker, Aligner, DetectionFilter   | <-- /dock/detected_pose (PoseStamped)
 +-------------------------------------------+ <-- /safety_stop (Bool, read only)
     | NavigateToPose(staging)    | dock_vel (prio 30)     ^ TF map->odom->base_footprint
     v                            v                        |
   Nav2 --nav_vel(20)--> twist_mux <-- joy(99) key(90) safety_stop lock(255)
                            | /bumperbot_controller/cmd_vel_unstamped -> diff_drive
                            v
               MuJoCo / Gazebo / real robot --> /scan, odom, /mujoco/ground_truth [SIM ONLY]
                                                        |
                         dock_marker_sim [SIM ONLY] ----+--> /dock/detected_pose
```

In the node tests, Nav2 + simulator + twist_mux are replaced by `MockDockWorld`. It is one rclpy
node in the test process:
- a kinematic diff-drive robot integrating `dock_vel` at 50 Hz, with true pose T;
- TF `odom -> base_footprint` = T, plus an optional slip that odom does not see;
- `map -> odom` = configurable localization error E;
- a mock `navigate_to_pose` that sets T so that E*T equals the goal plus a configurable error, with
  scripted aborts;
- optional detections from T, and optional outliers / NaN / bad frames;
- `/safety_stop` gating (commands ignored while locked);
- stall mode (commands ignored);
- an optional `/clock` publisher that can be stopped;
- recording of every command, Nav2 goal and cancel.

## Files

| Path | create/modify | Purpose |
|---|---|---|
| `src/bumperbot_msgs/action/DockRobot.action` | create | Dock / undock action |
| `src/bumperbot_msgs/msg/DockStatus.msg` | create | `/dock/status` message |
| `src/bumperbot_msgs/CMakeLists.txt` | modify | Generate the two new interfaces |
| `src/bumperbot_navigation/include/bumperbot_navigation/dock_logic.hpp` | create | C++ pure dock logic API |
| `src/bumperbot_navigation/src/dock_logic.cpp` | create | C++ pure dock logic |
| `src/bumperbot_navigation/include/bumperbot_navigation/dock_manager.hpp` | create | C++ node class |
| `src/bumperbot_navigation/src/dock_manager.cpp` | create | C++ node (joinable worker, signal handling) |
| `src/bumperbot_navigation/bumperbot_navigation/dock_logic.py` | create | Python pure dock logic |
| `src/bumperbot_navigation/bumperbot_navigation/dock_manager.py` | create | Python node (executable `dock_manager.py`) |
| `src/bumperbot_navigation/bumperbot_navigation/battery_logic.py` | create | `BatteryModel` + `BatteryMonitorLogic` |
| `src/bumperbot_navigation/bumperbot_navigation/battery_monitor.py` | create | Battery monitor node |
| `src/bumperbot_navigation/bumperbot_navigation/battery_sim.py` | create | SIM-ONLY battery node |
| `src/bumperbot_navigation/config/dock_manager.yaml` | create | dock_manager parameters |
| `src/bumperbot_navigation/config/battery_monitor.yaml` | create | battery_monitor parameters |
| `src/bumperbot_navigation/config/battery_sim.yaml` | create | SIM-ONLY battery model parameters |
| `src/bumperbot_navigation/launch/docking.launch.py` | create | dock_manager + battery_monitor |
| `src/bumperbot_navigation/CMakeLists.txt` | modify | `dock_logic` lib (links `mission_logic`), `dock_manager` exe, install 4 Python executables, register tests |
| `src/bumperbot_navigation/package.xml` | modify | `sensor_msgs`, `std_msgs`, `std_srvs`, `action_msgs` depend |
| `src/bumperbot_navigation/test/data/dock_cases.yaml` | create | Shared parity cases (geometry, controller, filter, timer, tracker) |
| `src/bumperbot_navigation/test/data/battery_cases.yaml` | create | BatteryModel + BatteryMonitorLogic cases |
| `src/bumperbot_navigation/test/test_dock_logic.py` | create | AC2 (Python) |
| `src/bumperbot_navigation/test/test_dock_logic.cpp` | create | AC2 (C++ gtest) |
| `src/bumperbot_navigation/test/test_dock_geometry_on_map.py` | create | AC4 |
| `src/bumperbot_navigation/test/mock_dock_world.py` | create | `MockDockWorld` helper |
| `src/bumperbot_navigation/test/dock_harness.py` | create | Process-level helpers (start exe, signals, clock) |
| `src/bumperbot_navigation/test/test_dock_manager.launch.py` | create | AC5-AC13, parametrized cpp/py |
| `src/bumperbot_navigation/test/test_dock_robustness.py` | create | AC3, AC14 (process-level, both impls) |
| `src/bumperbot_navigation/test/test_battery_logic.py` | create | AC15 |
| `src/bumperbot_navigation/test/test_battery_nodes.launch.py` | create | AC16, AC17 |
| `src/bumperbot_navigation/test/test_dock_static.py` | create | AC1, AC20 |
| `src/bumperbot_navigation/test/test_dock_docs.py` | create | AC23 |
| `src/bumperbot_mujoco/bumperbot_mujoco/mujoco_sim_node.py` | modify | `/mujoco/ground_truth` publisher |
| `src/bumperbot_mujoco/bumperbot_mujoco/dock_marker_sim.py` | create | SIM-ONLY detector emulator + `emulate_detection` |
| `src/bumperbot_mujoco/config/dock_marker_sim.yaml` | create | Charger pose in world, noise, FOV |
| `src/bumperbot_mujoco/setup.py` | modify | `dock_marker_sim` entry point, install `config/` |
| `src/bumperbot_mujoco/package.xml` | modify | `nav_msgs`, `geometry_msgs` exec_depend; `bumperbot_navigation` test_depend |
| `src/bumperbot_mujoco/test/test_dock_precision_sim.py` | create | AC21 (BumperbotSim closed loop, no ROS) |
| `src/bumperbot_mujoco/test/test_dock_marker_sim.py` | create | AC22 |
| `src/bumperbot_controller/config/twist_mux_topics.yaml` | modify | `docking` input `dock_vel`, priority 30 |
| `src/bumperbot_bringup/launch/dock_sim.launch.py` | create | Simulation launch |
| `src/bumperbot_bringup/test/test_dock_launch_config.py` | create | AC19 |
| `src/bumperbot_bringup/test/test_dock_twist_mux.launch.py` | create | AC18 (real twist_mux + repo configs) |
| `src/bumperbot_bringup/test/test_dock_sim_mujoco.launch.py` | create | AC24 smoke + AC25 opt-in E2E |
| `src/bumperbot_bringup/CMakeLists.txt` | modify | Register the 3 new tests |
| `src/bumperbot_bringup/package.xml` | modify | `twist_mux` test depend (exec already through controller) |
| `README.md` | modify | Capabilities, "Auto-docking and battery" quick start, commands |
| `doc/10_navigation.md` | modify | Docking design, detection choice, phases, retries, safety, battery monitor |
| `doc/03_ros2_nodes_topics_services.md` | modify | New nodes, topics, actions, interfaces, `dock_vel` |
| `doc/04_tf_and_coordinate_frames.md` | modify | Frames used by docking, detection frame, `world == map` in MuJoCo |
| `doc/05_robot_description_and_simulation.md` | modify | `/mujoco/ground_truth`, `dock_marker_sim`, `battery_sim` (sim only) |
| `doc/01_repository_structure.md` | modify | New files in the tree |
| `doc/02_system_architecture.md` | modify | Docking layer, twist_mux priorities |
| `doc/11_build_run_and_command_reference.md` | modify | Build / run / test commands |
| `doc/12_troubleshooting.md` | modify | Safety lock near a physical dock, missing detection, AMCL bias |

Existing fixtures reused: `test/data/stations_test.yaml`, `stations_other_frame.yaml`,
`valid_exponent.yaml` and all `invalid_*.yaml`.

## Acceptance criteria

- AC1: `bumperbot_msgs` builds `DockRobot.action` and `DockStatus.msg` with exactly the fields and
  constants above. `ros2 interface show` succeeds for both, and `ExecuteMission`, `MissionStatus`,
  `Fibonacci`, `AddTwoInts` and `GetTransform` are still generated.
- AC2: Python and C++ `dock_logic` give identical results for every case in
  `test/data/dock_cases.yaml`. Numbers match within 1e-9 and step sequences and final fields match
  exactly. The table covers at least:
  - geometry: `compute_dock_pose` backwards and forwards, `normalize_angle` at +-pi, compose/inverse
    round trip;
  - controller: clamping to the max speeds; `v <= 0` during backwards insertion and `>= 0` during
    undock; `done` / `failed` flags;
  - `DetectionFilter`: NaN rejected, a jump > `max_jump` rejected, EMA with yaw wrap;
  - `PhaseTimer`: node-time expiry, and wall fallback when node time stops for
    `clock_stall_timeout`;
  - `DockTracker`: success on the first attempt; Nav2 skipped when near staging; Nav2 fails
    `nav_max_retries + 1` times -> STAGING_FAILED; `required` without a detection ->
    DETECTION_FAILED; `optional` without a detection -> continues; verify failure -> retry ->
    success (attempts = 2); `max_retries + 1` failures (TIMEOUT / STALL / SAFETY_TIMEOUT /
    MISALIGNED) -> DOCKING_FAILED; cancel in every phase -> CANCELED with `docked = false`; abort ->
    ERROR; UNDOCK success and failure; DOCK while docked -> ALREADY_DOCKED; reuse of the same tracker
    after a failure starts with attempt 1, an empty message and `docked = false` (no flag survives).
- AC3: config parsing parity at node level. Both executables are started with `config/stations.yaml`,
  `test/data/stations_test.yaml` and `test/data/valid_exponent.yaml`. Each publishes a first
  `/dock/status` (UNDOCKED) with identical `staging_pose`, `dock_pose` and `frame_id` (1e-9). For the
  shipped config these are (0, 0, 0) and (-0.5, 0, 0). Each executable exits with code != 0 within
  5 s, without publishing on `dock_vel`, for:
  - every `test/data/invalid_*.yaml` (duplicate key, quoted number, null, NaN, missing field, no
    dock, empty, not a mapping);
  - `stations_other_frame.yaml`;
  - `odom_frame == robot_frame`;
  - `detection_mode: bogus`;
  - `staging_distance: 0.0`.
- AC4: the charger pose computed from the shipped config, and every point of the straight insertion
  segment from the staging pose to the charger pose (5 cm steps), lie on free cells of
  `bumperbot_mapping/maps/small_house/map.pgm` with >= 0.30 m clearance.
  `bumperbot_mujoco/config/dock_marker_sim.yaml` `dock_x/y/yaw` equals that charger pose (1e-6).
- AC5: (node, `impl=cpp` and `impl=py`, `MockDockWorld`, perfect localization, detections on). The
  robot starts 2 m from staging and a DOCK goal is sent.
  - Exactly one NavigateToPose goal is sent, with `frame_id == map` and the pose equal to the staging
    pose (1e-6).
  - The mock puts the robot at staging with an error of (+0.08, -0.06, +0.2 rad).
  - The result is SUCCEEDED, `OUTCOME_DOCKED`, `docked == true`, `attempts == 1`.
  - The true final pose error vs the charger pose is <= 0.03 m and <= 0.05 rad.
  - `/dock/status` passes NAVIGATING_TO_STAGING -> ALIGNING -> DOCKING -> DOCKED in that order.
  - Every `dock_vel` command satisfies `|v| <= max_linear_speed` and `|w| <= max_angular_speed`;
    in DOCKING `v <= 0`; the last command is exactly zero.
- AC6: (node, both impls) a robot already within `staging_tolerance` of staging gets a DOCK goal. Zero
  NavigateToPose goals are sent and it docks with the AC5 tolerances.
- AC7: (node, both impls) detection refinement, with a mock localization error E = (0.08 m, -0.05 m,
  0.04 rad).
  - (a) `detection_mode: optional` with detections: true final error <= 0.03 m / 0.05 rad, and
    `detection_used == true` in the result feedback/status.
  - (b) the same with `detection_mode: none`: true final error >= 0.06 m. This shows the detection
    is what removes the bias.
  - (c) one outlier detection 0.5 m off plus NaN, zero-quaternion, empty-frame and unknown-frame
    messages during DOCKING: they are ignored, the node keeps running and (a) still holds.
  - (d) `detection_mode: required` and no detections: OUTCOME_DETECTION_FAILED within
    `detection_timeout + 2` s after reaching the pre-dock pose, `docked == false`, last command zero,
    no DOCKING state.
- AC8: (node, both impls) retries and failures.
  - (a) The mock applies an unobserved lateral slip of 0.06 m during the first insertion only, with
    detections on: a second ALIGN + DOCKING happens, the result is OUTCOME_DOCKED with
    `attempts == 2`, and the true final error is within the AC5 tolerances.
  - (b) Stall mode (commands ignored): `max_retries + 1` attempts, then ABORTED,
    `OUTCOME_DOCKING_FAILED`, `docked == false`, state FAILED, last command zero, total time <=
    `(max_retries + 1) * (stall_timeout + 2)` s (test uses `stall_timeout: 2.0`).
  - (c) Nav2 always aborts: exactly `nav_max_retries + 1` Nav2 goals, then
    `OUTCOME_STAGING_FAILED`, no non-zero `dock_vel`.
- AC9: (node, both impls) safety lock.
  - (a) `/safety_stop` true for 2 s in the middle of DOCKING: while it is true every published
    `dock_vel` is zero within one control period, and after release docking completes with
    OUTCOME_DOCKED.
  - (b) lock held longer than `safety_stop_timeout` (test: 3 s) on every attempt:
    `OUTCOME_DOCKING_FAILED`, last command zero.
  - (c) the node never publishes on `safety_stop` (topic info shows 0 publishers from
    `dock_manager`).
- AC10: (node, both impls) cancel and error paths end in a safe, defined state.
  - (a) Cancel during NAVIGATING_TO_STAGING: the mock gets a cancel for the Nav2 goal, the result is
    CANCELED, `OUTCOME_CANCELED`, and no non-zero `dock_vel` follows.
  - (b) Cancel during ALIGNING and (c) cancel during DOCKING: a zero command within 0.2 s, no
    non-zero command after it, CANCELED, `docked == false`, state UNDOCKED.
  - (d) Cancel within 50 ms of goal acceptance (race): CANCELED within 1 s, no non-zero command.
  - (e) The mock stops publishing `odom -> base_footprint` during DOCKING: zero command within
    `tf_stale_timeout + 0.2` s, `OUTCOME_ERROR` within `tf_timeout + 1` s, state FAILED.
  - (f) The mock Nav2 server is destroyed mid-goal: STAGING_FAILED or ERROR within
    `nav_server_timeout + 2` s and zero velocity.
  - After each of (a)-(f) a new DOCK goal is accepted and its first feedback shows `attempt == 1`.
- AC11: (node, both impls) docked flag lifecycle.
  - After OUTCOME_DOCKED, a DOCK goal returns `OUTCOME_ALREADY_DOCKED` with no non-zero command.
  - The mock then moves the robot 0.15 m away: within 1 s `/dock/status` shows UNDOCKED with
    `docked == false`.
  - The next DOCK goal runs a full docking: ALIGNING is seen and the result is OUTCOME_DOCKED, not
    ALREADY_DOCKED.
  - While docked, a mock `/battery_state` with CHARGING gives state CHARGING and `charging == true`;
    FULL gives back DOCKED.
- AC12: (node, both impls) goal acceptance, each decision within 1 s.
  - Rejected: a second goal while one is active, and the first still succeeds.
  - Rejected: UNDOCK while UNDOCKED.
  - Rejected: `command = 7`.
  - Rejected: DOCK while `/mission/status` state is NAVIGATING.
  - Rejected: DOCK while no TF `map -> base_footprint` exists. No Nav2 goal, no `dock_vel` message
    and no attempt are recorded. After the mock starts TF, the same goal is accepted within 2 s.
  - Rejected: DOCK needing staging while the Nav2 server is absent. Accepted after it appears.
- AC13: (node, both impls) undock.
  - From DOCKED, UNDOCK gives SUCCEEDED, `OUTCOME_UNDOCKED`, `docked == false`, state UNDOCKED.
  - The true final pose is within 0.05 m / 0.1 rad of the staging pose, `v >= 0` throughout
    (backwards docking) and the last command is zero.
  - A cancel during UNDOCKING gives a zero command within 0.2 s, CANCELED and state UNDOCKED.
- AC14: (process level, both impls) robustness.
  - (a) SIGINT and (b) SIGTERM during DOCKING: the last `dock_vel` message is zero, an active Nav2
    goal (signal during NAVIGATING_TO_STAGING) is canceled, the client sees ABORTED with
    `OUTCOME_ERROR`, and the process exits with code 0 within 10 s.
  - (c) `use_sim_time: true` with a test `/clock` that stops during DOCKING: a zero command and
    ABORTED `OUTCOME_ERROR` within `clock_stall_timeout + 2` s. `/dock/status` keeps arriving at
    least once per 1.5 s wall time during the stall.
  - (d) `use_sim_time: true` with a running `/clock`: AC5 also passes.
- AC15: (pytest, pure Python) `battery_logic` passes every case in `test/data/battery_cases.yaml`:
  - `BatteryModel`: idle drain, faster drain when moving, charge while docked, clamp to [0, 1],
    negative/huge dt clamped, status DISCHARGING/CHARGING/FULL, voltage mapping, override;
  - `BatteryMonitorLogic`:
    - low with no mission -> DOCK;
    - low with an active mission -> cancel-all first, DOCK only after the terminal mission state
      (and also after `mission_cancel_timeout`);
    - no retrigger before `low_threshold + rearm_hysteresis`;
    - a stale or NaN battery never triggers;
    - rejected DOCK -> retries not counted -> ERROR after `dock_request_timeout`;
    - `max_dock_attempts` failures -> ERROR;
    - CHARGING -> UNDOCK at `full_threshold`;
    - `docked` false while CHARGING -> MONITORING;
    - UNDOCK failure -> ERROR, and reset -> MONITORING.
- AC16: (node, `launch_testing`) `battery_monitor` against a mock DockRobot server, a mock
  ExecuteMission server publishing `/mission/status`, and a mock `/battery_state`:
  - (a) percentage 0.5 -> 0.15 with no mission: exactly one DOCK goal within 2 s.
  - (b) with an active mission: a CancelGoal request with a zero goal id reaches
    `execute_mission` before any DOCK goal, and DOCK is sent only after the status turns CANCELED.
  - (c) dock result DOCKED, then a mock charge to 0.96 with CHARGING: exactly one UNDOCK goal.
    `battery_monitor/state` goes MONITORING -> DOCKING -> CHARGING -> UNDOCKING -> MONITORING.
  - (d) every DOCK rejected: retries every `dock_retry_delay` (test 0.5 s), then ERROR after
    `dock_request_timeout` (test 3 s).
  - (e) battery silent for longer than `battery_timeout` (test 1 s) after a low value:
    WAITING_FOR_BATTERY and no goal.
  - (f) SIGINT while DOCK is active: the mock DockRobot server receives a cancel.
- AC17: (node, `launch_testing`) `battery_sim` publishes `/battery_state` at 2 Hz (+-30 %) with
  `present == true` and `0 <= percentage <= 1`.
  - Status is DISCHARGING with no dock status and CHARGING after a mock `/dock/status` with
    `docked == true`; the percentage rises while docked.
  - With a mock odom twist of 0.2 m/s it drains faster than idle.
  - `ros2 param set battery_sim percentage_override 0.1` gives 0.1 +- 0.01 within 1 s.
  - Statically, `battery_sim` and `dock_marker_sim` appear only in `dock_sim.launch.py`, never in
    `docking.launch.py`, `real_robot.launch.py` or any non-test file of `bumperbot_firmware`.
- AC18: (`launch_testing`, real `twist_mux` with the repo's `twist_mux_topics.yaml` /
  `twist_mux_locks.yaml`, output remapped as in `joystick_teleop.launch.py`):
  - (a) with `dock_vel` at 0.1 m/s and `safety_stop` true, no non-zero Twist appears on
    `bumperbot_controller/cmd_vel_unstamped` for 2 s;
  - (b) with `safety_stop` false, `dock_vel` passes;
  - (c) with `nav_vel` and `dock_vel` both publishing, the output equals `dock_vel`;
  - (d) `key_vel` overrides `dock_vel`.
  - Statically, docking priority is > navigation and < keyboard, and `twist_mux_locks.yaml` still
    has `safety_stop` at 255 (unchanged vs `main`).
- AC19: launch and configuration.
  - `ros2 launch bumperbot_navigation docking.launch.py --show-args` and
    `ros2 launch bumperbot_bringup dock_sim.launch.py --show-args` succeed and list the args above.
  - No string literal in either file starts with `/` (except topic names) and both use
    `get_package_share_directory`.
  - `dock_sim.launch.py` includes `mission_sim.launch.py` and `docking.launch.py`
    (`use_sim_time` true), creates exactly one `dock_manager` (C++ by default, Python with
    `use_python:=true`), one `battery_sim`, and `dock_marker_sim` only for `simulator:=mujoco`.
  - `mission_manager.yaml` and `dock_manager.yaml` have equal `global_frame` / `robot_frame`, and
    Nav2 `robot_base_frame == base_footprint`.
  - `real_robot.launch.py`, `simulated_robot.launch.py` and `mission_sim.launch.py` are unchanged
    vs `main`, and their `--show-args` still succeed.
  - The existing `test_launch_config.py` / `test_mission_launch_config.py` still pass.
- AC20: static checks.
  - `dock_manager.cpp/.hpp/.py`, `dock_logic.*`, `battery_monitor.py` and `battery_logic.py` contain
    no `mujoco`, `gazebo`, `\bgz\b` or `/clock` tokens (case-insensitive).
  - All new and modified source, YAML and launch files have ASCII-only comments.
  - Both `dock_manager` sources declare the same parameter names with the same defaults, equal to
    `config/dock_manager.yaml` and the table above.
  - At runtime (inside the AC5 launch test), `list_parameters` / `get_parameters` on both running
    nodes return identical name/value sets.
- AC21: (headless MuJoCo, pytest, no ROS) `BumperbotSim` on `small_house.xml` runs the Python
  `dock_logic` controller closed loop (wheel commands by diff-drive inverse kinematics r = 0.033,
  b = 0.17; odometry from wheel encoders; detections emulated with `emulate_detection` and noise).
  - Start poses are 6 perturbed staging poses (dx, dy in [-0.1, 0.1] m, dyaw in [-0.25, 0.25] rad,
    fixed seed), run in two modes:
    - with detections, where the initial map estimate is biased by 0.05 m;
    - map-only, without bias.
  - Every run docks within 60 s of sim time. The ground-truth `base_pose()` error to (-0.5, 0, 0) is
    <= 0.05 m and <= 0.1 rad, and the final wheel command is zero.
  - Undock from the docked pose ends within 0.05 m / 0.1 rad of staging.
- AC22: `emulate_detection` returns the exact relative pose without noise, None beyond `max_range` or
  outside the FOV, and noise std within 30 % of the configured values over 2000 samples.
  `mujoco_sim_node` publishes `/mujoco/ground_truth` equal to `BumperbotSim.base_pose()` (checked in
  the AC24 launch). The existing `bumperbot_mujoco` tests still pass.
- AC23: `README.md`, `doc/10_navigation.md`, `doc/03_ros2_nodes_topics_services.md`,
  `doc/04_tf_and_coordinate_frames.md`, `doc/05_robot_description_and_simulation.md`,
  `doc/01_repository_structure.md` and `doc/11_build_run_and_command_reference.md` mention:
  - `dock_manager`, `/dock_robot`, `DockRobot`, `/dock/status`, `/dock/detected_pose`;
  - `battery_monitor`, `battery_sim` (marked simulation only), `/battery_state`;
  - `dock_sim.launch.py`, `dock_vel`.
  `doc/12_troubleshooting.md` mentions the `safety_stop` lock near a physical dock.
- AC24: headless MuJoCo smoke test: `dock_sim.launch.py viewer:=false use_rviz:=false` gives, within
  150 s:
  - an available `/dock_robot` server;
  - `/dock/status` UNDOCKED with `dock_pose == (-0.5, 0, 0)`;
  - `/battery_state` messages;
  - `battery_monitor/state` MONITORING;
  - `/mujoco/ground_truth` within 0.02 m of (0, 0).
- AC25 (opt-in, `BUMPERBOT_DOCK_E2E=1`): on the same headless stack (battery start 1.0, low 0.3,
  full 0.95, test `charge_rate` 0.05/s):
  - Mission `[pickup, dropoff]` is sent. Once `/mission/status` is NAVIGATING, the test sets
    `percentage_override 0.2`.
  - The mission ends CANCELED (`docked == true` at staging). A DOCK goal follows automatically and
    ends OUTCOME_DOCKED.
  - Ground truth to (-0.5, 0, 0) is <= 0.05 m and <= 0.1 rad.
  - `/dock/status` goes to CHARGING. At >= 0.95 an UNDOCK follows automatically and ends
    OUTCOME_UNDOCKED, with ground truth within 0.1 m of staging.
  - The last twist_mux output is zero and `battery_monitor/state` is MONITORING.
  - Everything finishes within 900 s.

## Test plan

Build once (no `--symlink-install`):
```
cd <ws> && colcon build --packages-select bumperbot_msgs bumperbot_controller bumperbot_navigation bumperbot_mujoco bumperbot_bringup && source install/setup.bash
```
All pytest commands are prefixed with `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`. Launch tests run with
`ROS_DOMAIN_ID=90 ROS_LOCALHOST_ONLY=1` (MuJoCo: 91). Every `add_launch_test` /
`ament_add_pytest_test` gets a generous registered TIMEOUT (shown below).

| AC | Type | File | Command | Pass condition |
|---|---|---|---|---|
| AC1 | pytest (CLI) | `src/bumperbot_navigation/test/test_dock_static.py -k interfaces` | `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/bumperbot_navigation/test/test_dock_static.py -k interfaces -v` | `ros2 interface show` returns 0 for 2 new + 5 old interfaces, all fields/constants present |
| AC2 | pytest unit + gtest | `test/test_dock_logic.py`, `test/test_dock_logic.cpp` | `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/bumperbot_navigation/test/test_dock_logic.py -v` and `./build/bumperbot_navigation/test_dock_logic_cpp` | every case passes in both languages (TIMEOUT 120) |
| AC3, AC14 | pytest, process level (both executables, MockDockWorld, test `/clock`) | `test/test_dock_robustness.py` | `ROS_DOMAIN_ID=90 ROS_LOCALHOST_ONLY=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 timeout 600 python3 -m pytest src/bumperbot_navigation/test/test_dock_robustness.py -v` | all parametrized cases pass (TIMEOUT 600) |
| AC4 | pytest unit (numpy/scipy) | `test/test_dock_geometry_on_map.py` | `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/bumperbot_navigation/test/test_dock_geometry_on_map.py -v` | clearance >= 0.30 m along the segment; marker config equals charger pose |
| AC5-AC13 (+ runtime part of AC20) | launch_testing, `@launch_testing.parametrize("impl", ["cpp", "py"])`, MockDockWorld | `test/test_dock_manager.launch.py` | `ROS_DOMAIN_ID=90 ROS_LOCALHOST_ONLY=1 timeout 900 launch_test src/bumperbot_navigation/test/test_dock_manager.launch.py` | all methods pass for both impls (`test_dock_success`, `test_skip_staging`, `test_detection_refinement`, `test_detection_required_missing`, `test_retry_on_slip`, `test_stall_fails_safe`, `test_staging_failed`, `test_safety_lock`, `test_cancel_paths`, `test_error_paths`, `test_docked_flag_reset`, `test_goal_rejection`, `test_undock`, `test_parameter_parity_runtime`) (TIMEOUT 900) |
| AC15 | pytest unit | `test/test_battery_logic.py` | `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/bumperbot_navigation/test/test_battery_logic.py -v` | all `battery_cases.yaml` cases pass |
| AC16, AC17 | launch_testing (mocks, no simulator) + static grep | `test/test_battery_nodes.launch.py` | `ROS_DOMAIN_ID=90 ROS_LOCALHOST_ONLY=1 timeout 400 launch_test src/bumperbot_navigation/test/test_battery_nodes.launch.py` | all methods pass (TIMEOUT 400) |
| AC18 | launch_testing (real twist_mux) | `src/bumperbot_bringup/test/test_dock_twist_mux.launch.py` | `ROS_DOMAIN_ID=90 ROS_LOCALHOST_ONLY=1 timeout 180 launch_test src/bumperbot_bringup/test/test_dock_twist_mux.launch.py` | (a)-(d) hold; static priority checks pass; `git diff --quiet main -- src/bumperbot_controller/config/twist_mux_locks.yaml` returns 0 (TIMEOUT 180) |
| AC19 | pytest (static + `--show-args`) | `src/bumperbot_bringup/test/test_dock_launch_config.py` | `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/bumperbot_bringup/test/test_dock_launch_config.py src/bumperbot_bringup/test/test_launch_config.py src/bumperbot_bringup/test/test_mission_launch_config.py -v` | all pass; `git diff --quiet main -- src/bumperbot_bringup/launch/real_robot.launch.py src/bumperbot_bringup/launch/simulated_robot.launch.py src/bumperbot_bringup/launch/mission_sim.launch.py` returns 0 |
| AC20 | pytest static | `test/test_dock_static.py -k "sim_independence or ascii or parameter"` | `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/bumperbot_navigation/test/test_dock_static.py -v` | no forbidden tokens; ASCII comments; same parameter set in both sources + YAML |
| AC21 | pytest, headless MuJoCo (no ROS) | `src/bumperbot_mujoco/test/test_dock_precision_sim.py` | `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/bumperbot_mujoco/test/test_dock_precision_sim.py -v` | 12 docking runs + undock within tolerances (PYTHONPATH includes `src/bumperbot_navigation` if not installed) |
| AC22 | pytest unit | `src/bumperbot_mujoco/test/test_dock_marker_sim.py` | `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/bumperbot_mujoco/test -v` | new tests + existing mujoco tests pass |
| AC23 | pytest (grep) | `test/test_dock_docs.py` | `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/bumperbot_navigation/test/test_dock_docs.py -v` | all required strings found |
| AC24 | launch_testing, headless MuJoCo | `src/bumperbot_bringup/test/test_dock_sim_mujoco.launch.py::test_stack_up` | `ROS_DOMAIN_ID=91 ROS_LOCALHOST_ONLY=1 timeout 300 launch_test src/bumperbot_bringup/test/test_dock_sim_mujoco.launch.py` | all conditions met within 150 s (TIMEOUT 1200 for the file) |
| AC25 | launch_testing, headless MuJoCo, opt-in | same file, `test_low_battery_dock_charge_undock` (`skipUnless BUMPERBOT_DOCK_E2E=1`) | `BUMPERBOT_DOCK_E2E=1 ROS_DOMAIN_ID=91 ROS_LOCALHOST_ONLY=1 timeout 1100 launch_test src/bumperbot_bringup/test/test_dock_sim_mujoco.launch.py` | sequence completes, ground-truth error <= 0.05 m / 0.1 rad, within 900 s |

Full suite:
```
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon test --packages-select bumperbot_msgs bumperbot_navigation bumperbot_mujoco bumperbot_bringup --event-handlers console_direct+ && colcon test-result --verbose
```
It must show 0 failures. AC25 and the earlier opt-in E2E tests are reported as skipped. The earlier
mission tests must still pass.

`MockDockWorld` (in `test/mock_dock_world.py`, own executor thread in the test process):
- configurable `localization_error`, `staging_error`, `nav_script` (`abort_first`, `always_abort`,
  `duration`), `detections` (on/off, rate 10 Hz, noise 0), `inject` (outlier / NaN / bad frame
  list), `slip_first_insert`, `stall`, `tf_enabled`, `safety_lock` (set by test), `publish_clock` /
  `stop_clock`;
- it records `(t_wall, v, w, state)` for each `dock_vel` message, Nav2 goals and cancels;
- the node under test runs with `stations_file = test/data/stations_test.yaml`, `stall_timeout 2.0`,
  `safety_stop_timeout 3.0`, `detection_timeout 2.0`, `nav_server_timeout 3.0`,
  `nav_goal_timeout 10.0`, `use_sim_time false` (except AC14 c/d).

## Implementation steps

1. `bumperbot_msgs`: add `DockRobot.action` and `DockStatus.msg`, update CMake, build, then
   `ros2 interface show` (AC1).
2. `twist_mux_topics.yaml`: add the `docking` input (`dock_vel`, 30) with an English comment, and
   check that the existing bringup tests still pass.
3. Python `dock_logic.py`: geometry, `Aligner` / `insert_command` / `undock_command`,
   `DetectionFilter`, `PhaseTimer`, `DockTracker`. Write `test/data/dock_cases.yaml` and
   `test_dock_logic.py` (AC2 Python).
4. C++ `dock_logic.hpp/.cpp` with the same API. The `dock_logic` library links `mission_logic`.
   Write the gtest `test_dock_logic.cpp` reading the same YAML through `TEST_DATA_DIR` (AC2 C++).
5. `bumperbot_mujoco`: `emulate_detection` + `dock_marker_sim.py`, the `/mujoco/ground_truth`
   publisher, `config/dock_marker_sim.yaml`, and setup.py/package.xml. Write
   `test_dock_marker_sim.py` (AC22) and `test_dock_precision_sim.py` (AC21). Tune the controller
   gains and defaults until AC21 passes, then mirror any default change into C++ and
   `dock_cases.yaml`.
6. `test_dock_geometry_on_map.py` (AC4).
7. Python `dock_manager.py`:
   - parameters + startup validation (reuse `load_stations`);
   - action server with goal/cancel callbacks and the joinable worker thread with try/finally;
   - Nav2 client, TF handling, detection subscription + filter, safety lock, battery/mission
     subscriptions;
   - control loop, docked watch, wall status timer, signal handling.
8. `test/mock_dock_world.py` + `test_dock_manager.launch.py` (start with `impl=py`). Make AC5-AC13
   pass for Python.
9. C++ `dock_manager.hpp/.cpp` mirroring step 7 (same names, defaults and logs where practical,
   custom signal handler, joinable `std::thread`). Enable `impl=cpp` and make AC5-AC13 pass.
10. `dock_harness.py` + `test_dock_robustness.py` (AC3, AC14) for both impls.
11. `battery_logic.py` + `battery_cases.yaml` + `test_battery_logic.py` (AC15). Then
    `battery_monitor.py`, `battery_sim.py` and `test_battery_nodes.launch.py` (AC16, AC17).
12. Config files `dock_manager.yaml`, `battery_monitor.yaml`, `battery_sim.yaml`, and
    `launch/docking.launch.py`. Update the CMake install and test registration (only when the
    files exist, with `APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:${CMAKE_CURRENT_SOURCE_DIR}/test`,
    TIMEOUTs as in the test plan) and the package.xml deps.
13. `bumperbot_bringup/launch/dock_sim.launch.py`, `test_dock_launch_config.py` (AC19),
    `test_dock_twist_mux.launch.py` (AC18), and the CMake + package.xml updates.
14. `test_dock_static.py` (AC1, AC20).
15. `test_dock_sim_mujoco.launch.py`: run AC24, then AC25 once with `BUMPERBOT_DOCK_E2E=1`. Tune
    `dock_manager.yaml` (detection FOV/range, speeds) if needed.
16. Docs: README, doc/10, doc/03, doc/04, doc/05, doc/01, doc/02, doc/11 and doc/12, including the
    commands
    `ros2 launch bumperbot_bringup dock_sim.launch.py simulator:=mujoco`,
    `ros2 action send_goal /dock_robot bumperbot_msgs/action/DockRobot "{command: 0}" --feedback`,
    `ros2 topic echo /dock/status` and `ros2 param set /battery_sim percentage_override 0.15`.
    Then run `test_dock_docs.py` (AC23).
17. Full `colcon test` on the four packages plus `bumperbot_mapping` to confirm there is no
    regression.

## Risks and open questions

- **AMCL bias vs. 0.05 m target**: map-only docking can miss 0.05 m when AMCL is off by several
  cm. That is why the detection refinement exists. AC7b documents the map-only limit, and AC21/AC25
  use detections. With Gazebo (no marker emulator) precision is not guaranteed; this is documented.
- **Physical dock and `safety_stop` on the real robot**: a real charger body within 0.2 m of the
  LiDAR triggers the 255 lock and docking fails safely (SAFETY_TIMEOUT). The fix is hardware/config
  (dock below the scan plane, or a tuned `danger_distance` reviewed by the user). It must never be
  done by docking code bypassing the lock. This goes into doc/12. Out of scope: wiring
  `docking.launch.py` into `real_robot.launch.py` and a real fiducial detector.
- **Charging contact confirmation**: `battery_sim` trusts `/dock/status.docked`. On real hardware the
  BMS reports CHARGING independently. A `require_charging_confirmation` parameter is a possible
  follow-up (open question).
- **Concurrent mission sent by a user while docking or charging**: `dock_manager` rejects DOCK
  during an active mission, but cannot stop a mission sent during DOCKED. The docked watch (AC11)
  then clears `docked`, and `battery_monitor` goes back to MONITORING and re-docks if the battery is
  still low. Undocking before a mission is not automatic (open question: should `mission_manager`
  call UNDOCK first?).
- **Mission cancel takes minutes** (`mission_manager` drives back to the dock station before it
  ends CANCELED). This is bounded by `mission_cancel_timeout` (180 s).
- **`battery_monitor` / `battery_sim` Python only**: this follows the request (parity is mandated for
  `dock_manager`). If the reviewer requires a C++ twin, `battery_logic` is pure and can be ported
  with the `battery_cases.yaml` table, the same way as `dock_logic`.
- **`/mujoco/ground_truth` adds traffic** (50 Hz) to every MuJoCo run. It is harmless and sim only,
  but could get a parameter `publish_ground_truth` (default true) if load matters.
- **Cancel-all on `/execute_mission`** cancels every mission goal, including ones not started by
  `battery_monitor`. This is intended for a single-mission server.
- **Detection timestamps**: lookup at the message stamp can fail when TF history is short. The
  fallback uses the latest transform; with sim time the marker stamps come from ground truth, which
  shares the sim clock.
- `launch_testing.parametrize` with many methods makes `test_dock_manager.launch.py` long (~10 min).
  If the 900 s TIMEOUT is tight, split it into two files (`..._dock` / `..._paths`).
