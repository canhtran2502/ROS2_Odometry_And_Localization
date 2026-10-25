# 03 - ROS 2 Nodes, Topics, Services & Actions Reference

## A. Objective

This document provides a comprehensive, verified catalog of every ROS 2 node, topic, service, action, and parameter in the BumperBot repository. It serves as an exhaustive API reference for developers writing new software or debugging communication across the stack.

---

## B. Complete ROS 2 Nodes Inventory

### 1. Actuation & Motion Control Nodes

#### `simple_controller` (`simple_controller.cpp` / `simple_controller.py`)
- **Package**: `bumperbot_controller`
- **Purpose**: Analytical differential-drive controller. Converts velocity commands into left/right wheel angular velocities and integrates encoder increments into planar odometry.
- **Subscriptions**:
  - `/bumperbot_controller/cmd_vel` (`geometry_msgs/msg/TwistStamped`, queue: 10)
  - `/joint_states` (`sensor_msgs/msg/JointState`, queue: 10)
- **Publications**:
  - `/bumperbot_controller/odom` (`nav_msgs/msg/Odometry`, queue: 10)
  - `/simple_velocity_controller/commands` (`std_msgs/msg/Float64MultiArray`, queue: 10)
- **TF Broadcasts**: `odom` -> `base_footprint`
- **Parameters**:
  - `wheel_radius` (double, default: `0.033` m)
  - `wheel_separation` (double, default: `0.17` m)

#### `noisy_controller` (`noisy_controller.cpp` / `noisy_controller.py`)
- **Package**: `bumperbot_controller`
- **Purpose**: Generates realistic corrupted odometry by injecting zero-mean Gaussian noise into wheel encoder readings and kinematic integrations.
- **Subscriptions**:
  - `/joint_states` (`sensor_msgs/msg/JointState`, queue: 10)
- **Publications**:
  - `/bumperbot_controller/odom_noisy` (`nav_msgs/msg/Odometry`, queue: 10)
- **TF Broadcasts**: `odom` -> `base_footprint_noisy`
- **Parameters**:
  - `wheel_radius` (double, default: `0.033` m)
  - `wheel_separation` (double, default: `0.17` m)

#### `twist_relay` (`twist_relay.cpp` / `twist_relay.py`)
- **Package**: `bumperbot_controller`
- **Purpose**: Message bridge reconciling differences between stamped and unstamped Twist formats across nodes.
- **Subscriptions**:
  - `/bumperbot_controller/cmd_vel_unstamped` (`geometry_msgs/msg/Twist`, queue: 10)
  - `/input_joy/cmd_vel_stamped` (`geometry_msgs/msg/TwistStamped`, queue: 10)
- **Publications**:
  - `/bumperbot_controller/cmd_vel` (`geometry_msgs/msg/TwistStamped`, queue: 10)
  - `/input_joy/cmd_vel` (`geometry_msgs/msg/Twist`, queue: 10)

---

### 2. Supervisory & Safety Nodes

#### `safety_supervisor` (`safety_supervisor` C++ / `safety_supervisor.py`)
- **Package**: `bumperbot_utils`; started by `simulated_robot.launch.py` (default `safety:=supervisor`, `safety_use_python:=false`) and by `bumperbot_utils/launch/safety_supervisor.launch.py`.
- **Purpose**: Direction-aware speed limiting around the footprint, latched e-stop, sensor watchdog and the `twist_mux` lock. See doc/07 section C2.
- **Subscriptions**: `scan` (`LaserScan`, best effort), `bumperbot_controller/odom` (`Odometry`), `safety/cmd_vel_in` (`Twist`, the remapped `twist_mux` output; in `enforcement: lock_only` the node reads `bumperbot_controller/cmd_vel_unstamped` read only instead), `safety/estop` (`std_msgs/Bool`), `dock/status` (`DockStatus`, transient_local)
- **Publications**: `bumperbot_controller/cmd_vel_unstamped` (`Twist`, filter mode only), `safety_stop` (`std_msgs/Bool`, the `twist_mux` lock, every tick), `safety/status` (`bumperbot_msgs/msg/SafetyStatus`, transient_local)
- **Services** (`std_srvs/srv/Trigger`): `/safety_supervisor/trigger_estop`, `/safety_supervisor/reset` (refused while the `safety/estop` input is true, a fault exists, or an obstacle is in the contact zone)
- **TF**: reads `base_footprint <- laser_link` and, for the dock profile, `DockStatus.frame_id -> base_footprint`; publishes none
- **Parameters**: `config/safety_supervisor.yaml` (`enforcement` filter/lock_only, `robot_frame`, `footprint` (default rectangle x +-0.10 m, y +-0.125 m, includes the wheels), `stop_margin`, `slowdown_distance`, `contact_margin`, `max_linear_speed`, timeouts, `latch_faults`, `dock_approach_enabled`, `dock_max_speed`, ...); configuration is ROS parameters only

#### `safety_stop_node` (`safety_stop.cpp` / `safety_stop.py`, legacy, `safety:=legacy`)
- **Package**: `bumperbot_utils`
- **Purpose**: Two-tier reactive safety supervisor inspecting 2D LiDAR ranges. Modulates teleop velocity via actions and triggers emergency lock on `twist_mux`.
- **Subscriptions**:
  - `scan` (`sensor_msgs/msg/LaserScan`, queue: 10)
- **Publications**:
  - `safety_stop` (`std_msgs/msg/Bool`, queue: 10)
  - `zones` (`visualization_msgs/msg/MarkerArray`, queue: 10)
- **Action Clients**:
  - `/joy_turbo_decrease` (`twist_mux_msgs/action/JoyTurbo`)
  - `/joy_turbo_increase` (`twist_mux_msgs/action/JoyTurbo`)
- **Parameters**:
  - `danger_distance` (double, default: `0.2` m)
  - `warning_distance` (double, default: `0.6` m)
  - `scan_topic` (string, default: `"scan"`)
  - `safety_stop_topic` (string, default: `"safety_stop"`)

---

### 3. State Estimation & Sensor Fusion Nodes

#### `imu_republisher_node` (`imu_republisher.cpp` / `imu_republisher.py`)
- **Package**: `bumperbot_localization`
- **Purpose**: Adapts IMU coordinate sign conventions between the Gazebo sensor plugin and the robot body coordinate system.
- **Subscriptions**:
  - `imu/out` (`sensor_msgs/msg/Imu`, queue: 10)
- **Publications**:
  - `imu_ekf` (`sensor_msgs/msg/Imu`, queue: 10)

#### `kalman_filter` (`kalman_filter.cpp` / `kalman_filter.py`)
- **Package**: `bumperbot_localization`
- **Purpose**: 1D linear Kalman Filter fusing corrupted odometry measurements with a dynamic system prediction model.
- **Subscriptions**:
  - `bumperbot_controller/odom_noisy` (`nav_msgs/msg/Odometry`, queue: 10)
- **Publications**:
  - `bumperbot_controller/odom_kalman` (`nav_msgs/msg/Odometry`, queue: 10)

#### `odometry_motion_model` (`odometry_motion_model.cpp` / `odometry_motion_model.py`)
- **Package**: `bumperbot_localization`
- **Purpose**: Monte Carlo sample generator implementing Thrun's probabilistic odometry motion model.
- **Subscriptions**:
  - `bumperbot_controller/odom` (`nav_msgs/msg/Odometry`, queue: 10)
- **Publications**:
  - `odometry_motion_model/samples` (`geometry_msgs/msg/PoseArray`, queue: 10)
- **Parameters**:
  - `alpha1` (double, default: `0.1` rad/rad)
  - `alpha2` (double, default: `0.1` rad/m)
  - `alpha3` (double, default: `0.1` m/m)
  - `alpha4` (double, default: `0.1` m/rad)
  - `nr_samples` (int, default: `300`)

---

### 4. Mapping & SLAM Nodes

#### `mapping_with_known_poses` (`mapping_with_known_poses.cpp` / `mapping_with_known_poses.py`)
- **Package**: `bumperbot_mapping`
- **Purpose**: Generates a 2D `nav_msgs/msg/OccupancyGrid` map using ray endpoint projection from laser scans at known TF poses.
- **Subscriptions**:
  - `scan` (`sensor_msgs/msg/LaserScan`, queue: 10)
- **Publications**:
  - `map` (`nav_msgs/msg/OccupancyGrid`, queue: 1, timer: 1.0 Hz)
- **TF Lookups**: `odom` -> `laser_link` evaluated at `scan.header.stamp`
- **Parameters**:
  - `width` (double, default: `50.0` m)
  - `height` (double, default: `50.0` m)
  - `resolution` (double, default: `0.1` m/pixel)

---

#### `frontier_explorer` (`frontier_explorer.py`)
- **Package**: `bumperbot_mapping`
- **Purpose**: Autonomous frontier exploration on top of `slam_toolbox`: frontier detection, grid A*, pure pursuit, reactive safety layer, map saving on completion. Algorithms live in `bumperbot_mapping/frontier_exploration.py` (no rclpy).
- **Python-only**: this is a new node (the C++/Python parity rule covers nodes that already exist in both languages). Its logic is shared with the headless pytest and MuJoCo tests, which a C++ port could not run; a C++ port would be a separate feature.
- **Subscriptions**: `map` (`nav_msgs/msg/OccupancyGrid`, reliable + transient_local), `scan` (`sensor_msgs/msg/LaserScan`, sensor-data QoS)
- **Publications**: `explore_vel` (`geometry_msgs/msg/Twist`, 10 Hz), `/explore/status` (`std_msgs/msg/String`, transient_local), `/explore/path` (`nav_msgs/msg/Path`), `/explore/frontiers` (`visualization_msgs/msg/MarkerArray`)
- **Service clients**: `/map_saver/save_map` (`nav2_msgs/srv/SaveMap`), fallback `/slam_toolbox/save_map` (`slam_toolbox/srv/SaveMap`)
- **TF Lookups**: `map` -> `base_footprint` (every tick), `base_footprint` -> `laser_link` (laser yaw offset)
- **Parameters**: see `bumperbot_mapping/config/frontier_explorer.yaml` (e.g. `inflation_radius`, `min_frontier_size`, `max_linear_speed`, `stop_distance`, `side_clearance`, `completion_hysteresis`, `completion_timeout`, `map_save_path`, `shutdown_on_complete`)
- **Launch**: `bumperbot_bringup/launch/auto_explore.launch.py` includes `simulated_robot.launch.py` (`use_slam:=True`) and starts this node with `frontier_explorer.yaml` (arguments `simulator`, `world_name`, `use_rviz`, `viewer`, `map_save_path`, `explorer_config`; see doc/08 section E2).

---

### 4b. Navigation & Mission Nodes

#### `mission_manager` (`mission_manager.cpp` / `mission_manager.py`)
- **Package**: `bumperbot_navigation`
- **Purpose**: AGV waypoint missions. Visits the stations of `stations.yaml` in order through Nav2, checks the reached pose with TF, dwells, retries and skips failed stations, and returns to the dock at the end, on error and on cancel. The state machine is the pure library `mission_logic` (C++) / `mission_logic.py`; both languages are checked against the same case table.
- **Action server**: `/execute_mission` (`bumperbot_msgs/action/ExecuteMission`); rejects an empty list, unknown stations and a goal while a mission is active
- **Action client**: `navigate_to_pose` (`nav2_msgs/action/NavigateToPose`)
- **Publications**: `/mission/status` (`bumperbot_msgs/msg/MissionStatus`, reliable + transient_local)
- **TF Lookups**: `map` -> `base_footprint` (latest, after each Nav2 success)
- **Parameters**: `stations_file`, `nav_action_name`, `global_frame`, `robot_frame`, `goal_tolerance`, `max_retries`, `retry_delay`, `max_consecutive_failures`, `dock_max_retries`, `return_to_dock`, `nav_server_timeout`, `nav_goal_timeout`, `tf_timeout`, `status_rate`, `behavior_tree` (see `config/mission_manager.yaml`)
- **Launch**: `bumperbot_bringup/launch/mission_sim.launch.py` (see doc/10)

#### `dock_manager` (`dock_manager.cpp` / `dock_manager.py`)
- **Package**: `bumperbot_navigation`
- **Purpose**: auto-docking. DOCK drives to the dock staging pose with Nav2 (skipped when already within `staging_tolerance`), aligns on the pre-dock line, backs in closed loop on odometry and verifies the final pose (retries on failure); UNDOCK drives straight out to the staging pose. The pure library `dock_logic` (C++) / `dock_logic.py` is checked against one shared case table.
- **Action server**: `/dock_robot` (`bumperbot_msgs/action/DockRobot`); rejects an unknown command, a second goal, UNDOCK while not docked, DOCK during an active mission and any goal while TF or Nav2 is not ready
- **Action client**: `navigate_to_pose` (`nav2_msgs/action/NavigateToPose`)
- **Publications**: `/dock/status` (`bumperbot_msgs/msg/DockStatus`, reliable + transient_local), `dock_vel` (`geometry_msgs/Twist`, `twist_mux` input, priority 30)
- **Subscriptions**: `/dock/detected_pose` (`PoseStamped`, abstract detector output), `/safety_stop` (read only), `/battery_state`, `/mission/status`
- **TF Lookups**: `map -> base_footprint`, `map -> odom`, `odom -> base_footprint`; detections are transformed into `odom`. No TF is published
- **Parameters**: see `config/dock_manager.yaml` (`stations_file`, `dock_station`, `staging_distance`, `detection_mode` none/optional/required, tolerances, speeds, timeouts, `safety_stop_timeout`, `max_retries`, ...)
- **Launch**: `bumperbot_navigation/launch/docking.launch.py`, `bumperbot_bringup/launch/dock_sim.launch.py`

#### `battery_monitor` (`battery_monitor.py`)
- **Package**: `bumperbot_navigation` (Python only, simulator independent)
- **Purpose**: below `low_threshold` cancels an active mission (cancel-all on `execute_mission/_action/cancel_goal`), sends `DockRobot` DOCK, stays in CHARGING until `full_threshold`, then sends UNDOCK. States: WAITING_FOR_BATTERY, MONITORING, CANCELING_MISSION, DOCKING, CHARGING, UNDOCKING, ERROR (`~/reset` leaves ERROR)
- **Subscriptions**: `/battery_state`, `/dock/status`, `/mission/status`; **Action client**: `dock_robot`; **Service client**: `execute_mission/_action/cancel_goal` (`action_msgs/srv/CancelGoal`); **Publications**: `battery_monitor/state` (`std_msgs/String`, transient_local)

#### `battery_sim` (`battery_sim.py`) -- SIMULATION ONLY
- **Package**: `bumperbot_navigation`; started only by `dock_sim.launch.py`
- **Purpose**: publishes `/battery_state` (`sensor_msgs/BatteryState`, 2 Hz). Drains while idle, faster with the `bumperbot_controller/odom` twist, charges while `/dock/status.docked`. `ros2 param set /battery_sim percentage_override 0.15` jumps to a charge

#### `dock_marker_sim` (`dock_marker_sim.py`) -- SIMULATION ONLY
- **Package**: `bumperbot_mujoco`; started only by `dock_sim.launch.py` with `simulator:=mujoco`
- **Purpose**: emulates a rear-facing fiducial detector. Reads `/mujoco/ground_truth`, gates by range and field of view, adds seeded Gaussian noise and publishes the charger pose in `base_footprint` on `/dock/detected_pose`. Parameters in `bumperbot_mujoco/config/dock_marker_sim.yaml`

#### Nav2 servers (`navigation.launch.py`)
`controller_server` and `behavior_server` (`cmd_vel` remapped to `nav_vel`), `planner_server`, `bt_navigator` and `lifecycle_manager_navigation`, configured by `bumperbot_navigation/config/nav2_params.yaml`.

---

### 5. Physical Hardware & Serial Bridge Nodes

#### `mpu6050_driver` (`mpu6050_driver.py`)
- **Package**: `bumperbot_firmware`
- **Purpose**: Direct Linux I2C SMBus driver reading raw accelerometer and gyroscope registers from an MPU6050 sensor at 100 Hz.
- **Publications**:
  - `/imu/out` (`sensor_msgs/msg/Imu`, QoS: SensorData)
- **Hardware Target**: I2C bus (`/dev/i2c-1`), Device address `0x68`

#### `simple_serial_receiver` (`simple_serial_receiver.cpp` / `simple_serial_receiver.py`)
- **Package**: `bumperbot_firmware`
- **Purpose**: Serial diagnostic reader streaming line-delimited ASCII messages from an external MCU onto a ROS topic.
- **Publications**:
  - `serial_receiver` (`std_msgs/msg/String`, queue: 10)
- **Parameters**:
  - `port` (string, default: `"/dev/ttyUSB0"`)
  - `baudrate` (int, default: `115200`)

#### `simple_serial_transmitter` (`simple_serial_transmitter.cpp` / `simple_serial_transmitter.py`)
- **Package**: `bumperbot_firmware`
- **Purpose**: Serial diagnostic writer streaming String topic data out to an external MCU.
- **Subscriptions**:
  - `serial_transmitter` (`std_msgs/msg/String`, queue: 10)
- **Parameters**:
  - `port` (string, default: `"/dev/ttyUSB0"`)

---

## C. Topics Reference Catalog

| Topic Name | Message Type | Primary Publisher(s) | Primary Subscriber(s) | Semantic Meaning |
|---|---|---|---|---|
| `/clock` | `rosgraph_msgs/msg/Clock` | `ros_gz_bridge` | All nodes (`use_sim_time:=True`) | Simulation clock synchronization |
| `/joint_states` | `sensor_msgs/msg/JointState` | `joint_state_broadcaster` / Sim | `robot_state_publisher`, controllers | Current joint positions & velocities |
| `/bumperbot_controller/cmd_vel` | `geometry_msgs/msg/TwistStamped` | `twist_relay` | `diff_drive_controller`, `simple_controller` | Velocity demand to drive wheels |
| `/bumperbot_controller/cmd_vel_unstamped` | `geometry_msgs/msg/Twist` | `twist_mux` | `twist_relay` | Multiplexed unstamped velocity |
| `/input_joy/cmd_vel_stamped` | `geometry_msgs/msg/TwistStamped` | `joy_teleop` | `twist_relay` | Raw joystick velocity demand |
| `/input_joy/cmd_vel` | `geometry_msgs/msg/Twist` | `twist_relay` | `twist_mux` | Unstamped joystick velocity demand |
| `/bumperbot_controller/odom` | `nav_msgs/msg/Odometry` | `diff_drive_controller` / `simple_controller` | `nav2_amcl`, `slam_toolbox`, EKF | Clean kinematic wheel odometry |
| `/bumperbot_controller/odom_noisy` | `nav_msgs/msg/Odometry` | `noisy_controller` | `robot_localization` (EKF), `kalman_filter` | Synthetic corrupted odometry |
| `/bumperbot_controller/odom_kalman` | `nav_msgs/msg/Odometry` | `kalman_filter` | RViz2 | Filtered 1D odometry estimate |
| `/odometry_motion_model/samples` | `geometry_msgs/msg/PoseArray` | `odometry_motion_model` | RViz2 | Particle cloud of motion model |
| `/imu/out` | `sensor_msgs/msg/Imu` | `ros_gz_bridge` / `mpu6050_driver` | `imu_republisher_node` | Raw 6-DoF inertial measurements |
| `/imu_ekf` | `sensor_msgs/msg/Imu` | `imu_republisher_node` | `robot_localization` (EKF) | Axis-corrected IMU data for fusion |
| `/scan` | `sensor_msgs/msg/LaserScan` | `ros_gz_bridge` (LiDAR) | `safety_stop`, `nav2_amcl`, `slam_toolbox` | 2D planar laser range scan |
| `safety/cmd_vel_in` | `geometry_msgs/msg/Twist` | `twist_mux` (output remapped in `simulated_robot.launch.py`, `safety:=supervisor`) | `safety_supervisor` | Unfiltered velocity demand; the supervisor limits it and publishes `bumperbot_controller/cmd_vel_unstamped` |
| `/safety_stop` | `std_msgs/msg/Bool` | `safety_supervisor` (or legacy `safety_stop`) | `twist_mux` lock (255), `dock_manager` (read only) | Lock; the supervisor republishes it at 20 Hz |
| `/safety/status` | `bumperbot_msgs/msg/SafetyStatus` | `safety_supervisor` | user, tests | State, zones, clearances, allowed speeds, lock, e-stop latch, reason, input ages (transient_local) |
| `/safety/estop` | `std_msgs/msg/Bool` | user / external e-stop | `safety_supervisor` | `true` latches the e-stop; reset needs it back to `false` |
| `explore_vel` | `geometry_msgs/msg/Twist` | `frontier_explorer` | `twist_mux` (input `explore`, priority 10) | Autonomous exploration velocity demand (lowest priority; joystick 99 > keyboard 90 > explore 10; the `safety_stop` lock 255 wins) |
| `nav_vel` | `geometry_msgs/msg/Twist` | Nav2 `controller_server`, `behavior_server` | `twist_mux` (input `navigation`, priority 20) | Nav2 velocity demand (joystick 99 > keyboard 90 > navigation 20 > explore 10; the `safety_stop` lock 255 wins; nothing publishes it on the real robot) |
| `dock_vel` | `geometry_msgs/msg/Twist` | `dock_manager` | `twist_mux` (input `docking`, priority 30) | Docking velocity demand (above `nav_vel`, below keyboard and joystick; the `safety_stop` lock still wins) |
| `/dock/status` | `bumperbot_msgs/msg/DockStatus` | `dock_manager` | `battery_sim`, `battery_monitor`, user | Dock state, docked/charging flags, errors, staging and charger pose (transient_local) |
| `/dock/detected_pose` | `geometry_msgs/msg/PoseStamped` | `dock_marker_sim` (sim only) / real detector | `dock_manager` | Charger pose (docked `base_footprint` pose) seen by a detector, any TF frame |
| `/battery_state` | `sensor_msgs/msg/BatteryState` | `battery_sim` (sim only) / battery driver | `battery_monitor`, `dock_manager` | Charge level and charging status |
| `/mujoco/ground_truth` | `nav_msgs/msg/Odometry` | `mujoco_sim_node` (sim only) | `dock_marker_sim`, tests | Exact `base_footprint` pose in the MuJoCo `world` frame (equals `map`), 50 Hz |
| `/mission/status` | `bumperbot_msgs/msg/MissionStatus` | `mission_manager` | User / tests | Mission state, current station, progress, visited and skipped stations (transient_local) |
| `/explore/status` | `std_msgs/msg/String` | `frontier_explorer` | User / tests | `WAITING_FOR_MAP`, `EXPLORING`, `COMPLETE`, `MAP_SAVED`, `MAP_SAVE_FAILED` (latched) |
| `/explore/path` | `nav_msgs/msg/Path` | `frontier_explorer` | RViz2 | Current A* path |
| `/explore/frontiers` | `visualization_msgs/msg/MarkerArray` | `frontier_explorer` | RViz2 | Frontier centroids and selected goal |
| `/safety_stop` | `std_msgs/msg/Bool` | `safety_stop_node` | `twist_mux` (Lock 255) | Emergency obstacle brake inhibitor |
| `/zones` | `visualization_msgs/msg/MarkerArray` | `safety_stop_node` | RViz2 | Visual warning/danger radius disks |
| `/map` | `nav_msgs/msg/OccupancyGrid` | `nav2_map_server` / `slam_toolbox` | RViz2, `nav2_amcl` | 2D discrete occupancy grid map |
| `/tf` | `tf2_msgs/msg/TFMessage` | Transform broadcasters | Transform listeners | Dynamic coordinate frames |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | `robot_state_publisher`, static TFs | Transform listeners | Static coordinate frames |

---

## D. Custom Interfaces (`bumperbot_msgs`)

The `bumperbot_msgs` package defines custom IDL interfaces: three used throughout the example suite and two for the mission manager (`action/ExecuteMission.action`, `msg/MissionStatus.msg`, section 4 and 5):

### 1. `srv/AddTwoInts.srv`
```idl
# Request
int64 a
int64 b
---
# Response
int64 sum
```

### 2. `srv/GetTransform.srv`
```idl
# Request 
string frame_id
string child_frame_id
---
# Response
geometry_msgs/TransformStamped transform
bool success
```

### 3. `action/Fibonacci.action`
```idl
# Goal
int32 order
---
# Result
int32[] sequence
---
# Feedback
int32[] partial_sequence
```

### 4. `action/ExecuteMission.action` (`/execute_mission`)
```idl
# Goal: ordered list of station names from stations.yaml
string[] stations
---
# Result
uint8 OUTCOME_COMPLETED=0
uint8 OUTCOME_COMPLETED_WITH_SKIPS=1
uint8 OUTCOME_ABORTED=2
uint8 OUTCOME_CANCELED=3
uint8 OUTCOME_DOCK_FAILED=4
uint8 outcome
bool docked
string[] visited
string[] skipped
string message
---
# Feedback
string state
string current_station
int32 current_index
uint32 total_stations
uint32 attempt
float32 progress
float32 distance_remaining
```

### 5. `msg/MissionStatus.msg` (`/mission/status`)
```idl
# State constants: IDLE, NAVIGATING, DWELLING, RETURNING_TO_DOCK, COMPLETED, ABORTED, CANCELED, DOCK_FAILED
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

### 6. `action/DockRobot.action` (`/dock_robot`)
```idl
# Goal: COMMAND_DOCK=0, COMMAND_UNDOCK=1
uint8 command
---
# Result outcome: DOCKED=0, UNDOCKED=1, ALREADY_DOCKED=2, CANCELED=3, STAGING_FAILED=4,
# DETECTION_FAILED=5, DOCKING_FAILED=6, UNDOCKING_FAILED=7, ERROR=8
uint8 outcome
bool docked
uint32 attempts
float32 position_error   # m, -1 if unknown
float32 yaw_error        # rad, -1 if unknown
string message
---
string state
uint32 attempt
float32 distance_to_dock
float32 yaw_error
bool detection_used
```
SUCCEEDED for DOCKED / UNDOCKED / ALREADY_DOCKED, CANCELED for CANCELED, ABORTED otherwise.

### 7. `msg/DockStatus.msg` (`/dock/status`)
State constants: UNDOCKED, NAVIGATING_TO_STAGING, WAITING_FOR_DETECTION, ALIGNING, DOCKING, DOCKED, CHARGING, UNDOCKING, FAILED. Fields: `stamp`, `state`, `docked`, `charging`, `battery_percentage`, `attempt`, `position_error`, `yaw_error`, `detection_used`, `frame_id`, `staging_pose`, `dock_pose` (`geometry_msgs/Pose2D`), `message`.
