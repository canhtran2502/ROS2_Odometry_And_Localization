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

#### `safety_stop_node` (`safety_stop.cpp` / `safety_stop.py`)
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
- **Parameters**: see `bumperbot_mapping/config/frontier_explorer.yaml` (e.g. `inflation_radius`, `min_frontier_size`, `max_linear_speed`, `stop_distance`, `completion_hysteresis`, `completion_timeout`, `map_save_path`, `shutdown_on_complete`)

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
| `explore_vel` | `geometry_msgs/msg/Twist` | `frontier_explorer` | `twist_mux` (input `explore`, priority 10) | Autonomous exploration velocity demand (lowest priority; joystick 99 > keyboard 90 > explore 10; the `safety_stop` lock 255 wins) |
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

The `bumperbot_msgs` package defines three custom IDL interfaces used throughout the example suite:

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

