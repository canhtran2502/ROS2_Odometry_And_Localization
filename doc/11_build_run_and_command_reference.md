# 11 - Build, Run & Comprehensive Command Reference

## A. Objective

This document provides an exhaustive, verified operational handbook for building, configuring, launching, inspecting, and testing every component within the BumperBot ROS 2 workspace.

---

## B. Prerequisites & System Installation

### 1. System Requirements
- **OS**: Ubuntu 22.04 LTS (x86_64 or ARM64)
- **ROS 2 Distribution**: Humble Hawksbill
- **Hardware Privileges**: Access to `/dev/ttyUSB*` (serial) and `/dev/i2c-*` (I2C) for physical robot execution.

### 2. Dependency Installation
Execute in a standard bash terminal:

```bash
# Update package indices
sudo apt update

# Install core ROS 2 simulation & control dependencies
sudo apt install -y \
  ros-humble-ros-gz \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-diff-drive-controller \
  ros-humble-joint-state-broadcaster \
  ros-humble-joy \
  ros-humble-joy-teleop \
  ros-humble-twist-mux \
  ros-humble-slam-toolbox \
  ros-humble-nav2-amcl \
  ros-humble-nav2-map-server \
  ros-humble-nav2-lifecycle-manager \
  ros-humble-robot-localization \
  ros-humble-tf-transformations \
  ros-humble-turtlesim \
  libserial-dev \
  python3-smbus \
  python3-colcon-common-extensions

# Automatic rosdep dependency resolution
cd ~/bumperbot_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

---

## C. Workspace Build Commands

### 1. Clean Build from Scratch
```bash
cd ~/bumperbot_ws

# Source underlay ROS 2 environment
source /opt/ros/humble/setup.bash

# Build all packages with symlink install
colcon build --symlink-install

# Source overlay workspace environment
source install/setup.bash
```

### 2. Building Specific Packages (Accelerated Incremental Builds)
```bash
# Build custom message definitions first if modified
colcon build --symlink-install --packages-select bumperbot_msgs

# Build core simulation and description
colcon build --symlink-install --packages-select bumperbot_description bumperbot_controller

# Build localization and mapping
colcon build --symlink-install --packages-select bumperbot_localization bumperbot_mapping bumperbot_utils

# Build firmware and bringup
colcon build --symlink-install --packages-select bumperbot_firmware bumperbot_bringup
```

---

## D. Complete Launch File Catalog

| Launch Command | Default Arguments | Nodes & Subsystems Started | Expected Verification |
|---|---|---|---|
| `ros2 launch bumperbot_bringup simulated_robot.launch.py` | `world_name:=small_house`<br>`use_slam:=false` | Ignition Gazebo, `diff_drive_controller`, `twist_mux`, `safety_stop`, `nav2_map_server`, `nav2_amcl`, `lifecycle_manager`, RViz2 | Gazebo window opens with small house; RViz shows map and laser scan |
| `ros2 launch bumperbot_bringup simulated_robot.launch.py use_slam:=true` | `world_name:=small_house`<br>`use_slam:=true` | Gazebo, `diff_drive_controller`, `twist_mux`, `safety_stop`, `slam_toolbox`, `map_saver_server`, RViz2 (SLAM layout) | RViz shows dynamic map generation as robot explores |
| `ros2 launch bumperbot_bringup auto_explore.launch.py` | `simulator:=gazebo`<br>`world_name:=small_house`<br>`use_rviz:=true`<br>`viewer:=true`<br>`map_save_path:=` (empty = YAML value) | `simulated_robot.launch.py use_slam:=True` + `frontier_explorer` | Robot explores the world, `/explore/status` goes to `MAP_SAVED`, map saved under `~/bumperbot_maps/` |
| `ros2 launch bumperbot_bringup mission_sim.launch.py` | `simulator:=gazebo`<br>`world_name:=small_house`<br>`use_rviz:=true`<br>`viewer:=true`<br>`use_python:=false`<br>`stations_file:=` (package default)<br>`mission_config:=` (package default) | `simulated_robot.launch.py use_slam:=False` + `navigation.launch.py` + `mission_manager` | `/execute_mission` and `/navigate_to_pose` available, `/mission/status` is `IDLE`; send a mission with `ros2 action send_goal /execute_mission bumperbot_msgs/action/ExecuteMission "{stations: [pickup, dropoff]}" --feedback` |
| `ros2 launch bumperbot_navigation navigation.launch.py` | `use_sim_time:=false`<br>`nav2_params:=` (package default) | `controller_server`, `planner_server`, `behavior_server`, `bt_navigator`, `lifecycle_manager_navigation` (velocity on `nav_vel`) | Nav2 lifecycle nodes active; needs a map, AMCL and TF |
| `ros2 launch bumperbot_bringup real_robot.launch.py` | N/A | `ros2_control_node` (`BumperbotInterface`), `diff_drive_controller`, `mpu6050_driver.py`, `twist_mux` | Robot drives via serial commands to Arduino; IMU publishes at 100 Hz |
| `ros2 launch bumperbot_description display.launch.py` | `model:=bumperbot.urdf.xacro` | `robot_state_publisher`, `joint_state_publisher_gui`, RViz2 | Interactive joint slider GUI and 3D visual robot model |
| `ros2 launch bumperbot_bringup simulated_robot.launch.py simulator:=mujoco` | `world_name:=small_house`<br>`use_slam:=false` | MuJoCo viewer + `mujoco_sim_node`, `ros2_control_node`, `diff_drive_controller`, `twist_mux`, `safety_stop`, `nav2_amcl`, RViz2 | Same behaviour as the Gazebo launch, physics by MuJoCo |
| `ros2 launch bumperbot_mujoco mujoco.launch.py` | `world_name:=small_house`<br>`viewer:=True` | MuJoCo viewer, `robot_state_publisher`, `ros2_control_node`, `mujoco_sim_node` | Robot spawned in the chosen MuJoCo world |
| `ros2 launch bumperbot_description gazebo.launch.py` | `world_name:=empty` | Ignition Gazebo, `robot_state_publisher`, `create` spawner, `parameter_bridge` | Robot spawned on flat ground plane in Gazebo |
| `ros2 launch bumperbot_controller controller.launch.py` | `use_simple_controller:=False`<br>`use_python:=False` | `joint_state_broadcaster`, `wheel_controller_spawner`, `noisy_controller` | `/bumperbot_controller/odom` and `odom_noisy` active |
| `ros2 launch bumperbot_controller joystick_teleop.launch.py` | `use_sim_time:=True` | `joy_node`, `joy_teleop`, `twist_mux`, `twist_relay` | Gamepad controls robot velocity via priority channels |
| `ros2 launch bumperbot_localization local_localization.launch.py` | `use_python:=False` | `ekf_node` (`robot_localization`), `imu_republisher`, `static_transform_publisher` | Publishes fused odometry on `/odometry/filtered` |
| `ros2 launch bumperbot_localization global_localization.launch.py` | `map_name:=small_house`<br>`use_sim_time:=true` | `nav2_map_server`, `nav2_amcl`, `lifecycle_manager_localization` | Map broadcast on `/map`, particle filter active |
| `ros2 launch bumperbot_mapping slam.launch.py` | `use_sim_time:=true` | `sync_slam_toolbox_node`, `map_saver_server`, `lifecycle_manager` | Live SLAM graph optimization active on `/map` |

---

## E. Executable Node Run Catalog

All C++ and Python nodes can be executed directly using `ros2 run`:

### 1. Controllers & Relays
```bash
# Run C++ simple analytical controller
ros2 run bumperbot_controller simple_controller

# Run Python simple analytical controller
ros2 run bumperbot_controller simple_controller.py

# Run C++ noisy odometry generator
ros2 run bumperbot_controller noisy_controller

# Run Python twist relay
ros2 run bumperbot_controller twist_relay.py
```

### 2. Localization & State Estimation
```bash
# Run 1D linear Kalman filter (C++)
ros2 run bumperbot_localization kalman_filter

# Run 1D linear Kalman filter (Python)
ros2 run bumperbot_localization kalman_filter.py

# Run Thrun probabilistic sample odometry model (C++)
ros2 run bumperbot_localization odometry_motion_model

# Run IMU coordinate republisher (C++)
ros2 run bumperbot_localization imu_republisher
```

### 2b. Navigation & Missions
```bash
# Build the mission packages (interfaces first)
colcon build --packages-select bumperbot_msgs bumperbot_controller bumperbot_navigation bumperbot_bringup

# Run the mission manager alone (C++ or Python); it needs a Nav2 navigate_to_pose server and TF
ros2 run bumperbot_navigation mission_manager
ros2 run bumperbot_navigation mission_manager.py

# Stations are defined in bumperbot_navigation/config/stations.yaml (override: stations_file:=<file>)
# Mission interfaces
ros2 action send_goal /execute_mission bumperbot_msgs/action/ExecuteMission "{stations: [pickup, dropoff]}" --feedback
ros2 topic echo /mission/status

# Tests (headless, mock Nav2); isolate them with a dedicated domain
ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 colcon test --packages-select bumperbot_navigation bumperbot_bringup
colcon test-result --verbose
```

### 3. Perception, Safety & Mapping
```bash
# Run 2-tier reactive LiDAR safety supervisor (C++)
ros2 run bumperbot_utils safety_stop

# Run 2-tier reactive LiDAR safety supervisor (Python)
ros2 run bumperbot_utils safety_stop.py

# Run custom known-pose occupancy grid mapper (C++)
ros2 run bumperbot_mapping mapping_with_known_poses

# Run custom known-pose occupancy grid mapper (Python)
ros2 run bumperbot_mapping mapping_with_known_poses.py
```

### 4. Hardware Firmware Drivers
```bash
# Run MPU6050 direct I2C Linux driver
ros2 run bumperbot_firmware mpu6050_driver.py

# Run serial receiver test node
ros2 run bumperbot_firmware simple_serial_receiver --ros-args -p port:=/dev/ttyUSB0

# Run serial transmitter test node
ros2 run bumperbot_firmware simple_serial_transmitter --ros-args -p port:=/dev/ttyUSB0
```

### 5. Communication Pattern Examples (`bumperbot_cpp_examples` / `py_examples`)
```bash
# Pub / Sub
ros2 run bumperbot_cpp_examples simple_publisher
ros2 run bumperbot_cpp_examples simple_subscriber

# Parameters
ros2 run bumperbot_cpp_examples simple_parameter --ros-args -p simple_int_param:=42

# Services (AddTwoInts)
ros2 run bumperbot_cpp_examples simple_service_server
ros2 run bumperbot_cpp_examples simple_service_client --ros-args -p a:=10 -p b:=25

# Actions (Fibonacci)
ros2 run bumperbot_cpp_examples simple_action_server
ros2 run bumperbot_cpp_examples simple_action_client --ros-args -p order:=10

# Managed Lifecycle Node
ros2 run bumperbot_cpp_examples simple_lifecycle_node
# (Transition state via: ros2 lifecycle set /simple_lifecycle_node configure)

# TF2 Kinematics
ros2 run bumperbot_cpp_examples simple_tf_kinematics
```

---

## F. Complete Multi-Terminal Testing Workflows

### Workflow 1: Simulation + Global AMCL Localization (Pre-Mapped House)
- **Terminal 1 (Simulation & Localization Bringup)**:
  ```bash
  source install/setup.bash
  ros2 launch bumperbot_bringup simulated_robot.launch.py world_name:=small_house use_slam:=False
  ```
- **Terminal 2 (Teleoperation)**:
  ```bash
  source install/setup.bash
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/key_vel
  ```
- **Terminal 3 (Inspection)**:
  ```bash
  source install/setup.bash
  # Check particle cloud publication
  ros2 topic hz /particlecloud
  # Verify TF map -> odom is active
  ros2 run tf2_ros tf2_echo map odom
  ```

---

### Workflow 2: Simulation + Real-Time SLAM & Map Saving
- **Terminal 1 (SLAM Bringup)**:
  ```bash
  source install/setup.bash
  ros2 launch bumperbot_bringup simulated_robot.launch.py world_name:=small_house use_slam:=True
  ```
- **Terminal 2 (Drive Robot to Explore)**:
  ```bash
  source install/setup.bash
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/key_vel
  ```
- **Terminal 3 (Save Map after Exploring)**:
  ```bash
  source install/setup.bash
  ros2 run nav2_map_server map_saver_cli -f ~/my_house_map --ros-args -p use_sim_time:=True
  ```

---

### Workflow 2b: Autonomous Exploration + Map Saving
- **Run** (headless MuJoCo example; use `simulator:=gazebo` for Gazebo):
  ```bash
  source install/setup.bash
  ros2 launch bumperbot_bringup auto_explore.launch.py simulator:=mujoco world_name:=small_house viewer:=false use_rviz:=false
  ```
- **Watch progress / result**:
  ```bash
  ros2 topic echo /explore/status      # EXPLORING -> COMPLETE -> MAP_SAVED
  ls ~/bumperbot_maps/                 # explored_map.yaml / explored_map.pgm
  ```
- **Take over**: any joystick or keyboard command overrides the explorer (`twist_mux` priority 99 / 90 vs. 10).
- **Tests**:
  ```bash
  PYTHONPATH=src/bumperbot_mapping:$PYTHONPATH python3 -m pytest src/bumperbot_mapping/test -q
  colcon test --packages-select bumperbot_mapping bumperbot_bringup && colcon test-result --verbose
  ```
- **MuJoCo odometry and map quality tests** (headless):
  ```bash
  PYTHONPATH=src/bumperbot_mapping:src/bumperbot_mujoco:$PYTHONPATH python3 -m pytest -q src/bumperbot_mujoco/test src/bumperbot_mapping/test/test_map_compare.py
  # Opt-in full-stack SLAM map quality test (up to ~30 min, dedicated ROS_DOMAIN_ID=87 is set by the test)
  BUMPERBOT_SLAM_E2E=1 colcon test --packages-select bumperbot_bringup --ctest-args -R test_slam_map_quality_mujoco
  ```
  When mapping by hand, use a dedicated `ROS_DOMAIN_ID` (for example `export ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1`) and check `ros2 topic info /clock` shows one publisher.

---

### Workflow 3: Multi-Sensor EKF Odometry Fusion Comparison
- **Terminal 1 (Simulation Bringup)**:
  ```bash
  source install/setup.bash
  ros2 launch bumperbot_bringup simulated_robot.launch.py world_name:=empty
  ```
- **Terminal 2 (Launch Local EKF Node)**:
  ```bash
  source install/setup.bash
  ros2 launch bumperbot_localization local_localization.launch.py
  ```
- **Terminal 3 (Drive Robot in Circle)**:
  ```bash
  source install/setup.bash
  ros2 topic pub /key_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.5}}"
  ```
- **Terminal 4 (Compare Trajectory Drift)**:
  ```bash
  source install/setup.bash
  # Echo ground-truth/clean odometry
  ros2 topic echo /bumperbot_controller/odom --field pose.pose.position
  # Echo corrupted odometry
  ros2 topic echo /bumperbot_controller/odom_noisy --field pose.pose.position
  # Echo EKF filtered odometry
  ros2 topic echo /odometry/filtered --field pose.pose.position
  ```

