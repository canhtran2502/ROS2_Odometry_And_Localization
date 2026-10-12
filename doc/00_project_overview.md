# 00 - Project Overview & System Scope

## A. Objective

This document establishes the high-level system context, technical scope, package responsibilities, and implementation maturity matrix for the **BumperBot** mobile robotics workspace. It is tailored for embedded software engineers transitioning into Autonomous Mobile Robots (AMRs), Automated Guided Vehicles (AGVs), and Physical AI under the ROS 2 Humble framework on Ubuntu 22.04 LTS.

---

## B. System Overview & Engineering Scope

The **BumperBot** platform is a two-wheel differential-drive Autonomous Mobile Robot (AMR) with passive front and rear caster support. The software stack covers the end-to-end robotics lifecycle:
1. **Kinematics & Control**: Forward/inverse differential kinematics, hardware abstraction via `ros2_control`, velocity teleoperation multiplexing, and fail-safe safety locks.
2. **Sensor Processing & Simulation**: Simulation in Ignition Gazebo Fortress (Gazebo 6), virtual sensor bridging (2D LiDAR, 6-DoF IMU, wheel encoders), and dual-tier reactive obstacle avoidance.
3. **State Estimation & Localization**: Analytical odometry computation, simulated Gaussian sensor corruption, 1D linear Kalman filtering, Thrun's probabilistic sample odometry motion model, Extended Kalman Filter (EKF) sensor fusion via `robot_localization`, and Monte Carlo Localization via `nav2_amcl`.
4. **Mapping & SLAM**: Custom occupancy grid generation via known-pose ray projection, along with graph-based Simultaneous Localization and Mapping via `slam_toolbox`.
5. **Physical Hardware Integration**: Arduino-based firmware sketches implementing closed-loop PID wheel velocity control, custom `hardware_interface::SystemInterface` plugin communicating over serial (`/dev/ttyUSB0`), and I2C driver for MPU6050 6-DoF IMU.

```
+---------------------------------------------------------------------------------------------------------+
|                                        BUMPERBOT AMR PLATFORM                                           |
+---------------------------------------------------------------------------------------------------------+
|  [Hardware Interface / Sim]  <--->  [ROS 2 Control (diff_drive)]  <--->  [Twist Mux / Joy Teleop]       |
|              ^                                    ^                                     ^               |
|              | (JointStates, IMU, Scan)           | (cmd_vel)                           | (cmd_vel)     |
|              v                                    v                                     v               |
|  [Odometry & Motion Models]  <--->  [Sensor Safety Supervisor]   <--->  [Localization & SLAM]          |
|  (Analytical, Noisy, Kalman,        (2-tier warning & danger             (EKF Fusion, AMCL Particles,   |
|   Probabilistic Sample Model)        LiDAR reactive brake)                SLAM Toolbox Graph Ceres)     |
+---------------------------------------------------------------------------------------------------------+
```

---

## C. Feature Implementation Matrix

To maintain rigorous technical integrity, the following matrix documents the exact status of every feature in the repository based on source-code inspection:

| Robotics Domain | Feature / Component | Status | Source Location | Execution / Verification Mechanism |
|---|---|---|---|---|
| **Robot Modeling** | URDF / Xacro Geometry & Inertials | **Implemented** | `src/bumperbot_description/urdf/bumperbot.urdf.xacro` | `display.launch.py`, RViz2, Joint State Publisher GUI |
| **Simulation** | Ignition Gazebo Fortress Integration | **Implemented** | `src/bumperbot_description/urdf/bumperbot_gazebo.xacro` | `gazebo.launch.py`, `simulated_robot.launch.py` |
| **Simulation** | Virtual Worlds (Empty, Small House, Warehouse) | **Implemented** | `src/bumperbot_description/worlds/` | Worlds loaded via `world_name:=<name>` argument |
| **Actuation & Control** | `ros2_control` Diff-Drive Controller | **Implemented** | `src/bumperbot_controller/config/bumperbot_controllers.yaml` | Spawned by `controller.launch.py` via `controller_manager` |
| **Actuation & Control** | Custom Analytical Controller (C++ & Python) | **Implemented** | `src/bumperbot_controller/src/simple_controller.cpp` | Selectable via `use_simple_controller:=True` in launch |
| **Actuation & Control** | Noisy Odometry Generator (C++ & Python) | **Implemented** | `src/bumperbot_controller/src/noisy_controller.cpp` | Publishes `odom_noisy` with synthetic Gaussian noise |
| **Teleoperation** | Joystick & Priority Multiplexing | **Implemented** | `src/bumperbot_controller/launch/joystick_teleop.launch.py` | Uses `twist_mux`, `joy`, `joy_teleop`, and `twist_relay` |
| **Hardware Integration** | Custom `SystemInterface` Hardware Plugin | **Implemented** | `src/bumperbot_firmware/src/bumperbot_interface.cpp` | `hardware_interface.launch.py`, loaded via pluginlib |
| **Hardware Integration** | Serial Receiver & Transmitter Nodes | **Implemented** | `src/bumperbot_firmware/src/simple_serial_receiver.cpp` | Bidirectional serial bridge for Arduino communication |
| **Hardware Integration** | MPU6050 I2C Linux Driver Node | **Implemented** | `src/bumperbot_firmware/bumperbot_firmware/mpu6050_driver.py` | I2C direct register reader for real hardware |
| **Embedded Firmware** | Arduino Dual PID & Encoder Sketches | **Implemented** | `src/bumperbot_firmware/firmware/` | C++ Arduino `.ino` files for low-level microcontrollers |
| **Reactive Safety** | 2-Tier LiDAR Safety Supervisor (C++ & Python) | **Implemented** | `src/bumperbot_utils/src/safety_stop.cpp` | `safety_stop` node in `simulated_robot.launch.py` |
| **Filtering & Estimation** | 1D Discrete Linear Kalman Filter (C++ & Py) | **Implemented** | `src/bumperbot_localization/src/kalman_filter.cpp` | Fuses noisy odometry with 1D dynamic system model |
| **Probabilistic Robotics**| Thrun's Sample Odometry Motion Model (C++ & Py)| **Implemented** | `src/bumperbot_localization/src/odometry_motion_model.cpp` | Monte Carlo particle propagation published on `samples` topic |
| **Local Localization** | Multi-Sensor EKF Fusion (`robot_localization`)| **Implemented** | `src/bumperbot_localization/config/ekf.yaml` | `local_localization.launch.py`, fuses `odom_noisy` + IMU |
| **Global Localization** | AMCL Particle Filter Localization (`nav2_amcl`) | **Implemented** | `src/bumperbot_localization/config/amcl.yaml` | `global_localization.launch.py`, `simulated_robot.launch.py` |
| **Mapping** | Known-Pose Ray Projection Grid Mapper (C++ & Py)| **Implemented** | `src/bumperbot_mapping/src/mapping_with_known_poses.cpp` | Standalone node converting LiDAR ranges to OccupancyGrid |
| **SLAM** | Graph-Based 2D SLAM (`slam_toolbox`) | **Implemented** | `src/bumperbot_mapping/config/slam_toolbox.yaml` | `slam.launch.py`, `simulated_robot.launch.py use_slam:=True` |
| **Pre-built Maps** | Small House Occupancy Grid Map | **Implemented** | `src/bumperbot_mapping/maps/small_house/` | `map.yaml` and `map.pgm` (0.05m resolution) |
| **Path Planning (Nav2)** | Global Path Planner (NavFn / Smac) | **NOT FOUND** | N/A | Not implemented in repository. See `10_navigation.md` |
| **Trajectory Execution**| Local Controller Server (DWB / TEB / Regulated)| **NOT FOUND** | N/A | Not implemented in repository. See `10_navigation.md` |
| **Costmaps (Nav2)** | 2D Costmap Layering (Inflation, Obstacles) | **NOT FOUND** | N/A | Not implemented in repository. See `10_navigation.md` |
| **Behavior Trees** | Nav2 BT Navigator & Recovery Execution | **NOT FOUND** | N/A | Not implemented in repository. See `10_navigation.md` |
| **Examples / Education**| 12 ROS 2 Communication Pattern Modules (C++ & Py)| **Implemented** | `src/bumperbot_cpp_examples/`, `src/bumperbot_py_examples/` | Pub/Sub, Param, Srv, Action, QoS, Lifecycle, TF2, Turtlesim |

---

## D. Technology Stack & Environment Assumptions

- **Target OS**: Ubuntu 22.04 LTS (Jammy Jellyfish) x86_64.
- **Middleware**: ROS 2 Humble Hawksbill (CycloneDDS or FastDDS default).
- **Build System**: `ament_cmake` and `ament_python` via `colcon`.
- **Simulation**: Ignition Gazebo Fortress (`ign gazebo` / `ros_gz_sim`, version 6.x).
- **Coordinate Transformations**: TF2 (`tf2_ros`, `tf2_geometry_msgs`, `tf_transformations`).
- **Control Middleware**: `ros2_control` (`controller_manager`, `diff_drive_controller`, `joint_state_broadcaster`).
- **Visualization**: RViz2 (`librviz2`).
- **Serial Communication**: `libserial-dev` (Linux userspace serial API).
- **I2C Protocol**: `python3-smbus` (Linux userspace I2C dev API).

---

## E. Architectural Constraints & Discovered Gaps

1. **Autonomous Navigation vs. Localization & SLAM**: The workspace comprehensively addresses the perception and localization portions of AMR engineering (SLAM Toolbox, Known-Pose Mapping, AMCL, and EKF). Autonomous path planning, obstacle inflation costmaps, and behavior tree navigation are **not implemented** in the repository.
2. **LiDAR Rate vs. Robot Dynamics**: The simulated LiDAR scan rate is configured in `bumperbot_gazebo.xacro`. At high rotational speeds ($\omega > 1.5\text{ rad/s}$), low update rates create rotational smearing if scans are looked up at time zero rather than `scan.header.stamp`.
3. **Serial Device Assumptions**: `bumperbot_firmware` targets `/dev/ttyUSB0` at 115200 baud. Real-robot bringup requires udev permission configuration (`dialout` group).
4. **Environment Variables for Simulation Models**: Ignition Gazebo Fortress in ROS 2 Humble relies on `IGN_GAZEBO_RESOURCE_PATH` to resolve mesh assets referenced by `model://` in SDF/World files.
