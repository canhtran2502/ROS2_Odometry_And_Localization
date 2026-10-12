# 04 - TF2 & Coordinate Frame Transformations

## A. Objective

This document explains the coordinate frame conventions, spatial kinematics, transformation trees, and TF2 infrastructure implemented in BumperBot according to **REP-105** (Coordinate Frames for Mobile Platforms). It provides the theoretical foundation and practical debugging tools for coordinate transformations in ROS 2.

---

## B. Fundamental Theory & Standards

### 1. REP-105 Mobile Platform Frame Conventions

ROS 2 robotics systems define spatial state using four primary coordinate frames:

```
[map]  <--- (Discrete corrections from AMCL / SLAM)
  |
  v
[odom] <--- (Continuous, smooth, drift-prone integration from encoders)
  |
  v
[base_footprint] <--- (Planar projection on ground z=0)
  |
  v
[base_link]      <--- (Rigid physical body center of differential axle)
```

- **`map` (World Fixed Frame)**: World-fixed reference frame. The origin is arbitrary (typically where the SLAM session started or map origin). Pose estimates in `map` do not drift over long periods, but discrete jumps occur when global localization algorithms (AMCL / loop closures) update the `map -> odom` transform.
- **`odom` (Odometric Local Fixed Frame)**: Continuous, smooth coordinate system. Position and orientation are calculated by integrating high-rate wheel encoders or IMU measurements. While continuous and locally accurate (ideal for obstacle avoidance), it drifts indefinitely due to wheel slip and kinematic errors.
- **`base_footprint`**: The 2D ground projection of the robot body ($z = 0$). It isolates planar navigation and 2D mapping from pitch and roll vibrations.
- **`base_link`**: Rigidly attached to the robot's physical chassis, centered at the midpoint of the drive wheel axle.

### 2. Mathematical Representation of Coordinate Transforms

A transform from frame $A$ to frame $B$ is represented as a homogeneous transformation matrix $\mathbf{T}_A^B \in SE(3)$:

$$\mathbf{T}_A^B = \begin{bmatrix} \mathbf{R}_A^B & \mathbf{t}_A^B \\ \mathbf{0}_{1 \times 3} & 1 \end{bmatrix}$$

In ROS 2 (`geometry_msgs/msg/TransformStamped`):
- Translation vector: $\mathbf{t} = [x, y, z]^T$
- Rotation: Unit quaternion $\mathbf{q} = [q_x, q_y, q_z, q_w]^T$, where $\|\mathbf{q}\| = 1$.
Quaternions avoid singularities (gimbal lock) that affect Euler angles (Roll-Pitch-Yaw).

---

## C. BumperBot Complete TF Tree Hierarchy

```mermaid
graph TD
    map["map (Global Fixed)"] -->|nav2_amcl OR slam_toolbox| odom["odom (Odometric Smooth)"]
    
    odom -->|diff_drive_controller OR simple_controller| base_footprint["base_footprint (z=0 Ground Plane)"]
    odom -.->|noisy_controller| base_footprint_noisy["base_footprint_noisy (Corrupted State)"]
    odom -.->|robot_localization EKF| base_footprint_ekf["base_footprint_ekf (Fused State)"]
    
    base_footprint_ekf -.->|static_transform_publisher| imu_link_ekf["imu_link_ekf (Inverted Axis)"]
    
    base_footprint -->|base_joint (z=0.033m)| base_link["base_link (Chassis Axle Center)"]
    
    base_link -->|wheel_left_joint (continuous)| wheel_left_link["wheel_left_link (y=+0.0701m, r=0.033m)"]
    base_link -->|wheel_right_joint (continuous)| wheel_right_link["wheel_right_link (y=-0.0701m, r=0.033m)"]
    base_link -->|caster_front_joint (fixed)| caster_front_link["caster_front_link (x=+0.0476m, z=-0.0275m)"]
    base_link -->|caster_rear_joint (fixed)| caster_rear_link["caster_rear_link (x=-0.0476m, z=-0.0275m)"]
    base_link -->|imu_joint (fixed)| imu_link["imu_link (z=+0.0699m)"]
    base_link -->|laser_joint (fixed, yaw=3.14 rad)| laser_link["laser_link (x=-0.0051m, y=-0.0023m, z=+0.1208m)"]
```

---

## D. Detailed Joint & Transform Offsets Table

| Parent Frame | Child Frame | Transform Type | Source Broadcaster | Translation $(x, y, z)$ [m] | Rotation $(R, P, Y)$ [rad] |
|---|---|---|---|---|---|
| `map` | `odom` | Dynamic (TF) | `nav2_amcl` / `slam_toolbox` | Dynamic based on localization | Dynamic based on heading |
| `odom` | `base_footprint` | Dynamic (TF) | `diff_drive_controller` / `simple_controller` | Kinematic integration | $[\text{roll}=0, \text{pitch}=0, \theta]$ |
| `odom` | `base_footprint_noisy` | Dynamic (TF) | `noisy_controller` | Corrupted integration | $[\text{roll}=0, \text{pitch}=0, \theta_{\text{noisy}}]$ |
| `odom` | `base_footprint_ekf` | Dynamic (TF) | `robot_localization` (`ekf_node`) | EKF fused estimate | Fused quaternion |
| `base_footprint` | `base_link` | Static (URDF) | `robot_state_publisher` | $(0.0, 0.0, 0.033)$ | $(0.0, 0.0, 0.0)$ |
| `base_link` | `wheel_right_link` | Dynamic (Joint)| `robot_state_publisher` + JointStates | $(0.0, -0.0701, 0.0)$ | Continuous rotation around $y$ |
| `base_link` | `wheel_left_link` | Dynamic (Joint)| `robot_state_publisher` + JointStates | $(0.0, +0.0701, 0.0)$ | Continuous rotation around $y$ |
| `base_link` | `caster_front_link`| Static (URDF) | `robot_state_publisher` | $(+0.04755, 0.0, -0.0275)$ | $(0.0, 0.0, 0.0)$ |
| `base_link` | `caster_rear_link` | Static (URDF) | `robot_state_publisher` | $(-0.04755, 0.0, -0.0275)$ | $(0.0, 0.0, 0.0)$ |
| `base_link` | `imu_link` | Static (URDF) | `robot_state_publisher` | $(0.0, 0.0, +0.0699)$ | $(0.0, 0.0, 0.0)$ |
| `base_link` | `laser_link` | Static (URDF) | `robot_state_publisher` | $(-0.0051, -0.0023, +0.1208)$ | $(0.0, 0.0, 3.14)$ *(Reversed mount!)* |
| `base_footprint_ekf`| `imu_link_ekf` | Static (Launch) | `static_transform_publisher` | $(0.0, 0.0, 0.103)$ | Quaternion: $(1.0, 0.0, 0.0, 0.0)$ |

> [!IMPORTANT]
> **Reversed LiDAR Mounting**: In `bumperbot.urdf.xacro`, `laser_joint` has an orientation offset of `rpy="0 0 3.14"`. This means the physical LiDAR sensor is mounted backwards on the robot chassis. TF2 handles this automatically during ray transformation, mapping rear sensor angles to forward robot headings.

---

## E. Critical TF Engineering Lessons & Troubleshooting

### 1. Timestamp Matching vs. TimePointZero (`lookup_transform`)
- **The Issue**: Calling `lookup_transform("odom", "laser_link", TimePointZero)` requests the **latest available transform** currently buffered. When the robot rotates rapidly ($\omega > 1.5\text{ rad/s}$), the 30–60ms latency between scan capture and callback execution causes the robot's pose to be evaluated several degrees in the future.
- **The Fix**: Query the transform at the exact header timestamp of the sensor reading:
  ```python
  scan_time = Time.from_msg(scan.header.stamp)
  t = self.tf_buffer.lookup_transform(
      self.map_.header.frame_id,
      scan.header.frame_id,
      scan_time,
      timeout=Duration(seconds=0.05)
  )
  ```
- **Concurrency Consideration**: When calling `lookup_transform` with a timeout, the node must run on a `MultiThreadedExecutor` (or dedicated listener thread); otherwise, the single executor thread blocks, preventing `/tf` subscriber callbacks from receiving incoming data and causing a timeout.

---

## F. Practical TF Inspection Commands

```bash
# 1. View live ASCII representation of the complete TF tree
ros2 run tf2_tools view_frames
# (Generates frames.pdf in the current working directory)

# 2. Output the active transform between two specific frames
ros2 run tf2_ros tf2_echo odom base_footprint

# 3. Monitor live broadcast frequency and latency of all frames
ros2 run tf2_ros tf2_monitor

# 4. Verify static transform between laser and base_link
ros2 run tf2_ros tf2_echo base_link laser_link
```

