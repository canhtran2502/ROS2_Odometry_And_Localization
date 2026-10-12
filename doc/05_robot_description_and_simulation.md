# 05 - Robot Description & Simulation Infrastructure

## A. Objective

This document covers the mechanical modeling, kinematic descriptions, inertial properties, and physics simulation of BumperBot using URDF, Xacro, and Ignition Gazebo Fortress in ROS 2 Humble.

---

## B. Robot Mechanical Architecture & Kinematics

BumperBot is a compact two-wheel differential drive mobile robot supported by passive front and rear casters:
- **Drive Wheel Radius ($r$)**: $0.033\text{ m}$ ($33\text{ mm}$).
- **Track Width / Wheel Separation ($L$)**: $0.170\text{ m}$ ($170\text{ mm}$ nominal axle separation; distance from origin to each wheel center is $0.07011\text{ m}$).
- **Chassis Dimensions**: Circular planar footprint with radius $\approx 0.10\text{ m}$.
- **Ground Clearance**: $0.033\text{ m}$ from ground to chassis axle center.
- **Support Mechanism**: Passive spherical casters located along the longitudinal axis at $x = \pm 0.04755\text{ m}$, radius $0.005\text{ m}$.

```
                 Front (+X)
                     ^
                     |
               [Front Caster] (x=+0.04755m)
                     |
 [Left Wheel] -------+------- [Right Wheel] (y=±0.07011m)
 (r=0.033m)          |        (r=0.033m)
               [Rear Caster]  (x=-0.04755m)
                     |
```

---

## C. URDF & Xacro Structural Decomposition

The robot description resides in `src/bumperbot_description/urdf/` and is divided into three modular Xacro files:

### 1. `bumperbot.urdf.xacro` (Core Mechanical Structure)
Defines all links, visual geometry, collision boundaries, and inertial tensors:
- **`base_footprint`**: Virtual root link on the ground surface ($z = 0$). No inertia or collision.
- **`base_link`**: Rigid chassis body centered at $(0, 0, 0.033)$ above `base_footprint`. Mass: $\approx 0.83\text{ kg}$. Visual and collision use `base_link.STL`.
- **`wheel_right_link` & `wheel_left_link`**: Differential drive wheels attached via continuous joints (`wheel_right_joint` and `wheel_left_joint`) rotating about the $y$-axis $[0, 1, 0]$.
  - Mass: $\approx 0.053\text{ kg}$ per wheel.
  - Visual: `wheel_*.STL`.
  - Collision: Spheres of radius $0.033\text{ m}$. Spheres provide stable single-point ground contact for ODE physics engines, avoiding mesh penetration artifacts.
- **`caster_front_link` & `caster_rear_link`**: Attached via fixed joints to `base_link`. Collision is a sphere of radius $0.005\text{ m}$.
- **`imu_link`**: Rigidly mounted at $z = +0.0699\text{ m}$ above `base_link`.
- **`laser_link`**: Rigidly mounted at $x = -0.0051\text{ m}, y = -0.0023\text{ m}, z = +0.1208\text{ m}$ with yaw $= 3.14\text{ rad}$ (180° reversed mounting).

### 2. `bumperbot_gazebo.xacro` (Physics & Sensors Plugin Integration)
Configures ODE surface contacts, simulation plugins, and virtual sensors:
- **Wheel Surface Friction**:
  ```xml
  <gazebo reference="wheel_left_link">
    <mu1>1000000000000000.0</mu1>
    <mu2>1000000000000000.0</mu2>
    <kp>1000000000000.0</kp>
    <kd>10.0</kd>
    <minDepth>0.001</minDepth>
    <maxVel>0.1</maxVel>
    <fdir1>1 0 0</fdir1>
  </gazebo>
  ```
  High Coulomb friction coefficients ($\mu_1, \mu_2$) with primary friction direction `fdir1="1 0 0"` ensure positive forward traction without lateral slippage.
- **Caster Surface Friction**:
  Low friction ($\mu_1 = 0.1, \mu_2 = 0.1$) allows the spherical casters to slide omnidirectionally over the ground without causing mechanical bind.
- **Distro Compatibility**:
  Uses Xacro conditionals (`<xacro:if value="$(arg is_ignition)">`) to configure plugins between ROS 2 Humble (`ign_ros2_control-system`, `ignition-gazebo-imu-system`) and Iron/Jazzy (`gz_ros2_control-system`, `gz-sim-imu-system`).
- **Virtual Sensors**:
  1. **6-DoF IMU**: 100 Hz update rate, topic `/imu`. Gaussian noise: $\sigma = 2 \times 10^{-4}\text{ rad/s}$ (gyro), $\sigma = 1.7 \times 10^{-2}\text{ m/s}^2$ (accel).
  2. **2D LiDAR**: `gpu_ray` sensor on `laser_link`, 20 Hz update rate, 360 horizontal samples across $2\pi$ radians, range $0.12\text{ m} - 12.0\text{ m}$, Gaussian noise $\sigma = 0.01\text{ m}$.

### 3. `bumperbot_ros2_control.xacro` (Hardware System Abstraction)
Defines the `ros2_control` hardware tag:
- **Simulation (`is_sim:=True`)**: Loads `ign_ros2_control/IgnitionSystem`.
- **Real Robot (`is_sim:=False`)**: Loads `bumperbot_firmware/BumperbotInterface` communicating over `/dev/ttyUSB0`.
- **Exposed Interfaces**: Velocity command interface ($[-1, 1]\text{ rad/s}$ limits), position state interface, and velocity state interface for each wheel joint.

---

## D. Ignition Gazebo Simulation Pipeline & Bridge

The launch file `src/bumperbot_description/launch/gazebo.launch.py` manages the simulation lifecycle:

1. **Environment Variables**:
   Sets `IGN_GAZEBO_RESOURCE_PATH` and `GZ_SIM_RESOURCE_PATH` to include workspace model directories and AWS RoboMaker model packs.
2. **Xacro Evaluation**:
   Converts Xacro to plain URDF via `xacro $(find bumperbot_description)/urdf/bumperbot.urdf.xacro is_ignition:=True`.
3. **World Selection**:
   Loads the requested `.world` file (`empty.world`, `small_house.world`, or `small_warehouse.world`) via `ros_gz_sim/launch/gz_sim.launch.py`.
4. **Entity Spawning**:
   Spawns the robot model into Gazebo using `ros_gz_sim/create` subscribing to `/robot_description` topic.
5. **Bidirectional Communication Bridge**:
   Runs `ros_gz_bridge/parameter_bridge` to transfer messages between Ignition transport and ROS 2:
   - `/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock` (Simulation time)
   - `/imu@sensor_msgs/msg/Imu[gz.msgs.IMU` (Inertial measurements, remapped to `/imu/out`)
   - `/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan` (LiDAR scans)

```
+-----------------------------------------------------------------------------------+
|                            IGNITION GAZEBO FORTRESS                               |
|   Physics Engine (ODE)  <--->  Sensors (IMU, GPU Lidar)  <--->  ign_ros2_control |
+-----------------------------------------------------------------------------------+
         | (gz.msgs.Clock)              | (gz.msgs.IMU, Lidar)          ^ (Joint Cmds)
         v                              v                               |
+-----------------------------------------------------------------------------------+
|                        ros_gz_bridge (parameter_bridge)                           |
+-----------------------------------------------------------------------------------+
         | (/clock)                     | (/imu/out, /scan)             |
         v                              v                               |
+-----------------------------------------------------------------------------------+
|                                ROS 2 HUMBLE GRAPH                                 |
|   robot_state_publisher   <--->   controller_manager   <--->   Perception/SLAM    |
+-----------------------------------------------------------------------------------+
```

---

## E. Practical Simulation Commands

```bash
# 1. Preview the robot URDF in RViz2 with Joint State Slider GUI
ros2 launch bumperbot_description display.launch.py

# 2. Launch simulation in empty world
ros2 launch bumperbot_description gazebo.launch.py world_name:=empty

# 3. Launch simulation in small residential house world
ros2 launch bumperbot_description gazebo.launch.py world_name:=small_house

# 4. Launch simulation in small industrial warehouse world
ros2 launch bumperbot_description gazebo.launch.py world_name:=small_warehouse
```

