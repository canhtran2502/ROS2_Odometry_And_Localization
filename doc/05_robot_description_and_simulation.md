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

---

## E. MuJoCo Simulation (`bumperbot_mujoco`)

MuJoCo is an alternative to Ignition Gazebo. Both use the same xacro (`simulator:=gazebo|mujoco`), the same worlds and the same sensor topics, so controllers, EKF, AMCL, SLAM and `safety_stop` run unchanged.

```text
robot xacro --(simulator:=mujoco)--> topic_based_ros2_control/TopicBasedSystem
                                          |  JointState topics
                                          v
 /mujoco/joint_commands  --> mujoco_sim_node (MuJoCo physics) --> /mujoco/joint_states
                                          |--> /clock, /scan (5 Hz), /imu/out (100 Hz)
```

- `ros2_control_node` hosts `diff_drive_controller` (there is no simulator plugin to host it); the hardware plugin exchanges wheel velocity commands and joint states with `mujoco_sim_node`. The xacro sets `trigger_joint_command_threshold=-1` for this plugin, otherwise it publishes commands only once the wheel positions have drifted past 1e-5 (see doc/12, 3b).
- `mujoco_sim_node` wraps `bumperbot_mujoco/sim_core.py` (pure Python, no ROS). Wheels use velocity servos, the IMU uses MuJoCo `gyro`/`accelerometer`/`framequat` sensors with the Gazebo noise values, and the lidar casts 360 rays with `mj_multiRay` against the world collision geoms only.
- Geom groups: 1 robot visual, 2 world visual, 3 robot collision, 4 world collision (hidden in the viewer, hit by the lidar).

### Ground truth and dock marker emulator (simulation only)

- `mujoco_sim_node` publishes `/mujoco/ground_truth` (`nav_msgs/Odometry`, `frame_id world`, `child_frame_id base_footprint`, 50 Hz, simulator stamp) from `BumperbotSim.base_pose()`. The world frame equals `map` (the robot spawns at the origin where AMCL starts). Existing topics are unchanged.
- `dock_marker_sim` (`config/dock_marker_sim.yaml`, charger pose `dock_x/y/yaw` = (-0.5, 0, 0), `max_range` 2.0 m, `fov` 2.1 rad, `facing: rear`, seeded noise) turns the ground truth into `/dock/detected_pose`, standing in for a fiducial detector. It is started only by `dock_sim.launch.py` with `simulator:=mujoco`.
- `battery_sim` (`bumperbot_navigation`, **simulation only**) publishes `/battery_state`; it is also started only by `dock_sim.launch.py`. The world has no dock model on purpose (the saved map and the AMCL tests depend on the world), see [10_navigation.md](10_navigation.md).

### Generated assets

`mjcf/` and `meshes/` are generated and committed, so normal users do not need the converters. Regenerate them after changing the URDF or the SDF worlds (`pip install -r src/bumperbot_mujoco/tools/requirements.txt`):

```bash
python3 src/bumperbot_mujoco/tools/generate_robot.py   # xacro -> mjcf/robot.xml + meshes/robot/
python3 src/bumperbot_mujoco/tools/sdf_to_mjcf.py      # worlds/*.world -> mjcf/<world>.xml + meshes/models/
```

Differences from Gazebo to be aware of:
- MuJoCo cannot load Collada; visual meshes are decimated STL with one flat colour per material (textures are not carried over) and collision meshes are convex decompositions (CoACD).
- Collision geometry follows two paths in `sdf_to_mjcf.py`. Small concave models (furniture) are split into at most `MAX_HULLS = 8` convex hulls with CoACD. Large thin concave structures (XY extent >= `FOOTPRINT_MIN_EXTENT` = 4 m, in practice the outer house wall `residential_HouseWallB_01`) are NOT decomposed: eight hulls cannot follow a 19 m x 11 m wall, and each hull then fills whole strips of the house (diagonal walls in the lidar scan and in every SLAM map). They become axis-aligned thin boxes from the mesh footprint: the mesh is sliced at the middle of z layers (the layer edges include every horizontal face level, so window sills and door heads are kept), rasterised at 0.05 m, the gap between the two faces of a wall (up to 0.45 m) is filled, and the raster is merged into rectangles stacked over consecutive layers. The boxes need no mesh files and the result is deterministic. Regeneration needs `scipy` in addition to the converter dependencies; install them in a virtualenv (`python3 -m venv --system-site-packages <dir>`), not into the system Python.
- Fidelity check: a lidar-style ground truth of the MuJoCo world (horizontal `mj_ray` at the laser height from free reference cells) compared with `maps/small_house/map.pgm` through `map_compare.compare_maps` gives inlier fraction ~0.99, max tile rotation < 1 deg and max tile shift 0.10 m (AC11).
- All world models are static (Gazebo treats a few warehouse props as dynamic).
- Wheel/ground friction and caster behaviour are tuned for MuJoCo; expect slightly different wheel slip than Gazebo.

### MuJoCo contact parameters (why the model has `priority` and `solref`)

Wheel odometry in the simulation must agree with the true motion, otherwise SLAM gets a biased odometry prior and draws a warped (fanned) map. Two contact settings in `generate_robot.py` (and therefore `mjcf/robot.xml`) make that true:

| Geom | Setting | Reason |
|---|---|---|
| `caster_front_link_collision`, `caster_rear_link_collision` | `condim="1" priority="1"` | MuJoCo mixes the contact parameters of the two geoms: with equal priority it takes the larger `condim`, and the ground has `condim 3`, so a caster `condim="1"` alone is ignored (contacts stay `dim 3`, frictional). The sticky casters then dragged the robot and over-reported rotation by about 4 %. `priority="1"` makes the caster's frictionless contact win. |
| `wheel_left_link_collision`, `wheel_right_link_collision` | `priority="1" solref="0.004 1"` | The default soft contact (`solref 0.02 1`) lets the wheels sink about 0.5 mm, which shrinks the effective wheel radius (about 1 % heading scale error). A stiffer contact fixes it; keep `solref[0]` at least 2 x the timestep (0.002 s). `priority="1"` also keeps the wheel friction (1.5) and `solref` in effect. |

Result (`BumperbotSim`, encoder dead reckoning, empty world): 2 m square heading error below 1 deg, 4 turns in place below 3 deg, 3 m straight below 2 mm. Only the MuJoCo model changed; the URDF, Gazebo, `ros2_control` parameters and the real robot are untouched. `test/test_odometry_drift.py` pins this behavior. The generator needs `fast_simplification` (trimesh decimation) for the meshes; the contact settings can be checked with `python3 -c "import mujoco; ..."` by looking at `contact.dim` of the casters (must be 1).

```bash
ros2 launch bumperbot_mujoco mujoco.launch.py world_name:=small_house   # empty | small_house | small_warehouse
```
