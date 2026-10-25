# 12 - Fault Diagnosis & Troubleshooting Guide

## A. Objective

This document provides a systematic debugging guide and root-cause analysis for common failure modes encountered during build, simulation, coordination, perception, and hardware interfacing on BumperBot.

---

## B. Master Diagnostic Matrix

| Error Message or Symptom | Root Cause | Immediate Remediation |
|---|---|---|
| `failed to create symbolic link ... Is a directory` | Switched to `--symlink-install` after a non-symlink build; existing physical directory cannot be overwritten by symlink | `rm -rf build/<package_name> install/<package_name>` and rebuild |
| `PackageNotFoundError: package 'twist_mux' not found` | Required ROS 2 system dependency not installed on Ubuntu host | `sudo apt install ros-humble-twist-mux` or run `rosdep install` |
| `Could not contact service /controller_manager/list_controllers` | Orphaned Gazebo process running in background, or Gazebo hanging while trying to download missing models | Kill zombie processes: `killall -9 ign ruby gz`<br>Set `IGN_GAZEBO_RESOURCE_PATH` |
| `Found additional publishers on /stats /clock` | Two Gazebo physics engines running concurrently, causing transport collisions | Run `killall -9 ign ruby gz 2>/dev/null` before launching |
| `Lookup would require extrapolation into the future` | Sensor message timestamp is slightly ahead of buffered TF; or `lookup_transform` called without timeout | Use `lookup_transform(..., scan_time, timeout=Duration(seconds=0.05))` with `MultiThreadedExecutor` |
| Walls blurred / curved like a fan when robot rotates | Transform looked up at `TimePointZero` instead of historical `scan.header.stamp` | Pass exact `Time.from_msg(scan.header.stamp)` to `lookup_transform` |
| SLAM map warped / fanned, doubled walls, or a seam, with MuJoCo | (a) A second (stale) simulator stack on the same `ROS_DOMAIN_ID`: two `/clock`, `/scan`, odom TF publishers and two `twist_mux` nodes. (b) Odometry slip in the MuJoCo model (see the caster `condim` row below) | `ros2 topic info /clock` and `/scan` must each show 1 publisher; kill old stacks (`pkill -f mujoco_sim_node`, section C.4) or use a dedicated `ROS_DOMAIN_ID` plus `ROS_LOCALHOST_ONLY=1`. Make sure `mjcf/robot.xml` has the caster/wheel `priority="1"` fix (doc/05) |
| MuJoCo wheel odometry over-reports rotation (about 4 % heading scale error, 14 deg after a 2 m square) | MuJoCo contact mixing: the ground has `condim 3`, so a caster geom with only `condim="1"` still gets a frictional (`dim 3`) contact and drags the robot; the soft default wheel contact (`solref 0.02 1`) also shrinks the wheel radius | Caster collision geoms need `condim="1" priority="1"`, wheel geoms `priority="1" solref="0.004 1"` (`tools/generate_robot.py`). Verify: every contact with `caster_*_link_collision` has `contact.dim == 1` |
| Parameters silently ignored in YAML files | Typo: single underscore `ros_parameters:` instead of `ros__parameters:` | Ensure every ROS 2 YAML configuration uses double underscores: `ros__parameters:` |
| `rclcpp::exceptions::ParameterNotDeclaredException` | Parameter name mismatch between `declare_parameter()` and `get_parameter()` | Ensure identical spelling in both declaration and getter |
| `Permission denied: /dev/ttyUSB0` | Linux user lacks permissions for serial communication devices | `sudo usermod -aG dialout $USER` (log out and back in) |
| `Permission denied: /dev/i2c-1` | Linux user lacks permissions for I2C devices | `sudo usermod -aG i2c $USER` |
| Robot not moving despite `/cmd_vel` being published | Emergency brake active via `/safety_stop` (Lock 255 on `twist_mux`), or deadman button not held | Echo `/safety_stop` and `/safety/status` (`reason`) to see why the supervisor locks (INITIALIZING, FAULT, ESTOP, brake pulse; section 3d) or, with `safety:=legacy`, whether an obstacle is within 0.20m; verify `twist_mux` locks |
| Diagonal or slanted "walls" in the MuJoCo lidar scan and in every SLAM map (`small_house`), tiles rotated by ~16 deg against `maps/small_house/map.pgm` | The collision geometry of the outer house wall was decomposed into only 8 convex hulls, so some hulls filled whole strips of the house | Regenerate the world with the current `tools/sdf_to_mjcf.py` (large thin concave models use box footprints instead of CoACD hulls), see doc/05. Check with a ground-truth raycast that no wall geom is a wide slab |

---

## C. Deep-Dive Failure Mode Analysis

### 1. The Gazebo Hanging & Spawner Timeout Deadlock
- **Symptoms**:
  - `spawner_joint_state_broadcaster` repeatedly logs:
    ```
    [WARN] [spawner_joint_state_broadcaster]: Could not contact service /controller_manager/list_controllers
    [INFO] [spawner_joint_state_broadcaster]: waiting for service /controller_manager/list_controllers to become available...
    ```
  - Gazebo GUI opens but displays a blank black scene, and the robot never spawns.
- **Underlying Cause**:
  In ROS 2 Humble, `controller_manager` runs as a system plugin inside the Ignition Gazebo process (`ign_ros2_control`). If Gazebo hangs while loading models (e.g. searching online Fuel repositories because `IGN_GAZEBO_RESOURCE_PATH` is missing), Gazebo never initializes the simulation clock or entity spawner. Consequently, `controller_manager` is never instantiated.
- **Definitive Fix**:
  Ensure both environment variables are exported before simulation startup:
  ```bash
  export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:$(ros2 pkg prefix bumperbot_description)/share/bumperbot_description/models
  export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(ros2 pkg prefix bumperbot_description)/share/bumperbot_description/models
  ```

---

### 2. The Single-Threaded Executor TF Deadlock
- **Symptoms**:
  - Node throws `ExtrapolationException` or hangs for 50ms on every laser callback, causing high CPU usage and dropped scans.
- **Underlying Cause**:
  Inside `scan_callback`, `lookup_transform(target, source, scan_time, timeout=Duration(seconds=0.05))` loops while calling `sleep(0.02)`. If the node runs on a `SingleThreadedExecutor`, the single executor thread is blocked by the sleep loop, preventing incoming `/tf` messages from being processed.
- **Definitive Fix**:
  Run the node using a `MultiThreadedExecutor`:
  ```python
  def main():
      rclpy.init()
      node = MappingWithKnownPoses("mapping_with_known_poses")
      executor = MultiThreadedExecutor()
      executor.add_node(node)
      try:
          executor.spin()
      finally:
          node.destroy_node()
          rclpy.shutdown()
  ```

---

### 3. The `ros__parameters` Double Underscore Trap
- **Symptoms**:
  - Parameters specified in YAML appear to be loaded, but the node uses hardcoded default values.
  - No error is printed to the console.
- **Underlying Cause**:
  The ROS 2 parameter parser strictly requires the root dictionary under the node namespace to be named `ros__parameters` (with two underscores). If written as `ros_parameters` (single underscore), the parser treats it as an arbitrary user dictionary and discards it without warning.
- **Verification**:
  Verify live parameter values using the CLI:
  ```bash
  ros2 param list <node_name>
  ros2 param get <node_name> <parameter_name>
  ```

---

### 3b. MuJoCo: robot never moves although `cmd_vel` is non-zero

Symptom: `/explore_vel` and `/bumperbot_controller/cmd_vel` carry non-zero twists, but nothing is published on `/mujoco/joint_commands`, wheel joint positions stay near 1e-8 and `/odom` stays at 0.

Cause: `topic_based_ros2_control/TopicBasedSystem::write()` only publishes when the summed absolute wheel *position* exceeds `trigger_joint_command_threshold` (default 1e-5). With a velocity-only command interface on wheels that settle without creeping, this never happens (deadlock). `bumperbot_ros2_control.xacro` sets `trigger_joint_command_threshold` to `-1.0` for the MuJoCo hardware so a command is published on every write. Gazebo and real-robot hardware are unaffected.

---

### 3c. Docking fails: `safety_stop` lock near a physical dock, missing detection, AMCL bias

- **`safety_stop` lock near a physical dock (real robot).** A real charger body within `danger_distance` (0.2 m) of the LiDAR raises the `/safety_stop` lock (255) while the robot backs in. `dock_manager` then commands zero and the attempt ends with `SAFETY_TIMEOUT` after `safety_stop_timeout`, ending as `OUTCOME_DOCKING_FAILED`. This is the intended safe behavior. Fix it in hardware or configuration (mount the dock below the scan plane, or a `danger_distance` change reviewed by the user). Never bypass the lock in docking code.
- **No detection (`OUTCOME_DETECTION_FAILED`, or `detection_used: false`).** Check `ros2 topic hz /dock/detected_pose`. In simulation `dock_marker_sim` runs only with `simulator:=mujoco` and only sees the charger within `max_range` 2.0 m and the rear field of view; with Gazebo, `detection_mode: optional` falls back to the map pose. `required` needs a detection within `detection_timeout`.
- **Final error above 0.05 m without detections.** The map-only target is limited by the AMCL error. Use a detector, or relocalize before docking.
- **Known limitations (hardware).** (a) The detection gating is meant for the simulated marker: a real detector needs a stamp check (stamp 0 skips the age check), a yaw gate and a bound relative to the map prior. (b) A constant odom TF lag between `tf_stale_timeout` (0.5 s) and `tf_timeout` (1.0 s) pauses the control phases without their timers firing; synchronize the clocks. (c) A manual dock that runs longer than `dock_request_timeout` (120 s) while the battery is low can drive `battery_monitor` to ERROR (`~/reset` leaves it). (d) A `safety_stop` lock that was already active before `dock_manager` started is not seen (`safety_stop` publishes only on change); `twist_mux` still blocks the motion, the attempt then ends as STALL.
- **`safety_stop` on the real robot.** `real_robot.launch.py` does not start `safety_stop`; docking relies on the lock only if you start `safety_stop` yourself.
- **DOCK rejected.** The reply says why: mission active, `map -> base_footprint` or `odom -> base_footprint` missing, Nav2 `navigate_to_pose` not ready, or the state is not DOCKED for UNDOCK.

---

### 3d. Safety supervisor: locked at startup, FAULT, lock kept after exit, reset refusals

- **Robot does not move right after start (INITIALIZING).** The supervisor locks the robot (`/safety_stop` true, zero output) until it has a valid scan, the TF `base_footprint <- laser_link` and odometry. `ros2 topic echo /safety/status` shows the missing input in `reason` (`waiting for scan`, `waiting for TF ...`, `waiting for odom`). It resolves itself once the input arrives; start the simulator / LiDAR driver first.
- **FAULT `scan timeout` / `odom timeout`.** A scan older than `scan_timeout` (0.6 s; the MuJoCo scan runs at 5 Hz) or odometry older than `odom_timeout` (0.5 s) locks the robot; it recovers after `recovery_time` (0.5 s) of healthy data. With `latch_faults: true` it becomes an ESTOP that needs `/safety_supervisor/reset`. Other FAULT reasons: `invalid scan: nan ratio`, `invalid scan: empty ranges`, `invalid scan: angle_increment <= 0`, `scan frame changed`, `internal error: ...`.
- **Zone says CLEAR but the robot is slow.** WARNING means the speed limit is below `max_linear_speed`, so `max_forward_speed` in `/safety/status` can already be below the maximum while `front_zone` still reads CLEAR (the braking-distance limit starts earlier than the ramp). Read the speed fields.
- **Lock stays engaged after the supervisor exits.** The `twist_mux` lock timeout is 0.0, so the last value (`true` on exit or on a startup error) stays in force and the robot stays stopped, and the joystick is disabled too. This is the fail-safe behavior. Start a new supervisor, or for an intentional removal release it: `ros2 topic pub --once /safety_stop std_msgs/msg/Bool "{data: false}"`.
- **`/safety_supervisor/reset` refused.** `estop input active`: the `safety/estop` topic is still true. `fault: <reason>`: a scan/odom/TF fault exists. `obstacle in contact zone (front)` / `(rear)`: back away manually first (the rear is exempt during the dock profile). An obstacle in the STOP zone does not block the reset; the filter then still blocks motion toward it and lets you reverse out. `not latched` is a success reply.
- **Supervisor refuses to start.** A wrong parameter type (quoted number, int for a double) or an invalid value (`footprint`, margins, `enforcement`) makes the node publish `safety_stop: true`, log the reason and exit with code 1. Fix the parameter file. A custom `footprint` must contain the wheels (default y +-0.125 m), otherwise the robot can touch obstacles beside the wheels without a stop.
- **Legacy node kept.** `ros2 launch bumperbot_bringup simulated_robot.launch.py safety:=legacy` restores the old `safety_stop` node and wiring (see the matrix and section 3c).
- **Real robot.** `real_robot.launch.py` does not start `safety_supervisor` (nor `safety_stop`), because no LiDAR driver is started and a fail-safe supervisor would sit in INITIALIZING with the lock engaged forever. Start your LiDAR driver and `ros2 launch bumperbot_utils safety_supervisor.launch.py enforcement:=lock_only`; without it nothing provides the lock.
- **Known limitation.** Returns closer than the LiDAR `range_min` (0.12 m from the laser) are invisible, so an obstacle hugging the front edge can be missed. Zones limit linear motion only; a corner can touch a side obstacle during in-place rotation.

### 4. Cleaning Zombie Background Processes
(MuJoCo: an old `mujoco_sim_node`, `ros2_control_node` or `twist_mux` from an earlier launch keeps publishing `/clock`, `/scan` and TF on the same domain and corrupts a new run. Check with `ros2 topic info /clock`; clean up with `pkill -f mujoco_sim_node`, `pkill -f ros2_control_node`, `pkill -f twist_mux`.)

When terminating ROS 2 launch files with `Ctrl+C`, subsidiary processes (especially Ignition Gazebo GUI, Ruby daemon, or controllers) may remain active as orphaned processes holding socket ports.

Always execute this cleanup command when encountering unexpected communication collisions:
```bash
killall -9 ign ruby gz 2>/dev/null
killall -9 robot_state_publisher 2>/dev/null
pkill -f "controller_manager"
pkill -f "ros_gz_bridge"
```

