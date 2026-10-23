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
| Robot not moving despite `/cmd_vel` being published | Emergency brake active via `/safety_stop` (Lock 255 on `twist_mux`), or deadman button not held | Echo `/safety_stop` to check if obstacle is within 0.20m; verify `twist_mux` locks |

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

