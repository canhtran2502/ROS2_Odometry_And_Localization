# 07 - LiDAR & Sensor Processing

## A. Objective

This document covers sensor data structures, polar-to-Cartesian transformations, IMU reference frame conversions, and the autonomous reactive safety supervisor implemented in `bumperbot_utils`.

---

## B. 2D LiDAR Principles & `LaserScan` Processing

### 1. `sensor_msgs/msg/LaserScan` Anatomy
Planar range data is communicated via the standard ROS 2 `LaserScan` message:
- `header.stamp`: The exact time the scan was initiated.
- `header.frame_id`: Coordinate frame of the laser emitter (`laser_link`).
- `angle_min` / `angle_max`: Start and end angles of the scan arc ($[0, 2\pi]\text{ rad}$).
- `angle_increment`: Angular resolution between adjacent beams ($\Delta \theta = \frac{2\pi}{360} \approx 0.01745\text{ rad} = 1^\circ$).
- `time_increment`: Time between successive beam measurements ($\approx 0.0\text{ s}$ for simultaneous GPU simulation).
- `scan_time`: Period between complete rotations ($0.05\text{ s}$ at 20 Hz).
- `range_min` / `range_max`: Hardware cutoff limits ($0.12\text{ m}$ to $12.0\text{ m}$).
- `ranges[]`: Array of float range measurements of length $N = 360$.

### 2. Polar to Cartesian Transformation with Sensor Offsets
For beam index $i \in [0, N-1]$ with distance $r = \text{ranges}[i]$:
1. Polar beam angle in the sensor frame:
   $$\theta_L = \text{angle\_min} + i \cdot \text{angle\_increment}$$
2. Point in `laser_link` coordinates:
   $$x_L = r \cos(\theta_L), \quad y_L = r \sin(\theta_L)$$
3. Transformation to robot base frame (`base_link` or `odom`):
   Because the sensor is mounted with a $180^\circ$ ($3.14\text{ rad}$) yaw rotation on BumperBot, the orientation transforms through:
   $$x_O = r \cos(\theta_L + \text{yaw}_{\text{sensor}}) + t_x$$
   $$y_O = r \sin(\theta_L + \text{yaw}_{\text{sensor}}) + t_y$$

> [!WARNING]
> **Invalid Range Filtering**: Raw LiDAR data routinely contains non-finite values (`std::isinf` or `std::isnan`) when beams do not return (free space or sky) or when specular reflections fail. All processing algorithms must explicitly validate:
> ```cpp
> if (std::isinf(r) || std::isnan(r) || r < scan.range_min || r > scan.range_max) {
>     continue; // Skip invalid measurements
> }
> ```

---

## C. Autonomous Reactive Safety Supervisor (`safety_stop`, legacy)

> The simulation stack now starts `safety_supervisor` (section C2) instead of this node. `safety_stop` stays in the package unchanged and is started only with `ros2 launch bumperbot_bringup simulated_robot.launch.py safety:=legacy`.

Implemented in `src/bumperbot_utils/src/safety_stop.cpp` and `bumperbot_utils/safety_stop.py`, this node acts as an autonomous virtual bumper protecting the robot against collisions.

```
       +---------------------------------------------+
       |               FREE SPACE                    |
       |  (Range > 0.60m) - Normal Max Speed         |
       |                                             |
       |       +-----------------------------+       |
       |       |       WARNING ZONE          |       |
       |       |  (0.20m < Range <= 0.60m)   |       |
       |       |  Triggers /joy_turbo_decrease       |
       |       |                             |       |
       |       |       +-------------+       |       |
       |       |       | DANGER ZONE |       |       |
       |       |       | (r <= 0.20m)|       |       |
       |       |       | Hard Brake  |       |       |
       |       |       | Lock 255    |       |       |
       |       |       +-------------+       |       |
       +-------+-----------------------------+-------+
```

### 1. Finite State Machine Logic
```
          [FREE] (Range > 0.60m)
            |               ^
            | r <= 0.60m    | r > 0.60m (Calls /joy_turbo_increase)
            v               |
        [WARNING] ----------+
            |
            | r <= 0.20m
            v
         [DANGER] (Publishes /safety_stop = True -> twist_mux Lock 255)
```

- **`State::FREE`**: All obstacles $> 0.60\text{ m}$. Calls action server `/joy_turbo_increase` to restore velocity scale. Publishes `safety_stop: False`.
- **`State::WARNING`**: Any valid beam $\le 0.60\text{ m}$ but $> 0.20\text{ m}$. Asynchronously dispatches a goal to `/joy_turbo_decrease`, cutting manual teleop speed. Disks on `/zones` highlight yellow.
- **`State::DANGER`**: Any valid beam $\le 0.20\text{ m}$. Immediately publishes `True` on `/safety_stop`. `twist_mux` activates lock 255 and stops the robot. Disks on `/zones` highlight red.

### 2. RViz2 Visual Zones
The node generates two concentric visual disks using `visualization_msgs/msg/MarkerArray` on `/zones`:
- Marker ID 0: Yellow translucent cylinder representing the warning threshold ($2 \times \text{warning\_distance} = 1.2\text{ m}$ diameter).
- Marker ID 1: Red translucent cylinder representing the danger threshold ($2 \times \text{danger\_distance} = 0.4\text{ m}$ diameter).

---

## C2. Safety Supervisor (`safety_supervisor`)

`safety_supervisor` (`bumperbot_utils`, C++ `safety_supervisor` and Python `safety_supervisor.py`, same parameters, topics, services and reason strings) is the safety layer of the simulation stack. It works independent of Nav2 and uses only `/scan`, odometry, TF and the velocity command stream. The pure logic lives in `safety_logic.hpp/.cpp` and `bumperbot_utils/safety_logic.py` (no ROS; one shared case table runs in pytest and gtest). Parameters: `config/safety_supervisor.yaml`. The legacy `safety_stop` node (section C) stays in the package unchanged and is started only with `safety:=legacy`.

**Zones.** Each valid scan point is transformed into `base_footprint` (the LiDAR is yawed by pi, so the TF `base_footprint <- laser_link` is required). Points in the front corridor (ahead of the footprint front edge, footprint half width + `lateral_margin`) limit only forward motion; points in the rear corridor limit only reverse motion; points beside the robot or inside the footprint limit nothing. With `d` the clearance from the footprint edge:

| Zone | Condition | Effect |
|---|---|---|
| CONTACT | `d <= contact_margin` (0.015 m) | hard stop; latches the e-stop while moving toward it (`latch_on_contact`) |
| STOP | `d <= stop_margin` (0.05 m); left at `d > stop_margin + stop_release_hysteresis` | hard stop in that direction; a brake pulse if the robot moves toward it |
| WARNING | `d < stop_margin + slowdown_distance` (0.35 m) | speed limit that never increases as `d` decreases |
| CLEAR | otherwise | `max_linear_speed` (0.7 m/s) |

The speed limit is `min(max_linear_speed, ramp, braking)` with `ramp = max_linear_speed * (d - stop_margin) / slowdown_distance` and `braking = max_decel * (-response_time + sqrt(response_time^2 + 2 (d - stop_margin) / max_decel))`. Note the exact meaning of the zone label: WARNING is set when the speed limit is below `max_linear_speed`, which the braking term can already cause before `d` reaches `stop_margin + slowdown_distance`. So the allowed speed (`max_forward_speed`, `max_reverse_speed`) can be below `max_linear_speed` while the zone label is still CLEAR; use the speed fields, not the label, to know the limit. An obstacle behind the robot never limits forward motion and an obstacle ahead never limits reverse motion.

**Two enforcement channels (`enforcement: filter`, default).**
- Speed filter: in the simulation stack the `twist_mux` output is remapped to `safety/cmd_vel_in`. The supervisor publishes the limited command on `bumperbot_controller/cmd_vel_unstamped` (consumed by `twist_relay`), so every source (joystick, keyboard, Nav2, docking, exploration) is slowed without Nav2. `v` is clamped to `[-max_reverse, +max_forward]`, `w` to `max_angular_speed`; a stale (`cmd_timeout`) or non-finite command gives zero (not a fault, no lock).
- The `twist_mux` `safety_stop` lock (priority 255, `twist_mux_locks.yaml` unchanged). The supervisor is its only publisher in the default stack and republishes it at 20 Hz. It is `true` in INITIALIZING, FAULT, ESTOP, SHUTDOWN and during a brake pulse.

**Brake pulse.** When the zone of the travel direction becomes STOP (or CONTACT) while the robot moves toward it, the lock is raised until odometry reports `|v| < stopped_speed` for at least `brake_hold_time`. Afterwards the filter keeps blocking motion toward the obstacle and lets motion away from it pass, so the robot can always back out. Travel direction comes from the odometry sign (command sign when the robot is stopped).

**States.** `INITIALIZING` (no scan yet, TF missing, no odometry yet: locked, zero output, the reason names the input), `ACTIVE`, `DOCK_APPROACH`, `FAULT` (locked, zero output, auto-recovers after `recovery_time` of healthy data, or latches the e-stop with `latch_faults: true`), `ESTOP` (latched) and `SHUTDOWN`. Priority: SHUTDOWN > ESTOP > INITIALIZING > FAULT > DOCK_APPROACH / ACTIVE.

**Reason strings (`/safety/status` `reason`).** `waiting for scan`, `waiting for TF base_footprint <- laser_link`, `waiting for odom`, `scan timeout`, `odom timeout`, `no valid scan`, `invalid scan: empty frame_id`, `scan frame changed`, `invalid scan: empty ranges`, `invalid scan: angle_increment <= 0`, `invalid scan: nan ratio <r>`, `internal error: <what>`, `estop input`, `contact zone (front|rear)`, `fault: <reason>` (latched by a fault), `shutdown`; in ACTIVE `front zone <ZONE>`, `rear zone <ZONE>`, `brake pulse`, `dock approach`, `dock approach time limit`, `dock status stale`, `no dock status`, `dock state <STATE>`, `dock out of radius`, `dock TF unavailable`, `dock approach disabled`, `dock profile unavailable in lock_only`. The reason is empty only in ACTIVE with both zones CLEAR.

**Watchdog.** Ages use the steady clock and the loop runs on a wall timer, so protection continues when `/clock` stalls. A scan older than `scan_timeout` (0.6 s), odometry older than `odom_timeout` (0.5 s) or an invalid scan gives FAULT. A tick exception gives FAULT `internal error: ...` and the node keeps running. On SIGINT/SIGTERM the node publishes status SHUTDOWN, the lock `true` and a zero command three times, and exits with code 0. The lock stays engaged afterwards (see doc/12).

**Latched e-stop.** Triggered by the service `/safety_supervisor/trigger_estop` (`std_srvs/Trigger`), by `safety/estop` (`std_msgs/Bool`, true) or by a contact-zone violation while moving toward it. It stays latched until `/safety_supervisor/reset` succeeds. The reset is refused (`success: false`) with `estop input active` while `safety/estop` is still true, `fault: <reason>` while an input fault exists, and `obstacle in contact zone (front)` / `(rear)` while an obstacle is inside the contact zone (the rear is exempt while the dock profile holds). An obstacle in a STOP zone does not block the reset, but the filter keeps blocking motion toward it afterwards. Reset when not latched returns `success: true`, `not latched`.

**Dock-approach profile.** Active only while `dock_approach_enabled`, `enforcement: filter`, `/dock/status` is fresh (`dock_status_timeout`) and `DOCKING`, the robot is within `dock_approach_radius` of `DockStatus.dock_pose` (TF `DockStatus.frame_id -> base_footprint`) and `dock_approach_max_time` has not elapsed in this episode. It changes only the rear direction: the rear stop margin becomes `dock_stop_margin`, the rear warning ramp becomes a hard reverse cap `dock_max_speed` (0.06 m/s) and the rear contact latch is suspended. Front protection, the lock, the e-stop and the watchdog are unchanged. The profile re-arms only after `/dock/status` leaves DOCKING. The lock is never bypassed. See doc/10.

**`lock_only` mode.** `enforcement:=lock_only` makes the node subscribe read-only to the observed command (`bumperbot_controller/cmd_vel_unstamped`), publish no velocity and drive only the lock (brake on motion toward an occupied stop zone, e-stop, watchdog). Slowdown and the dock profile are unavailable and the status says so. This is the hardware recipe:

```bash
# Real robot: start a LiDAR driver (none is provided in this repo), then
ros2 launch bumperbot_utils safety_supervisor.launch.py enforcement:=lock_only
```

**Decision: `real_robot.launch.py` is not changed and does not start `safety_supervisor` (or `safety_stop`).** No LiDAR driver is started by it, so a fail-safe supervisor would stay in INITIALIZING with the 255 lock engaged forever and would also disable the joystick, a silent breaking change on hardware. The `filter` channel also needs the `twist_mux` output rewired (`joystick_teleop.launch.py cmd_vel_out:=safety/cmd_vel_in`), which is a wiring decision for the user. Without starting the supervisor yourself, nothing provides the lock on hardware, as before.

**Interfaces.** `safety/status` (`bumperbot_msgs/msg/SafetyStatus`, transient_local, 5 Hz and on every change): state, enforcement, active/front/rear zone, clearances, nearest obstacle distance and bearing, allowed speeds, lock, brake, e-stop latch, dock profile flag, reason and input ages. Not implemented on purpose: virtual keepout / speed-limit polygons in `map` (the safety layer must stay valid when AMCL is lost; use Nav2 costmap filters).

```bash
ros2 topic echo /safety/status
ros2 service call /safety_supervisor/trigger_estop std_srvs/srv/Trigger
ros2 service call /safety_supervisor/reset std_srvs/srv/Trigger
```

---

## D. IMU Processing & Frame Sign Conversion (`imu_republisher`)

Implemented in `src/bumperbot_localization/src/imu_republisher.cpp` and `imu_republisher.py`:
- **The Problem**: In Ignition Gazebo Fortress, the IMU system plugin aligns with world orientation conventions where angular velocity around $z$ and linear accelerations may have opposite sign conventions compared to standard ROS ENU (East-North-Up) frames.
- **The Solution**: The `imu_republisher_node` subscribes to `/imu/out`, applies axis sign transformations:
  ```cpp
  imu_ekf.angular_velocity.z = -msg.angular_velocity.z;
  imu_ekf.linear_acceleration.y = -msg.linear_acceleration.y;
  imu_ekf.linear_acceleration.z = -msg.linear_acceleration.z;
  ```
  and publishes the corrected output to `/imu_ekf` with `header.frame_id = "imu_link_ekf"`.

---

## E. Physical MPU6050 Linux I2C Driver (`mpu6050_driver`)

For real hardware, `src/bumperbot_firmware/bumperbot_firmware/mpu6050_driver.py` directly interfaces with the MPU6050 sensor via Linux I2C userspace (`smbus` on `/dev/i2c-1`):
1. **Power Configuration**: Writes `0x00` to `PWR_MGMT_1` (register `0x6B`) to wake the sensor from sleep.
2. **Register Polling**: At 100 Hz, reads 16-bit signed registers:
   - Accel $X, Y, Z$: Registers `0x3B` through `0x40`
   - Gyro $X, Y, Z$: Registers `0x43` through `0x48`
3. **Unit Conversion**:
   $$\mathbf{a} = \frac{\text{raw\_accel}}{1670.13}\text{ m/s}^2, \quad \boldsymbol{\omega} = \frac{\text{raw\_gyro}}{7509.55}\text{ rad/s}$$
4. **Publishing**: Outputs to `/imu/out` with `qos_profile_sensor_data` (Best-Effort, Volatile).

---

## F. Practical Inspection & Testing Commands

```bash
# 1. Inspect live LiDAR scan topic metadata and publish frequency
ros2 topic hz /scan
ros2 topic info /scan

# 2. Echo a single LiDAR scan to inspect range values
ros2 topic echo --once /scan --field ranges

# 3. View live safety supervisor state changes
ros2 topic echo /safety_stop
ros2 topic echo /safety/status      # SafetyStatus of safety_supervisor

# 4. Inspect IMU publication rates and covariance matrices
ros2 topic hz /imu/out
ros2 topic echo --once /imu/out --field linear_acceleration
```

